/* -----------------------------------------------------------------------------
 * CC2500
 * I-Grebot RF Module CC2500 Library
 * -----------------------------------------------------------------------------
 * File        : cc2500.c
 * Language    : C
 * Author      : Paul M.
 * Creation    : 2013-02-23
 * -----------------------------------------------------------------------------
 * Description
 *   This file is a library to be used with Microchip's dsPIC33F microcontroller
 *   familiy. This library contains necessary functions (not everything is
 *   supported/handled here) for managing a Radio-Frequency module CC2500.
 * -----------------------------------------------------------------------------
 * Versionning informations
 * Repository: http://svn2.assembla.com/svn/paranoid_android/
 * -----------------------------------------------------------------------------
 * $Rev:: 1170                                                                 $
 * $LastChangedBy:: sebastien.brulais                                          $
 * $LastChangedDate:: 2014-10-31 12:12:12 +0100 (ven., 31 oct. 2014)           $
 * -----------------------------------------------------------------------------
 * Version     Comment                                   Author       Date
 * 1.0         Initial release                           Paul M.      2013-02-23
 * 1.1         New procedure implementation              Paul M.      2014-02-15
 * -----------------------------------------------------------------------------
 */

#include "cc2500.h"

// Constant CC2500 registers configuration
// Declared in FLASH
const uint8_t cc2500_reg_cfg[CC2500_NB_REG] __attribute__((space(psv))) = {
  CC2500_CFG_IOCFG2
 ,CC2500_CFG_IOCFG1
 ,CC2500_CFG_IOCFG0
 ,CC2500_CFG_FIFOTHR
 ,CC2500_CFG_SYNC1
 ,CC2500_CFG_SYNC0
 ,CC2500_CFG_PKTLEN
 ,CC2500_CFG_PKTCTRL1
 ,CC2500_CFG_PKTCTRL0
 ,CC2500_CFG_ADDR
 ,CC2500_CFG_CHANNR
 ,CC2500_CFG_FSCTRL1
 ,CC2500_CFG_FSCTRL0
 ,CC2500_CFG_FREQ2
 ,CC2500_CFG_FREQ1
 ,CC2500_CFG_FREQ0
 ,CC2500_CFG_MDMCFG4
 ,CC2500_CFG_MDMCFG3
 ,CC2500_CFG_MDMCFG2
 ,CC2500_CFG_MDMCFG1
 ,CC2500_CFG_MDMCFG0
 ,CC2500_CFG_DEVIATN
 ,CC2500_CFG_MCSM2
 ,CC2500_CFG_MCSM1
 ,CC2500_CFG_MCSM0
 ,CC2500_CFG_FOCCFG
 ,CC2500_CFG_BSCFG
 ,CC2500_CFG_AGCTRL2
 ,CC2500_CFG_AGCTRL1
 ,CC2500_CFG_AGCTRL0
 ,CC2500_CFG_WOREVT1
 ,CC2500_CFG_WOREVT0
 ,CC2500_CFG_WORCTRL
 ,CC2500_CFG_FREND1
 ,CC2500_CFG_FREND0
 ,CC2500_CFG_FSCAL3
 ,CC2500_CFG_FSCAL2
 ,CC2500_CFG_FSCAL1
 ,CC2500_CFG_FSCAL0
 ,CC2500_CFG_RCCTRL1
 ,CC2500_CFG_RCCTRL0
};

const uint8_t cc2500_patable_cfg[CC2500_NB_PATABLE] __attribute__((space(psv))) = {
  CC2500_CFG_PATABLE0
 ,CC2500_CFG_PATABLE1
 ,CC2500_CFG_PATABLE2
 ,CC2500_CFG_PATABLE3
 ,CC2500_CFG_PATABLE4
 ,CC2500_CFG_PATABLE5
 ,CC2500_CFG_PATABLE6
 ,CC2500_CFG_PATABLE7
};

// Contains the buisness of each channel for the programmed base frequency
uint16_t chan_busy_cnt[CC2500_CHANNR_MAX] ;

// -----------------------------------------------------------------------------
// HARDWARE-RELATED FUNCTIONS
// -----------------------------------------------------------------------------

void cc2500_init_spi(void) {

  // Setup hardware I/Os configuration
  CC2500_SS_TRIS  = 0 ;
  CC2500_SCK_TRIS = 0 ;
  CC2500_SDO_TRIS = 0 ;
  CC2500_SDI_TRIS = 1 ;

  // Define I/Os remappable registers & values
  CC2500_SCK_RPN  = CC2500_SCK_PIN ;
  CC2500_SDO_RPN  = CC2500_SDO_PIN ;
  CC2500_SDI_RPN  = CC2500_SDI_PIN ;

  // Default un-active chip-select value
  CC2500_SS = 1 ;

  IFS0bits.SPI1IF = 0 ; // Clear interrupt flag
  IEC0bits.SPI1IE = 0 ; // Disable interrupt

  // Baudrate configuration
  SPI1CON1bits.PPRE   = CC2500_SPI_PPRE ;
  SPI1CON1bits.SPRE   = CC2500_SPI_SPRE ;

  SPI1CON1bits.DISSCK = 0 ; // Internal serial clock
  SPI1CON1bits.DISSDO = 0 ; // SDOx is controlled by the module
  SPI1CON1bits.MODE16 = 0 ; // 8 bit mode
  SPI1CON1bits.CKP    = 0 ; // SCK idle state is LOW level
  SPI1CON1bits.SMP    = 1 ; // Sample SDI on SCK rising edge (slave emits on falling)
  SPI1CON1bits.CKE    = 1 ; // Emits SDO on SCK falling edge (slave samples on rising)  
  SPI1CON1bits.MSTEN  = 1 ; // Master mode is enabled

  SPI1STATbits.SPIEN  = 1 ; // Enable SPI module

  SPI1BUF         = 0 ; // Clear data to be transmitted
  IFS0bits.SPI1IF = 0 ; // Clear interrupt flag
  IEC0bits.SPI1IE = 1 ; // Enable interrupt

}

inline void cc2500_enable_sdi_isr(void) {

  // Clear flag, enable interrupt
  IFS1bits.INT1IF = 0 ;
  IEC1bits.INT1IE = 1 ;
}

inline void cc2500_disable_sdi_isr(void) {

  // Clear flag, disable interrupt
  IFS1bits.INT1IF = 0 ;
  IEC1bits.INT1IE = 0 ;
}

// Setup External interrupt on SDI
void cc2500_init_sdi_isr(void) {
    
  // Just in case
  CC2500_SDI_TRIS = 1 ;

  // Map INT1 on SDI
  RPINR0bits.INT1R = CC2500_SDI_PIN ;

  // Interrupt on positive edge
  INTCON2bits.INT1EP = 0 ;

  // Define INT1 priority to middle
  //IPC5bits.INT1IP = 0b100 ;

  cc2500_disable_sdi_isr();
  
}

// -----------------------------------------------------------------------------
// SPI LOW-LEVEL FUNCTIONS
// -----------------------------------------------------------------------------

// Single or Burst TX/RX transfer (depends on _size)
// Arrays endianness: data #0       is transfered/received at first
//                    data #_size-1 ...................... at last
void cc2500_spi_txrx(uint8_t* _wr_datas, uint8_t* _rd_datas, uint8_t _size) {

  uint8_t dummy ;
  uint8_t cnt ;

  // Size must be non-null
  if(_size == 0)
    return;

  // Inidicates to slave that TX/RX will start
  CC2500_SS = 0 ;

  // Loop on all data
  for(cnt = 0; cnt < _size ; cnt++) {

    // Wait until the previous TX data was transfered
    // in the SPIxTXB buffer
    while(SPI1STATbits.SPITBF);

    // Load TX buffer with next data
    SPI1BUF = _wr_datas[cnt] ; // Mapped on SPIxTXB during writes

    // Clear RX buffer before read
    dummy = SPI1BUF;

    // Wait until the RX buffer becomes full (data is received)
    // It will happen once interrupt will be generated
    // (cleared in dedicated ISR)
    while(!SPI1STATbits.SPIRBF);

    _rd_datas[cnt] = SPI1BUF ;

    // Wait until the TX buffer becomes free again
    while(SPI1STATbits.SPITBF);

  }

  // Stop TX/RX
  CC2500_SS = 1 ;

}

// -----------------------------------------------------------------------------
// CC2500 BASIC ACCESS ROUTINES
// -----------------------------------------------------------------------------

// Send a strobe and return chip status
uint8_t cc2500_send_strobe(uint8_t _strobe_addr) {

  uint8_t wr_datas[2];
  uint8_t rd_datas[2];

  // Build HEADER_strobe
  wr_datas[0] = CC2500_BUILD_HEADER(0, 0, _strobe_addr) ;
  wr_datas[1] = 0;
  
  // Send strobe and get status
  cc2500_spi_txrx(wr_datas, rd_datas, 2);

  return rd_datas[1] ;
}


// Write a single register and return chip status
uint8_t cc2500_write_reg(uint8_t _addr, uint8_t _value) {

  uint8_t wr_datas[2];
  uint8_t rd_datas[2];

  // Build HEADER_write
  wr_datas[0] = CC2500_BUILD_HEADER(0, 0, _addr) ;
  wr_datas[1] = _value;

  // Send strobe and get status
  cc2500_spi_txrx(wr_datas, rd_datas, 2);

  return rd_datas[1] ;
}

// Read a single status register
uint8_t cc2500_read_status(uint8_t _addr) {

  uint8_t wr_datas[2];
  uint8_t rd_datas[2];

  // Build HEADER_status (burst must be set!)
  wr_datas[0] = CC2500_BUILD_HEADER(1, 1, _addr) ;
  wr_datas[1] = 0;

  // Send strobe and get status
  cc2500_spi_txrx(wr_datas, rd_datas, 2);

  return (uint8_t) rd_datas[1] ;
}

// Send some RF data
void cc2500_send_rf_data(uint8_t* _rf_datas, uint8_t _size) {

  uint8_t idx ;

  // +1 For Header byte
  uint8_t wr_datas[CC2500_FIFO_TX_DEPTH+1];
  uint8_t rd_datas[CC2500_FIFO_TX_DEPTH+1];

  if(_size > CC2500_FIFO_TX_DEPTH)
    _size = CC2500_FIFO_TX_DEPTH;
  else if(_size == 0)
    return ;

  // Build HEADER_fifo
  wr_datas[0] = CC2500_BUILD_HEADER(0, (_size==1?0:1), CC2500_ADDR_FIFO) ;
  for(idx = 1; idx <= _size ; idx++)
    wr_datas[idx] = _rf_datas[idx-1];

  // Send datas and get status
  cc2500_spi_txrx(wr_datas, rd_datas, _size+1);

}

// Receive some RF data
void cc2500_receive_rf_data(uint8_t* _rf_datas, uint8_t _size) {

  uint8_t idx ;

  // +1 For Header byte
  uint8_t wr_datas[CC2500_FIFO_TX_DEPTH+1];
  uint8_t rd_datas[CC2500_FIFO_TX_DEPTH+1];

  if(_size > CC2500_FIFO_TX_DEPTH)
    _size = CC2500_FIFO_TX_DEPTH;
  else if(_size == 0)
    return ;

  // Build HEADER_fifo
  wr_datas[0] = CC2500_BUILD_HEADER(1, (_size==1?0:1), CC2500_ADDR_FIFO) ;
  for(idx = 1; idx <= _size ; idx++)
    wr_datas[idx] = 0;

  // Send datas and get status
  cc2500_spi_txrx(wr_datas, rd_datas, _size+1);

  for(idx = 0; idx < _size ; idx++)
    _rf_datas[idx] = rd_datas[1+idx];

}

inline uint8_t cc2500_get_chip_state(void) {
  uint8_t status ;
  status = cc2500_send_strobe(CC2500_ADDR_STROBE_SNOP);
  return (status & CC2500_MASK_STATE) >> CC2500_SHIFT_STATE ;
}

inline uint8_t cc2500_get_partnum(void) {
  return cc2500_read_status(CC2500_ADDR_STATUS_PARTNUM);
}

inline uint8_t cc2500_get_version(void) {
  return cc2500_read_status(CC2500_ADDR_STATUS_VERSION);
}

inline sint8_t cc2500_get_rssi(void) {
  uint8_t rssi_reg ;
  rssi_reg = cc2500_read_status(CC2500_ADDR_STATUS_RSSI);
  return (((sint8_t) rssi_reg)>>1) - 70;
}

inline uint8_t cc2500_get_chip_marcstate(void) { 
  return cc2500_read_status(CC2500_ADDR_STATUS_MARCSTATE);
}

inline uint8_t cc2500_get_packet_status(void) {
  return cc2500_read_status(CC2500_ADDR_STATUS_PKTSTATUS);
}

// -----------------------------------------------------------------------------
// CONFIGURATION ROUTINES
// -----------------------------------------------------------------------------

void cc2500_write_cfg(void) {

  uint8_t idx ;

  // +1 For Header byte
  uint8_t wr_datas[CC2500_NB_REG+1];
  uint8_t rd_datas[CC2500_NB_REG+1];
  
  // Request IDLE state before writing any configuration
  cc2500_send_strobe(CC2500_ADDR_STROBE_SIDLE);
    
  // Wait for chip to be in IDLE
  while(cc2500_get_chip_state() != CC2500_STATE_IDLE);

  // Build HEADER_regburst, start from 1st address
  wr_datas[0] = CC2500_BUILD_HEADER(0, 1, 0) ;
  for(idx = 1; idx <= CC2500_NB_REG ; idx++)
    wr_datas[idx] = cc2500_reg_cfg[idx-1];

  // Write configuration registers
  cc2500_spi_txrx(wr_datas, rd_datas, CC2500_NB_REG+1);

  // Build HEADER_patable
  wr_datas[0] = CC2500_BUILD_HEADER(0, 1, CC2500_ADDR_PATABLE) ;
  for(idx = 1; idx <= CC2500_NB_PATABLE ; idx++)
    wr_datas[idx] = cc2500_patable_cfg[idx-1];

  // Write patable configuration
  cc2500_spi_txrx(wr_datas, rd_datas, CC2500_NB_PATABLE+1);

}

// Returns 0 if all configuration registers are correctly written
uint8_t cc2500_check_cfg(void) {

  uint8_t idx ;
  uint8_t wr_datas[CC2500_NB_PATABLE];
  uint8_t rd_datas[CC2500_NB_PATABLE];

  // Read-back all configuration registers 1 by 1
  for(idx = 0; idx < CC2500_NB_REG; idx++) {
    
    // Build header
    wr_datas[0] = CC2500_BUILD_HEADER(1, 0, idx) ;
    wr_datas[1] = 0 ;

    // Read register
    cc2500_spi_txrx(wr_datas, rd_datas, 2);

    // Check it against static configuration
    if(rd_datas[1] != cc2500_reg_cfg[idx])
      return 1;
  }

  // Prepare burst datas
  for(idx = 0; idx <= CC2500_NB_PATABLE; idx++) {
    wr_datas[idx] = CC2500_BUILD_HEADER(1, 1, CC2500_ADDR_PATABLE) ;
    rd_datas[idx] = 0;
  }

  // Send it
  cc2500_spi_txrx(wr_datas, rd_datas, CC2500_NB_PATABLE+1);

  // Check registers against static configuration
  for(idx = 1; idx < CC2500_NB_PATABLE+1; idx++)
    if(rd_datas[idx] != cc2500_patable_cfg[idx-1])
      return 2;

  // Everything is OK !
  return 0 ;

}

// -----------------------------------------------------------------------------
// PROCEDURE HANDLING ROUTINES
// -----------------------------------------------------------------------------

// Returns != 0 when procedure is finished
uint8_t cc2500_do_procedure(cc2500_proc_t _proc, uint8_t _chan) {

  static cc2500_proc_fsm_t proc_fsm = {cc2500_proc_state_ready, 0, 0, 0};

  // FSM parameters are initialized at startup (when in ready mode)
  // Also, the FSM automatically goes back to ready state once
  // finish state is reached.
  if(proc_fsm.cur_state == cc2500_proc_state_ready) {
    proc_fsm.proc = _proc;
    proc_fsm.strobe_occured = 0;
    proc_fsm.chan = _chan;
  }

  // Call processing function
  (proc_fsm.cur_state)(&proc_fsm);

  return (proc_fsm.cur_state == cc2500_proc_state_finish);
}

// The first action for all procedures is always to go
// to IDLE state at first
void cc2500_proc_state_ready(cc2500_proc_fsm_t* fsm) {
  fsm->cur_state = cc2500_proc_state_idle;  
}

// Manages the strobe to IDLE state and wait for it
void cc2500_proc_state_idle(cc2500_proc_fsm_t* fsm){
  if(!fsm->strobe_occured) {
    cc2500_send_strobe(CC2500_ADDR_STROBE_SIDLE);
    fsm->strobe_occured = 1;

  // Strobe occured, check to see if IDLE state is reached.
  // If so, cleanup strobe flag and move to next state depending on procedure
  } else if(cc2500_get_chip_marcstate() == CC2500_MARCSTATE_IDLE) {
    fsm->strobe_occured = 0;
    switch(fsm->proc) {
      case CC2500_PROCEDURE_RX:
        fsm->cur_state = cc2500_proc_state_flush_rx;
        break;

      case CC2500_PROCEDURE_CHAN_RX:
        fsm->cur_state = cc2500_proc_state_channel;
        break;

      case CC2500_PROCEDURE_FAST_TX:
        fsm->cur_state = cc2500_proc_state_cal;
        break;
    } // switch
  }
}

void cc2500_proc_state_channel(cc2500_proc_fsm_t* fsm){
  cc2500_write_reg(CC2500_ADDR_REG_CHANNR, fsm->chan);
  fsm->cur_state = cc2500_proc_state_cal;
}

void cc2500_proc_state_cal(cc2500_proc_fsm_t* fsm){
  if(!fsm->strobe_occured) {
    cc2500_send_strobe(CC2500_ADDR_STROBE_SCAL);
    fsm->strobe_occured = 1;
  } else if(cc2500_get_chip_marcstate() == CC2500_MARCSTATE_IDLE) {
    fsm->strobe_occured = 0;
    switch(fsm->proc) {
      case CC2500_PROCEDURE_CHAN_RX:
        fsm->cur_state = cc2500_proc_state_flush_rx;
        break;

      case CC2500_PROCEDURE_FAST_TX:
        fsm->cur_state = cc2500_proc_state_fstxon;
        break;
    } // switch
  }
}

void cc2500_proc_state_flush_rx(cc2500_proc_fsm_t* fsm){
  if(!fsm->strobe_occured) {
    cc2500_send_strobe(CC2500_ADDR_STROBE_SFRX);
    fsm->strobe_occured = 1;
  } else if(cc2500_get_chip_marcstate() == CC2500_MARCSTATE_IDLE) {
    fsm->strobe_occured = 0;
    fsm->cur_state = cc2500_proc_state_rx;
  }
}

void cc2500_proc_state_rx(cc2500_proc_fsm_t* fsm){
  if(!fsm->strobe_occured) {
    cc2500_send_strobe(CC2500_ADDR_STROBE_SRX);
    fsm->strobe_occured = 1;
  } else if(cc2500_get_chip_marcstate() == CC2500_MARCSTATE_RX) {
    fsm->strobe_occured = 0;
    fsm->cur_state = cc2500_proc_state_finish;    
  }
}

void cc2500_proc_state_fstxon(cc2500_proc_fsm_t* fsm){
  if(!fsm->strobe_occured) {
    cc2500_send_strobe(CC2500_ADDR_STROBE_SFSTXON);
    fsm->strobe_occured = 1;
  } else if(cc2500_get_chip_marcstate() == CC2500_MARCSTATE_FSTXON) {
    fsm->strobe_occured = 0;
    fsm->cur_state = cc2500_proc_state_finish;
  }
}

// Automatically goes back to ready state once finished
void cc2500_proc_state_finish(cc2500_proc_fsm_t* fsm){
  fsm->cur_state = cc2500_proc_state_ready;
}

// -----------------------------------------------------------------------------
// SYSTEM ROUTINES
// -----------------------------------------------------------------------------

// Scan through all channels and return the best (clearest) channel number
uint8_t cc2500_scan_channels(void) {

  // Flag indicating that a SCAN is currently running
  static uint8_t scan_started = 0;

  // Channel number of current SCAN
  static uint16_t chan;

  // Changing and calibrating current channel
  static uint8_t chan_cal;

  // Current burst index
  static uint16_t burst;

  // Re-init indexes if scan is finished
  if(!scan_started) {
    chan = 0;
    chan_cal = 0;
    burst = 0;
    scan_started = 1;
    
  // Else, if scan is started and reached last burst of last channel, stop scanning
  } else if((chan == CC2500_CHANNR_MAX-1) && (burst == CC2500_CHANNEL_SCAN_RATE-1)){
    scan_started = 0;
  }

  // Procedure-call for channel change, continue until finished
  // and memorized that channel is calibrated
  if(!chan_cal) {
    if(cc2500_do_procedure(CC2500_PROCEDURE_CHAN_RX, chan)) {
      chan_cal = 1;      
    }

  // Channel is calibrated: normal processing
  } else {

    // Burst not finished: update business counter and increment burst index
    if(burst < CC2500_CHANNEL_SCAN_RATE) {
      if(!(cc2500_get_packet_status() & CC2500_MASK_CCA)) {
        chan_busy_cnt[chan] ++;
      }
      burst++;

    // Finished current burst, move to next channel and request a new cal
    } else {
      burst = 0;
      chan++;
      chan_cal = 0;
      chan_busy_cnt[chan] = 0;
    } // burst

  } // !chan_cal

  // Returns !=0 when finished
  return !scan_started;
  
}


// Best channel selected is the middle channel in the longest
// continuous unused channels
uint8_t cc2500_find_best_channel(void) {
  
  uint8_t chan ;

  uint8_t continuous_free_nb;
  uint8_t chan_free_indexes[CC2500_CHANNR_MAX];
  uint8_t continuous_current_max;
  uint8_t continuous_current_max_idx;

  uint8_t lowest_busy_chan ;

  // 1st pass: fill indexes table
  continuous_free_nb = 0;
  for(chan = 0; chan < CC2500_CHANNR_MAX; chan++) {

      if(chan_busy_cnt[chan]) {
          //printf("[%2u] %u\n", chan, chan_busy_cnt[chan]);
          continuous_free_nb = 0;
      } else {
          continuous_free_nb++;
      }

      chan_free_indexes[chan] = continuous_free_nb;

  } // for chan

  // 2nd pass: lookup for the max of chan_free_indexes[]
  continuous_current_max = 0;
  for(chan = 0; chan < CC2500_CHANNR_MAX; chan++) {
      if(chan_free_indexes[chan] > continuous_current_max) {
          continuous_current_max = chan_free_indexes[chan] ;
          continuous_current_max_idx = chan;
      }
  }

  lowest_busy_chan = continuous_current_max_idx-(continuous_current_max>>1);

  //printf("Best chan = %u (max = %u, idx = %u)\n", lowest_busy_chan, continuous_current_max, continuous_current_max_idx);

  return lowest_busy_chan ;
}

// -----------------------------------------------------------------------------
// INTERRUPT ROUTINES
// -----------------------------------------------------------------------------

// SPI RX ISR
void __attribute__((interrupt, no_auto_psv)) _SPI1Interrupt(void) {
  IFS0bits.SPI1IF = 0 ; // Clear interrupt flag
}

