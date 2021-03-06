/* -----------------------------------------------------------------------------
 * pid
 * I-Grebot Dc motor PID library
 * -----------------------------------------------------------------------------
 * File        : pid.c
 * Language    : C
 * Author      : Sebastien Brulais
 * Creation    : 2013-06-07
 * -----------------------------------------------------------------------------
 * Description
 *   This is a double PID (position and speed) control library for dc motors.
 *   With dspi33F, it can handle up to 4 independant motor controls
 * -----------------------------------------------------------------------------
 * Dependency:
 *   dc_motor library
 *   encoders library
 * -----------------------------------------------------------------------------
 * Library usage
 *
 * - If you're not using QEI peripheral, a PID_GET_POSITION(_motor_channel) macro
 * has to be defined and the pPID->cfg->use_QEI should be reset.
 * - If you're using QEI peripheral, make sure the encoders library is included
 * - In the user defined program declare a PID process structure and configure it.
 * - Then the PID_Process() function should be called at constant rate, for 
 * instance by a Timer
 * - Update reference position of the motor with PID_Set_Ref_Postion()
 *
 *  /!\ Keep in mind that PID gains are sample time dependent
 *
 *  /!\ Do not forget to set a heap size in your linker for mallocs
 *  /!\ At least 4096 bytes /!\
 *
 * See ./example for more informations
 *
 * -----------------------------------------------------------------------------
 * Versionning informations
 * Repository: http://svn2.assembla.com/svn/paranoid_android/
 * -----------------------------------------------------------------------------
 * $Rev: 1320 $
 * $LastChangedBy: Pierrick_Boissard $
 * $LastChangedDate: 2015-04-30 23:13:02 +0200 (jeu., 30 avr. 2015) $
 * -----------------------------------------------------------------------------
 * Version     Comment                                   Author       Date
 * 1.0         Initial release                           Seb B.      2013-06-07
 * 1.1	       Separation speed/position update		 Pierrick B. 2013-12-11
 * 1.2         Adding PID Process + Testing              Pierrick B. 2014-01-04
 * -----------------------------------------------------------------------------
 */

#include "pid.h"
#include <stdlib.h>

sint32_t PID_Process(PID_struct_t *PID, sint32_t error){
    sint32_t command;
    sint32_t err_D;

    PID->last_err = PID->err;
    PID->err = error;
    PID->err_I += error;
    err_D = PID->err - PID->last_err;

    if(PID->KI!=0)
    {
        if(PID->err_I > PID->I_limit)
        {
            PID->err_I = PID->I_limit;
        }else if(PID->err_I < -PID->I_limit)
        {
            PID->err_I = -PID->I_limit;
        }
    }
    command = PID->err*PID->KP + PID->err_I*PID->KI/100 - err_D*PID->KD;

    return command;
 }

void PID_Process_Speed(PID_process_t *sPID, uint32_t position){
    sint32_t command=0;

    sPID->curr = (position - sPID->last);

    // Compute Speed errors
    command = PID_Process(sPID->PID, sPID->ref - sPID->curr);
   
    // Speed saturation
    if(sPID->speed_Limit)
    {
	if (command > sPID->speed_Limit)
	{
            command = sPID->speed_Limit;
        }else if (command < - sPID->speed_Limit)
	{
            command = - sPID->speed_Limit;
	}
    }

    // Acceleration saturation
    if(sPID->acceleration_Limit)
    {
        if ( (command - sPID->last_ref) > sPID->acceleration_Limit)
	{
            command = sPID->last_ref + sPID->acceleration_Limit;
	}else if (  (sPID->last_ref - command )  >  sPID->acceleration_Limit )
	{
            command = sPID->last_ref - sPID->acceleration_Limit;
	}
    }

    // Send new motor reference
    dc_motor_set_speed(sPID->DC_Motor_Channel,command);
    sPID->last_ref = command;
    sPID->last = position;
}

void PID_Process_Position(PID_process_t *pPID, PID_process_t *sPID, sint32_t position){
    
    sint32_t ref_speed;

    pPID->curr = position;
	
    // Compute position errors
    ref_speed = PID_Process(pPID->PID, pPID->ref - pPID->curr);
 
    // Speed saturation
    if(pPID->speed_Limit)
    {
	if (ref_speed > pPID->speed_Limit)
	{
            ref_speed = pPID->speed_Limit;
        }else if (ref_speed < - pPID->speed_Limit)
	{
            ref_speed = - pPID->speed_Limit;
	}
    }
	
    // Acceleration saturation
    if(pPID->acceleration_Limit)
    {
        if ( (ref_speed - pPID->last_ref) > pPID->acceleration_Limit)
	{
            ref_speed = pPID->last_ref + pPID->acceleration_Limit;
	}else if (  (pPID->last_ref - ref_speed )  >  pPID->acceleration_Limit )
	{
            ref_speed = pPID->last_ref - pPID->acceleration_Limit;
	}
    }

    
    if (sPID != NULL)
    {
        sPID->ref=ref_speed;
    	PID_Process_Speed(sPID,position);
    }
    else
    {
        // Send new motor reference
        dc_motor_set_speed(pPID->DC_Motor_Channel,ref_speed);
        pPID->last_ref = ref_speed;
    }
}

void PID_Process_Polar(PID_process_t *pDIST, PID_process_t *pROT, sint32_t positionR, sint32_t positionL){
    sint32_t ref_distance_command;
    sint32_t ref_rotation_command;
    
    pDIST->curr = (positionR + positionL)/2;    // compute distance
    pROT->curr = positionR - positionL;         // compute angle

    // Compute distance and rotation errors
    ref_distance_command = PID_Process(pDIST->PID, pDIST->ref - pDIST->curr);
    ref_rotation_command = PID_Process(pROT->PID, pROT->ref - pROT->curr);

    // Send new motors references
    if((ref_distance_command+ref_rotation_command)>0x7FFF)
        dc_motor_set_speed(pDIST->DC_Motor_Channel,0x7FFF);
    else
    {
        if((ref_distance_command+ref_rotation_command)<-32768)
            dc_motor_set_speed(pDIST->DC_Motor_Channel,0x8000);
        else
            dc_motor_set_speed(pDIST->DC_Motor_Channel,ref_distance_command+ref_rotation_command);
    }


    if((ref_distance_command-ref_rotation_command)>0x7FFF)
        dc_motor_set_speed(pROT->DC_Motor_Channel,0x7FFF);
    else
    {
        if((ref_distance_command-ref_rotation_command)<-32768)
            dc_motor_set_speed(pROT->DC_Motor_Channel,0x8000);
        else
            dc_motor_set_speed(pROT->DC_Motor_Channel,ref_distance_command-ref_rotation_command);
    }
  
    pDIST->last_ref = ref_distance_command;
    pROT->last_ref = ref_rotation_command;
}

void PID_Set_Coefficient(PID_struct_t *PID,sint8_t KP,sint8_t KI,sint8_t KD,uint32_t I_limit){
    // Set coefficients
    PID->KP = KP;
    PID->KI = KI;
    PID->KD = KD;
    PID->I_limit = I_limit;
}

void PID_Reset(PID_process_t *xPID){
    xPID->PID->I_limit = 0;
    xPID->PID->err = 0;
    xPID->PID->err_I = 0;
    xPID->PID->last_err = 0;
    xPID->last = 0;
    xPID->last_ref = 0;

    // Reset PID gains
    PID_Set_Coefficient(xPID->PID,0,0,0,0);
	
    PID_Set_limitation(xPID,0,0);
}

void PID_Set_limitation(PID_process_t *xPID,sint32_t S_limit, sint32_t A_limit){
    // Set Saturations
    xPID->speed_Limit = S_limit;
    xPID->acceleration_Limit = A_limit;
}

PID_process_t* pid_init(uint8_t M_channel,uint8_t E_channel,uint8_t use_QEI){
    PID_process_t *xPID;
    xPID =(PID_process_t *) malloc(sizeof(PID_process_t));
    xPID->PID=(PID_struct_t *) malloc(sizeof(PID_struct_t));
    PID_Reset(xPID);

    // Configure PID process
    xPID->DC_Motor_Channel = M_channel;
    xPID->encoder_Channel = E_channel;
    xPID->use_QEI = use_QEI;

    return xPID;
}

void PID_Set_Cur_Position(PID_process_t *pPID, sint32_t position) {
    pPID->curr = position;
}

void PID_Set_Ref_Position(PID_process_t *pPID, sint32_t position) {
    pPID->ref = position;
}

sint32_t PID_Get_Cur_Position(PID_process_t *pPID){
    return pPID->curr;
}

void PID_Set_Ref_Speed(PID_process_t *sPID, sint16_t speed) {
    sPID->ref = speed;
}

sint32_t PID_Get_Cur_Speed(PID_process_t *sPID){
    return sPID->curr;
}
