/*******************************************************************************
*  Filename:        hal_int.c
*  Revised:         $Date: 2013-04-19 15:42:53 +0200 (fr, 19 apr 2013) $
*  Revision:        $Revision: 9885 $
*
*  Description:     HAL interrupt control.
*
*  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
*
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*******************************************************************************/


/******************************************************************************
* INCLUDES
*/
#include "hal_types.h"
#include "hal_defs.h"
#include "hal_int.h"
#include "hal_board.h"


/******************************************************************************
* GLOBAL FUNCTIONS
*/
/*******************************************************************************
* @fn      halIntOn
*
* @brief   Enable global interrupts.
*
* @param   none
*
* @return  none
*/
void halIntOn(void) {
    HAL_INT_ON();
}


/*******************************************************************************
* @fn      halIntOff
*
* @brief   Turns global interrupts off.
*
* @param   none
*
* @return  none
*/
void halIntOff(void) {
    HAL_INT_OFF();
}


/*******************************************************************************
* @fn      halIntLock
*
* @brief   Turns global interrupts off and returns current interrupt state.
*          Should always be used together with halIntUnlock().
*
* @param   none
*
* @return  uint16 - current interrupt state
*/
uint16 halIntLock(void) {
    istate_t key;
    HAL_INT_LOCK(key);
    return(key);
}


/*******************************************************************************
* @fn      halIntUnlock
*
* @brief   Set interrupt state back to the state it had before calling halIntLock().
*          Should always be used together with halIntLock().
*
* @param   key
*
* @return  none
*/
void halIntUnlock(uint16 key) {
    HAL_INT_UNLOCK(key);
}