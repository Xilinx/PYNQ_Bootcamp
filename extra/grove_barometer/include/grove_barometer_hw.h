/******************************************************************************
 *  Copyright (c) 2021, Xilinx, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1.  Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2.  Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *  3.  Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS py_intERRUPTION). HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#pragma once

// Register address definitions
#define BPS_REG_RESET            0x0C        /**< Reset register                                               */
#define BPS_REG_PRSCFG           0x06        /**< Pressure configuration register                              */
#define BPS_REG_TMPCFG           0x07        /**< Temperature configuration register                           */
#define BPS_REG_MEASCFG          0x08        /**< Sensor Operating Mode and Status register                    */
#define BPS_REG_CFGREG           0x09        /**< py_interrupt and FIFO configuration register                    */
#define BPS_REG_TMP_BASE         0x03        /**< Temperature Data base register                               */
#define BPS_REG_PRS_BASE         0x00        /**< Pressure Data base register                                  */
#define BPS_REG_TMPSRC           0x28        /**< Coefficient Source register                                  */
#define BPS_REG_ID               0x0D        /**< Product and Revision ID register                             */
#define BPS_REG_COEFF_BASE       0x10        /**< Calibration Coefficients register                            */
#define BPS_REG_FIFO_STS         0x0B        /**< FIFO status Register                                         */

#define BPS_COEFFICIENT_COUNT    9           /**< Number of coefficients                                       */
#define BPS_COEFFICIENT_SIZE     18          /**< Size of all of the coefficients in bytes except c00 and c010 */

#define BPS_CMD_RESET            0x89        /**< Reset command                                                */
#define BPS_CMD_T_SHIFT    		 0x08        /**< Temperature and pressure result shift command                */
#define BPS_CMD_P_SHIFT    		 0x04        /**< Temperature and pressure result shift command                */
