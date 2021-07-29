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

#include <grove_constants.h>
#include <grove_constants.h>
#include <pytypes.h>
#include <sys/errno.h>

// Device typedef 
typedef py_int grove_barometer;

// Device lifetime functions
grove_barometer grove_barometer_open(py_int grove_id);
grove_barometer grove_barometer_open_at_address(py_int grove_id, py_int address);
void grove_barometer_close(grove_barometer p);

// Device functions
py_int grove_barometer_configure(grove_barometer p);
py_int grove_barometer_start_conversion(grove_barometer p, int measurement_type);
py_void grove_barometer_pressure_oversample_rate(grove_barometer p, py_int value);
py_void grove_barometer_temperature_oversample_rate(grove_barometer p, py_int value);
py_void grove_barometer_pressure_measurement_rate(grove_barometer p, py_int value);
py_void grove_barometer_temperature_measurement_rate(grove_barometer p, py_int value);
py_void grove_barometer_mode(grove_barometer p, py_int value);
py_int grove_barometer_temperature_raw(grove_barometer p);
py_int grove_barometer_pressure_raw(grove_barometer p);
py_float grove_barometer_temperature(grove_barometer p);
py_float grove_barometer_pressure(grove_barometer p);
py_int grove_barometer_fifo_empty(grove_barometer p);
py_int grove_barometer_fifo_full(grove_barometer p);
py_int grove_barometer_enable_fifo(grove_barometer p, py_int value);
py_int grove_barometer_read_fifo(grove_barometer p);
py_int grove_barometer_reset(grove_barometer p);
py_int grove_barometer_status(grove_barometer p);
py_int grove_barometer_reg_read(grove_barometer p, unsigned char addr);
py_int grove_barometer_reg_write(grove_barometer p, unsigned char addr, unsigned char val);
py_float grove_barometer_calculate_pressure(grove_barometer p, int raw_temp, int raw_press);
py_float grove_barometer_calculate_temperature(grove_barometer p, int raw_temp, int raw_press);
