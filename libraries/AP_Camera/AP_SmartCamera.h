// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>

#define INLEN 128
#define SCALE 10
#define DISTANCE 50
 
class SmartCamera
{
public:

    SmartCamera(AP_SerialManager &_serial_manager);

    enum SmartCamera_Status {
        Camera_NotConnected = 0,
        Camera_NoData,
        Camera_SetModeSuccess,
        Camera_Good
    };

    // The Camera_State structure is filled in by the backend driver
    struct SmartCamera_State {
        int16_t               target_x; 
        int16_t               target_y;  
                                            
        enum SmartCamera_Status status;
        bool bALIVE;     
    };


    // detect and initialise any available Cameras
    bool init(void);

    // update state of all Cameras. Should be called at around
    // 10Hz from main loop
    void update(void);

    int read_target_x(void);

    int read_target_y(void);
    
    bool read_camera_ALIVE(void);//zing_modi

    void set_whitebalance(bool bOn);
    
    SmartCamera_State state;

private:    
    uint8_t num_instances;
    char instr[INLEN + 1];
    int16_t pangain = 400;
    int16_t tiltgain = 300;
    int16_t distance = 50;//cm

    AP_HAL::UARTDriver *uart = nullptr;

    AP_SerialManager &serial_manager;

};
