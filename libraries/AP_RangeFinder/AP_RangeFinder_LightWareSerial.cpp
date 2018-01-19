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

#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_LightWareSerial.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <stdio.h> //zing_debug 可以使用print函数

extern const AP_HAL::HAL& hal;

/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_LightWareSerial::AP_RangeFinder_LightWareSerial(RangeFinder::RangeFinder_State &_state,
                                                               AP_SerialManager &serial_manager) :
    AP_RangeFinder_Backend(_state)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Lidar, 0));
        //printf("zing_debug_botelv %d",serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Lidar, 0));
    }
}

/* 
   detect if a Lightware rangefinder is connected. We'll detect by
   trying to take a reading on Serial. If we get a result the sensor is
   there.
*/
bool AP_RangeFinder_LightWareSerial::detect(AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar, 0) != nullptr;
}

// read - return last value measured by sensor
/*bool AP_RangeFinder_LightWareSerial::get_reading(uint16_t &reading_cm)
{
    if (uart == nullptr) {
        return false;
    }

    // read any available lines from the lidar
    float sum = 0;
    uint16_t count = 0;
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        char c = uart->read();
        if (c == '\r') {
            linebuf[linebuf_len] = 0;
            sum += (float)atof(linebuf);
            count++;
            linebuf_len = 0;
        } else if (isdigit(c) || c == '.') {
            linebuf[linebuf_len++] = c;
            if (linebuf_len == sizeof(linebuf)) {
                // too long, discard the line
                linebuf_len = 0;
            }
        }
    }

    // we need to write a byte to prompt another reading
    uart->write('d');

    if (count == 0) {
        return false;
    }
    reading_cm = 100 * sum / count;
    return true;
}*/
bool AP_RangeFinder_LightWareSerial::get_reading(uint16_t &reading_cm)
{
    if (uart == nullptr) {
        return false;
    }
    // read any available lines from the lidar
    int16_t sum = 0;
    int32_t distance_sum = 0;
    //int16_t strength;
    int8_t byte_count = 0;
    char state_switch = 0;
    int16_t count = 0;
    int16_t nbytes = uart->available();//缓冲区字节数
    unsigned char byte[9];
    
    
    //hal.uartA->printf("zing-c  %x\n", c);
    while (nbytes-- > 0) 
    {
        unsigned char c = uart->read();   
        //printf("zing_debug c %x \n",c);
        switch (state_switch)
        {
            case 0://检测第一个帧头
                if(c == 0x59)
                {         
                    byte[byte_count] = c;
                    sum = sum + byte[byte_count];
                    byte_count = byte_count + 1;
                    state_switch = 1;
                }
                else
                {
                    byte_count = 0;
                    sum = 0;
                    state_switch = 0;
                }
                break;
            case 1://检测第二个帧头
                if(c == 0x59)
                {
                    byte[byte_count] = c;
                    sum = sum + byte[byte_count];
                    byte_count = byte_count + 1;
                    state_switch = 2;
                }
                else
                {
                    byte_count = 0;
                    sum = 0;
                    state_switch = 0;
                }
                break;
            case 2://读取数据字节
                byte[byte_count] = c;
                sum = sum + byte[byte_count];
                byte_count = byte_count + 1;
                if(byte_count == 8)//zing 7 
                {
                    state_switch = 3;
                }
                break;
            case 3://校验
                if((sum & 0xff) == c)
                {
                    distance_sum = distance_sum + ((int16_t(byte[3]) <<8) | int16_t(byte[2]));
                    //strength = (byte[5] << 8) |  byte[4];
                    count = count + 1;

                    //hal.uartA->printf("zing-c  %x\n", reading_cm);
                }
                else
                {
                    byte_count = 0;
                    sum = 0;
                    state_switch = 0;
                }
                break;
        }
    }
    if(count == 0)
    {
        return false;
    }
    reading_cm = distance_sum/count;
    //printf("zing_debug reading_cm %d \n",reading_cm);//zing_debug
    return true;
}

/* 
   update the state of the sensor
*/
void AP_RangeFinder_LightWareSerial::update(void)
{
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - last_reading_ms > 200) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}
