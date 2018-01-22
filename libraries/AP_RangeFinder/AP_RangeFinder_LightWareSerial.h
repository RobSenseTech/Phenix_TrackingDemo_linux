#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"
//zing_modi 读取L10雷达
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <arpa/inet.h>
#include <errno.h>

class AP_RangeFinder_LightWareSerial : public AP_RangeFinder_Backend
{

public:
    // constructor
    AP_RangeFinder_LightWareSerial(RangeFinder::RangeFinder_State &_state,
                                AP_SerialManager &serial_manager);

    // static detection function
    static bool detect(AP_SerialManager &serial_manager);

    // update state
    void update(void);

    struct Lidar_frame{
        uint8_t head;
        uint8_t dev_addr;
        uint8_t cmd;
        uint8_t reg_H;
        uint8_t reg_L;
        uint8_t data_H;
        uint8_t data_L;
        uint16_t reg;
        uint16_t data;
        uint16_t crc;
    };

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    // get a reading
    bool get_reading(uint16_t &reading_cm);

    AP_HAL::UARTDriver *uart = nullptr;
    uint32_t last_reading_ms = 0;
    char linebuf[10];
    uint8_t linebuf_len = 0;

};
