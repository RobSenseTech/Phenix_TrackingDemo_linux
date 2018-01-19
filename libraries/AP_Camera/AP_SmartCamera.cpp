#include "AP_SmartCamera.h"
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <ctype.h>
#include <stdio.h> //zing_debug 可以使用print函数

extern const AP_HAL::HAL& hal; //zing_debug 添加
SmartCamera::SmartCamera(AP_SerialManager &_serial_manager) :
    num_instances(0),
    serial_manager(_serial_manager)
{}

/*
  zing_modi
 */
bool SmartCamera::init(void)
{   
    if (num_instances != 0) {
        // init called a 2nd time?
        //printf("SmartCemera failed\n");//zing_debug
        return false;
    }

    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_SmartCamera , 0);//zingdebug SerialProtocol_SmartCamera
    if (uart == nullptr)
    {
        return false;
    }
    else{
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_SmartCamera, 0));//zingdebug SerialProtocol_SmartCamera
        //hal.uartA->printf("zing_botelv  %d\n", serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Lidar, 0)  );//zingdebug SerialProtocol_SmartCamera
        num_instances++;
    }

    uart->println("setmapping2 YUYV 640 480 20.0 JeVois DemoArUco");
    uart->println("streamon");
    uart->println("setpar serout Hard");
    return true;
}

/*
  update SmartCamera state for all instances. This should be called at
  around 10Hz by main loop
 */
//zing_modi 读取摄像头数据
void SmartCamera::update(void)
{   
    //printf("zing_debug camera_update \n");
    if (num_instances == 0) {
        return;
    }

    int16_t nbytes = uart->available();


    int16_t nCnt = 0;
    
    if(nbytes!=0){ 
        while (nbytes-- >= 0){
            char c = uart->read();
            
            if(nCnt < INLEN){
                instr[nCnt] = c;
                nCnt++;
            }
        }
        instr[nCnt] = 0;
        char * tok = strtok(instr, " \r\n");
        //hal.uartA->printf("zing_huifu  %s\n", tok  ); //zing_dubug

        int mode = 0; int targx = 0, targy = 0;
        while (tok){
            //printf("zing_debug camera %s \n",tok); //zing_debug
            switch (mode){
                case 0:
                if (strcmp(tok, "T2") == 0) mode = 1;
                else if (strcmp(tok, "ALIVE") == 0) mode = 3;//
                else if (strcmp(tok, "ALIVE") == 1) mode = 4;//正常运行
                else  mode = 1000;
                break;
                
                case 1: 
                    targx = atoi(tok); 
                    mode = 2; 
                    state.target_x = targx;
                    //hal.uartA->printf("zing_x  %d\n", state.target_x  );
                    printf("zing_debug target_x %d \n",state.target_x);//zing_debug  
                    break;
                case 2: 
                    targy = atoi(tok); 
                    mode = 0; 
                    state.target_y = targy;
                    //hal.uartA->printf("zing_y  %d\n", state.target_y  );
                    break;
                case 3: 
                    mode = 0; 
                    state.bALIVE = 0;  //zing_todo 标志位有点问题
                    break;
                case 4:
                    state.status = Camera_SetModeSuccess;//应该等于2
                    state.bALIVE = 1;
                default: 
                    state.target_x = 0;
                    state.target_y = 0;
                    mode = 0; 
                    break; // Skip any additional tokens
            }
            tok = strtok(0, " \r\n");
        }
    }
    else{
        state.target_x = 0;
        state.target_y = 0;
    }
        
    static int cnt = 0;
    if (cnt == 10){
        cnt = 0;
        uart->println("ping");
    }else{
        cnt++;
    }
}

int SmartCamera::read_target_x(void)
{
    // if (state.target_x > -50 && state.target_x < 50){
    //     return 0;
    // }else{
    //     return state.target_x;
    // }
    return state.target_x;
}

int SmartCamera::read_target_y(void)
{
    // if (state.target_y > -50 && state.target_y < 50){
    //     return 0;
    // }else{
    //     return state.target_y;
    // }
    return state.target_y;
}

bool SmartCamera::read_camera_ALIVE(void){
    return state.bALIVE;
}

void SmartCamera::set_whitebalance(bool bOn)
{
    if (bOn){
        uart->println("setcam autowb 1");
    }else{
        uart->println("setcam autowb 0");
    }  
}
