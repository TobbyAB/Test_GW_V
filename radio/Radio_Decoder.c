/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-11-22     Rick       the first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <stdio.h>
#include "drv_spi.h"
#include <string.h>
#include "AX5043.h"
#include "Radio_Decoder.h"
#include "Radio_Encoder.h"
#include "Flashwork.h"
#include "led.h"
#include "key.h"
#include "pin_config.h"
#include "wifi-api.h"

#define DBG_TAG "RF_DE"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

uint8_t Learn_Flag=1;
uint32_t Main_ID = 0;

extern uint32_t Self_Id;

uint8_t Check_Valid(uint32_t From_id)
{
    return Flash_Get_Key_Valid(From_id);
}
void Device_Learn(Message buf)
{
    if(Flash_Get_Key_Valid(buf.From_ID) != RT_EOK)
    {
        LOG_D("Flash_Get_Key_Valid ERROR");
        if(Get_MainNums()!=RT_EOK)
        {
            learn_fail();
            return;
        }
    }



    if(buf.From_ID>=10000000 && buf.From_ID<20000000)
    {
        switch(buf.Data)
        {
        case 1:
            RadioEnqueue(buf.From_ID,buf.Counter,3,2);
            LOG_I("Learn Decoder Data is 1 and Send BackPackage\r\n");
            break;
        case 2:
            LOG_I("Learn Success\r\n");
            Del_Device(buf.From_ID);
            Device_Add2Flash_Wifi(buf.From_ID,0);
            learn_success();
        }
    }
}
void NormalSolve(int rssi,uint8_t *rx_buffer,uint8_t rx_len)
{
    Message Rx_message;
    if(rx_buffer[rx_len]==0x0A&&rx_buffer[rx_len-1]==0x0D)
     {
         sscanf((const char *)&rx_buffer[1],"{%ld,%ld,%d,%d,%d}",&Rx_message.Target_ID,&Rx_message.From_ID,&Rx_message.Counter,&Rx_message.Command,&Rx_message.Data);
         if(Rx_message.Target_ID==Self_Id)
         {
             rf_led(3);
             LOG_D("NormalSolve verify ok\r\n");
             switch(Rx_message.Command)
             {
             case 3://学习
                 LOG_D("Start_Learn Step_2");
                 Device_Learn(Rx_message);
                 break;
             }
             Heart_Upload(Rx_message.From_ID,1);
         }
     }
}
void GatewaySyncSolve(int rssi,uint8_t *rx_buffer,uint8_t rx_len)
{
    Message Rx_message;
    if(rx_buffer[rx_len]=='A')
    {
        sscanf((const char *)&rx_buffer[2],"{%d,%d,%ld,%ld,%ld,%d,%d}",&Rx_message.ack,&Rx_message.type,&Rx_message.Target_ID,&Rx_message.From_ID,&Rx_message.Device_ID,&Rx_message.Rssi,&Rx_message.Data);
        if(Rx_message.Target_ID == Self_Id && Check_Valid(Rx_message.From_ID) == RT_EOK)
        {
            rf_led(3);
            if(Rx_message.ack)
            {
                GatewayDataEnqueue(Rx_message.From_ID,0,0,7,0);
            }
            Main_Rssi_Report(Rx_message.From_ID,rssi);//上报主控的RSSI
            Heart_Upload(Rx_message.From_ID,1);//向WIFI上报主控的心跳，设置FLASH中的心跳
            switch(Rx_message.type)
            {
            case 1:
                DeviceCheck(Rx_message.Device_ID,Rx_message.From_ID);//将心跳的数据 载入内存及FLASH中
                Slave_Heart(Rx_message.Device_ID,Rx_message.Rssi);//终端 门控 心跳及RSSI 上报
                break;
            case 2:
                Local_Delete(Rx_message.Device_ID);//门 控删除
                Del_Device(Rx_message.Device_ID);//删除终端
                break;
            case 3://同步在线设备
                Sync_Refresh();
                Device_Add2Flash_Wifi(Rx_message.Device_ID,Rx_message.From_ID);//增加终端
                Flash_Set_Heart(Rx_message.Device_ID,1);
                Device_Up(Rx_message.Device_ID);//上报设备在线信息
                Slave_Heart(Rx_message.Device_ID,Rx_message.Rssi);//子设备RSSI上报
                WariningUpload(Rx_message.From_ID,Rx_message.Device_ID,2,Rx_message.Data);//报警信息上传给WIFI模块
                break;
            case 4://删除全部
                Del_MainBind(Rx_message.From_ID);
                break;
            case 5://同步离线设备
                Sync_Refresh();
                Device_Add2Flash_Wifi(Rx_message.Device_ID,Rx_message.From_ID);//增加终端
                Flash_Set_Heart(Rx_message.Device_ID,0);//设置离线子设备 全局变量 为0
                Slave_Heart(Rx_message.Device_ID,Rx_message.Rssi);//心跳
                WariningUpload(Rx_message.From_ID,Rx_message.Device_ID,2,Rx_message.Data);//终端低电量
                break;
            case 6://添加设备
                LOG_I("Add Device6\r\n");
                Device_Add2Flash_Wifi(Rx_message.Device_ID,Rx_message.From_ID);//增加终端
                Flash_Set_Heart(Rx_message.Device_ID,1);
                Device_Up(Rx_message.Device_ID);
                Slave_Heart(Rx_message.Device_ID,Rx_message.Rssi);//心跳
                WariningUpload(Rx_message.From_ID,Rx_message.Device_ID,2,Rx_message.Data);//报警信息上传给WIFI模块
                break;
            }
        }
        else {
            LOG_W("GatewaySyncSolve ID %ld Error\r\n",Rx_message.From_ID);
        }
    }
}
void GatewayWarningSolve(int rssi,uint8_t *rx_buffer,uint8_t rx_len)
{
    Message Rx_message;
    if(rx_buffer[rx_len]=='B')
    {
        sscanf((const char *)&rx_buffer[2],"{%d,%ld,%ld,%ld,%d,%d,%d}",&Rx_message.ack,&Rx_message.Target_ID,&Rx_message.From_ID,&Rx_message.Device_ID,&Rx_message.Rssi,&Rx_message.Command,&Rx_message.Data);
        if(Rx_message.Target_ID == Self_Id && Check_Valid(Rx_message.From_ID) == RT_EOK)
        {
            rf_led(3);
            if(Rx_message.ack)
            {
                GatewayDataEnqueue(Rx_message.From_ID,0,0,7,0);
            }
            Heart_Upload(Rx_message.From_ID,1);//主控心跳上报给WIFI，及FLASH存放标志位
            Main_Rssi_Report(Rx_message.From_ID,rssi);//主控上报RSSI值
            LOG_D("WariningUpload From ID is %ld,Device ID is %ld,type is %d,value is %d\r\n",Rx_message.From_ID,Rx_message.Device_ID,Rx_message.Command,Rx_message.Data);
            switch(Rx_message.Command)
            {
            case 1:
                WariningUpload(Rx_message.From_ID,Rx_message.Device_ID,1,Rx_message.Data);//水警上报
                break;
            case 2:
                WariningUpload(Rx_message.From_ID,Rx_message.Device_ID,0,Rx_message.Data);//掉线上报
                break;
            case 3:
                WariningUpload(Rx_message.From_ID,Rx_message.Device_ID,2,Rx_message.Data);//电量上报
                break;
            case 4:
                Flash_Set_Heart(Rx_message.Device_ID,0);//子设备离线 重要信息
                break;
            case 5:
                DeviceCheck(Rx_message.Device_ID,Rx_message.From_ID);
                Slave_Heart(Rx_message.Device_ID,Rx_message.Rssi);//设备RSSI更新
                WariningUpload(Rx_message.From_ID,Rx_message.Device_ID,1,Rx_message.Data);//终端水警
                MotoUpload(Rx_message.From_ID,0);//主控开关阀状态上报
                break;
            case 6:
                DeviceCheck(Rx_message.Device_ID,Rx_message.From_ID);
                Slave_Heart(Rx_message.Device_ID,Rx_message.Rssi);//设备RSSI更新
                WariningUpload(Rx_message.From_ID,Rx_message.Device_ID,2,Rx_message.Data);//电量上报
                break;
            case 7:
                InitWarn_Main(Rx_message.From_ID);//初始化主控报警
                break;
            case 8://NTC报警
                WariningUpload(Rx_message.From_ID,Rx_message.Device_ID,3,Rx_message.Data);
                break;
            case 9://终端测水线掉落
                WariningUpload(Rx_message.From_ID,Rx_message.Device_ID,3,Rx_message.Data);
                break;
            }
        }
        else {
            LOG_W("GatewayWarningSolve ID %ld Error\r\n",Rx_message.From_ID);
        }
    }
}
void GatewayControlSolve(int rssi,uint8_t *rx_buffer,uint8_t rx_len)
{
    Message Rx_message;
    if(rx_buffer[rx_len]=='C')
    {
        sscanf((const char *)&rx_buffer[2],"{%d,%ld,%ld,%ld,%d,%d,%d}",&Rx_message.ack,&Rx_message.Target_ID,&Rx_message.From_ID,&Rx_message.Device_ID,&Rx_message.Rssi,&Rx_message.Command,&Rx_message.Data);
        if(Rx_message.Target_ID == Self_Id && Check_Valid(Rx_message.From_ID) == RT_EOK)
        {
            rf_led(3);
            if(Rx_message.ack)
            {
                GatewayDataEnqueue(Rx_message.From_ID,0,0,7,0);
            }
            Main_Rssi_Report(Rx_message.From_ID,rssi);
            Heart_Upload(Rx_message.From_ID,1);
            switch(Rx_message.Command)
            {
            case 1:
                MotoUpload(Rx_message.From_ID,Rx_message.Data);//主控开关阀
                if(Rx_message.Data == 0)
                {
                    CloseWarn_Main(Rx_message.From_ID);
                }
                break;
            case 2:
                DeviceCheck(Rx_message.Device_ID,Rx_message.From_ID);
                Slave_Heart(Rx_message.Device_ID,Rx_message.Rssi);//设备RSSI更新
                if(Rx_message.Data == 0 || Rx_message.Data == 1)
                {
                    MotoUpload(Rx_message.From_ID,Rx_message.Data);//主控开关阀
                    if(Rx_message.Data == 0)
                    {
                        CloseWarn_Slave(Rx_message.Device_ID);
                    }
                }
                else
                {
                    MotoUpload(Rx_message.From_ID,0);//主控开关阀
                }
                break;
            case 3:
                if(Rx_message.Device_ID)//Delay远程关闭
                {
                    Door_Delay_WiFi(Rx_message.From_ID,Rx_message.Device_ID,Rx_message.Data);
                    DeviceCheck(Rx_message.Device_ID,Rx_message.From_ID);
                    Slave_Heart(Rx_message.Device_ID,Rx_message.Rssi);//设备RSSI更新
                }
                break;
            case 4:
                MotoUpload(Rx_message.From_ID,Rx_message.Data);
                break;
            case 5:
                MotoUpload(Rx_message.From_ID,Rx_message.Data);//主控开关阀
                Ack_Report(Rx_message.From_ID);
                InitWarn_Main(Rx_message.From_ID);//报警状态
                break;
            case 6:
                DoorUpload(Rx_message.Device_ID,Rx_message.Data);//主控开关阀
                DeviceCheck(Rx_message.Device_ID,Rx_message.From_ID);
                Slave_Heart(Rx_message.Device_ID,Rx_message.Rssi);//设备RSSI更新
                MotoUpload(Rx_message.From_ID,Rx_message.Data);//主控开关阀
                if(Rx_message.Data == 0)
                {
                    CloseWarn_Slave(Rx_message.Device_ID);
                }
                break;
            }
        }
        else
         {
             LOG_W("GatewayControlSolve ID %ld Error\r\n",Rx_message.From_ID);
         }
    }
}
void rf433_rx_callback(int rssi,uint8_t *buffer,uint8_t len)
{
    switch(buffer[1])
    {
    case '{':
        NormalSolve(rssi,buffer,len);
        break;
    case 'A':
        LOG_D("RX 433 is %s,RSSI is %d\r\n",buffer,rssi);
        GatewaySyncSolve(rssi,buffer,len);
        break;
    case 'B':
        LOG_D("RX 433 is %s,RSSI is %d\r\n",buffer,rssi);
        GatewayWarningSolve(rssi,buffer,len);
        break;
    case 'C':
        LOG_D("RX 433 is %s,RSSI is %d\r\n",buffer,rssi);
        GatewayControlSolve(rssi,buffer,len);
        break;
    }
}

