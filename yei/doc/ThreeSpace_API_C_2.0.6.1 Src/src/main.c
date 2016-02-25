/********************************************//**
 * This file should not be included for compiling the threespace API
 * This file is mainly a testing ground for various functionality of the API and threespace sensor
 ***********************************************/
#include <stdio.h>
#include <stdlib.h>
#include <windows.h>

#include "yei_threespace_api.h"


#pragma pack(push,1)
typedef struct {
    float quat[4];
    float linear_accel[3];
    char battery;
    char button;
} Steam_packet;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct{
    float raw_data[9];
//    char button;
} Steam_packet2;
#pragma pack(pop)

int pk_count=0;
unsigned int last_timestamp;

void printStreamData(const Steam_packet * packet);
void printStreamData2(const Steam_packet2 * packet);

void CALLBACK DataPrinter(TSS_Device_Id device, char * output_data,
                          unsigned int output_data_len,  unsigned int * timestamp){
    if(output_data_len == sizeof(Steam_packet2)){
        Steam_packet2 * packet;
        packet = (Steam_packet2 *)output_data;
        printf("t:%8u ",(*timestamp)-last_timestamp);
        printStreamData2(packet);
//        printf("Quat: %f,%f,%f,%f", packet->quat[0],
//                                     packet->quat[1],
//                                     packet->quat[2],
//                                     packet->quat[3]);
//        printf("Batt:%u Butt: %u\n",   packet->battery,
//                                            packet->button);
        last_timestamp = *timestamp;

        pk_count++;
    }
}

void printStreamData(const Steam_packet * packet){
    printf("Quat: %f,%f,%f,%f\n", packet->quat[0],
                                 packet->quat[1],
                                 packet->quat[2],
                                 packet->quat[3]);
    printf("Linear Accel: %f,%f,%f\n", packet->linear_accel[0],
                                 packet->linear_accel[1],
                                 packet->linear_accel[2]);
    printf("Battery:%u Button: %u\n",   packet->battery,
                                        packet->button);
}

void printStreamData2(const Steam_packet2 * packet){
    printf("\n    Gyro: %f,%f,%f\n    Accel:%f,%f,%f\n    Comps:%f,%f,%f\n", packet->raw_data[0],
                                               packet->raw_data[1],
                                               packet->raw_data[2],
                                               packet->raw_data[3],
                                               packet->raw_data[4],
                                               packet->raw_data[5],
                                               packet->raw_data[6],
                                               packet->raw_data[7],
                                               packet->raw_data[8]);
//    printf("Button: %u\n",   packet->button);
}

int TSS_single_wired_stream_BT(){
    float quat[4];
    float new_led_color[3]= {1,1,1};
//  TSS_Stream_Command stream_slots[8] = {TSS_GET_TARED_ORIENTATION_AS_QUATERNION,
//                                    TSS_GET_BATTERY_PERCENT_REMAINING,
//                                    TSS_GET_BUTTON_STATE,
//                                    TSS_NULL,TSS_NULL,TSS_NULL,TSS_NULL,TSS_NULL};
    TSS_Stream_Command stream_slots[8] = { TSS_GET_ALL_CORRECTED_COMPONENT_SENSOR_DATA,
                                    TSS_NULL,
                                    TSS_NULL,
                                    TSS_NULL,TSS_NULL,TSS_NULL,TSS_NULL,TSS_NULL};
    TSS_Stream_Command return_stream_slots[8];
    unsigned int interval, duration, delay;
    //Steam_packet packet;
    //TSS_Device_Id device = tss_createTSDeviceStr("COM71", TSS_TIMESTAMP_SENSOR);

    //tss_closeTSDevice(device);

    //device = tss_createTSDeviceStr("COM71", TSS_TIMESTAMP_SENSOR);

    unsigned int device;
    TSS_ComPort comportlist;
    unsigned int found_count;
    unsigned int d_device;
    unsigned int timestamp;

    LARGE_INTEGER frequency;        // ticks per second
    LARGE_INTEGER t1, t2;           // ticks
    Steam_packet2 packet;
    int pk_count =0;

    found_count = tss_getComPorts(&comportlist, 1, 0,TSS_FIND_DNG);
    printf("found_count %u\n", found_count);
    if(found_count){
        printf("TSS_ComPort:%s Name:%s\n",comportlist.com_port, comportlist.friendly_name);
        d_device = tss_createTSDeviceStr(comportlist.com_port, TSS_TIMESTAMP_SENSOR);
    }
    else{
        printf("No sensor found\n");
        return 1;
    }
    tss_getSensorFromDongle(d_device,0,&device);
    if(device == TSS_NO_DEVICE_ID){
        printf("No sensor found\n");
        return 1;
    }
    printf("device: %X\n",device);

    //tss_setNewDataCallBack(device,DataPrinter);

    //tss_setStreamingTiming(device,0, 1000000, 0, NULL);
    tss_setStreamingTiming(device,0, TSS_INFINITE_DURATION, 10000, NULL);


    tss_getStreamingTiming(device, &interval, &duration, &delay, NULL);
    printf("delay:%u duration:%u delay:%u\n",interval,duration, delay);

    tss_setStreamingSlots(device, stream_slots, NULL);

    tss_getStreamingSlots(device, return_stream_slots, NULL);
    printf("Command bytes: %X, %X, %X, %X, %X, %X, %X, %X\n", return_stream_slots[0],
                                                            return_stream_slots[1],
                                                            return_stream_slots[2],
                                                            return_stream_slots[3],
                                                            return_stream_slots[4],
                                                            return_stream_slots[5],
                                                            return_stream_slots[6],
                                                            return_stream_slots[7]);

    tss_getTaredOrientationAsQuaternion(device, quat, NULL);
    printf("Quat: %f,%f,%f,%f\n",quat[0],quat[1],quat[2],quat[3]);
    printf("Setting the LED to Red\n");
    tss_setLEDColor(device, new_led_color, NULL);
    tss_updateCurrentTimestamp(device,500000000,&timestamp);
    printf("timestamp %u\n", timestamp);
    Sleep(100);
//    int error = tss_broadcastSynchronizationPulse(d_device,&timestamp);
//    printf("error= %d timestamp %u\n",error, timestamp);
//    tss_broadcastSynchronizationPulse(d_device,NULL);
//    tss_broadcastSynchronizationPulse(d_device,NULL);
    tss_startStreaming(device,NULL);
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&t1);
    QueryPerformanceCounter(&t2);
    while( 2.0 > ((t2.QuadPart-t1.QuadPart)*1.0f/frequency.QuadPart)){
        QueryPerformanceCounter(&t2);
        tss_getLatestStreamData(device,(char*)&packet,sizeof(packet),1000,&timestamp);
        //tss_getLastStreamData(device,(char*)&packet,sizeof(packet),&timestamp);
        printf("t:%8u ",timestamp);
        printStreamData2(&packet);
        pk_count++;
    }
    //tss_stopStreaming(device,NULL);
    return 0;
}

int TSS_multi_wired_stream(){
    float quat[4];
    //float forwardDown[6];
    float new_led_color[3]= {1,1,1};
//  TSS_Stream_Command stream_slots[8] = {TSS_GET_TARED_ORIENTATION_AS_QUATERNION,
//                                    TSS_GET_BATTERY_PERCENT_REMAINING,
//                                    TSS_GET_BUTTON_STATE,
//                                    TSS_NULL,TSS_NULL,TSS_NULL,TSS_NULL,TSS_NULL};
    TSS_Stream_Command stream_slots[8] = { TSS_GET_ALL_CORRECTED_COMPONENT_SENSOR_DATA,
                                    TSS_NULL,
                                    TSS_NULL,
                                    TSS_NULL,TSS_NULL,TSS_NULL,TSS_NULL,TSS_NULL};
    TSS_Stream_Command return_stream_slots[8];
    unsigned int interval, duration, delay;
    //Steam_packet packet;
    //TSS_Device_Id device = tss_createTSDeviceStr("COM71", TSS_TIMESTAMP_SENSOR);

    //tss_closeTSDevice(device);

    //device = tss_createTSDeviceStr("COM71", TSS_TIMESTAMP_SENSOR);

    unsigned int device[5];
    TSS_ComPort comportlist;
    unsigned int found_count;
    unsigned int d_device;
    unsigned int timestamp;

    LARGE_INTEGER frequency;        // ticks per second
    LARGE_INTEGER t1, t2;           // ticks
    //Steam_packet2 packet;
    //int pk_count =0;

    found_count = tss_getComPorts(&comportlist, 1, 0,TSS_FIND_DNG);
    printf("found_count %u\n", found_count);
    if(found_count){
        printf("TSS_ComPort:%s Name:%s\n",comportlist.com_port, comportlist.friendly_name);
        d_device = tss_createTSDeviceStr(comportlist.com_port, TSS_TIMESTAMP_SENSOR);
    }
    else{
        printf("No sensor found\n");
        return 1;
    }
    //tss_setTimestampMode(d_device,TSS_TIMESTAMP_SENSOR);
    //tss_setTimestampMode(d_device,TSS_TIMESTAMP_SYSTEM);
    tss_getSensorFromDongle(d_device,0,&device[0]);
    tss_getSensorFromDongle(d_device,1,&device[1]);
    tss_getSensorFromDongle(d_device,2,&device[2]);
    tss_getSensorFromDongle(d_device,3,&device[3]);
    tss_getSensorFromDongle(d_device,4,&device[4]);
    if(device[0] == TSS_NO_DEVICE_ID){
        printf("No sensor found\n");
        return 1;
    }
    printf("device: %X\n",device[0]);

    //tss_setNewDataCallBack(device,DataPrinter);

    //tss_setStreamingTiming(device,0, 1000000, 0, NULL);
    tss_setStreamingTiming(device[0],0, TSS_INFINITE_DURATION, 100000, NULL);
    tss_setStreamingTiming(device[1],0, TSS_INFINITE_DURATION, 100000, NULL);
    tss_setStreamingTiming(device[2],0, TSS_INFINITE_DURATION, 100000, NULL);
    tss_setStreamingTiming(device[3],0, TSS_INFINITE_DURATION, 100000, NULL);
    tss_setStreamingTiming(device[4],0, TSS_INFINITE_DURATION, 100000, NULL);


    tss_getStreamingTiming(device[0], &interval, &duration, &delay, NULL);
    printf("delay:%u duration:%u delay:%u\n",interval,duration, delay);

    tss_setStreamingSlots(device[0], stream_slots, NULL);
    tss_setStreamingSlots(device[1], stream_slots, NULL);
    tss_setStreamingSlots(device[2], stream_slots, NULL);
    tss_setStreamingSlots(device[3], stream_slots, NULL);
    tss_setStreamingSlots(device[4], stream_slots, NULL);

    tss_getStreamingSlots(device[0], return_stream_slots, NULL);
    printf("Command bytes: %X, %X, %X, %X, %X, %X, %X, %X\n", return_stream_slots[0],
                                                            return_stream_slots[1],
                                                            return_stream_slots[2],
                                                            return_stream_slots[3],
                                                            return_stream_slots[4],
                                                            return_stream_slots[5],
                                                            return_stream_slots[6],
                                                            return_stream_slots[7]);

    tss_getTaredOrientationAsQuaternion(device[0], quat, NULL);
    printf("Quat: %f,%f,%f,%f\n",quat[0],quat[1],quat[2],quat[3]);
    printf("Setting the LED to Red\n");
    tss_setLEDColor(device[0], new_led_color, NULL);
    tss_updateCurrentTimestamp(device[0],500000000,&timestamp);
    printf("timestamp %u\n", timestamp);
    //Sleep(100);
//    int error = tss_broadcastSynchronizationPulse(d_device,&timestamp);
//    printf("error= %d timestamp %u\n",error, timestamp);
//    tss_broadcastSynchronizationPulse(d_device,NULL);
//    tss_broadcastSynchronizationPulse(d_device,NULL);
    tss_startStreaming(device[0],NULL);
    tss_startStreaming(device[1],NULL);
    tss_startStreaming(device[2],NULL);
    tss_startStreaming(device[3],NULL);
    tss_startStreaming(device[4],NULL);
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&t1);
    QueryPerformanceCounter(&t2);
//    while( 2.0 > ((t2.QuadPart-t1.QuadPart)*1.0f/frequency.QuadPart)){
//        QueryPerformanceCounter(&t2);
//        tss_getLatestStreamData(device[0],(char*)&packet,sizeof(packet),250,&timestamp);
//        //tss_getLastStreamData(device,(char*)&packet,sizeof(packet),&timestamp);
//        printf("t:%8u ",timestamp);
//        printStreamData2(&packet);
//        pk_count++;
//    }
//    tss_stopStreaming(device[0],NULL);
//    tss_stopStreaming(device[1],NULL);
//    tss_stopStreaming(device[2],NULL);
//    tss_stopStreaming(device[3],NULL);
//    tss_stopStreaming(device[4],NULL);
    return 0;
}

int TSS_single_wired_stream(){
    float quat[4];
    float forward_vector[3];
    float down_vector[3];
    float new_led_color[3]= {1,0,0};
    TSS_Stream_Command stream_slots[8] = {TSS_GET_TARED_ORIENTATION_AS_QUATERNION,
                                    TSS_GET_BATTERY_PERCENT_REMAINING,
                                    TSS_GET_BUTTON_STATE,
                                    TSS_NULL,TSS_NULL,TSS_NULL,TSS_NULL,TSS_NULL};
    TSS_Stream_Command return_stream_slots[8];
    unsigned int interval, duration, delay;
    Steam_packet packet;
    unsigned int packet_count;
    unsigned int device = tss_createTSDeviceStr("COM103", TSS_TIMESTAMP_NONE);
    if(device == TSS_NO_DEVICE_ID){
        printf("No sensor found\n");
        return 1;
    }
    printf("device: %X\n",device);


    packet_count = 0;
//  Steam_packet * packet;

    tss_setStreamingTiming(device,0, 0xffffffff, 0, NULL);

    tss_getStreamingTiming(device, &interval, &duration, &delay, NULL);
    printf("delay:%u duration:%u delay:%u\n",interval,duration, delay);

    tss_setStreamingSlots(device, stream_slots, NULL);

    tss_getStreamingSlots(device, return_stream_slots, NULL);
    printf("Command bytes: %d, %d, %d, %d, %d, %d, %d, %d\n", return_stream_slots[0],
                                                            return_stream_slots[1],
                                                            return_stream_slots[2],
                                                            return_stream_slots[3],
                                                            return_stream_slots[4],
                                                            return_stream_slots[5],
                                                            return_stream_slots[6],
                                                            return_stream_slots[7]);

    tss_getTaredOrientationAsQuaternion(device, quat, NULL);
    printf("Quat: %f,%f,%f,%f\n",quat[0],quat[1],quat[2],quat[3]);
    printf("Setting the LED to Red\n");
    tss_setLEDColor(device, new_led_color, NULL);
    tss_startStreaming(device,NULL);

    Sleep(100);
    tss_getLastStreamData(device,(char *)&packet,sizeof(Steam_packet),NULL);
    printStreamData(&packet);
    Sleep(100);
    tss_getLastStreamData(device,(char *)&packet,sizeof(Steam_packet),NULL);
    printStreamData(&packet);
    Sleep(100);
    tss_getLastStreamData(device,(char *)&packet,sizeof(Steam_packet),NULL);
    printStreamData(&packet);
    Sleep(100);
    tss_getLastStreamData(device,(char *)&packet,sizeof(Steam_packet),NULL);
    printStreamData(&packet);
    tss_stopStreaming(device,NULL);
    tss_getUntaredTwoVectorInSensorFrame(device,forward_vector, down_vector,NULL);
    printf("tss_getUntaredTwoVectorInSensorFrame= %f,%f,%f\n,%f,%f,%f\n",
           forward_vector[0],
           forward_vector[1],
           forward_vector[2],
           down_vector[0],
           down_vector[1],
           down_vector[2]);
    tss_getUntaredOrientationAsQuaternion(device,quat,NULL);
    printf("UntaredQuat: %f,%f,%f,%f\n",quat[0],quat[1],quat[2],quat[3]);
    tss_tareWithQuaternion(device,quat,NULL);
    Sleep(400);
    tss_startStreaming(device,NULL);
    Sleep(900);
    tss_getLastStreamData(device,(char *)&packet,sizeof(Steam_packet),NULL);
    printStreamData(&packet);
    tss_stopStreaming(device,NULL);
    printf("packet_count=%u\n",packet_count);
//    free(packet);
    new_led_color[0]=0;
    new_led_color[1]=0;
    new_led_color[2]=1;
    printf("Setting the LED to Blue\n");
    tss_setLEDColor(device, new_led_color, NULL);
    tss_getTaredOrientationAsQuaternion(device, quat, NULL);
    printf("Quat: %f,%f,%f,%f\n",quat[0],quat[1],quat[2],quat[3]);
    return 0;
}

int TSS_single_wireless_stream(){
    unsigned int d_device;
    float quat[4];
    //float new_led_color[3]= {1,0,1};
    unsigned int device;
    unsigned int interval, duration, delay;
    TSS_ComPort comportlist;
    unsigned int found_count;
    unsigned int serial;
    Steam_packet2 packet;
    //unsigned char axis_settings[4];
    TSS_Stream_Command stream_slots[8] = {TSS_GET_ALL_RAW_COMPONENT_SENSOR_DATA,
//                                        TSS_GET_TARED_ORIENTATION_AS_QUATERNION,
//                                        TSS_GET_BATTERY_PERCENT_REMAINING,
//                                        TSS_GET_BUTTON_STATE,
                                        TSS_NULL,
                                        TSS_NULL,
                                        TSS_NULL,TSS_NULL,TSS_NULL,TSS_NULL,TSS_NULL};
    TSS_Stream_Command return_stream_slots[8];
    found_count = tss_getComPorts(&comportlist, 1, 0,4);
    printf("found_count %u\n", found_count);
    if(found_count){
        printf("TSS_ComPort:%s Name:%s\n",comportlist.com_port, comportlist.friendly_name);
        d_device = tss_createTSDeviceStr(comportlist.com_port, TSS_TIMESTAMP_NONE);
    }
    else{
        printf("No sensor found\n");
        return 1;
    }
    tss_getSensorFromDongle(d_device,6,&device);

    printf("device: %X\n",d_device);
    printf("device: %X\n",device);

    tss_getSerialNumber(device,&serial,NULL);
    printf("Sensor Serial:%x\n", serial);

//  axis_settings[0]= 2;
//  axis_settings[1]= 1;
//  axis_settings[2]= 0;
//  axis_settings[3]= 1;
//
//  tss_setAxisDirectionsuc4(device,axis_settings, NULL);


//  unsigned int packet_count = 0;
//  Steam_packet * packet;

    tss_setStreamingTiming(device,0, 60000000, 0, NULL);

    tss_getStreamingTiming(device, &interval, &duration, &delay, NULL);
    printf("delay:%u duration:%u delay:%u\n",interval,duration, delay);



    tss_setStreamingSlots(device, stream_slots, NULL);

    tss_getStreamingSlots(device, return_stream_slots, NULL);
    printf("Command bytes: %d, %d, %d, %d, %d, %d, %d, %d\n", return_stream_slots[0],
                                                            return_stream_slots[1],
                                                            return_stream_slots[2],
                                                            return_stream_slots[3],
                                                            return_stream_slots[4],
                                                            return_stream_slots[5],
                                                            return_stream_slots[6],
                                                            return_stream_slots[7]);

//    tss_getTaredOrientationAsQuaternion(device, quat, NULL);
//    printf("Quat: %f,%f,%f,%f\n",quat[0],quat[1],quat[2],quat[3]);
//    printf("Setting the LED to Red\n");
//    tss_setLEDColor(device, new_led_color, NULL);
    tss_startStreaming(device,NULL);
    //Sleep(10000);

    tss_getLatestStreamData(device,(char *)&packet,sizeof(Steam_packet2), 1000 ,NULL);
    printStreamData2(&packet);
    Sleep(250);
    tss_getLastStreamData(device,(char *)&packet,sizeof(Steam_packet2),NULL);
    printStreamData2(&packet);
    Sleep(250);
    tss_getLastStreamData(device,(char *)&packet,sizeof(Steam_packet2),NULL);
    printStreamData2(&packet);
    Sleep(250);
    tss_getLastStreamData(device,(char *)&packet,sizeof(Steam_packet2),NULL);
    printStreamData2(&packet);
    tss_stopStreaming(device,NULL);
//    printf("Record Count=%d\n", tss_getRecordCount(device));
//    free(packet);
//    new_led_color[0]=0;
//    new_led_color[1]=0;
//    new_led_color[2]=1;
//    printf("Setting the LED to Blue\n");
//    tss_setLEDColor(device, new_led_color, NULL);
    tss_getTaredOrientationAsQuaternion(device, quat, NULL);
    printf("Quat: %f,%f,%f,%f\n",quat[0],quat[1],quat[2],quat[3]);
    return 0;
}

int TSS_double_wired_stream(){
    float quat[4];
    float new_led_color[3]= {1,0,0};
    unsigned int interval, duration, delay;
    unsigned int device, device2, device3;
    unsigned int packet_count;
    Steam_packet packet;
    TSS_Stream_Command stream_slots[8] = {TSS_GET_TARED_ORIENTATION_AS_QUATERNION,
                                        TSS_GET_BATTERY_PERCENT_REMAINING,
                                        TSS_GET_BUTTON_STATE,
                                        TSS_NULL,TSS_NULL,TSS_NULL,TSS_NULL,TSS_NULL};
    TSS_Stream_Command return_stream_slots[8];
    //unsigned int d_device = tss_createTSDeviceStr("COM6", TSS_TIMESTAMP_NONE);
    //if(d_device == TSS_NO_DEVICE_ID){
    //    printf("No sensor found\n");
    //    return 1;
    //}
    
    //tss_getSensorFromDongle(d_device,0,&device);
    //tss_getSensorFromDongle(d_device,1,&device2);
    //tss_getSensorFromDongle(d_device,2,&device3);

    device = tss_createTSDeviceStr("COM104", TSS_TIMESTAMP_NONE);
    device2 = tss_createTSDeviceStr("COM105", TSS_TIMESTAMP_NONE);
    device3 = tss_createTSDeviceStr("COM106", TSS_TIMESTAMP_NONE);
//    if(device == TSS_NO_DEVICE_ID){
//        printf("No sensor found\n");
//        return 1;
//    }
    printf("device: %X\n",device);
    printf("device: %X\n",device2);
    printf("device: %X\n",device3);

    packet_count = 0;
//  Steam_packet * packet;

    tss_setStreamingTiming(device,0, 20000000, 0, NULL);
    tss_setStreamingTiming(device2,0, 20000000, 0, NULL);
    tss_setStreamingTiming(device3,0, 20000000, 0, NULL);

    tss_getStreamingTiming(device, &interval, &duration, &delay, NULL);
    tss_getStreamingTiming(device2, &interval, &duration, &delay, NULL);
    tss_getStreamingTiming(device3, &interval, &duration, &delay, NULL);
    printf("delay:%u duration:%u delay:%u\n",interval,duration, delay);

    tss_setStreamingSlots(device, stream_slots, NULL);
    tss_setStreamingSlots(device2, stream_slots, NULL);
    tss_setStreamingSlots(device3, stream_slots, NULL);

    tss_getStreamingSlots(device, return_stream_slots, NULL);
    tss_getStreamingSlots(device2, return_stream_slots, NULL);
    tss_getStreamingSlots(device3, return_stream_slots, NULL);
    printf("Command bytes: %d, %d, %d, %d, %d, %d, %d, %d\n", return_stream_slots[0],
                                                            return_stream_slots[1],
                                                            return_stream_slots[2],
                                                            return_stream_slots[3],
                                                            return_stream_slots[4],
                                                            return_stream_slots[5],
                                                            return_stream_slots[6],
                                                            return_stream_slots[7]);

    tss_getTaredOrientationAsQuaternion(device, quat, NULL);
    tss_getTaredOrientationAsQuaternion(device2, quat, NULL);
    tss_getTaredOrientationAsQuaternion(device3, quat, NULL);
    printf("Quat: %f,%f,%f,%f\n",quat[0],quat[1],quat[2],quat[3]);
    printf("Setting the LED to Red\n");
    tss_setLEDColor(device, new_led_color, NULL);
    tss_setLEDColor(device2, new_led_color, NULL);
    tss_setLEDColor(device3, new_led_color, NULL);
    tss_startStreaming(device,NULL);
    tss_startStreaming(device2,NULL);
    tss_startStreaming(device3,NULL);

    Sleep(2500);
    tss_getLastStreamData(device2,(char *)&packet,sizeof(Steam_packet),NULL);
    printStreamData(&packet);
    Sleep(2500);
    tss_getLastStreamData(device,(char *)&packet,sizeof(Steam_packet),NULL);
    printStreamData(&packet);
    Sleep(2500);
    tss_getLastStreamData(device2,(char *)&packet,sizeof(Steam_packet),NULL);
    printStreamData(&packet);
    Sleep(2500);
    tss_getLastStreamData(device,(char *)&packet,sizeof(Steam_packet),NULL);
    printStreamData(&packet);
    tss_stopStreaming(device,NULL);
    tss_stopStreaming(device2,NULL);
    tss_stopStreaming(device3,NULL);

    //printf("Record Count=%d\n", tss_getRecordCount(device));
    //printf("Record Count=%d\n", tss_getRecordCount(device2));
    //printf("Record Count=%d\n", tss_getRecordCount(device3));
//    WaitForSingleObject(readerthread,INFINITE);

    printf("packet_count=%u\n",packet_count);
//    free(packet);
    new_led_color[0]=0;
    new_led_color[1]=0;
    new_led_color[2]=1;
    printf("Setting the LED to Blue\n");
    tss_setLEDColor(device, new_led_color, NULL);
    tss_setLEDColor(device2, new_led_color, NULL);
    tss_setLEDColor(device3, new_led_color, NULL);
    tss_getTaredOrientationAsQuaternion(device, quat, NULL);
    printf("Quat: %f,%f,%f,%f\n",quat[0],quat[1],quat[2],quat[3]);
    tss_getTaredOrientationAsQuaternion(device2, quat, NULL);
    tss_getTaredOrientationAsQuaternion(device2, quat, NULL);
    printf("Quat: %f,%f,%f,%f\n",quat[0],quat[1],quat[2],quat[3]);
    return 0;

}

int TSS_single_dongle(){
    //float quat[4];
    float new_led_color[3]= {1,0,0};
    unsigned int old_sensor = 0x00003EEF;
    unsigned int new_sensor = 0x02000119;
//  unsigned int interval, duration, delay;
    unsigned int w_device;
    unsigned int device = tss_createTSDeviceStr("COM18", TSS_TIMESTAMP_NONE);
    if(device == TSS_NO_DEVICE_ID){
        printf("No sensor found\n");
        return 1;
    }
    tss_getSensorFromDongle(device,0,&w_device);
    printf("device: %X\n",device);
    printf("device: %X\n",w_device);

    printf("Setting the LED to Red\n");
    tss_setLEDColor(device, new_led_color, NULL);
    tss_setLEDColor(w_device, new_led_color, NULL);


    printf("Setting new sensor\n");
    tss_setSensorToDongle(device, 0, new_sensor);
    //tss_getSensorFromDongle(device,0,&w_device);
    printf("Setting the LED to Red\n");
    tss_setLEDColor(w_device, new_led_color, NULL);


    Sleep(20000);
//    tss_getTaredOrientationAsQuaternion(w_device, quat, NULL);
//    printf("Quat: %f,%f,%f,%f\n",quat[0],quat[1],quat[2],quat[3]);
    new_led_color[0]=0;
    new_led_color[1]=0;
    new_led_color[2]=1;
    printf("Setting the LED to Blue\n");
    tss_setLEDColor(device, new_led_color, NULL);
    tss_setLEDColor(w_device, new_led_color, NULL);
    printf("Setting old sensor\n");
    tss_setSensorToDongle(device, 0, old_sensor);
    //tss_getSensorFromDongle(device,0,&w_device);
    printf("Setting the LED to Red\n");
    tss_setLEDColor(w_device, new_led_color, NULL);
    return 0;
}

int TSS_Pairing(){
    //float quat[4];
    #define DEBUG_PRINT
    float red[3]= {1,0,0};
    float blue[3]= {0,0,1};
    unsigned int device;
    TSS_Error error;
    //unsigned int sensor = 0x02000121;
//  unsigned int interval, duration, delay;
    unsigned int wl_device;
    printf("Getting device\n");
    device = tss_createTSDeviceStr("COM8", TSS_TIMESTAMP_NONE);
    if(device == TSS_NO_DEVICE_ID){
        printf("No sensor found\n");
        return 1;
    }
    printf("Getting sensor\n");
    error = tss_getSensorFromDongle(device, 0, &wl_device);
    printf("Error: %s\n",TSS_Error_String[error]);
    printf("DNG device: %X\n",device);
    printf("WL  device: %X\n",wl_device);

    printf("Setting the LED to Red\n");
    tss_setLEDColor(wl_device, red, NULL);

    Sleep(20000);

    printf("Setting the LED to Blue\n");
    tss_setLEDColor(wl_device, blue, NULL);

    printf("Closing Dongle\n");
    tss_closeTSDevice(device);

    return 0;
}

int TSS_Repairing(){
    TSS_Device_Id  wl_sensor;
    TSS_Device_Id  wl_sensor_wireless;
    TSS_Device_Id  dongle;
    TSS_ComPort comport;
    unsigned int serial;
    float quat[4];
    dongle = TSS_NO_DEVICE_ID;
    if(tss_getComPorts(&comport,1,0,TSS_FIND_DNG)){
        dongle =tss_createTSDeviceStr(comport.com_port, TSS_TIMESTAMP_SENSOR);
        if( dongle == TSS_NO_DEVICE_ID){
            printf("Failed to create a dongle on %s\n",comport.com_port);
            return 1;
        }
    }
    tss_getSensorFromDongle(dongle, 0, &wl_sensor);
    tss_getSerialNumber(wl_sensor, &serial, NULL);
    printf("Serial at 0: %08X", serial);

    tss_setSensorToDongle(dongle,0,0x020002A5);
    tss_getSensorFromDongle(dongle, 0, &wl_sensor_wireless);

    if( wl_sensor_wireless != TSS_NO_DEVICE_ID){
        printf("Sensor pair successful!!!\n");
        tss_getTaredOrientationAsQuaternion(wl_sensor_wireless, quat, NULL);
        printf("Quat: %f,%f,%f,%f\n",quat[0],quat[1],quat[2],quat[3]);
    }
    else{
        printf("Failed to pair sensor\n");
    }

    tss_setSensorToDongle(dongle,0,0x02000002);
    //tss_getSensorFromDongle(dongle, 0, &wl_sensor_wireless);

    if( wl_sensor_wireless != TSS_NO_DEVICE_ID){
        printf("Sensor pair2 successful!!!\n");
        tss_getTaredOrientationAsQuaternion(wl_sensor_wireless, quat, NULL);
        printf("Quat: %f,%f,%f,%f\n",quat[0],quat[1],quat[2],quat[3]);
    }
    else{
        printf("Failed to pair sensor2\n");
    }

    printf("Finished press Enter to continue");
    getchar();
    return 0;
}

int TSS_get_sensor_info2(){
    TSS_ComPort comport_list[20];
    int device_count;
    int i;
    device_count =tss_getComPorts(comport_list,20,0,TSS_FIND_UNKNOWN);//TSS_FIND_UNKNOWN);
    printf("Found %d Devices\n", device_count);
    for( i=0; i< device_count; ++i){
        TSS_ComInfo com_info;
        TSS_Error error= tss_getTSDeviceInfoFromComPort(comport_list[i].com_port,&com_info);
        if( error == TSS_NO_ERROR){
            printf("============(%s)=============\n",comport_list[i].com_port);
            printf("DeviceType:%s\nSerial:%08X\nHardwareVersion:%s\nFirmwareVersion:%s\n",
               TSS_Type_String[com_info.device_type],
               com_info.serial_number,
               com_info.hardware_version,
               com_info.firmware_version);
            printf("================================\n");
        }
        else{
            printf("Failed to communicate on %s\n",comport_list[i].com_port);
        }
    }
    return 0;
}
int TSS_get_sensor_info(){
    TSS_ComPort comport;
    int offset=0;
    while(tss_getComPorts(&comport,1,offset,TSS_FIND_UNKNOWN)){
        TSS_ComInfo com_info;
        tss_getTSDeviceInfoFromComPort(comport.com_port,&com_info);
        printf("============(%s)=============\n",comport.com_port);
        printf("DeviceType:%s\nSerial:%08X\nHardwareVersion:%s\nFirmwareVersion:%s\n",
               TSS_Type_String[com_info.device_type],com_info.serial_number, com_info.hardware_version, com_info.firmware_version);
        printf("================================\n");
        offset++;
    }
    return 0;
}

int trust_test(){
    TSS_Device_Id  device;
    TSS_Error tss_error;
    TSS_ComPort comport;
    float min_trust_value, max_trust_value;
    float temp_min_trust_value, temp_max_trust_value;
    unsigned int timestamp;

    if(tss_getComPorts(&comport,1,0,TSS_FIND_ALL_KNOWN^TSS_FIND_DNG)){
        device =tss_createTSDeviceStr(comport.com_port, TSS_TIMESTAMP_SENSOR);
        if( device == TSS_NO_DEVICE_ID){
            printf("Failed to create a sensor on %s\n",comport.com_port);
            return 1;
        }
    }
    else{
        printf("No sensors found\n");
        return 1;
    }
    printf("==================================================\n");
    printf("Getting the filtered tared quaternion orientation.(xyzw)\n");

    tss_error= tss_getAccelerometerTrustValues(device, &min_trust_value, &max_trust_value, &timestamp);
    if(!tss_error){
        printf("tss_getAccelerometerTrustValues: %f, %f, Timestamp=%u\n", min_trust_value, max_trust_value, timestamp);
    }
    else{
        printf("TSS_Error: %s\n", TSS_Error_String[tss_error]);
    }
    tss_error= tss_setConfidenceAccelerometerTrustValues(device, 0.0080f, 0.15f, &timestamp);
    if(!tss_error){
    }
    else{
        printf("TSS_Error: %s\n", TSS_Error_String[tss_error]);
    }
    tss_error= tss_getAccelerometerTrustValues(device, &temp_min_trust_value, &temp_max_trust_value, &timestamp);
    if(!tss_error){
        printf("tss_getAccelerometerTrustValues: %f, %f, Timestamp=%u\n",temp_min_trust_value, temp_max_trust_value, timestamp);
    }
    else{
        printf("TSS_Error: %s\n", TSS_Error_String[tss_error]);
    }
    tss_error= tss_setConfidenceAccelerometerTrustValues(device, min_trust_value, max_trust_value, &timestamp);
    if(!tss_error){
    }
    else{
        printf("TSS_Error: %s\n", TSS_Error_String[tss_error]);
    }
    tss_error= tss_getAccelerometerTrustValues(device, &min_trust_value, &max_trust_value, &timestamp);
    if(!tss_error){
        printf("tss_getAccelerometerTrustValues: %f, %f, Timestamp=%u\n", min_trust_value, max_trust_value, timestamp);
    }
    else{
        printf("TSS_Error: %s\n", TSS_Error_String[tss_error]);
    }


    min_trust_value = max_trust_value = 0.0;
    tss_error= tss_getCompassTrustValues(device, &min_trust_value, &max_trust_value, &timestamp);
    if(!tss_error){
        printf("tss_getCompassTrustValues: %f, %f, Timestamp=%u\n", min_trust_value, max_trust_value, timestamp);
    }
    else{
        printf("TSS_Error: %s\n", TSS_Error_String[tss_error]);
    }
    printf("==================================================\n");
    tss_closeTSDevice(device);
    return 0;
}

int TSS_batch_command(){
    TSS_Stream_Command stream_slots[8] = {  TSS_GET_TARED_ORIENTATION_AS_QUATERNION,
                                            TSS_GET_CORRECTED_LINEAR_ACCELERATION_IN_GLOBAL_SPACE,
                                            TSS_GET_BATTERY_PERCENT_REMAINING,
                                            TSS_GET_BUTTON_STATE,
                                            TSS_NULL,TSS_NULL,TSS_NULL,TSS_NULL};
    Steam_packet packet;
    float quat[4];
    TSS_Error tss_error;

    unsigned int device = tss_createTSDeviceStr("COM37", TSS_TIMESTAMP_NONE);
    if(device == TSS_NO_DEVICE_ID){
        printf("No sensor found\n");
        return 1;
    }
    printf("device: %X\n",device);

    tss_setStreamingSlots(device, stream_slots, NULL);

    tss_error = tss_getStreamingBatch(device, (char*)&packet,sizeof(Steam_packet), NULL);
    printStreamData(&packet);
    printf("tss_error= %s\n", TSS_Error_String[tss_error] );

    tss_getTaredOrientationAsQuaternion(device, quat, NULL);
    printf("Quat: %f,%f,%f,%f\n",quat[0],quat[1],quat[2],quat[3]);

    return 0;
}

#ifdef TSS_STATIC_LIB
//function uses a few convinances for the endian swap in threespace
//if being used by end user they will have their own endian swap func or do not need to change the endianness
int TSS_custom_command(){
    float quat[4];
    float red_color[]= {1,0,0};
    float blue_color[]= {0,0,1};

    unsigned int device = tss_createTSDeviceStr("COM71", TSS_TIMESTAMP_NONE);
    if(device == TSS_NO_DEVICE_ID){
        printf("No sensor found\n");
        return 1;
    }
    printf("device: %X\n",device);

    tss_sendRawCommandFormatted(device, 0xee, (char*)red_color, "fff", NULL, "", NULL);

    tss_sendRawCommandFormatted(device, 0x00, NULL, "", (char*)quat, "ffff", NULL);
    printf("Quat: %f,%f,%f,%f\n",quat[0],quat[1],quat[2],quat[3]);

    Sleep(2000);
    //endian_swap_32 is in the yei_threespace_core.h, normally no one needs to do an endian swap
    endian_swap_32((unsigned int*)&blue_color[0]);
    endian_swap_32((unsigned int*)&blue_color[1]);
    endian_swap_32((unsigned int*)&blue_color[2]);
    tss_sendRawCommand(device, 0xee, (char*)blue_color, 12, NULL, 0, NULL);

    tss_sendRawCommand(device, 0x00, NULL, 0, (char*)quat, 16, NULL);
    endian_swap_32((unsigned int*)&quat[0]);
    endian_swap_32((unsigned int*)&quat[1]);
    endian_swap_32((unsigned int*)&quat[2]);
    endian_swap_32((unsigned int*)&quat[3]);
    printf("Quat: %f,%f,%f,%f\n",quat[0],quat[1],quat[2],quat[3]);

    return 0;
}
#endif

int TSS_BT_find(){
    TSS_ComPort comport[20];
    float quat[4];
    unsigned int device;

    if(tss_getComPorts(comport,20,0,TSS_FIND_BT)){
        device =tss_createTSDeviceStr(comport[0].com_port, TSS_TIMESTAMP_SENSOR);
        if( device == TSS_NO_DEVICE_ID){
            printf("Failed to create a sensor on %s\n",comport[0].com_port);
            return 1;
        }
    }
    else{
        printf("No sensors found\n");
        return 1;
    }

    tss_getTaredOrientationAsQuaternion(device, quat, NULL);
    printf("Quat: %f,%f,%f,%f\n",quat[0],quat[1],quat[2],quat[3]);
    return 0;
}

int TSS_createBySerialNumber(){
    //unsigned int serial_list[] = {0x02000005 , 0x02000009, 0x0200000F, 0x02000011, 0x02000002};
    unsigned int serial_list[] = {0x0200001c , 0x0200001d,0x02000015, 0x0200002d};
    unsigned int device_list[sizeof(serial_list)/sizeof(unsigned int)];
    int list_length = sizeof(serial_list)/sizeof(unsigned int);
    int device_count;
    int i;
    device_count = tss_createTSDevicesBySerialNumber(serial_list, device_list, list_length, 1, 1, TSS_TIMESTAMP_NONE);

    if(device_count == list_length){
        float quat[4];
        for( i=0; i< list_length ; i++){
            tss_getTaredOrientationAsQuaternion(device_list[i], quat, NULL);
            printf("Serial: %08X Quat: %f,%f,%f,%f\n", serial_list[i], quat[0],quat[1],quat[2],quat[3]);
        }
    }
    else{
        printf("!!Error:Not all devices were found\n");
        for( i=0; i< list_length ; i++){
            if(device_list[i] == TSS_NO_DEVICE_ID){
                printf("Sensor with serial %08X was not found\n",serial_list[i]);
            }
        }
        return 1;
    }
    
    printf("============TSS_FIND_DNG=========\n");
    {
        
        unsigned int more_device_list[10];
        device_count = tss_getDeviceIDs(more_device_list, 10, 0, TSS_FIND_DNG);
        if(device_count){
            for( i=0; i< device_count ; i++){
                TSS_ComInfo com_info;
                if( tss_getTSDeviceInfo(more_device_list[i],&com_info) == TSS_NO_ERROR){
                    printf("============================\n");
                    printf("DeviceType:%s\nSerial:%08X\nHardwareVersion:%s\nFirmwareVersion:%s\nCompatibility:%d\n",
                       TSS_Type_String[com_info.device_type],
                       com_info.serial_number,
                       com_info.hardware_version,
                       com_info.firmware_version,
                       com_info.fw_compatibility);
                    printf("================================\n");
                }
            }
        }
        printf("============TSS_FIND_ALL=========\n");
        device_count = tss_getDeviceIDs(more_device_list, 10, 0, TSS_FIND_ALL);
        if(device_count){
            for( i=0; i< device_count ; i++){
                TSS_ComInfo com_info;
                if( tss_getTSDeviceInfo(more_device_list[i],&com_info) == TSS_NO_ERROR){
                    printf("============================\n");
                    printf("DeviceType:%s\nSerial:%08X\nHardwareVersion:%s\nFirmwareVersion:%s\nCompatibility:%d\n",
                       TSS_Type_String[com_info.device_type],
                       com_info.serial_number,
                       com_info.hardware_version,
                       com_info.firmware_version,
                       com_info.fw_compatibility);
                    printf("================================\n");
                }
            }
        }

    }


    return 0;
}

int compatability_test(){
    TSS_Device_Id  device;
    const char * com_port = "COM103";
    printf("====Creating TS Devices on %s ====\n", com_port);
    device = tss_createTSDeviceStr(com_port, TSS_TIMESTAMP_SENSOR);
    if( device == TSS_NO_DEVICE_ID){
        printf("Could not create device\n");
    }
    else{
        TSS_ComInfo com_info;
        if( tss_getTSDeviceInfo(device,&com_info) == TSS_NO_ERROR){
            printf("============(%s)=============\n",com_port);
            printf("DeviceType:%s\nSerial:%08X\nHardwareVersion:%s\nFirmwareVersion:%s\nCompatibility:%d\n",
               TSS_Type_String[com_info.device_type],
               com_info.serial_number,
               com_info.hardware_version,
               com_info.firmware_version,
               com_info.fw_compatibility);
            printf("================================\n");
        }
    }
    tss_closeTSDevice(device);
    return 0;
}

int main()
{
#ifdef TSS_STATIC_LIB
    tss_initThreeSpaceAPI();
#endif
  //TSS_single_wired_stream_BT();
//    TSS_get_sensor_info2();
    //TSS_single_wired_stream();
    //trust_test();
    //TSS_Pairing();
    //TSS_Repairing();
    //TSS_multi_wired_stream();
    //TSS_single_wireless_stream();
    //TSS_double_wired_stream();
   // TSS_batch_command();
    TSS_createBySerialNumber();
    //compatability_test();
    //TSS_BT_find();
    //TSS_custom_command();
#ifdef TSS_STATIC_LIB
   tss_delThreeSpaceAPI();
#endif
    return 0;
}

















