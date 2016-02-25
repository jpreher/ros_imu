/********************************************//**
 * \file yei_enum_ports.h
 * \brief  ThreeSpace API 2.0
 * \author Chris George
 * \author Daniel Morrison
 * \copyright Copyright 1998-2014, YEI Corporation.
 *
 * The YEI 3-Space C API is released under the YEI 3-Space Open Source License, which allows for both
 * non-commercial use and commercial use with certain restrictions.
 *
 * For Non-Commercial Use, your use of Covered Works is governed by the GNU GPL v.3, subject to the YEI 3-Space Open
 * Source Licensing Overview and Definitions.
 *
 * For Commercial Use, a YEI Commercial/Redistribution License is required, pursuant to the YEI 3-Space Open Source
 * Licensing Overview and Definitions. Commercial Use, for the purposes of this License, means the use, reproduction
 * and/or Distribution, either directly or indirectly, of the Covered Works or any portion thereof, or a Compilation,
 * Improvement, or Modification, for Pecuniary Gain. A YEI Commercial/Redistribution License may or may not require
 * payment, depending upon the intended use.
 *
 * Full details of the YEI 3-Space Open Source License can be found in license.txt
 * License also available online at http://www.yeitechnology.com/yei-3-space-open-source-license
 ***********************************************/
#include "yei_enum_ports.h"

#if defined(__GNUC__)
#include "ddk\ntddser.h" // defines  GUID_DEVINTERFACE_COMPORT and GUID_DEVINTERFACE_COMPORT in ming
#endif

//DEFINE_GUID(GUID_DEVINTERFACE_COMPORT,                0x86e0d1e0L, 0x8089, 0x11d0, 0x9c, 0xe4, 0x08, 0x00, 0x3e, 0x30, 0x1f, 0x73);
//DEFINE_GUID(GUID_DEVINTERFACE_SERENUM_BUS_ENUMERATOR, 0x4D36E978L, 0xE325, 0x11CE, 0xBF, 0xC1, 0x08, 0x00, 0x2B, 0xE1, 0x03, 0x18);
int tss_getComPorts(TSS_ComPort * ports_vec,unsigned int ports_vec_size, unsigned int offset, unsigned int filter)
{

    // Build vector list of device types to search for
    GUID guid_vec[2];// = {GUID_DEVINTERFACE_COMPORT, GUID_DEVINTERFACE_SERENUM_BUS_ENUMERATOR};
    int i;
    int guid_vec_size;
    unsigned int vec_size= 0;
    if(filter & (TSS_FIND_BT|TSS_FIND_UNKNOWN)){
        guid_vec_size=2;
    }
    else{
        guid_vec_size=1;
    }
    guid_vec[0]= GUID_DEVINTERFACE_SERENUM_BUS_ENUMERATOR; // YEI 3-Space sensors via USB, some of the devices in GUID_DEVINTERFACE_COMPORT but not all......
    guid_vec[1]= GUID_DEVINTERFACE_COMPORT; //BT serial, physical ports, ftdi serial adaptors, and other usb serial adaptors
    for( i=0; i<guid_vec_size; ++i){
        BOOL more_items;
        DWORD idx;
        SP_DEVINFO_DATA dev_info;
        GUID guid = guid_vec[i];
        // Get the class of the device
        HDEVINFO dev_set = SetupDiGetClassDevsA(&guid, NULL, NULL, DIGCF_PRESENT | DIGCF_INTERFACEDEVICE);
        if(dev_set == INVALID_HANDLE_VALUE){  // Check if INVALID_HANDLE_VALUE

            #ifdef DEBUG_PRINT
            // Retrieve the system error message for the last-error code
            LPVOID msg_buf;
            DWORD last_error = GetLastError();

            FormatMessageA(
                FORMAT_MESSAGE_ALLOCATE_BUFFER |
                FORMAT_MESSAGE_FROM_SYSTEM |
                FORMAT_MESSAGE_IGNORE_INSERTS,
                NULL,
                last_error,
                MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                (LPSTR) &msg_buf,
                0, NULL );

            // Display the error message
            printf("Failed to get device info handle ERROR[%d]: %s", last_error, (LPCSTR)msg_buf);

            // Free memory
            LocalFree(msg_buf);

            system("PAUSE");
            #endif
        }
        more_items = TRUE;
        idx = 0;

        while(more_items){
            dev_info.cbSize = sizeof(SP_DEVINFO_DATA);
            more_items = SetupDiEnumDeviceInfo(dev_set, idx, &dev_info);
            if(more_items){
                DWORD hid_size;
                DWORD req_hid_size;
                //string friendly_name_string = "";
                // Get the device's hardware id
                BYTE hardware_id[256];
                BYTE friendly_name[256];
                DWORD friendly_size;
                hardware_id[0] = '\0';
                hid_size = sizeof(hardware_id);
                if (SetupDiGetDeviceRegistryPropertyA(dev_set, &dev_info, SPDRP_HARDWAREID, NULL, hardware_id, hid_size, &req_hid_size) == FALSE){
                    DWORD last_error = GetLastError();
                    if (last_error != ERROR_INSUFFICIENT_BUFFER){  // Ignore ERROR_INSUFFICIENT_BUFFER
                        #ifdef DEBUG_PRINT
                        // Retrieve the system error message for the last-error code
                        LPVOID msg_buf;
                        FormatMessageA(
                            FORMAT_MESSAGE_ALLOCATE_BUFFER |
                            FORMAT_MESSAGE_FROM_SYSTEM |
                            FORMAT_MESSAGE_IGNORE_INSERTS,
                            NULL,
                            last_error,
                            MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                            (LPSTR) &msg_buf,
                            0, NULL );
                        // Display the error message
                        printf("Failed to get device registry for hardware_id ERROR[%d]: %s", last_error, (LPCSTR)msg_buf);
                        // Free memory
                        LocalFree(msg_buf);
                        system("PAUSE");
                        #endif
                    }
                }

                // The real com port name has to read differently...
                friendly_name[0] = '\0';
                friendly_size = sizeof(friendly_name);
                if(SetupDiGetDeviceRegistryPropertyA(dev_set, &dev_info, SPDRP_FRIENDLYNAME, NULL, friendly_name, friendly_size, NULL)){
                    HKEY dev_key = SetupDiOpenDevRegKey(dev_set, &dev_info, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
                    BYTE port_name[256];
                    DWORD port_size = sizeof(port_name);
                    if(RegQueryValueExA(dev_key, "PortName", NULL, NULL, port_name, &port_size) == ERROR_SUCCESS){
                        int sensor_type = TSS_UNKNOWN;
                        RegCloseKey(dev_key);
                        if(memcmp(hardware_id+8,"2476",4) == 0){
                            if((filter & TSS_FIND_BTL)     && memcmp(hardware_id+17,"1000",4) == 0){
                                sensor_type = TSS_BTL;
                            }
                            else if((filter & TSS_FIND_USB)&& memcmp(hardware_id+17,"1010",4) == 0){
                                sensor_type = TSS_USB;
                            }
                            else if((filter & TSS_FIND_DNG)&& memcmp(hardware_id+17,"1020",4) == 0){
                                sensor_type = TSS_DNG;
                            }
                            else if((filter & TSS_FIND_WL) && memcmp(hardware_id+17,"1030",4) == 0){
                                sensor_type = TSS_WL;
                            }
                            else if((filter & TSS_FIND_EM) && memcmp(hardware_id+17,"1040",4) == 0){
                                sensor_type = TSS_EM;
                            }
                            else if((filter & TSS_FIND_DL) && memcmp(hardware_id+17,"1050",4) == 0){
                                sensor_type = TSS_DL;
                            }
                            else if((filter & TSS_FIND_BT) && memcmp(hardware_id+17,"1060",4) == 0){
                                sensor_type = TSS_BT;
                            }
                            else{
                                //unknown TSS device?!
                            }

                            if( sensor_type && (ports_vec_size > vec_size)){
                                if(offset){
                                    offset--;
                                }
                                else{
                                    memcpy(ports_vec[vec_size].com_port,port_name,32);
                                    memcpy(ports_vec[vec_size].friendly_name,friendly_name,64);
                                    ports_vec[vec_size].sensor_type = sensor_type;
                                    vec_size++;
                                    if(vec_size == ports_vec_size){
                                        break;
                                    }
                                }
                            }
                        }
                        else if( (i==1) && (ports_vec_size > vec_size) ){
                            //can be used to detect bluetooth sensors, it may false positive on other bt serial devices needs additional testing
                            if( memcmp(hardware_id,"BTHENUM\\{00001101-0000-1000-8000-00805f9b34fb}_LOCALMFG&000f",62) == 0){
                                if(filter & TSS_FIND_BT){
                                    sensor_type = TSS_BT;
                                }
                                else{
                                    ++idx;
                                    continue;
                                }
                            }
                            else if(!(filter & TSS_FIND_UNKNOWN)){
                                ++idx;
                                continue;
                            }
                            if(offset){
                                offset--;
                            }
                            else{
                                //realloc(ports_vec,sizeof(TSS_ComPort)*(vec_size+1));
                                memcpy(ports_vec[vec_size].com_port,port_name,32);
                                memcpy(ports_vec[vec_size].friendly_name,friendly_name,64);
                                ports_vec[vec_size].sensor_type = sensor_type;
                                vec_size++;
                                if(vec_size == ports_vec_size){
                                    break;
                                }
                            }
                        }
                    }
                    else{
                        RegCloseKey(dev_key);
                        #ifdef DEBUG_PRINT
                        // Retrieve the system error message for the last-error code
                        LPVOID msg_buf;
                        DWORD last_error = GetLastError();

                        FormatMessageA(
                            FORMAT_MESSAGE_ALLOCATE_BUFFER |
                            FORMAT_MESSAGE_FROM_SYSTEM |
                            FORMAT_MESSAGE_IGNORE_INSERTS,
                            NULL,
                            last_error,
                            MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                            (LPSTR) &msg_buf,
                            0, NULL );

                        // Display the error message
                        printf("Failed to do a registry query for \"PortName\" ERROR[%d]: %s", last_error, (LPCSTR)msg_buf);

                        // Free memory
                        LocalFree(msg_buf);

                        system("PAUSE");
                        #endif
                    }
                }
                else{
                    DWORD last_error = GetLastError();
                    if(last_error != ERROR_INSUFFICIENT_BUFFER){  // Ignore ERROR_INSUFFICIENT_BUFFER
                        #ifdef DEBUG_PRINT
                        // Retrieve the system error message for the last-error code
                        LPVOID msg_buf;

                        FormatMessageA(
                            FORMAT_MESSAGE_ALLOCATE_BUFFER |
                            FORMAT_MESSAGE_FROM_SYSTEM |
                            FORMAT_MESSAGE_IGNORE_INSERTS,
                            NULL,
                            last_error,
                            MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                            (LPSTR) &msg_buf,
                            0, NULL );

                        // Display the error message
                        printf("Failed to device registry for friendly name ERROR[%d]: %s", last_error, (LPCSTR)msg_buf);

                        // Free memory
                        LocalFree(msg_buf);

                        system("PAUSE");
                        #endif
                    }
                }
            }
            ++idx;
        }
        SetupDiDeleteDeviceInfo(dev_set, &dev_info);
        SetupDiDestroyDeviceInfoList(dev_set);
    }
    return vec_size;
}
