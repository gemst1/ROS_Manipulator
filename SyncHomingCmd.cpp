//
// Created by jeon on 18. 10. 21.
//

#include <iostream>
#include "Definitions.h"
#include <string.h>
#include <sstream>
#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdio.h>
#include <list>
#include <math.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/times.h>
#include <sys/time.h>

typedef void* HANDLE;
typedef int BOOL;

enum EAppMode
{
    AM_UNKNOWN,
    AM_DEMO,
    AM_INTERFACE_LIST,
    AM_PROTOCOL_LIST,
    AM_VERSION_INFO,
    AM_HOMING,
    AM_POSITION_CON
};

using namespace std;

void* g_pKeyHandle = 0;
unsigned short g_usNodeId = 1;
string g_deviceName;
string g_protocolStackName;
string g_interfaceName;
string g_portName;
int g_baudrate = 0;
EAppMode g_eAppMode = AM_HOMING;

// Joint Parameters
unsigned short g_usNodeId_arr[] = {1, 2, 3, 4, 5, 6, 7};
int g_NbofJoint = sizeof(g_usNodeId_arr)/ sizeof(*g_usNodeId_arr);
int g_GearRatio[] = {160, 160, 160, 120, 100, 100, 100};
double g_AxisVelRatio[] = {1.6, 1.6, 1.6, 1.2, 1, 1, 1};
int g_PulseRev[] = {4096, 4096, 4096, 4096, 2048, 2048, 2048};
int g_HomeOffset_arr[] = {72818, 72818, 72818, 54613, 22378, 22378, 22378};

// Position Control Parameters
long g_PosDesired;
unsigned int g_ProfileVelocity;
long g_PosCenter[] = {0, 0, 0, 0, 0, 0, 0};
long g_PosPositiveLimit[] = {0, 0, 0, 0, 0, 0, 0};
int g_CapturedPosition;

// Homing Parameters
unsigned  short g_HomingFirstId[] = {2, 5, 6};
unsigned  short g_HomingSecondId[] = {1, 3, 5, 7};
unsigned int g_HomingAcceleration;
unsigned int g_SpeedSwitch;
unsigned int g_SpeedIndex;
unsigned int l_SpeedSwitch;
unsigned int l_SpeedIndex;
int g_HomeOffset;
int l_HomeOffset;
unsigned short g_CurrentThreshold;
int g_HomePosition;
int g_IsQuickStopped;

// Digital Input/Output Parameters
unsigned short g_digitalIOIndex = 0x3142;
unsigned short g_digitalIPropIndex = 0x3141;
unsigned char g_digiatlIPol = 0x02; // Digital Input Polarity
unsigned char g_digitalIO_1 = 0x01; // Digital Input Pin for Limit
unsigned char g_digitalIO_4 = 0x04; // Digital Input Pin for QuickStop
unsigned int g_NbofBytesToWrite_1 = 1;
unsigned int g_NbofBytesToWrite_2 = 2;
unsigned int g_NbofBytesToWrite_4 = 4;
// Digital Input/Output Pin Configuration Parameters
unsigned char g_NegativeLimit = 0;
unsigned char g_GeneralD = 19;
unsigned char g_QuickStop = 28;
unsigned char g_None = 255;
unsigned short g_LowActive = 0x0001;
unsigned short g_HighActive = 0x0000;
unsigned char data;

const string g_programName = "HomingCmd";

#ifndef MMC_SUCCESS
#define MMC_SUCCESS 0
#endif

#ifndef MMC_FAILED
#define MMC_FAILED 1
#endif

#ifndef MMC_MAX_LOG_MSG_SIZE
#define MMC_MAX_LOG_MSG_SIZE 512
#endif

// Functions
void  LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode);
void  LogInfo(string message);
void  PrintUsage();
void  PrintHeader();
void  PrintSettings();
int   OpenDevice(unsigned int* p_pErrorCode);
int   CloseDevice(unsigned int* p_pErrorCode);
void  SetDefaultParameters();
int   ParseArguments(int argc, char** argv);
int   HomingMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode);
int   BothHomingMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode);
int   SyncHomingMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode);
int   PositionControl(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode, long PosDesired);
int   DemoProfilePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode);
int   Demo(unsigned int* p_pErrorCode);
int   Home(unsigned int* p_pErrorCode);
int   DemoSafetyPos(unsigned int* p_pErrorCode, long PosDesired);
int   PrepareDemo(unsigned int* p_pErrorCode);
int   PrintAvailableInterfaces();
int	  PrintAvailablePorts(char* p_pInterfaceNameSel);
int	  PrintAvailableProtocols();
int   PrintDeviceVersion();

void PrintUsage()
{
    cout << "Usage: HomingCmd" << endl;
    cout << "\t-h : this help" << endl;
    cout << "\t-n : node id (default 1)" << endl;
    cout << "\t-d   : device name (EPOS2, EPOS4, default - EPOS4)"  << endl;
    cout << "\t-s   : protocol stack name (MAXON_RS232, CANopen, MAXON SERIAL V2, default - MAXON SERIAL V2)"  << endl;
    cout << "\t-i   : interface name (RS232, USB, CAN_ixx_usb 0, CAN_kvaser_usb 0,... default - USB)"  << endl;
    cout << "\t-p   : port name (COM1, USB0, CAN0,... default - USB0)" << endl;
    cout << "\t-b   : baudrate (115200, 1000000,... default - 1000000)" << endl;
    cout << "\t-l   : list available interfaces (valid device name and protocol stack required)" << endl;
    cout << "\t-r   : list supported protocols (valid device name required)" << endl;
    cout << "\t-v   : display device version" << endl;
}

void LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode)
{
    cerr << g_programName << ": " << functionName << " failed (result=" << p_lResult << ", errorCode=0x" << std::hex << p_ulErrorCode << ")"<< endl;
}

void LogInfo(string message)
{
    cout << message << endl;
}

void SeparatorLine()
{
    const int lineLength = 65;
    for(int i=0; i<lineLength; i++)
    {
        cout << "-";
    }
    cout << endl;
}

void PrintSettings()
{
    stringstream msg;

    msg << "default settings:" << endl;
    msg << "node id             = " << g_usNodeId << endl;
    msg << "device name         = '" << g_deviceName << "'" << endl;
    msg << "protocal stack name = '" << g_protocolStackName << "'" << endl;
    msg << "interface name      = '" << g_interfaceName << "'" << endl;
    msg << "port name           = '" << g_portName << "'"<< endl;
    msg << "baudrate            = " << g_baudrate;

    LogInfo(msg.str());

    SeparatorLine();
}

void SetDefaultParameters()
{
    //USB
    g_usNodeId = 1;
    g_deviceName = "EPOS4";
    g_protocolStackName = "MAXON SERIAL V2";
    g_interfaceName = "USB";
    g_portName = "USB1";
    g_baudrate = 1000000;
}

int OpenDevice(unsigned int* p_pErrorCode)
{
    int lResult = MMC_FAILED;

    char* pDeviceName = new char[255];
    char* pProtocolStackName = new char[255];
    char* pInterfaceName = new char[255];
    char* pPortName = new char[255];

    strcpy(pDeviceName, g_deviceName.c_str());
    strcpy(pProtocolStackName, g_protocolStackName.c_str());
    strcpy(pInterfaceName, g_interfaceName.c_str());
    strcpy(pPortName, g_portName.c_str());

    LogInfo("Open device...");

    g_pKeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, p_pErrorCode);

    if(g_pKeyHandle!=0 && *p_pErrorCode == 0)
    {
        unsigned int lBaudrate = 0;
        unsigned int lTimeout = 0;

        if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
        {
            if(VCS_SetProtocolStackSettings(g_pKeyHandle, g_baudrate, lTimeout, p_pErrorCode)!=0)
            {
                if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
                {
                    if(g_baudrate==(int)lBaudrate)
                    {
                        lResult = MMC_SUCCESS;
                    }
                }
            }
        }
    }
    else
    {
        g_pKeyHandle = 0;
    }

    delete []pDeviceName;
    delete []pProtocolStackName;
    delete []pInterfaceName;
    delete []pPortName;

    return lResult;
}

int CloseDevice(unsigned int* p_pErrorCode)
{
    int lResult = MMC_FAILED;

    *p_pErrorCode = 0;

    LogInfo("Close device");

    if(VCS_CloseDevice(g_pKeyHandle, p_pErrorCode)!=0 && *p_pErrorCode == 0)
    {
        lResult = MMC_SUCCESS;
    }

    return lResult;
}

int ParseArguments(int argc, char** argv)
{
    int lOption;
    int lResult = MMC_SUCCESS;

    opterr = 0;

    while((lOption = getopt(argc, argv, "hlrvHd:s:i:p:b:n:P:")) != -1)
    {
        switch (lOption)
        {
            case 'h':
                PrintUsage();
                lResult = 1;
                break;
            case 'd':
                g_deviceName = optarg;
                break;
            case 's':
                g_protocolStackName = optarg;
                break;
            case 'i':
                g_interfaceName = optarg;
                break;
            case 'p':
                g_portName = optarg;
                break;
            case 'b':
                g_baudrate = atoi(optarg);
                break;
            case 'n':
                g_usNodeId = (unsigned short)atoi(optarg);
                break;
            case 'l':
                g_eAppMode = AM_INTERFACE_LIST;
                break;
            case 'r':
                g_eAppMode = AM_PROTOCOL_LIST;
                break;
            case 'v':
                g_eAppMode = AM_VERSION_INFO;
                break;
            case 'H':
                g_eAppMode = AM_HOMING;
                break;
            case 'P':
                g_PosDesired = (long)atoi(optarg);
                g_eAppMode = AM_POSITION_CON;
                break;
            case '?':  // unknown option...
                stringstream msg;
                msg << "Unknown option: '" << char(optopt) << "'!";
                LogInfo(msg.str());
                PrintUsage();
                lResult = MMC_FAILED;
                break;
        }
    }

    return lResult;
}

//int HomingMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
//{
//    int lResult = MMC_SUCCESS;
//    stringstream msg;
//
//    g_HomingAcceleration = 1000;
//    g_SpeedSwitch = 200;
//    g_SpeedIndex = 10;
//    g_HomeOffset = 24576;
//
//    msg << "set Homing mode, node = " << p_usNodeId;
//    LogInfo(msg.str());
//
//    if(VCS_SetObject(p_DeviceHandle, p_usNodeId, g_digitalIOIndex, g_digitalIO_4, &g_None, g_NbofBytesToWrite_1, &g_NbofBytesToWrite_1, &p_rlErrorCode) == 0)
//    {
//        LogError("VCS_SetObject", lResult, p_rlErrorCode);
//        lResult = MMC_FAILED;
//    }
//    else
//    {
//        if(VCS_SetHomingParameter(p_DeviceHandle, p_usNodeId, g_HomingAcceleration, g_SpeedSwitch, g_SpeedIndex, g_HomeOffset, g_CurrentThreshold,
//                                  g_HomePosition, &p_rlErrorCode) == 0)
//        {
//            LogError("VCS_SetHomingParameter", lResult, p_rlErrorCode);
//            lResult = MMC_FAILED;
//        }
//        else
//        {
//            if(VCS_GetHomingParameter(p_DeviceHandle, p_usNodeId, &g_HomingAcceleration, &g_SpeedSwitch, &g_SpeedIndex, &g_HomeOffset, &g_CurrentThreshold,
//                                      &g_HomePosition, &p_rlErrorCode) == 0)
//            {
//                LogError("VCS_GetHomingParameter", lResult, p_rlErrorCode);
//                lResult = MMC_FAILED;
//            }
//            else
//            {
//                stringstream msg;
//                msg << "Homing Parameters, node = " << p_usNodeId << endl;
//                msg << "HomingAcceleration  = " << g_HomingAcceleration << endl;
//                msg << "SpeedSwitch         = " << g_SpeedSwitch << endl;
//                msg << "SpeedIndex          = " << g_SpeedIndex << endl;
//                msg << "HomeOffset          = " << g_HomeOffset << endl;
//                msg << "CurrentThreshold    = " << g_interfaceName << endl;
//                msg << "HomePosition        = " << g_HomePosition;
//                LogInfo(msg.str());
//                SeparatorLine();
//                if(VCS_ActivateHomingMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
//                {
//                    LogError("VCS_ActivateHomingMode", lResult, p_rlErrorCode);
//                    lResult = MMC_FAILED;
//                }
//                else
//                {
//                    if(VCS_FindHome(p_DeviceHandle, p_usNodeId, HM_NEGATIVE_LIMIT_SWITCH_AND_INDEX, &p_rlErrorCode) == 0)
//                    {
//                        LogError("VCS_FindHome", lResult, p_rlErrorCode);
//                        lResult = MMC_FAILED;
//                    }
//                    else
//                    {
//                        stringstream msg;
//                        LogInfo("Finding Home....");
//                        VCS_WaitForHomingAttained(p_DeviceHandle, p_usNodeId, 100000, &p_rlErrorCode);
//                        msg << "Home position is set, node = " << p_usNodeId;
//                        LogInfo(msg.str());
//                        if(VCS_SetObject(p_DeviceHandle, p_usNodeId, g_digitalIOIndex, g_digitalIO_4, &g_QuickStop, g_NbofBytesToWrite_1, &g_NbofBytesToWrite_1, &p_rlErrorCode) == 0)
//                        {
//                            LogError("VCS_SetObject", lResult, p_rlErrorCode);
//                            lResult = MMC_FAILED;
//                        }
//                        else
//                        {
//                            if(VCS_GetObject(p_DeviceHandle, p_usNodeId, g_digitalIOIndex, g_digitalIO_4, &data, g_NbofBytesToWrite_1, &g_NbofBytesToWrite_1, &p_rlErrorCode) == 0)
//                            {
//                                LogError("VCS_GetObject", lResult, p_rlErrorCode);
//                                lResult = MMC_FAILED;
//                            }
//                            else
//                            {
//                                if(int(data) == 28)
//                                {
//                                    LogInfo("Homing is complete, DigitalIO 4 is set as QuickStop");
//                                    SeparatorLine();
//                                }
//                                else
//                                {
//                                    LogInfo("Homing is not complete, DigitalIO 4 is NOT set as QuickStop");
//                                    SeparatorLine();
//                                }
//                            }
//                        }
//                    }
//
//                }
//            }
//        }
//    }
//
//    return lResult;
//}

//int BothHomingMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
//{
//    int lResult = MMC_SUCCESS;
//    stringstream msg;
//
//    g_HomingAcceleration = 1000;
//    g_SpeedSwitch = 200;
//    g_SpeedIndex = 10;
//    g_HomeOffset = 24576;
//
//    LogInfo("set Synchronized Homing mode");
//
//    if(VCS_SetObject(p_DeviceHandle, p_usNodeId, g_digitalIOIndex, 0x04, &g_None, g_NbofBytesToWrite_1, &g_NbofBytesToWrite_1, &p_rlErrorCode) &&
//       VCS_SetObject(p_DeviceHandle, p_usNodeId, g_digitalIOIndex, 0x01, &g_NegativeLimit, g_NbofBytesToWrite_1, &g_NbofBytesToWrite_1, &p_rlErrorCode) == 0)
//    {
//
//        lResult = MMC_FAILED;
//        LogError("VCS_SetObject", lResult, p_rlErrorCode);
//    }
//    else
//    {
//        if(VCS_SetHomingParameter(p_DeviceHandle, p_usNodeId, g_HomingAcceleration, g_SpeedSwitch, g_SpeedIndex, g_HomeOffset, g_CurrentThreshold,
//                                  g_HomePosition, &p_rlErrorCode) == 0)
//        {
//            lResult = MMC_FAILED;
//            LogError("VCS_SetHomingParameter", lResult, p_rlErrorCode);
//        }
//        else
//        {
//            if(VCS_GetHomingParameter(p_DeviceHandle, p_usNodeId, &g_HomingAcceleration, &g_SpeedSwitch, &g_SpeedIndex, &g_HomeOffset, &g_CurrentThreshold,
//                                      &g_HomePosition, &p_rlErrorCode) == 0)
//            {
//                lResult = MMC_FAILED;
//                LogError("VCS_GetHomingParameter", lResult, p_rlErrorCode);
//            }
//            else
//            {
//                stringstream msg;
//                msg << "Homing Parameters, node = " << p_usNodeId << endl;
//                msg << "HomingAcceleration  = " << g_HomingAcceleration << endl;
//                msg << "SpeedSwitch         = " << g_SpeedSwitch << endl;
//                msg << "SpeedIndex          = " << g_SpeedIndex << endl;
//                msg << "HomeOffset          = " << g_HomeOffset << endl;
//                msg << "CurrentThreshold    = " << g_interfaceName << endl;
//                msg << "HomePosition        = " << g_HomePosition;
//                LogInfo(msg.str());
//                SeparatorLine();
//                if(VCS_ActivateHomingMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
//                {
//                    lResult = MMC_FAILED;
//                    LogError("VCS_ActivateHomingMode", lResult, p_rlErrorCode);
//                }
//                else
//                {
//                    if(VCS_FindHome(p_DeviceHandle, p_usNodeId, HM_NEGATIVE_LIMIT_SWITCH_AND_INDEX, &p_rlErrorCode) == 0)
//                    {
//                        lResult = MMC_FAILED;
//                        LogError("VCS_FindHome", lResult, p_rlErrorCode);
//                    }
//                    else
//                    {
//                        stringstream msg;
//                        LogInfo("Finding Home....");
//                        VCS_WaitForHomingAttained(p_DeviceHandle, p_usNodeId, 100000, &p_rlErrorCode);
//                        msg << "Home position is set, node = " << p_usNodeId;
//                        LogInfo(msg.str());
//
//                        if(VCS_SetObject(p_DeviceHandle, p_usNodeId, g_digitalIOIndex, 0x04, &g_None, g_NbofBytesToWrite_1, &g_NbofBytesToWrite_1, &p_rlErrorCode)
//                           && VCS_SetObject(p_DeviceHandle, p_usNodeId, g_digitalIOIndex, 0x01, &g_QuickStop, g_NbofBytesToWrite_1, &g_NbofBytesToWrite_1, &p_rlErrorCode)== 0)
//                        {
//                            lResult = MMC_FAILED;
//                            LogError("VCS_SetObject", lResult, p_rlErrorCode);
//                        }
//                        else
//                        {
//                            if(VCS_ActivateProfileVelocityMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
//                            {
//                                lResult = MMC_FAILED;
//                                LogError("VCS_ActivateProfileVelocityMode", lResult, p_rlErrorCode);
//                            }
//                            else
//                            {
//                                if(VCS_MoveWithVelocity(p_DeviceHandle, p_usNodeId, g_SpeedSwitch, &p_rlErrorCode) == 0)
//                                {
//                                    lResult = MMC_FAILED;
//                                    LogError("VCS_MoveWithVelocity", lResult, p_rlErrorCode);
//                                }
//                                else
//                                {
//                                    LogInfo("Finding Positive Limit Switch...");
//                                    do
//                                    {
//                                        if(VCS_GetQuickStopState(p_DeviceHandle, p_usNodeId, &g_IsQuickStopped, &p_rlErrorCode) == 0)
//                                        {
//                                            lResult = MMC_FAILED;
//                                            LogError("VCS_GetQuickStopState", lResult, p_rlErrorCode);
//                                        }
//                                    }
//                                    while (g_IsQuickStopped == 0);
//                                    LogInfo("Found Positive Limit Switch!");
//                                    if(VCS_SetObject(p_DeviceHandle, p_usNodeId, g_digitalIOIndex, 0x01, &g_QuickStop, g_NbofBytesToWrite_1, &g_NbofBytesToWrite_1, &p_rlErrorCode)
//                                       && VCS_SetObject(p_DeviceHandle, p_usNodeId, 0x3141, 0x02, &g_LowActive, g_NbofBytesToWrite_2, &g_NbofBytesToWrite_2, &p_rlErrorCode)== 0)
//                                    {
//                                        lResult = MMC_FAILED;
//                                        LogError("VCS_SetObject", lResult, p_rlErrorCode);
//                                    }
//                                    else
//                                    {
//                                        if(VCS_MoveWithVelocity(p_DeviceHandle, p_usNodeId, -g_SpeedIndex, &p_rlErrorCode) == 0)
//                                        {
//                                            lResult = MMC_FAILED;
//                                            LogError("VCS_MoveWithVelocity", lResult, p_rlErrorCode);
//                                        }
//                                        else
//                                        {
//                                            LogInfo("Finding Positive Home...");
//                                            do
//                                            {
//                                                if(VCS_GetQuickStopState(p_DeviceHandle, p_usNodeId, &g_IsQuickStopped, &p_rlErrorCode) == 0)
//                                                {
//                                                    lResult = MMC_FAILED;
//                                                    LogError("VCS_GetQuickStopState", lResult, p_rlErrorCode);
//                                                }
//                                                else
//                                                {
//                                                    if(VCS_GetPositionIs(p_DeviceHandle, p_usNodeId, &g_CapturedPosition, &p_rlErrorCode) == 0)
//                                                    {
//                                                        lResult = MMC_FAILED;
//                                                        LogError("VCS_GetPositionIs", lResult, p_rlErrorCode);
//                                                    }
//                                                }
//                                            }
//                                            while (g_IsQuickStopped == 0);
//                                            stringstream msg;
//
//                                            g_PosPositiveLimit = (long)(g_CapturedPosition - g_HomeOffset);
//                                            g_PosCenter = (long)(g_PosPositiveLimit/2);
//
//                                            msg << "Found Positive Limit    : " << g_PosPositiveLimit << endl;
//                                            msg << "Found Center Position   : " << g_PosCenter;
//                                            LogInfo(msg.str());
//
//                                            if(VCS_SetObject(p_DeviceHandle, p_usNodeId, 0x3141, 0x02, &g_HighActive, g_NbofBytesToWrite_2, &g_NbofBytesToWrite_2, &p_rlErrorCode)
//                                               && VCS_SetObject(p_DeviceHandle, p_usNodeId, g_digitalIOIndex, 0x01, &g_None, g_NbofBytesToWrite_1, &g_NbofBytesToWrite_1, &p_rlErrorCode)== 0)
//                                            {
//                                                lResult = MMC_FAILED;
//                                                LogError("VCS_SetObject", lResult, p_rlErrorCode);
//                                            }
//                                            else
//                                            {
//                                                if(VCS_ActivateProfilePositionMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
//                                                {
//                                                    lResult = MMC_FAILED;
//                                                    LogError("VCS_ActivateProfilePositionMode", lResult, p_rlErrorCode);
//                                                }
//                                                else
//                                                {
//                                                    if(VCS_MoveToPosition(p_DeviceHandle, p_usNodeId, g_PosCenter, 1, 1, &p_rlErrorCode) == 0)
//                                                    {
//                                                        lResult = MMC_FAILED;
//                                                        LogError("VCS_MoveToPosition", lResult, p_rlErrorCode);
//                                                    }
//                                                    else
//                                                    {
//                                                        VCS_WaitForTargetReached(p_DeviceHandle, p_usNodeId, 100000, &p_rlErrorCode);
//                                                        if(VCS_SetObject(p_DeviceHandle, p_usNodeId, g_digitalIOIndex, 0x01, &g_NegativeLimit, g_NbofBytesToWrite_1, &g_NbofBytesToWrite_1, &p_rlErrorCode)
//                                                           && VCS_SetObject(p_DeviceHandle, p_usNodeId, g_digitalIOIndex, 0x04, &g_QuickStop, g_NbofBytesToWrite_1, &g_NbofBytesToWrite_1, &p_rlErrorCode)== 0)
//                                                        {
//                                                            lResult = MMC_FAILED;
//                                                            LogError("VCS_SetObject", lResult, p_rlErrorCode);
//                                                        }
//                                                        else
//                                                        {
//                                                            if(VCS_SetHomingParameter(p_DeviceHandle, p_usNodeId, g_HomingAcceleration, g_SpeedSwitch, g_SpeedIndex, 0, g_CurrentThreshold,
//                                                                                      g_HomePosition, &p_rlErrorCode) == 0)
//                                                            {
//                                                                lResult = MMC_FAILED;
//                                                                LogError("VCS_SetHomingParameter", lResult, p_rlErrorCode);
//                                                            }
//                                                            else
//                                                            {
//                                                                if(VCS_ActivateHomingMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
//                                                                {
//                                                                    lResult = MMC_FAILED;
//                                                                    LogError("VCS_ActivateHomingMode", lResult, p_rlErrorCode);
//                                                                }
//                                                                else
//                                                                {
//                                                                    if(VCS_FindHome(p_DeviceHandle, p_usNodeId, HM_ACTUAL_POSITION, &p_rlErrorCode) == 0)
//                                                                    {
//                                                                        lResult = MMC_FAILED;
//                                                                        LogError("VCS_FindHome", lResult, p_rlErrorCode);
//                                                                    }
//                                                                    else
//                                                                    {
//                                                                        VCS_WaitForHomingAttained(p_DeviceHandle, p_usNodeId, 15000, &p_rlErrorCode);
//                                                                        if(VCS_GetHomingParameter(p_DeviceHandle, p_usNodeId, &g_HomingAcceleration, &g_SpeedSwitch, &g_SpeedIndex, &g_HomeOffset, &g_CurrentThreshold,
//                                                                                                  &g_HomePosition, &p_rlErrorCode) == 0)
//                                                                        {
//                                                                            lResult = MMC_FAILED;
//                                                                            LogError("VCS_GetHomingParameter", lResult, p_rlErrorCode);
//                                                                        }
//                                                                        else
//                                                                        {
//                                                                            stringstream msg;
//                                                                            msg << "Homing Parameters, node = " << p_usNodeId << endl;
//                                                                            msg << "HomingAcceleration  = " << g_HomingAcceleration << endl;
//                                                                            msg << "SpeedSwitch         = " << g_SpeedSwitch << endl;
//                                                                            msg << "SpeedIndex          = " << g_SpeedIndex << endl;
//                                                                            msg << "HomeOffset          = " << g_HomeOffset << endl;
//                                                                            msg << "CurrentThreshold    = " << g_interfaceName << endl;
//                                                                            msg << "HomePosition        = " << g_HomePosition;
//                                                                            LogInfo(msg.str());
//                                                                            SeparatorLine();
//                                                                        }
//                                                                    }
//                                                                }
//                                                            }
//                                                        }
//                                                    }
//                                                }
//                                            }
//                                        }
//                                    }
//                                }
//                            }
//                        }
//                        if(VCS_GetObject(p_DeviceHandle, p_usNodeId, g_digitalIOIndex, g_digitalIO_4, &data, g_NbofBytesToWrite_1, &g_NbofBytesToWrite_1, &p_rlErrorCode) == 0)
//                        {
//                            lResult = MMC_FAILED;
//                            LogError("VCS_GetObject", lResult, p_rlErrorCode);
//                        }
//                        else
//                        {
//                            if(int(data) == 28)
//                            {
//                                LogInfo("Homing is complete, DigitalIO 4 is set as QuickStop");
//                                SeparatorLine();
//                            }
//                            else
//                            {
//                                LogInfo("Homing is not complete, DigitalIO 4 is NOT set as QuickStop");
//                                SeparatorLine();
//                            }
//                        }
//                    }
//                }
//
//            }
//        }
//    }
//
//    return lResult;
//}

int SyncHomingMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
{
    int lResult = MMC_SUCCESS;
    stringstream msg;

    g_HomingAcceleration = 1000;
    g_SpeedSwitch = 200;
    g_SpeedIndex = 10;
    g_HomePosition = 0;

    LogInfo("set Synchronized Homing mode");

    // Parameter setting for Homing mode
    for (int i=0; i<g_NbofJoint; i++)
    {
        if(VCS_SetObject(p_DeviceHandle, g_usNodeId_arr[i], g_digitalIOIndex, g_digitalIO_4, &g_None, g_NbofBytesToWrite_1, &g_NbofBytesToWrite_1, &p_rlErrorCode) &&
           VCS_SetObject(p_DeviceHandle, g_usNodeId_arr[i], g_digitalIOIndex, g_digitalIO_1, &g_NegativeLimit, g_NbofBytesToWrite_1, &g_NbofBytesToWrite_1, &p_rlErrorCode) == 0)
        {

            lResult = MMC_FAILED;
            LogError("VCS_SetObject", lResult, p_rlErrorCode);
        }
        else
        {
            if(VCS_SetHomingParameter(p_DeviceHandle, g_usNodeId_arr[i], g_HomingAcceleration, (unsigned int)(g_SpeedSwitch*g_HomeOffset_arr[i]),
                                      (unsigned int)(g_SpeedIndex*g_AxisVelRatio[i]), g_HomeOffset_arr[i], g_CurrentThreshold, g_HomePosition, &p_rlErrorCode) == 0)
            {
                lResult = MMC_FAILED;
                LogError("VCS_SetHomingParameter", lResult, p_rlErrorCode);
            }
            else
            {
                if(VCS_ActivateHomingMode(p_DeviceHandle, g_usNodeId_arr[i], &p_rlErrorCode) == 0)
                {
                    lResult = MMC_FAILED;
                    LogError("VCS_ActivateHomingMode", lResult, p_rlErrorCode);
                }
            }
        }
    }

    // Find Negative limit of Joint 2, 4, 6
    for(int i=2; i>=0; i--)
    {
        if(lResult == MMC_SUCCESS)
        {
            if(VCS_FindHome(p_DeviceHandle, g_HomingFirstId[i], HM_NEGATIVE_LIMIT_SWITCH_AND_INDEX, &p_rlErrorCode) == 0)
            {
                lResult = MMC_FAILED;
                LogError("VCS_FindHome", lResult, p_rlErrorCode);
            }
        }
    }

    // Waiting for Negative Limit
    if(lResult == MMC_SUCCESS)
    {
        stringstream msg;
        LogInfo("Finding Negative Limit....");
        VCS_WaitForHomingAttained(p_DeviceHandle, g_HomingFirstId[0], 100000, &p_rlErrorCode);
        VCS_WaitForHomingAttained(p_DeviceHandle, g_HomingFirstId[1], 100000, &p_rlErrorCode);
        VCS_WaitForHomingAttained(p_DeviceHandle, g_HomingFirstId[2], 100000, &p_rlErrorCode);
        msg << "Negative Limit is set, node = " << g_HomingFirstId[0] << ", " << g_HomingFirstId[1] << ", " << g_HomingFirstId[2];
        LogInfo(msg.str());
    }

    // Ready for finding Positive Limit
    if(lResult == MMC_SUCCESS)
    {
        for (int i=0; i<3; i++)
        {
            if(VCS_SetObject(p_DeviceHandle, g_HomingFirstId[i], g_digitalIOIndex, g_digitalIO_4, &g_None, g_NbofBytesToWrite_1, &g_NbofBytesToWrite_1, &p_rlErrorCode)
               && VCS_SetObject(p_DeviceHandle, g_HomingFirstId[i], g_digitalIOIndex, g_digitalIO_1, &g_QuickStop, g_NbofBytesToWrite_1, &g_NbofBytesToWrite_1, &p_rlErrorCode)== 0)
            {
                lResult = MMC_FAILED;
                LogError("VCS_SetObject", lResult, p_rlErrorCode);
            }
            else
            {
                if(VCS_ActivateProfileVelocityMode(p_DeviceHandle, g_HomingFirstId[i], &p_rlErrorCode) == 0)
                {
                    lResult = MMC_FAILED;
                    LogError("VCS_ActivateProfileVelocityMode", lResult, p_rlErrorCode);
                }
            }
        }
    }

    // Find Positive limit Switch of Joint 2, 4, 6
    for (int i=2; i>=0; i--)
    {
        if(lResult == MMC_SUCCESS)
        {
            if(VCS_MoveWithVelocity(p_DeviceHandle, g_HomingFirstId[i], (long)(g_SpeedSwitch*g_HomeOffset_arr[g_HomingFirstId[i]-1]), &p_rlErrorCode) == 0)
            {
                lResult = MMC_FAILED;
                LogError("VCS_MoveWithVelocity", lResult, p_rlErrorCode);
            }
        }
    }

    // Waiting for Positive Limit Switch
    if(lResult == MMC_SUCCESS)
    {
        stringstream msg;
        LogInfo("Finding Positive Limit Switch...");
        for (int i=0; i<3; i++)
        {
            do
            {
                if(VCS_GetQuickStopState(p_DeviceHandle, g_HomingFirstId[i], &g_IsQuickStopped, &p_rlErrorCode) == 0)
                {
                    lResult = MMC_FAILED;
                    LogError("VCS_GetQuickStopState", lResult, p_rlErrorCode);
                }
            }
            while (g_IsQuickStopped == 0);
        }
    }

    for (int i=0; i<3; i++)
    {
        if(lResult == MMC_SUCCESS)
        {
            if(VCS_SetObject(p_DeviceHandle, g_HomingFirstId[i], g_digitalIOIndex, g_digitalIO_1, &g_QuickStop, g_NbofBytesToWrite_1, &g_NbofBytesToWrite_1, &p_rlErrorCode)
               && VCS_SetObject(p_DeviceHandle, g_HomingFirstId[i], g_digitalIPropIndex, g_digiatlIPol, &g_LowActive, g_NbofBytesToWrite_2, &g_NbofBytesToWrite_2, &p_rlErrorCode)== 0)
            {
                lResult = MMC_FAILED;
                LogError("VCS_SetObject", lResult, p_rlErrorCode);
            }
        }
    }

    // Find Positive Limit of Joint 2, 4, 6
    for (int i=2; i>=0; i--)
    {
        if(lResult == MMC_SUCCESS)
        {
            if(VCS_MoveWithVelocity(p_DeviceHandle, g_HomingFirstId[i], -(long)(g_SpeedIndex*g_HomeOffset_arr[g_HomingFirstId[i]-1]), &p_rlErrorCode) == 0)
            {
                lResult = MMC_FAILED;
                LogError("VCS_MoveWithVelocity", lResult, p_rlErrorCode);
            }
        }
    }

    // Waiting for Positive Limit
    if(lResult == MMC_SUCCESS)
    {
        stringstream msg;
        for (int i=0; i<3; i++)
        {
            do
            {
                if(VCS_GetQuickStopState(p_DeviceHandle, g_HomingFirstId[i], &g_IsQuickStopped, &p_rlErrorCode) == 0)
                {
                    lResult = MMC_FAILED;
                    LogError("VCS_GetQuickStopState", lResult, p_rlErrorCode);
                }
                else
                {
                    if(VCS_GetPositionIs(p_DeviceHandle, g_HomingFirstId[i], &g_CapturedPosition, &p_rlErrorCode) == 0)
                    {
                        lResult = MMC_FAILED;
                        LogError("VCS_GetPositionIs", lResult, p_rlErrorCode);
                    }
                }
            }
            while (g_IsQuickStopped == 0);
            stringstream msg;

            g_PosPositiveLimit[g_HomingFirstId[i]-1] = (long)(g_CapturedPosition - g_HomeOffset_arr[g_HomingFirstId[i]-1]);
            g_PosCenter[g_HomingFirstId[i]-1] = (long)(g_PosPositiveLimit[g_HomingFirstId[i]-1]/2);
        }
        msg << "Positive Limit is set, node = " << g_HomingFirstId[0] << ", " << g_HomingFirstId[1] << ", " << g_HomingFirstId[2];
        LogInfo(msg.str());
    }

    // Set Limit Switch as None
    if(lResult == MMC_SUCCESS)
    {
        for (int i=0; i<3; i++)
        {
            if(VCS_SetObject(p_DeviceHandle, g_HomingFirstId[i], g_digitalIPropIndex, g_digiatlIPol, &g_HighActive, g_NbofBytesToWrite_2, &g_NbofBytesToWrite_2, &p_rlErrorCode)
               && VCS_SetObject(p_DeviceHandle, g_HomingFirstId[i], g_digitalIOIndex, g_digitalIO_1, &g_None, g_NbofBytesToWrite_1, &g_NbofBytesToWrite_1, &p_rlErrorCode)== 0)
            {
                lResult = MMC_FAILED;
                LogError("VCS_SetObject", lResult, p_rlErrorCode);
            }
            else
            {
                if(VCS_ActivateProfilePositionMode(p_DeviceHandle, g_HomingFirstId[i], &p_rlErrorCode) == 0)
                {
                    lResult = MMC_FAILED;
                    LogError("VCS_ActivateProfilePositionMode", lResult, p_rlErrorCode);
                }
            }
        }
    }

    // Move to Center Position
    for (int i=2; i>=0; i--)
    {
        if(lResult == MMC_SUCCESS)
        {
            if(VCS_MoveToPosition(p_DeviceHandle, g_HomingFirstId[i], g_PosCenter[g_HomingFirstId[i]-1], 1, 1, &p_rlErrorCode) == 0)
            {
                lResult = MMC_FAILED;
                LogError("VCS_MoveToPosition", lResult, p_rlErrorCode);
            }
        }
    }

    if(lResult == MMC_SUCCESS)
    {
        VCS_WaitForTargetReached(p_DeviceHandle, g_HomingFirstId[2], 100000, &p_rlErrorCode);
        VCS_WaitForTargetReached(p_DeviceHandle, g_HomingFirstId[1], 100000, &p_rlErrorCode);
        VCS_WaitForTargetReached(p_DeviceHandle, g_HomingFirstId[0], 100000, &p_rlErrorCode);
    }

    // Set Center position as Home
    for (int i=0; i<3; i++)
    {
        if(lResult == MMC_SUCCESS)
        {
            if(VCS_SetObject(p_DeviceHandle, g_HomingFirstId[i], g_digitalIOIndex, g_digitalIO_1, &g_NegativeLimit, g_NbofBytesToWrite_1, &g_NbofBytesToWrite_1, &p_rlErrorCode)
               && VCS_SetObject(p_DeviceHandle, g_HomingFirstId[i], g_digitalIOIndex, g_digitalIO_4, &g_QuickStop, g_NbofBytesToWrite_1, &g_NbofBytesToWrite_1, &p_rlErrorCode)== 0)
            {
                lResult = MMC_FAILED;
                LogError("VCS_SetObject", lResult, p_rlErrorCode);
            }
            else
            {
                if(VCS_SetHomingParameter(p_DeviceHandle, g_HomingFirstId[i], g_HomingAcceleration, (unsigned int)(g_SpeedSwitch*g_HomeOffset_arr[g_HomingFirstId[i]-1]),
                                          (unsigned int)(g_SpeedIndex*g_AxisVelRatio[g_HomingFirstId[i]-1]), 0, g_CurrentThreshold, g_HomePosition, &p_rlErrorCode) == 0)
                {
                    lResult = MMC_FAILED;
                    LogError("VCS_SetHomingParameter", lResult, p_rlErrorCode);
                }
                else
                {
                    if(VCS_ActivateHomingMode(p_DeviceHandle, g_HomingFirstId[i], &p_rlErrorCode) == 0)
                    {
                        lResult = MMC_FAILED;
                        LogError("VCS_ActivateHomingMode", lResult, p_rlErrorCode);
                    }
                    else
                    {
                        if(VCS_FindHome(p_DeviceHandle, g_HomingFirstId[i], HM_ACTUAL_POSITION, &p_rlErrorCode) == 0)
                        {
                            lResult = MMC_FAILED;
                            LogError("VCS_FindHome", lResult, p_rlErrorCode);
                        }
                        else
                        {
                            VCS_WaitForHomingAttained(p_DeviceHandle, g_HomingFirstId[i], 15000, &p_rlErrorCode);
                            if(VCS_GetHomingParameter(p_DeviceHandle, g_HomingFirstId[i], &g_HomingAcceleration, &l_SpeedSwitch, &l_SpeedIndex, &l_HomeOffset, &g_CurrentThreshold,
                                                      &g_HomePosition, &p_rlErrorCode) == 0)
                            {
                                lResult = MMC_FAILED;
                                LogError("VCS_GetHomingParameter", lResult, p_rlErrorCode);
                            }
                            else
                            {
                                stringstream msg;
                                msg << "Homing Parameters, node = " << g_HomingFirstId[i] << endl;
                                msg << "HomingAcceleration  = " << g_HomingAcceleration << endl;
                                msg << "SpeedSwitch         = " << l_SpeedSwitch << endl;
                                msg << "SpeedIndex          = " << l_SpeedIndex << endl;
                                msg << "HomeOffset          = " << l_HomeOffset << endl;
                                msg << "CurrentThreshold    = " << g_interfaceName << endl;
                                msg << "HomePosition        = " << g_HomePosition;
                                LogInfo(msg.str());
                                SeparatorLine();
                            }
                        }
                    }
                }
            }
        }
    }

    // Find Negative limit of Joint 1, 3, 5, 7
    for(int i=3; i>=0; i--)
    {
        if(lResult == MMC_SUCCESS)
        {
            if(VCS_FindHome(p_DeviceHandle, g_HomingSecondId[i], HM_NEGATIVE_LIMIT_SWITCH_AND_INDEX, &p_rlErrorCode) == 0)
            {
                lResult = MMC_FAILED;
                LogError("VCS_FindHome", lResult, p_rlErrorCode);
            }
        }
    }

    // Waiting for Negative Limit
    if(lResult == MMC_SUCCESS)
    {
        stringstream msg;
        LogInfo("Finding Negative Limit....");
        VCS_WaitForHomingAttained(p_DeviceHandle, g_HomingSecondId[0], 100000, &p_rlErrorCode);
        VCS_WaitForHomingAttained(p_DeviceHandle, g_HomingSecondId[1], 100000, &p_rlErrorCode);
        VCS_WaitForHomingAttained(p_DeviceHandle, g_HomingSecondId[2], 100000, &p_rlErrorCode);
        VCS_WaitForHomingAttained(p_DeviceHandle, g_HomingSecondId[3], 100000, &p_rlErrorCode);
        msg << "Negative Limit is set, node = " << g_HomingFirstId[0] << ", " << g_HomingFirstId[1] << ", " << g_HomingFirstId[2] << ", " << g_HomingFirstId[3];
        LogInfo(msg.str());
    }

    // Ready for finding Positive Limit
    if(lResult == MMC_SUCCESS)
    {
        for (int i=0; i<4; i++)
        {
            if(VCS_SetObject(p_DeviceHandle, g_HomingSecondId[i], g_digitalIOIndex, g_digitalIO_4, &g_None, g_NbofBytesToWrite_1, &g_NbofBytesToWrite_1, &p_rlErrorCode)
               && VCS_SetObject(p_DeviceHandle, g_HomingSecondId[i], g_digitalIOIndex, g_digitalIO_1, &g_QuickStop, g_NbofBytesToWrite_1, &g_NbofBytesToWrite_1, &p_rlErrorCode)== 0)
            {
                lResult = MMC_FAILED;
                LogError("VCS_SetObject", lResult, p_rlErrorCode);
            }
            else
            {
                if(VCS_ActivateProfileVelocityMode(p_DeviceHandle, g_HomingSecondId[i], &p_rlErrorCode) == 0)
                {
                    lResult = MMC_FAILED;
                    LogError("VCS_ActivateProfileVelocityMode", lResult, p_rlErrorCode);
                }
            }
        }
    }

    // Find Positive limit Switch of Joint 1, 3, 5, 7
    for (int i=3; i>=0; i--)
    {
        if(lResult == MMC_SUCCESS)
        {
            if(VCS_MoveWithVelocity(p_DeviceHandle, g_HomingSecondId[i], (long)(g_SpeedSwitch*g_HomeOffset_arr[g_HomingSecondId[i]-1]), &p_rlErrorCode) == 0)
            {
                lResult = MMC_FAILED;
                LogError("VCS_MoveWithVelocity", lResult, p_rlErrorCode);
            }
        }
    }

    // Waiting for Positive Limit Switch
    if(lResult == MMC_SUCCESS)
    {
        stringstream msg;
        LogInfo("Finding Positive Limit Switch...");
        for (int i=0; i<4; i++)
        {
            do
            {
                if(VCS_GetQuickStopState(p_DeviceHandle, g_HomingSecondId[i], &g_IsQuickStopped, &p_rlErrorCode) == 0)
                {
                    lResult = MMC_FAILED;
                    LogError("VCS_GetQuickStopState", lResult, p_rlErrorCode);
                }
            }
            while (g_IsQuickStopped == 0);
        }
    }

    for (int i=0; i<4; i++)
    {
        if(lResult == MMC_SUCCESS)
        {
            if(VCS_SetObject(p_DeviceHandle, g_HomingSecondId[i], g_digitalIOIndex, g_digitalIO_1, &g_QuickStop, g_NbofBytesToWrite_1, &g_NbofBytesToWrite_1, &p_rlErrorCode)
               && VCS_SetObject(p_DeviceHandle, g_HomingSecondId[i], g_digitalIPropIndex, g_digiatlIPol, &g_LowActive, g_NbofBytesToWrite_2, &g_NbofBytesToWrite_2, &p_rlErrorCode)== 0)
            {
                lResult = MMC_FAILED;
                LogError("VCS_SetObject", lResult, p_rlErrorCode);
            }
        }
    }

    // Find Positive Limit of Joint 1, 3, 5, 7
    for (int i=3; i>=0; i--)
    {
        if(lResult == MMC_SUCCESS)
        {
            if(VCS_MoveWithVelocity(p_DeviceHandle, g_HomingSecondId[i], -(long)(g_SpeedIndex*g_HomeOffset_arr[g_HomingSecondId[i]-1]), &p_rlErrorCode) == 0)
            {
                lResult = MMC_FAILED;
                LogError("VCS_MoveWithVelocity", lResult, p_rlErrorCode);
            }
        }
    }

    // Waiting for Positive Limit
    if(lResult == MMC_SUCCESS)
    {
        stringstream msg;
        for (int i=0; i<4; i++)
        {
            do
            {
                if(VCS_GetQuickStopState(p_DeviceHandle, g_HomingSecondId[i], &g_IsQuickStopped, &p_rlErrorCode) == 0)
                {
                    lResult = MMC_FAILED;
                    LogError("VCS_GetQuickStopState", lResult, p_rlErrorCode);
                }
                else
                {
                    if(VCS_GetPositionIs(p_DeviceHandle, g_HomingSecondId[i], &g_CapturedPosition, &p_rlErrorCode) == 0)
                    {
                        lResult = MMC_FAILED;
                        LogError("VCS_GetPositionIs", lResult, p_rlErrorCode);
                    }
                }
            }
            while (g_IsQuickStopped == 0);
            stringstream msg;

            g_PosPositiveLimit[g_HomingSecondId[i]-1] = (long)(g_CapturedPosition - g_HomeOffset_arr[g_HomingSecondId[i]-1]);
            g_PosCenter[g_HomingSecondId[i]-1] = (long)(g_PosPositiveLimit[g_HomingSecondId[i]-1]/2);
        }
        msg << "Positive Limit is set, node = " << g_HomingSecondId[0] << ", " << g_HomingSecondId[1] << ", " << g_HomingSecondId[2] << ", " << g_HomingSecondId[3];
        LogInfo(msg.str());
    }

    // Set Limit Switch as None
    if(lResult == MMC_SUCCESS)
    {
        for (int i=0; i<4; i++)
        {
            if(VCS_SetObject(p_DeviceHandle, g_HomingSecondId[i], g_digitalIPropIndex, g_digiatlIPol, &g_HighActive, g_NbofBytesToWrite_2, &g_NbofBytesToWrite_2, &p_rlErrorCode)
               && VCS_SetObject(p_DeviceHandle, g_HomingSecondId[i], g_digitalIOIndex, g_digitalIO_1, &g_None, g_NbofBytesToWrite_1, &g_NbofBytesToWrite_1, &p_rlErrorCode)== 0)
            {
                lResult = MMC_FAILED;
                LogError("VCS_SetObject", lResult, p_rlErrorCode);
            }
            else
            {
                if(VCS_ActivateProfilePositionMode(p_DeviceHandle, g_HomingSecondId[i], &p_rlErrorCode) == 0)
                {
                    lResult = MMC_FAILED;
                    LogError("VCS_ActivateProfilePositionMode", lResult, p_rlErrorCode);
                }
            }
        }
    }

    // Move to Center Position
    for (int i=3; i>=0; i--)
    {
        if(lResult == MMC_SUCCESS)
        {
            if(VCS_MoveToPosition(p_DeviceHandle, g_HomingSecondId[i], g_PosCenter[g_HomingSecondId[i]-1], 1, 1, &p_rlErrorCode) == 0)
            {
                lResult = MMC_FAILED;
                LogError("VCS_MoveToPosition", lResult, p_rlErrorCode);
            }
        }
    }

    if(lResult == MMC_SUCCESS)
    {
        VCS_WaitForTargetReached(p_DeviceHandle, g_HomingSecondId[3], 100000, &p_rlErrorCode);
        VCS_WaitForTargetReached(p_DeviceHandle, g_HomingSecondId[2], 100000, &p_rlErrorCode);
        VCS_WaitForTargetReached(p_DeviceHandle, g_HomingSecondId[1], 100000, &p_rlErrorCode);
        VCS_WaitForTargetReached(p_DeviceHandle, g_HomingSecondId[0], 100000, &p_rlErrorCode);
    }

    // Set Center position as Home
    for (int i=0; i<4; i++)
    {
        if(lResult == MMC_SUCCESS)
        {
            if(VCS_SetObject(p_DeviceHandle, g_HomingSecondId[i], g_digitalIOIndex, g_digitalIO_1, &g_NegativeLimit, g_NbofBytesToWrite_1, &g_NbofBytesToWrite_1, &p_rlErrorCode)
               && VCS_SetObject(p_DeviceHandle, g_HomingSecondId[i], g_digitalIOIndex, g_digitalIO_4, &g_QuickStop, g_NbofBytesToWrite_1, &g_NbofBytesToWrite_1, &p_rlErrorCode)== 0)
            {
                lResult = MMC_FAILED;
                LogError("VCS_SetObject", lResult, p_rlErrorCode);
            }
            else
            {
                if(VCS_SetHomingParameter(p_DeviceHandle, g_HomingSecondId[i], g_HomingAcceleration, (unsigned int)(g_SpeedSwitch*g_HomeOffset_arr[g_HomingSecondId[i]-1]),
                                          (unsigned int)(g_SpeedIndex*g_AxisVelRatio[g_HomingSecondId[i]-1]), 0, g_CurrentThreshold, g_HomePosition, &p_rlErrorCode) == 0)
                {
                    lResult = MMC_FAILED;
                    LogError("VCS_SetHomingParameter", lResult, p_rlErrorCode);
                }
                else
                {
                    if(VCS_ActivateHomingMode(p_DeviceHandle, g_HomingSecondId[i], &p_rlErrorCode) == 0)
                    {
                        lResult = MMC_FAILED;
                        LogError("VCS_ActivateHomingMode", lResult, p_rlErrorCode);
                    }
                    else
                    {
                        if(VCS_FindHome(p_DeviceHandle, g_HomingSecondId[i], HM_ACTUAL_POSITION, &p_rlErrorCode) == 0)
                        {
                            lResult = MMC_FAILED;
                            LogError("VCS_FindHome", lResult, p_rlErrorCode);
                        }
                        else
                        {
                            VCS_WaitForHomingAttained(p_DeviceHandle, g_HomingSecondId[i], 15000, &p_rlErrorCode);
                            if(VCS_GetHomingParameter(p_DeviceHandle, g_HomingSecondId[i], &g_HomingAcceleration, &l_SpeedSwitch, &l_SpeedIndex, &l_HomeOffset, &g_CurrentThreshold,
                                                      &g_HomePosition, &p_rlErrorCode) == 0)
                            {
                                lResult = MMC_FAILED;
                                LogError("VCS_GetHomingParameter", lResult, p_rlErrorCode);
                            }
                            else
                            {
                                stringstream msg;
                                msg << "Homing Parameters, node = " << g_HomingFirstId[i] << endl;
                                msg << "HomingAcceleration  = " << g_HomingAcceleration << endl;
                                msg << "SpeedSwitch         = " << l_SpeedSwitch << endl;
                                msg << "SpeedIndex          = " << l_SpeedIndex << endl;
                                msg << "HomeOffset          = " << l_HomeOffset << endl;
                                msg << "CurrentThreshold    = " << g_interfaceName << endl;
                                msg << "HomePosition        = " << g_HomePosition;
                                LogInfo(msg.str());
                                SeparatorLine();
                            }
                        }
                    }
                }
            }
        }
    }

    return lResult;
}

int PositionControl(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode, long PosDesired)
{
    int lResult = MMC_SUCCESS;
    stringstream msg;

    msg << "set profile position mode, node = " << p_usNodeId;
    LogInfo(msg.str());

    if(VCS_ActivateProfilePositionMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
    {
        lResult = MMC_FAILED;
        LogError("VCS_ActivateProfilePositionMode", lResult, p_rlErrorCode);
    }
    else
    {
        stringstream msg;
        msg << "move to position = " << PosDesired << ", node = " << p_usNodeId;
        LogInfo(msg.str());

        if(VCS_MoveToPosition(p_DeviceHandle, p_usNodeId, PosDesired, 1, 1, &p_rlErrorCode) == 0)
        {
            lResult = MMC_FAILED;
            LogError("VCS_MoveToPosition", lResult, p_rlErrorCode);
        }
        else
        {
            VCS_WaitForTargetReached(p_DeviceHandle, p_usNodeId, 30000, &p_rlErrorCode);
        }

        if(VCS_GetQuickStopState(p_DeviceHandle, p_usNodeId, &g_IsQuickStopped, &p_rlErrorCode) == 0)
        {
            lResult = MMC_FAILED;
            LogError("VCS_GetQuickStopState", lResult, p_rlErrorCode);
        }
        else
        {
            stringstream msg;
            if(g_IsQuickStopped == 1)
            {
                msg << "Motor is QuickStopped, node = " << p_usNodeId;
                LogInfo(msg.str());

                if(VCS_SetObject(p_DeviceHandle, p_usNodeId, g_digitalIOIndex, g_digitalIO_4, &g_None, g_NbofBytesToWrite_1, &g_NbofBytesToWrite_1, &p_rlErrorCode) == 0)
                {
                    lResult = MMC_FAILED;
                    LogError("VCS_SetObject", lResult, p_rlErrorCode);
                }
                else
                {
                    if(VCS_GetPositionIs(p_DeviceHandle, p_usNodeId, &g_CapturedPosition, &p_rlErrorCode) == 0)
                    {
                        lResult = MMC_FAILED;
                        LogError("VCS_GetPositionIs", lResult, p_rlErrorCode);
                    }
                    else
                    {
                        if(VCS_ActivateProfilePositionMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
                        {
                            LogError("VCS_ActivateProfilePositionMode", lResult, p_rlErrorCode);
                            lResult = MMC_FAILED;
                        }
                        else
                        {
                            if(g_CapturedPosition>0)
                            {
                                if (VCS_MoveToPosition(p_DeviceHandle, p_usNodeId, -30000, 0, 1, &p_rlErrorCode) == 0)
                                {
                                    LogError("VCS_MoveToPosition", lResult, p_rlErrorCode);
                                    lResult = MMC_FAILED;
                                }
                                else
                                {
                                    VCS_WaitForTargetReached(p_DeviceHandle, p_usNodeId, 30000, &p_rlErrorCode);
                                    if(VCS_SetObject(p_DeviceHandle, p_usNodeId, g_digitalIOIndex, g_digitalIO_4, &g_QuickStop, g_NbofBytesToWrite_1, &g_NbofBytesToWrite_1, &p_rlErrorCode) == 0)
                                    {
                                        LogError("VCS_SetObject", lResult, p_rlErrorCode);
                                        lResult = MMC_FAILED;
                                    }
                                }
                            }
                            else
                            {
                                if (VCS_MoveToPosition(p_DeviceHandle, p_usNodeId, 30000, 0, 1, &p_rlErrorCode) == 0)
                                {
                                    LogError("VCS_MoveToPosition", lResult, p_rlErrorCode);
                                    lResult = MMC_FAILED;
                                }
                                else
                                {
                                    VCS_WaitForTargetReached(p_DeviceHandle, p_usNodeId, 30000, &p_rlErrorCode);
                                    if(VCS_SetObject(p_DeviceHandle, p_usNodeId, g_digitalIOIndex, g_digitalIO_4, &g_QuickStop, g_NbofBytesToWrite_1, &g_NbofBytesToWrite_1, &p_rlErrorCode) == 0)
                                    {
                                        LogError("VCS_SetObject", lResult, p_rlErrorCode);
                                        lResult = MMC_FAILED;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        if(lResult == MMC_SUCCESS)
        {
            LogInfo("halt position movement");

            if(VCS_HaltPositionMovement(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
            {
                LogError("VCS_HaltPositionMovement", lResult, p_rlErrorCode);
                lResult = MMC_FAILED;
            }
        }
    }

    return lResult;
}

int DemoProfilePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
{
    int lResult = MMC_SUCCESS;
    stringstream msg;

    msg << "set profile position mode, node = " << p_usNodeId;
    LogInfo(msg.str());

    if(VCS_ActivateProfilePositionMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
    {
        LogError("VCS_ActivateProfilePositionMode", lResult, p_rlErrorCode);
        lResult = MMC_FAILED;
    }
    else
    {
        list<long> positionList;

        positionList.push_back(5000);
        positionList.push_back(-10000);
        positionList.push_back(5000);

        for(list<long>::iterator it = positionList.begin(); it !=positionList.end(); it++)
        {
            long targetPosition = (*it);
            stringstream msg;
            msg << "move to position = " << targetPosition << ", node = " << p_usNodeId;
            LogInfo(msg.str());

            if(VCS_MoveToPosition(p_DeviceHandle, p_usNodeId, targetPosition, 0, 1, &p_rlErrorCode) == 0)
            {
                LogError("VCS_MoveToPosition", lResult, p_rlErrorCode);
                lResult = MMC_FAILED;
                break;
            }

            sleep(1);
        }

        if(lResult == MMC_SUCCESS)
        {
            LogInfo("halt position movement");

            if(VCS_HaltPositionMovement(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
            {
                LogError("VCS_HaltPositionMovement", lResult, p_rlErrorCode);
                lResult = MMC_FAILED;
            }
        }
    }

    return lResult;
}

bool DemoProfileVelocityMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
{
    int lResult = MMC_SUCCESS;
    stringstream msg;

    msg << "set profile velocity mode, node = " << p_usNodeId;

    LogInfo(msg.str());

    if(VCS_ActivateProfileVelocityMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
    {
        LogError("VCS_ActivateProfileVelocityMode", lResult, p_rlErrorCode);
        lResult = MMC_FAILED;
    }
    else
    {
        list<long> velocityList;

        velocityList.push_back(100);
        velocityList.push_back(500);
        velocityList.push_back(1000);

        for(list<long>::iterator it = velocityList.begin(); it !=velocityList.end(); it++)
        {
            long targetvelocity = (*it);

            stringstream msg;
            msg << "move with target velocity = " << targetvelocity << " rpm, node = " << p_usNodeId;
            LogInfo(msg.str());

            if(VCS_MoveWithVelocity(p_DeviceHandle, p_usNodeId, targetvelocity, &p_rlErrorCode) == 0)
            {
                lResult = MMC_FAILED;
                LogError("VCS_MoveWithVelocity", lResult, p_rlErrorCode);
                break;
            }

            sleep(10);
        }

        if(lResult == MMC_SUCCESS)
        {
            LogInfo("halt velocity movement");

            if(VCS_HaltVelocityMovement(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
            {
                lResult = MMC_FAILED;
                LogError("VCS_HaltVelocityMovement", lResult, p_rlErrorCode);
            }
        }
    }

    return lResult;
}

int PrepareDemo(unsigned int* p_pErrorCode)
{
    int lResult = MMC_SUCCESS;
    BOOL oIsFault = 0;

    if(VCS_GetFaultState(g_pKeyHandle, g_usNodeId, &oIsFault, p_pErrorCode ) == 0)
    {
        LogError("VCS_GetFaultState", lResult, *p_pErrorCode);
        lResult = MMC_FAILED;
    }

    if(lResult==0)
    {
        if(oIsFault)
        {
            stringstream msg;
            msg << "clear fault, node = '" << g_usNodeId << "'";
            LogInfo(msg.str());

            if(VCS_ClearFault(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0)
            {
                LogError("VCS_ClearFault", lResult, *p_pErrorCode);
                lResult = MMC_FAILED;
            }
        }

        if(lResult==0)
        {
            BOOL oIsEnabled = 0;

            if(VCS_GetEnableState(g_pKeyHandle, g_usNodeId, &oIsEnabled, p_pErrorCode) == 0)
            {
                LogError("VCS_GetEnableState", lResult, *p_pErrorCode);
                lResult = MMC_FAILED;
            }

            if(lResult==0)
            {
                if(!oIsEnabled)
                {
                    if(VCS_SetEnableState(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0)
                    {
                        LogError("VCS_SetEnableState", lResult, *p_pErrorCode);
                        lResult = MMC_FAILED;
                    }
                }
            }
        }
    }
    return lResult;
}

int MaxFollowingErrorDemo(unsigned int& p_rlErrorCode)
{
    int lResult = MMC_SUCCESS;
    const unsigned int EXPECTED_ERROR_CODE = 0x8611;
    unsigned int lDeviceErrorCode = 0;

    lResult = VCS_ActivateProfilePositionMode(g_pKeyHandle, g_usNodeId, &p_rlErrorCode);

    if(lResult)
    {
        lResult = VCS_SetMaxFollowingError(g_pKeyHandle, g_usNodeId, 1, &p_rlErrorCode);
    }

    if(lResult)
    {
        lResult = VCS_MoveToPosition(g_pKeyHandle, g_usNodeId, 1000, 1, 1, &p_rlErrorCode);
    }

    if(lResult)
    {
        lResult = VCS_GetDeviceErrorCode(g_pKeyHandle, g_usNodeId, 1, &lDeviceErrorCode, &p_rlErrorCode);
    }

    if(lResult)
    {
        lResult = lDeviceErrorCode == EXPECTED_ERROR_CODE ? MMC_SUCCESS : MMC_FAILED;
    }

    return lResult;
}

int Demo(unsigned int* p_pErrorCode)
{
    int lResult = MMC_SUCCESS;
    unsigned int lErrorCode = 0;

    lResult = DemoProfileVelocityMode(g_pKeyHandle, g_usNodeId, lErrorCode);

    if(lResult != MMC_SUCCESS)
    {
        LogError("DemoProfileVelocityMode", lResult, lErrorCode);
    }
    else
    {
        lResult = DemoProfilePositionMode(g_pKeyHandle, g_usNodeId, lErrorCode);

        if(lResult != MMC_SUCCESS)
        {
            LogError("DemoProfilePositionMode", lResult, lErrorCode);
        }
        else
        {
            if(VCS_SetDisableState(g_pKeyHandle, g_usNodeId, &lErrorCode) == 0)
            {
                LogError("VCS_SetDisableState", lResult, lErrorCode);
                lResult = MMC_FAILED;
            }
        }
    }

    return lResult;
}

int DemoSafetyPos(unsigned int* p_pErrorCode, long PosDesired)
{
    int lResult = MMC_SUCCESS;
    unsigned int lErrorCode = 0;

    if(VCS_GetHomingParameter(g_pKeyHandle, g_usNodeId, &g_HomingAcceleration, &g_SpeedSwitch, &g_SpeedIndex, &g_HomeOffset, &g_CurrentThreshold,
                              &g_HomePosition, &lErrorCode) == 0)
    {
        lResult = MMC_FAILED;
        LogError("VCS_GetHomingParameter", lResult, lErrorCode);
    }
    else
    {
        stringstream msg;
        msg << "Homing Parameters, node = " << g_usNodeId << endl;
        msg << "HomingAcceleration  = " << g_HomingAcceleration << endl;
        msg << "SpeedSwitch         = " << g_SpeedSwitch << endl;
        msg << "SpeedIndex          = " << g_SpeedIndex << endl;
        msg << "HomeOffset          = " << g_HomeOffset << endl;
        msg << "CurrentThreshold    = " << g_interfaceName << endl;
        msg << "HomePosition        = " << g_HomePosition;
        LogInfo(msg.str());
        SeparatorLine();
    }

    lResult = PositionControl(g_pKeyHandle, g_usNodeId, lErrorCode, PosDesired);

    if(lResult != MMC_SUCCESS)
    {
        LogError("PositionControl", lResult, lErrorCode);
    }

    return lResult;
}

int Home(unsigned int* p_pErrorCode)
{
    int lResult = MMC_SUCCESS;
    unsigned int lErrorCode = 0;
//    lResult = HomingMode(g_pKeyHandle, g_usNodeId, lErrorCode);
//    lResult = BothHomingMode(g_pKeyHandle, g_usNodeId, lErrorCode);
    lResult = SyncHomingMode(g_pKeyHandle, g_usNodeId, lErrorCode);

    if(lResult != MMC_SUCCESS)
    {
        LogError("HomingMode", lResult, lErrorCode);
    }

    return lResult;
}

void PrintHeader()
{
    SeparatorLine();

    LogInfo("Epos Command Library Example Program, (c) maxonmotor ag 2014-2017");

    SeparatorLine();
}

int PrintAvailablePorts(char* p_pInterfaceNameSel)
{
    int lResult = MMC_FAILED;
    int lStartOfSelection = 1;
    int lMaxStrSize = 255;
    char* pPortNameSel = new char[lMaxStrSize];
    int lEndOfSelection = 0;
    unsigned int ulErrorCode = 0;

    do
    {
        if(!VCS_GetPortNameSelection((char*)g_deviceName.c_str(), (char*)g_protocolStackName.c_str(), p_pInterfaceNameSel, lStartOfSelection, pPortNameSel, lMaxStrSize, &lEndOfSelection, &ulErrorCode))
        {
            lResult = MMC_FAILED;
            LogError("GetPortNameSelection", lResult, ulErrorCode);
            break;
        }
        else
        {
            lResult = MMC_SUCCESS;
            printf("            port = %s\n", pPortNameSel);
        }

        lStartOfSelection = 0;
    }
    while(lEndOfSelection == 0);

    return lResult;
}

int PrintAvailableInterfaces()
{
    int lResult = MMC_FAILED;
    int lStartOfSelection = 1;
    int lMaxStrSize = 255;
    char* pInterfaceNameSel = new char[lMaxStrSize];
    int lEndOfSelection = 0;
    unsigned int ulErrorCode = 0;

    do
    {
        if(!VCS_GetInterfaceNameSelection((char*)g_deviceName.c_str(), (char*)g_protocolStackName.c_str(), lStartOfSelection, pInterfaceNameSel, lMaxStrSize, &lEndOfSelection, &ulErrorCode))
        {
            lResult = MMC_FAILED;
            LogError("GetInterfaceNameSelection", lResult, ulErrorCode);
            break;
        }
        else
        {
            lResult = MMC_SUCCESS;

            printf("interface = %s\n", pInterfaceNameSel);

            PrintAvailablePorts(pInterfaceNameSel);
        }

        lStartOfSelection = 0;
    }
    while(lEndOfSelection == 0);

    SeparatorLine();

    delete[] pInterfaceNameSel;

    return lResult;
}

int PrintDeviceVersion()
{
    int lResult = MMC_FAILED;
    unsigned short usHardwareVersion = 0;
    unsigned short usSoftwareVersion = 0;
    unsigned short usApplicationNumber = 0;
    unsigned short usApplicationVersion = 0;
    unsigned int ulErrorCode = 0;

    if(VCS_GetVersion(g_pKeyHandle, g_usNodeId, &usHardwareVersion, &usSoftwareVersion, &usApplicationNumber, &usApplicationVersion, &ulErrorCode))
    {
        printf("%s Hardware Version    = 0x%04x\n      Software Version    = 0x%04x\n      Application Number  = 0x%04x\n      Application Version = 0x%04x\n",
               g_deviceName.c_str(), usHardwareVersion, usSoftwareVersion, usApplicationNumber, usApplicationVersion);
        lResult = MMC_SUCCESS;
    }

    return lResult;
}

int PrintAvailableProtocols()
{
    int lResult = MMC_FAILED;
    int lStartOfSelection = 1;
    int lMaxStrSize = 255;
    char* pProtocolNameSel = new char[lMaxStrSize];
    int lEndOfSelection = 0;
    unsigned int ulErrorCode = 0;

    do
    {
        if(!VCS_GetProtocolStackNameSelection((char*)g_deviceName.c_str(), lStartOfSelection, pProtocolNameSel, lMaxStrSize, &lEndOfSelection, &ulErrorCode))
        {
            lResult = MMC_FAILED;
            LogError("GetProtocolStackNameSelection", lResult, ulErrorCode);
            break;
        }
        else
        {
            lResult = MMC_SUCCESS;

            printf("protocol stack name = %s\n", pProtocolNameSel);
        }

        lStartOfSelection = 0;
    }
    while(lEndOfSelection == 0);

    SeparatorLine();

    delete[] pProtocolNameSel;

    return lResult;
}

int main(int argc, char** argv)
{
    int lResult = MMC_FAILED;
    unsigned int ulErrorCode = 0;

    PrintHeader();

    SetDefaultParameters();

    if((lResult = ParseArguments(argc, argv))!=MMC_SUCCESS)
    {
        return lResult;
    }

    PrintSettings();

    switch(g_eAppMode)
    {
        case AM_DEMO:
        {
            if((lResult = OpenDevice(&ulErrorCode))!=MMC_SUCCESS)
            {
                LogError("OpenDevice", lResult, ulErrorCode);
                return lResult;
            }

            if((lResult = PrepareDemo(&ulErrorCode))!=MMC_SUCCESS)
            {
                LogError("PrepareDemo", lResult, ulErrorCode);
                return lResult;
            }

            if((lResult = Demo(&ulErrorCode))!=MMC_SUCCESS)
            {
                LogError("Demo", lResult, ulErrorCode);
                return lResult;
            }

            if((lResult = CloseDevice(&ulErrorCode))!=MMC_SUCCESS)
            {
                LogError("CloseDevice", lResult, ulErrorCode);
                return lResult;
            }
        } break;
        case AM_HOMING:
        {
            if((lResult = OpenDevice(&ulErrorCode))!=MMC_SUCCESS)
            {
                LogError("OpenDevice", lResult, ulErrorCode);
                return lResult;
            }

            if((lResult = PrepareDemo(&ulErrorCode))!=MMC_SUCCESS)
            {
                LogError("PrepareDemo", lResult, ulErrorCode);
                return lResult;
            }

            if((lResult = Home(&ulErrorCode))!=MMC_SUCCESS)
            {
                LogError("Home", lResult, ulErrorCode);
                return lResult;
            }

            if((lResult = CloseDevice(&ulErrorCode))!=MMC_SUCCESS)
            {
                LogError("CloseDevice", lResult, ulErrorCode);
                return lResult;
            }
        } break;
        case AM_POSITION_CON:
        {
            if((lResult = OpenDevice(&ulErrorCode))!=MMC_SUCCESS)
            {
                LogError("OpenDevice", lResult, ulErrorCode);
                return lResult;
            }

            if((lResult = PrepareDemo(&ulErrorCode))!=MMC_SUCCESS)
            {
                LogError("PrepareDemo", lResult, ulErrorCode);
                return lResult;
            }

            if((lResult = DemoSafetyPos(&ulErrorCode, g_PosDesired))!=MMC_SUCCESS)
            {
                LogError("DemoSafetyPos", lResult, ulErrorCode);
                return lResult;
            }

            if((lResult = CloseDevice(&ulErrorCode))!=MMC_SUCCESS)
            {
                LogError("CloseDevice", lResult, ulErrorCode);
                return lResult;
            }
        } break;
        case AM_INTERFACE_LIST:
            PrintAvailableInterfaces();
            break;
        case AM_PROTOCOL_LIST:
            PrintAvailableProtocols();
            break;
        case AM_VERSION_INFO:
        {
            if((lResult = OpenDevice(&ulErrorCode))!=MMC_SUCCESS)
            {
                LogError("OpenDevice", lResult, ulErrorCode);
                return lResult;
            }

            if((lResult = PrintDeviceVersion()) != MMC_SUCCESS)
            {
                LogError("PrintDeviceVersion", lResult, ulErrorCode);
                return lResult;
            }

            if((lResult = CloseDevice(&ulErrorCode))!=MMC_SUCCESS)
            {
                LogError("CloseDevice", lResult, ulErrorCode);
                return lResult;
            }
        } break;
        case AM_UNKNOWN:
            printf("unknown option\n");
            break;
    }

    return lResult;
}
