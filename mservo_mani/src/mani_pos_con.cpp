//
// Created by jeon on 18. 10. 22.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "Definitions.h"
#include "sensor_msgs/JointState.h"
#include <sstream>
#define _USE_MATH_DEFINES
#include <math.h>

#ifndef MMC_SUCCESS
#define MMC_SUCCESS 0
#endif

#ifndef MMC_FAILED
#define MMC_FAILED 1
#endif

#ifndef MMC_MAX_LOG_MSG_SIZE
#define MMC_MAX_LOG_MSG_SIZE 512
#endif

typedef void* HANDLE;
typedef int BOOL;

using namespace std;

// Device Parameters
void* g_pKeyHandle = 0;
unsigned short g_usNodeId = 1;
string g_deviceName;
string g_protocolStackName;
string g_interfaceName;
string g_portName;
int g_baudrate = 0;
const string g_programName = "ControlCmd";

// Manipulator Joint Parameters
unsigned short g_usNodeId_arr[] = {1, 2, 3, 4, 5, 6};
int g_NbofJoint = sizeof(g_usNodeId_arr)/ sizeof(*g_usNodeId_arr);
int g_GearRatio[] = {160, 160, 160, 120, 100, 100, 100};
double g_AxisVelRatio[] = {1.6, 1.6, 1.6, 1.2, 1, 1, 1};
int g_PulseRev[] = {4096, 4096, 4096, 4096, 2048, 2048, 2048};
int g_HomeOffset_arr[] = {72818, 72818, 72818, 54613, 22378, 22378, 22378};
long g_SafeRange[] = {2148124, 1121393, 1856853, 1004885, 575715, 423253, 637156};
int g_PosLimitMin[] = {-5000000, -5000000, -5000000, -5000000, -5000000, -5000000, -5000000};
int g_PosLimitMax[] = {5000000, 5000000, 5000000, 5000000, 5000000, 5000000, 5000000};

// Position Control Parameters
long g_PosDesired;
unsigned int g_ProfileVelocity = 1000;
unsigned int l_ProfileVelocity;
long g_PosCenter[] = {0, 0, 0, 0, 0, 0, 0};
long g_PosPositiveLimit[] = {0, 0, 0, 0, 0, 0, 0};
int g_CapturedPosition;

// Homing Parameters
unsigned  short g_HomingFirstId[] = {2, 4, 6};
unsigned  short g_HomingSecondId[] = {1, 3, 5};
unsigned int g_HomingAcceleration = 1000;
unsigned int g_SpeedSwitch = 200;
unsigned int g_SpeedIndex = 10;
unsigned int l_SpeedSwitch;
unsigned int l_SpeedIndex;
int l_HomeOffset;
unsigned short g_CurrentThreshold;
int g_HomePosition = 0;
int g_IsQuickStopped;

// Digital Input/Output Pin Configuration Parameters
unsigned short Negative_LowActive = 0x0001;
unsigned short Negative_HighActive = 0x0000;


// Functions
void  LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode);
void  LogInfo(string message);
void  PrintSettings();
void  SetDefaultParameters();
void  SeperatorLine();

int   OpenDevice(unsigned int* p_pErrorCode);
int   CloseDevice(unsigned int* p_pErrorCode);
int   PrepareDriver(unsigned int* p_pErrorCode);
int   SyncHomingProcess(HANDLE p_DeviceHandle, unsigned short NodeIDs[], unsigned int & p_rlErrorCode);
int   SyncHomingMode(HANDLE p_DeviceHandle, unsigned int & p_rlErrorCode);
int   PreparePosCon(HANDLE p_DeviceHandle, unsigned int & p_rlErrorCode);
int   PositionControl(HANDLE p_DeviceHandle, unsigned int & p_rlErrorCode);


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
    g_deviceName = "EPOS4";
    g_protocolStackName = "MAXON SERIAL V2";
    g_interfaceName = "USB";
    g_portName = "USB0";
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

int PrepareDriver(unsigned int* p_pErrorCode)
{
    int lResult = MMC_SUCCESS;
    BOOL oIsFault = 0;

    for (int i=0; i<g_NbofJoint; i++)
    {
        if(VCS_GetFaultState(g_pKeyHandle, g_usNodeId_arr[i], &oIsFault, p_pErrorCode ) == 0)
        {
            LogError("VCS_GetFaultState", lResult, *p_pErrorCode);
            lResult = MMC_FAILED;
        }

        if(lResult==0)
        {
            if(oIsFault)
            {
                stringstream msg;
                msg << "clear fault, node = '" << g_usNodeId_arr[i] << "'";
                LogInfo(msg.str());

                if(VCS_ClearFault(g_pKeyHandle, g_usNodeId_arr[i], p_pErrorCode) == 0)
                {
                    LogError("VCS_ClearFault", lResult, *p_pErrorCode);
                    lResult = MMC_FAILED;
                }
            }

            if(lResult==0)
            {
                BOOL oIsEnabled = 0;

                if(VCS_GetEnableState(g_pKeyHandle, g_usNodeId_arr[i], &oIsEnabled, p_pErrorCode) == 0)
                {
                    LogError("VCS_GetEnableState", lResult, *p_pErrorCode);
                    lResult = MMC_FAILED;
                }

                if(lResult==0)
                {
                    if(!oIsEnabled)
                    {
                        if(VCS_SetEnableState(g_pKeyHandle, g_usNodeId_arr[i], p_pErrorCode) == 0)
                        {
                            LogError("VCS_SetEnableState", lResult, *p_pErrorCode);
                            lResult = MMC_FAILED;
                        }
                    }
                }
            }
        }
    }


    return lResult;
}

int SyncHomingProcess(HANDLE p_DeviceHandle, unsigned short NodeIDs[], unsigned int & p_rlErrorCode)
{
    int Nbof_IDs = sizeof(NodeIDs)/ sizeof(*NodeIDs);
    int lResult = MMC_SUCCESS;

    // Find Negative limit of NodeIDs
    for(int i=0; i<Nbof_IDs; i++)
    {
        if(lResult == MMC_SUCCESS)
        {
            if(VCS_FindHome(p_DeviceHandle, NodeIDs[i], HM_NEGATIVE_LIMIT_SWITCH_AND_INDEX, &p_rlErrorCode) == 0)
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
        msg << "Negative Limit is set, node = ";
        for (int i=0; i<Nbof_IDs; i++)
        {
            VCS_WaitForHomingAttained(p_DeviceHandle, NodeIDs[i], 100000, &p_rlErrorCode);
            msg << NodeIDs[i] << ", ";
        }
        LogInfo(msg.str());
    }

    // Ready for finding Positive Limit
    if(lResult == MMC_SUCCESS)
    {
        for (int i=0; i<Nbof_IDs; i++)
        {
            if(VCS_SetObject(p_DeviceHandle, NodeIDs[i], INDEX_DIO_CONFIGURATION, SUB_INDEX_DIO_CONFIGURATION_1, &QUICK_STOP, NB_OF_BYTES_TO_WRITE_1, &NB_OF_BYTES_TO_WRITE_1, &p_rlErrorCode)== 0)
            {
                lResult = MMC_FAILED;
                LogError("VCS_SetObject", lResult, p_rlErrorCode);
            }
        }
    }

    // Approach to Positive Limit Switch Aggressively
    for (int i=0; i<Nbof_IDs; i--)
    {
        if(lResult == MMC_SUCCESS)
        {
            if(VCS_ActivateProfilePositionMode(p_DeviceHandle, NodeIDs[i], &p_rlErrorCode) == 0)
            {
                lResult = MMC_FAILED;
                LogError("VCS_ActivateProfilePositionMode", lResult, p_rlErrorCode);
            }
        }
    }
    for (int i=0; i<Nbof_IDs; i--)
    {
        if(lResult == MMC_SUCCESS)
        {
            if(VCS_MoveToPosition(p_DeviceHandle, NodeIDs[i], g_SafeRange[NodeIDs[i]-1], 1, 1, &p_rlErrorCode) == 0)
            {
                lResult = MMC_FAILED;
                LogError("VCS_MoveToPosition", lResult, p_rlErrorCode);
            }
        }
    }
    if(lResult == MMC_SUCCESS)
    {
        for (int i=Nbof_IDs; i>0; i--)
        {
            VCS_WaitForTargetReached(p_DeviceHandle, NodeIDs[i-1], 100000, &p_rlErrorCode);
        }
    }

    // Find Positive limit Switch of Joint
    if(lResult == MMC_SUCCESS)
    {
        for (int i=0; i<Nbof_IDs; i++)
        {
            if(VCS_ActivateProfileVelocityMode(p_DeviceHandle, NodeIDs[i], &p_rlErrorCode) == 0)
            {
                lResult = MMC_FAILED;
                LogError("VCS_ActivateProfileVelocityMode", lResult, p_rlErrorCode);
            }
        }
    }
    for (int i=0; i<Nbof_IDs; i--)
    {
        if(lResult == MMC_SUCCESS)
        {
            if(VCS_MoveWithVelocity(p_DeviceHandle, NodeIDs[i], (long)(g_SpeedSwitch*g_AxisVelRatio[NodeIDs[i]-1]), &p_rlErrorCode) == 0)
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
        for (int i=0; i<Nbof_IDs; i++)
        {
            do
            {
                if(VCS_GetQuickStopState(p_DeviceHandle, NodeIDs[i], &g_IsQuickStopped, &p_rlErrorCode) == 0)
                {
                    lResult = MMC_FAILED;
                    LogError("VCS_GetQuickStopState", lResult, p_rlErrorCode);
                }
            }
            while (g_IsQuickStopped == 0);
        }
    }

    // Find Positive Homing Index
    for (int i=0; i<Nbof_IDs; i++)
    {
        if(lResult == MMC_SUCCESS)
        {
            if(VCS_SetObject(p_DeviceHandle, NodeIDs[i], INDEX_DIO_CONFIGURATION, SUB_INDEX_DIO_CONFIGURATION_1, &QUICK_STOP, NB_OF_BYTES_TO_WRITE_1, &NB_OF_BYTES_TO_WRITE_1, &p_rlErrorCode) &&
               VCS_SetObject(p_DeviceHandle, NodeIDs[i], INDEX_DI_PROPERTIES, SUB_INDEX_DI_POLARITY, &Negative_LowActive, NB_OF_BYTES_TO_WRITE_2, &NB_OF_BYTES_TO_WRITE_2, &p_rlErrorCode)== 0)
            {
                lResult = MMC_FAILED;
                LogError("VCS_SetObject", lResult, p_rlErrorCode);
            }
        }
    }
    for (int i=0; i<Nbof_IDs; i--)
    {
        if(lResult == MMC_SUCCESS)
        {
            if(VCS_MoveWithVelocity(p_DeviceHandle, NodeIDs[i], -(long)(g_SpeedIndex*g_AxisVelRatio[NodeIDs[i]-1]), &p_rlErrorCode) == 0)
            {
                lResult = MMC_FAILED;
                LogError("VCS_MoveWithVelocity", lResult, p_rlErrorCode);
            }
        }
    }
    // Waiting for Positive Homing Index
    if(lResult == MMC_SUCCESS)
    {
        stringstream msg;
        msg << "Positive Limit is set, node = ";
        for (int i=0; i<Nbof_IDs; i++)
        {
            do
            {
                if(VCS_GetQuickStopState(p_DeviceHandle, NodeIDs[i], &g_IsQuickStopped, &p_rlErrorCode) == 0)
                {
                    lResult = MMC_FAILED;
                    LogError("VCS_GetQuickStopState", lResult, p_rlErrorCode);
                }
                else
                {
                    if(VCS_GetPositionIs(p_DeviceHandle, NodeIDs[i], &g_CapturedPosition, &p_rlErrorCode) == 0)
                    {
                        lResult = MMC_FAILED;
                        LogError("VCS_GetPositionIs", lResult, p_rlErrorCode);
                    }
                }
            }
            while (g_IsQuickStopped == 0);
            stringstream msg;
            g_PosPositiveLimit[NodeIDs[i]-1] = (long)(g_CapturedPosition - g_HomeOffset_arr[NodeIDs[i]-1]);
            g_PosCenter[NodeIDs[i]-1] = (long)(g_PosPositiveLimit[NodeIDs[i]-1]/2);
            g_PosLimitMin[NodeIDs[i]-1] = -g_PosCenter[NodeIDs[i]-1] - 5000;
            g_PosLimitMax[NodeIDs[i]-1] = g_PosCenter[NodeIDs[i]-1] + 5000;
            msg << NodeIDs[i] << ", ";
        }
        LogInfo(msg.str());
        SeparatorLine();
    }

    // Set Limit Switch as None
    if(lResult == MMC_SUCCESS)
    {
        for (int i=0; i<Nbof_IDs; i++)
        {
            if(VCS_SetObject(p_DeviceHandle, NodeIDs[i], INDEX_DI_PROPERTIES, SUB_INDEX_DI_POLARITY, &Negative_HighActive, NB_OF_BYTES_TO_WRITE_2, &NB_OF_BYTES_TO_WRITE_2, &p_rlErrorCode) &&
               VCS_SetObject(p_DeviceHandle, NodeIDs[i], INDEX_DIO_CONFIGURATION, SUB_INDEX_DIO_CONFIGURATION_1, &NONE, NB_OF_BYTES_TO_WRITE_1, &NB_OF_BYTES_TO_WRITE_1, &p_rlErrorCode)== 0)
            {
                lResult = MMC_FAILED;
                LogError("VCS_SetObject", lResult, p_rlErrorCode);
            }
            else
            {
                if(VCS_ActivateProfilePositionMode(p_DeviceHandle, NodeIDs[i], &p_rlErrorCode) == 0)
                {
                    lResult = MMC_FAILED;
                    LogError("VCS_ActivateProfilePositionMode", lResult, p_rlErrorCode);
                }
            }
        }
    }

    // Move to Center Position
    for (int i=0; i<Nbof_IDs; i--)
    {
        if(lResult == MMC_SUCCESS)
        {
            if(VCS_MoveToPosition(p_DeviceHandle, NodeIDs[i], g_PosCenter[NodeIDs[i]-1], 1, 1, &p_rlErrorCode) == 0)
            {
                lResult = MMC_FAILED;
                LogError("VCS_MoveToPosition", lResult, p_rlErrorCode);
            }
        }
    }
    if(lResult == MMC_SUCCESS)
    {
        for (int i=Nbof_IDs; i>0; i--)
        {
            VCS_WaitForTargetReached(p_DeviceHandle, NodeIDs[i-1], 100000, &p_rlErrorCode);
        }
    }

    // Set Center position as Home
    for (int i=0; i<Nbof_IDs; i++)
    {
        if(lResult == MMC_SUCCESS)
        {
            if(VCS_SetObject(p_DeviceHandle, NodeIDs[i], INDEX_DIO_CONFIGURATION, SUB_INDEX_DIO_CONFIGURATION_1, &NEGATIVE_LIMIT_SWITCH, NB_OF_BYTES_TO_WRITE_1, &NB_OF_BYTES_TO_WRITE_1, &p_rlErrorCode) &&
               VCS_SetObject(p_DeviceHandle, NodeIDs[i], INDEX_DIO_CONFIGURATION, SUB_INDEX_DIO_CONFIGURATION_4, &QUICK_STOP, NB_OF_BYTES_TO_WRITE_1, &NB_OF_BYTES_TO_WRITE_1, &p_rlErrorCode) == 0)
            {
                lResult = MMC_FAILED;
                LogError("VCS_SetObject", lResult, p_rlErrorCode);
            }
            else
            {
                if(VCS_SetHomingParameter(p_DeviceHandle, NodeIDs[i], g_HomingAcceleration, (unsigned int)(g_SpeedSwitch*g_AxisVelRatio[NodeIDs[i]-1]),
                                          (unsigned int)(g_SpeedIndex*g_AxisVelRatio[NodeIDs[i]-1]), 0, g_CurrentThreshold, g_HomePosition, &p_rlErrorCode) == 0)
                {
                    lResult = MMC_FAILED;
                    LogError("VCS_SetHomingParameter", lResult, p_rlErrorCode);
                }
                else
                {
                    if(VCS_ActivateHomingMode(p_DeviceHandle, NodeIDs[i], &p_rlErrorCode) == 0)
                    {
                        lResult = MMC_FAILED;
                        LogError("VCS_ActivateHomingMode", lResult, p_rlErrorCode);
                    }
                    else
                    {
                        if(VCS_FindHome(p_DeviceHandle, NodeIDs[i], HM_ACTUAL_POSITION, &p_rlErrorCode) == 0)
                        {
                            lResult = MMC_FAILED;
                            LogError("VCS_FindHome", lResult, p_rlErrorCode);
                        }
                        else
                        {
                            VCS_WaitForHomingAttained(p_DeviceHandle, NodeIDs[i], 15000, &p_rlErrorCode);
                            if(VCS_GetHomingParameter(p_DeviceHandle, NodeIDs[i], &g_HomingAcceleration, &l_SpeedSwitch, &l_SpeedIndex, &l_HomeOffset, &g_CurrentThreshold,
                                                      &g_HomePosition, &p_rlErrorCode) == 0)
                            {
                                lResult = MMC_FAILED;
                                LogError("VCS_GetHomingParameter", lResult, p_rlErrorCode);
                            }
                            else
                            {
                                stringstream msg;
                                msg << "Homing Parameters, node = " << NodeIDs[i] << endl;
                                msg << "HomingAcceleration  = " << g_HomingAcceleration << endl;
                                msg << "SpeedSwitch         = " << l_SpeedSwitch << endl;
                                msg << "SpeedIndex          = " << l_SpeedIndex << endl;
                                msg << "HomeOffset          = " << l_HomeOffset << endl;
                                msg << "CurrentThreshold    = " << g_interfaceName << endl;
                                msg << "Range of Motion     = " << "+-" << g_PosCenter[NodeIDs[i]-1] << endl;
                                msg << "HomePosition        = " << g_HomePosition;
                                LogInfo(msg.str());
                                SeparatorLine();
                                if(VCS_SetObject(p_DeviceHandle, NodeIDs[i], INDEX_SOFT_POS_LIM, SUB_INDEX_SOFT_POS_MIN, &g_PosLimitMin[NodeIDs[i]-1], NB_OF_BYTES_TO_WRITE_4, &NB_OF_BYTES_TO_WRITE_4, &p_rlErrorCode) &&
                                   VCS_SetObject(p_DeviceHandle, NodeIDs[i], INDEX_SOFT_POS_LIM, SUB_INDEX_SOFT_POS_MAX, &g_PosLimitMax[NodeIDs[i]-1], NB_OF_BYTES_TO_WRITE_4, &NB_OF_BYTES_TO_WRITE_4, &p_rlErrorCode) == 0)
                                {
                                    lResult = MMC_FAILED;
                                    LogError("VCS_SetObject", lResult, p_rlErrorCode);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    return lResult;
}

int SyncHomingMode(HANDLE p_DeviceHandle, unsigned int & p_rlErrorCode)
{
    int lResult = MMC_SUCCESS;
    stringstream msg;

    LogInfo("set Synchronized Homing mode");

    // Parameter setting for Homing mode
    for (int i=0; i<g_NbofJoint; i++)
    {
        if(lResult == MMC_SUCCESS)
        {
            l_ProfileVelocity = (unsigned int)(g_ProfileVelocity*g_AxisVelRatio[g_usNodeId_arr[i]-1]);
            if(VCS_SetObject(p_DeviceHandle, g_usNodeId_arr[i], INDEX_SOFT_POS_LIM, SUB_INDEX_SOFT_POS_MIN, &g_PosLimitMin[i], NB_OF_BYTES_TO_WRITE_4, &NB_OF_BYTES_TO_WRITE_4, &p_rlErrorCode) &&
               VCS_SetObject(p_DeviceHandle, g_usNodeId_arr[i], INDEX_SOFT_POS_LIM, SUB_INDEX_SOFT_POS_MAX, &g_PosLimitMax[i], NB_OF_BYTES_TO_WRITE_4, &NB_OF_BYTES_TO_WRITE_4, &p_rlErrorCode) &&
               VCS_SetObject(p_DeviceHandle, g_usNodeId_arr[i], INDEX_DIO_CONFIGURATION, SUB_INDEX_DIO_CONFIGURATION_4, &NONE, NB_OF_BYTES_TO_WRITE_1, &NB_OF_BYTES_TO_WRITE_1, &p_rlErrorCode) &&
               VCS_SetObject(p_DeviceHandle, g_usNodeId_arr[i], INDEX_DIO_CONFIGURATION, SUB_INDEX_DIO_CONFIGURATION_1, &NEGATIVE_LIMIT_SWITCH, NB_OF_BYTES_TO_WRITE_1, &NB_OF_BYTES_TO_WRITE_1, &p_rlErrorCode) &&
               VCS_SetObject(p_DeviceHandle, g_usNodeId_arr[i], INDEX_PROFILE_VELOCITY, SUB_INDEX_PROFILE_VELOCITY, &l_ProfileVelocity, NB_OF_BYTES_TO_WRITE_4, &NB_OF_BYTES_TO_WRITE_4, &p_rlErrorCode)== 0)
            {

                lResult = MMC_FAILED;
                LogError("VCS_SetObject", lResult, p_rlErrorCode);
            }
            else
            {
                if(VCS_SetHomingParameter(p_DeviceHandle, g_usNodeId_arr[i], g_HomingAcceleration, (unsigned int)(g_SpeedSwitch*g_AxisVelRatio[i]),
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
    }

    if(SyncHomingProcess(p_DeviceHandle, g_HomingSecondId, p_rlErrorCode)==0)
    {
        lResult == MMC_FAILED;
    }
    else
    {
        if(SyncHomingProcess(p_DeviceHandle, g_HomingFirstId, p_rlErrorCode)==0)
        {
            lResult == MMC_FAILED;
        }
    }

    return lResult;
}

int PreparePosCon(HANDLE p_DeviceHandle, unsigned int & p_rlErrorCode)
{
    int lResult = MMC_SUCCESS;
    stringstream msg;

    msg << "set profile position mode";
    LogInfo(msg.str());

    for (int i=0; i<g_NbofJoint; i++)
    {
        if(VCS_ActivateProfilePositionMode(p_DeviceHandle, g_usNodeId_arr[i], &p_rlErrorCode) == 0)
        {
            lResult = MMC_FAILED;
            LogError("VCS_ActivateProfilePositionMode", lResult, p_rlErrorCode);
        }
    }
    return lResult;
}

int PositionControl(HANDLE p_DeviceHandle, long PosDesired[], unsigned int & p_rlErrorCode)
{
    int lResult = MMC_SUCCESS;
    int Nbof_Ids = sizeof(PosDesired)/ sizeof(*PosDesired);
    stringstream msg;

    for (int i=0; i<Nbof_Ids; i++)
    {
        if(VCS_MoveToPosition(p_DeviceHandle, g_usNodeId_arr[i], PosDesired[i], 1, 1, &p_rlErrorCode) == 0)
        {
            lResult = MMC_FAILED;
            LogError("VCS_MoveToPosition", lResult, p_rlErrorCode);
        }
    }


    return lResult;
}

void commandCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    int lResult = MMC_SUCCESS;
    unsigned lErrorCode = 0;
    stringstream ss;

    long pos_desired[g_NbofJoint];
    double pos_desired_rad[g_NbofJoint];

    for (int i=0; i<g_NbofJoint; i++)
    {
        pos_desired_rad[i] = msg->position[i];
        pos_desired[i] = pos_desired_rad[i]/M_PI*g_PulseRev[i]*4*g_GearRatio[i]/2.0;
        ss << pos_desired[i] << ", ";
    }
    LogInfo(ss.str());
    if((lResult=PositionControl(g_pKeyHandle, pos_desired, lErrorCode))!=MMC_SUCCESS)
    {
        LogError("PositionControl", lResult, lErrorCode);
//        return lResult;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Controller");
    ros::NodeHandle n;

    int lResult = MMC_SUCCESS;
    unsigned int ulErrorCode = 0;

    SetDefaultParameters();
    PrintSettings();

    if((lResult = OpenDevice(&ulErrorCode))!=MMC_SUCCESS)
    {
        LogError("OpenDevice", lResult, ulErrorCode);
        return lResult;
    }

    if((lResult = PrepareDriver(&ulErrorCode))!=MMC_SUCCESS)
    {
        LogError("PrepareDriver", lResult, ulErrorCode);
        return lResult;
    }

    if((lResult = SyncHomingMode(g_pKeyHandle, ulErrorCode))!=MMC_SUCCESS)
    {
        LogError("SyncHoming", lResult, ulErrorCode);
        return lResult;
    }

    if((lResult = PreparePosCon(g_pKeyHandle, ulErrorCode)) !=MMC_SUCCESS)
    {
        LogError("PreparePosCon", lResult, ulErrorCode);
        return lResult;
    }

    ros::Subscriber sub = n.subscribe("ourarm/joint_states", 1000, commandCallback);
    ros::spin();

    if((lResult = CloseDevice(&ulErrorCode))!=MMC_SUCCESS)
    {
        LogError("CloseDevice", lResult, ulErrorCode);
        return lResult;
    }


}
