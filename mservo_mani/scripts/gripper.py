#!/usr/bin/env python
import rospy
import math
from dynamixel_sdk import *                    # Uses Dynamixel SDK library
from mservo_mani.msg import gripper_dis

# Control table address
ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_PROFILE_VELOCITY   = 112
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132

# Data Byte Length
LEN_PRO_GOAL_POSITION       = 4
LEN_PRO_PRESENT_POSITION    = 4
LEN_PRO_PROFILE_VELOCITY    = 4

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                     = 7                 # Dynamixel#1 ID : 1
DXL2_ID                     = 8                 # Dynamixel#1 ID : 2
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 1947           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 2547            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 10                # Dynamixel moving status threshold

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)
groupSyncWrite_ProfileVel = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_PROFILE_VELOCITY, LEN_PRO_PROFILE_VELOCITY)

# Initialize GroupSyncRead instace for Present Position
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

def dxl_init():
    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        quit()


    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        quit()

    # Enable Dynamixel#1 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL1_ID)

    # Enable Dynamixel#2 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL2_ID)

    # Add parameter storage for Dynamixel#1 present position value
    dxl_addparam_result = groupSyncRead.addParam(DXL1_ID)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncRead addparam failed" % DXL1_ID)
        quit()

    # Add parameter storage for Dynamixel#2 present position value
    dxl_addparam_result = groupSyncRead.addParam(DXL2_ID)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncRead addparam failed" % DXL2_ID)
        quit()

    dxl_profile_vel = 50
    param_profile_vel = [DXL_LOBYTE(DXL_LOWORD(dxl_profile_vel)),
                           DXL_HIBYTE(DXL_LOWORD(dxl_profile_vel)),
                           DXL_LOBYTE(DXL_HIWORD(dxl_profile_vel)),
                           DXL_HIBYTE(DXL_HIWORD(dxl_profile_vel))]

    dxl_addparam_result = groupSyncWrite_ProfileVel.addParam(DXL1_ID, param_profile_vel)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID)
        quit()

    dxl_addparam_result = groupSyncWrite_ProfileVel.addParam(DXL2_ID, param_profile_vel)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL2_ID)
        quit()

    dxl_comm_result = groupSyncWrite_ProfileVel.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWrite_ProfileVel.clearParam()


def dis2inc(distance):
    if distance > 0.1:
        distance = 0.1
    elif distance < 0:
        distance = 0

    distance = distance/2+0.0065
    inc = int(math.degrees(math.asin((distance-0.01756)/0.07))/0.088)

    return inc

def dxl_poscon(goal_pos):
    # Allocate goal position value into byte array
    dxl_goal_position_1 = 2048+goal_pos
    dxl_goal_position_2 = 2048-goal_pos
    param_goal_position_1 = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_1)),
                           DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_1)),
                           DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_1)),
                           DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_1))]
    param_goal_position_2 = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_2)),
                             DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_2)),
                             DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_2)),
                             DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_2))]

    # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position_1)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID)
        quit()

    # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position_2)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL2_ID)
        quit()

    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()

    while 1:
        # Syncread present position
        dxl_comm_result = groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Check if groupsyncread data of Dynamixel#1 is available
        dxl_getdata_result = groupSyncRead.isAvailable(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % DXL1_ID)
            quit()

        # Check if groupsyncread data of Dynamixel#2 is available
        dxl_getdata_result = groupSyncRead.isAvailable(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % DXL2_ID)
            quit()

        # Get Dynamixel#1 present position value
        dxl1_present_position = groupSyncRead.getData(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        # Get Dynamixel#2 present position value
        dxl2_present_position = groupSyncRead.getData(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL1_ID, dxl_goal_position_1, dxl1_present_position, DXL2_ID, dxl_goal_position_2, dxl2_present_position))

        if not ((abs(dxl_goal_position_1- dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) and (abs(dxl_goal_position_2 - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD)):
            break

    # # Clear syncread parameter storage
    # groupSyncRead.clearParam()

def callback(distance):
    goal_pos = dis2inc(distance.gripper_dis)
    dxl_poscon(goal_pos)

def gripper():
    rospy.init_node('gripper', anonymous=True)
    rospy.Subscriber('GripperDis', gripper_dis, callback)
    rospy.spin()

if __name__ == '__main__':
    dxl_init()
    gripper()

    # Disable Dynamixel#1 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE,
                                                              TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Disable Dynamixel#2 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE,
                                                              TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Close port
    portHandler.closePort()


