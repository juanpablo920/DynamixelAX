#include <DynamixelSDK.h>

// AX-series Control table address
#define ADDR_AX_TORQUE_ENABLE           24                 // Control table address is different in Dynamixel model
#define ADDR_AX_GOAL_POSITION           30
#define ADDR_AX_PRESENT_POSITION        36

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          1                   // Dynamixel ID: 1
#define BAUDRATE                        1000000
#define DEVICENAME                      "3"                 //DEVICENAME "1" -> Serial1(OpenCM9.04 DXL TTL Ports)
//DEVICENAME "2" -> Serial2
//DEVICENAME "3" -> Serial3(OpenCM 485 EXP)
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      100                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      1000                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

void waiting_master() {
  while (Serial.available() == 0);
  String msg;
  msg = Serial.readString();
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int dxl_comm_result = COMM_TX_FAIL; // Communication result
  uint8_t dxl_error = 0;              // Dynamixel error

  // Open port
  if ( portHandler->openPort() ) {
    Serial.println("succeeded_open_port");
  }
  else {
    Serial.println("failed_open_port");
    return;
  }

  waiting_master();

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE)) {
    Serial.println("succeeded_change_baudrate");
  }
  else {
    Serial.println("failed_change_baudrate");
    portHandler->closePort();
    return;
  }

  waiting_master();

  int torque_tmp = 1;

  // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    packetHandler->getTxRxResult(dxl_comm_result);
    Serial.println("dynamixel_failed_connected");
    torque_tmp = 0;
  }
  else if (dxl_error != 0) {
    packetHandler->getRxPacketError(dxl_error);
    Serial.println("dynamixel_failed_connected");
    torque_tmp = 0;
  }
  else {
    Serial.println("dynamixel_successfully_connected");
    torque_tmp = 1;
  }

  String input_tmp;
  int goal_posse = 0;
  int16_t present_pose = 0;

  while (torque_tmp) {

    if ( Serial.available() ) {

      input_tmp = Serial.readString();
      input_tmp.replace("\n", "");

      if (input_tmp == "get_pose") {

        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_AX_PRESENT_POSITION, (uint16_t*)&present_pose, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS) {
          packetHandler->getTxRxResult(dxl_comm_result);
          Serial.println("failed_get_pose");
          break;
        }
        else if (dxl_error != 0) {
          packetHandler->getRxPacketError(dxl_error);
          Serial.print("failed_get_pose");
          break;
        }
        Serial.println(present_pose);
      }

      if (input_tmp == "set_pose") {
        Serial.println("");

        while (Serial.available() == 0);
        input_tmp = Serial.readString();
        goal_posse = input_tmp.toInt();

        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_AX_GOAL_POSITION, goal_posse, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS) {
          packetHandler->getTxRxResult(dxl_comm_result);
          Serial.println("failed_set_pose");
          break;
        }
        else if (dxl_error != 0) {
          packetHandler->getRxPacketError(dxl_error);
          Serial.println("failed_set_pose");
          break;
        }
        Serial.println("succeeded_set_pose");
      }

      if (input_tmp == "go_origin") {

        int origin;
        int sleep;
        int delta_poses;
        int delta_error;
        int new_pos;

        Serial.println("");
        while (Serial.available() == 0);
        input_tmp = Serial.readString();
        origin = input_tmp.toInt();

        Serial.println("");
        while (Serial.available() == 0);
        input_tmp = Serial.readString();
        sleep = input_tmp.toInt();

        Serial.println("");
        while (Serial.available() == 0);
        input_tmp = Serial.readString();
        delta_error = input_tmp.toInt();

        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_AX_PRESENT_POSITION, (uint16_t*)&present_pose, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS) {
          packetHandler->getTxRxResult(dxl_comm_result);
          Serial.println("failed_go_origin");
          break;
        }
        else if (dxl_error != 0) {
          packetHandler->getRxPacketError(dxl_error);
          Serial.print("failed_go_origin");
          break;
        }

        delta_poses = abs(present_pose - origin);

        while (delta_poses > delta_error) {

          if ( delta_poses < sleep) {
            sleep = delta_poses;
          }

          if (present_pose > origin) {
            new_pos = present_pose - sleep;
          }
          else {
            new_pos = present_pose + sleep;
          }

          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_AX_GOAL_POSITION, new_pos, &dxl_error);

          if (dxl_comm_result != COMM_SUCCESS) {
            packetHandler->getTxRxResult(dxl_comm_result);
            Serial.println("failed_go_origin");
            break;
          }
          else if (dxl_error != 0) {
            packetHandler->getRxPacketError(dxl_error);
            Serial.println("failed_go_origin");
            break;
          }

          dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_AX_PRESENT_POSITION, (uint16_t*)&present_pose, &dxl_error);

          if (dxl_comm_result != COMM_SUCCESS) {
            packetHandler->getTxRxResult(dxl_comm_result);
            Serial.println("failed_go_origin");
            break;
          }
          else if (dxl_error != 0) {
            packetHandler->getRxPacketError(dxl_error);
            Serial.print("failed_go_origin");
            break;
          }
          delay(100);
          delta_poses = abs(present_pose - origin);
        }
        Serial.println("arrived_scan");
      }

      if (input_tmp == "exit") {
        Serial.println("succeeded_exit");
        break;
      }
    }
  }

  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_AX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }

  // Close port
  portHandler->closePort();

}

void loop() {
  // put your main code here, to run repeatedly:

}
