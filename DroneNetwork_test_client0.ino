/*

** Objective: 
** This test is performed with two Drone
** AP Drone & Client0
** AP Drone: Once Gided mode is active, AP Drone will go to the IDEL loacation 
   and look 
   for predefined objects. Once boject/s detected It will send the data to 
   client0 (Asyn.)
** Client0: Once Gided mode active, client0 drone will go to a IDEL location 
   and wait for messages from the AP Drone. Based on the received message, 
   if red(sig 1) block detected it will move 4m towards the west. If blue(sig 2)
   block detected it will move 4m towards the east.
** 
*/

// ** Client 0 **
#include <MAVLink.h>
#include <SPI.h>
#include <Pixy2.h>
#include "GIGACOMMS.h"
#include "WiFi.h"

Pixy2 pixy;
GigaComms comm;

IPAddress ip(192, 168, 3, 4); // Client0 IP address
IPAddress gateway(192, 168, 3, 1); // Gateway IP address
IPAddress subnet(255, 255, 255, 0); // Subnet mask

// message format: 3:1/3/4/   
IPAddress APDrone(192, 168, 3, 1);  //
IPAddress laptopIP(192, 168, 3, 2);

//String laptopMsg = "";
bool Guided = false;
bool reachedDestination = false;

const double idealLat = 35.309349; // Ideal latitude
const double idealLon = -80.737924; // Ideal longitude
const float hoverAltitude = 3.0; // Hover altitude in meters

// Variables to store battery voltage information
float battery_voltage = 0.0;
float battery_remaining = 0.0;

void setup() {
  Serial.begin(115200); // For debugging
  Serial1.begin(57600); // MAVLink communication with Pixhawk
  pixy.init();
  WiFi.config(ip, gateway, subnet);
  comm.connectToAP("Client_0");
  Serial.println("Ready");
}

void loop() {
  Mav_Request_Data();

  if (Serial1.available()) {
    mavlink_message_t msg;
    mavlink_status_t status;
    String laptopMsg = "";
    
   

    while (Serial1.available()) {
      uint8_t c = Serial1.read();
      String laptopMsg = "";
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
        if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
          mavlink_heartbeat_t heartbeat;
          mavlink_msg_heartbeat_decode(&msg, &heartbeat);
          
          Guided = (heartbeat.custom_mode == 4);
        }
        if (msg.msgid == MAVLINK_MSG_ID_BATTERY_STATUS){
          mavlink_battery_status_t battery_status;
          mavlink_msg_battery_status_decode(&msg, &battery_status);

          // Extract battery voltage and remaining percentage
          battery_voltage = battery_status.voltages[0] / 1000.0; // Voltage in Volts
          battery_remaining = battery_status.battery_remaining; // Remaining battery percentage

          // Print battery information
          Serial.print("Battery Voltage: ");
          Serial.print(battery_voltage);
          Serial.print(" V \n");
          laptopMsg += "C_Bat_Vol: " + String(battery_voltage) + " V \n";
          comm.sendUDP_laptop(laptopMsg.c_str(), laptopIP);
          //laptopMsg += "Battery Voltage: " + String(battery_voltage) + " V \n";
          //Serial.print(battery_remaining);
          //Serial.println(" %");
        }
        if (msg.msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
          mavlink_gps_raw_int_t gps_raw;
          mavlink_msg_gps_raw_int_decode(&msg, &gps_raw);

          Serial.println((gps_raw.lat / 1E7), 8);
          Serial.println((gps_raw.lon / 1E7), 8);
          Serial.println("Satellites: " + String(gps_raw.satellites_visible));
          Serial.println("---------------------------");
          laptopMsg += String(gps_raw.lat / 1E7, 8) + "\n" + String(gps_raw.lon / 1E7, 8) + "\n" + "Satellites: " + String(gps_raw.satellites_visible) + "\n---------------------------\n";
          //laptopMsg += String((gps_raw.lat / 1E7), 8) + "\n" + String((gps_raw.lat / 1E7), 8) + "\n" + "Satellites: " + String(gps_raw.satellites_visible) += "\n---------------------------\n";

          Serial.println(laptopMsg);
          //comm.sendUDP_laptop(laptopMsg.c_str(), laptopIP);

          double currentLat = gps_raw.lat / 1E7;
          double currentLon = gps_raw.lon / 1E7;

          if (Guided && !reachedDestination) {
            moveDroneToIdealLocation();
            String msg= "Guided Mode Active!";
            delay(10000);
            //comm.sendUDP_laptop(msg.c_str(), laptopIP);
            moveAPDrone();
            if (isAtIdealLocation(currentLat, currentLon)) {
              //reachedDestination = true;
              moveAPDrone();
            }
          }
        }
      }
    }
  }
}

void Mav_Request_Data() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAV_DATA_STREAM_ALL, 1, 1);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

void moveDroneToIdealLocation() {
  sWaypoint(idealLat, idealLon, hoverAltitude);
}

bool isAtIdealLocation(double currentLat, double currentLon) {
  const double threshold = 0.000010; // Adjust as needed  0.000025 = 1 m
  return (fabs(currentLat - idealLat) < threshold) && (fabs(currentLon - idealLon) < threshold);
}

void processPixy2Input() {
  pixy.ccc.getBlocks();
  String pixyData = "";
  pixyData = "Blocks: " + String(pixy.ccc.getBlocks());
  if (pixy.ccc.numBlocks) {
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      if (pixy.ccc.blocks[i].m_signature == 1) { // Assuming signature 1 is red
        pixyData += "Sig: " + String(pixy.ccc.blocks[i].m_signature);
        reachedDestination = true;
        moveDrone(0, -4); // Move west by 4 meters
      } else if (pixy.ccc.blocks[i].m_signature == 2) { // Assuming signature 2 is blue
        pixyData += " Sig: " + String(pixy.ccc.blocks[i].m_signature);
        reachedDestination = true;
        moveDrone(0, 4); // Move east by 4 meters
      }
    }
    //comm.sendUDP_laptop(pixyData.c_str(), laptopIP);
  } /*else {
    landDrone(); // Land if no color detected
  }*/
}

void moveDrone(int32_t north, int32_t east) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  int32_t lat = idealLat * 1E7; // Use ideal latitude
  int32_t lon = idealLon * 1E7 + east * 1E7 / (111111 * cos(idealLat * M_PI / 180));
  int32_t alt = hoverAltitude; // Maintain hover altitude

  mavlink_msg_set_position_target_global_int_pack(1, 200, &msg, millis(), 1, 1, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    0b0000111111111000, lat, lon, alt, 0, 0, 0, 0, 0, 0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

void landDrone() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_command_long_pack(1, 200, &msg, 1, 1, MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

void sWaypoint(float lat, float lon, float alt) {
  int32_t x = lat * 1E7;
  int32_t y = lon * 1E7;
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_set_position_target_global_int_pack(1, 200, &msg, millis(), 1, 1, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    0b0000111111111000, x, y, alt, 0, 0, 0, 0, 0, 0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

//MOves AP drone East or West
void moveAPDrone(){
  pixy.ccc.getBlocks();
  String pixyData = "";
  pixyData = "Blocks: " + String(pixy.ccc.getBlocks());
  if (pixy.ccc.numBlocks) {
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      if (pixy.ccc.blocks[i].m_signature == 1) { // Assuming signature 1 is red
        pixyData += "Sig: " + String(pixy.ccc.blocks[i].m_signature);
        reachedDestination = true;
        Serial.println("Calling the AP drone: Red");
        comm.sendUDP("1", APDrone);

      } else if (pixy.ccc.blocks[i].m_signature == 2) { // Assuming signature 2 is blue
        pixyData += " Sig: " + String(pixy.ccc.blocks[i].m_signature);
        reachedDestination = true;
        Serial.println("Calling the AP drone: Blue");

        comm.sendUDP("2", APDrone);
      }
    }
    //comm.sendUDP_laptop(pixyData.c_str(), laptopIP);
  } /*else {
    landDrone(); // Land if no color detected
  }*/
}

//Calls the AP Drone to the GPS location
void callAPDrone(){
  pixy.ccc.getBlocks();
  mavlink_gps_raw_int_t gps_raw; 
  String pixyData = "";
  pixyData = "Blocks: " + String(pixy.ccc.getBlocks());
  if (pixy.ccc.numBlocks) {
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      if (pixy.ccc.blocks[i].m_signature == 1) { // Assuming signature 1 is red
        pixyData += "Sig: " + String(pixy.ccc.blocks[i].m_signature);
        reachedDestination = true;
        Serial.println("Calling the AP drone: Red");

        int32_t lat = gps_raw.lat;
        int32_t lon = gps_raw.lon;
        comm.sendUDP((String("1,") + String(lat) + "," + String(lon)).c_str(), APDrone);

      } else if (pixy.ccc.blocks[i].m_signature == 2) { // Assuming signature 2 is blue
        pixyData += "Sig: " + String(pixy.ccc.blocks[i].m_signature);
        reachedDestination = true;
        Serial.println("Calling the AP drone: Blue");

        int32_t lat = gps_raw.lat;
        int32_t lon = gps_raw.lon;
        comm.sendUDP((String("2,") + String(lat) + "," + String(lon)).c_str(), APDrone);
      }
    }
    //comm.sendUDP_laptop(pixyData.c_str(), laptopIP);
  } /*else {
    landDrone(); // Land if no color detected
  }*/
}

