#include "PrimaryServo.h"

// This Example is compatible with both Esp32 and ArduinoUno boards, 
// if you are using ArduinoUno, you need to comment "#define Esp32_BOARD" on this line, 
// if you are using Esp32 it does not need to change.
#define ESP32_BOARD

#if defined(ESP32_BOARD)
#define SERVO_SERIAL Serial2
void readFunction(uint8_t* pack, uint8_t pack_len) {
    SERVO_SERIAL.read(pack, pack_len);
}
#else
#define SERVO_SERIAL Serial
void readFunction(uint8_t* pack, uint8_t pack_len) {
    SERVO_SERIAL.readBytes(pack, pack_len);
}
#endif

uint8_t ret;                              //Change Unknown Servo ID Test
uint8_t order_buffer[40];                 //Store Generated Instructions
uint8_t order_len;                        //Instruction Length
uint8_t pack[40];                         //Store the received status packet
uint8_t pack_len;                         //Response packet length.
uint8_t write_buffer[20];                 //Write data to the memory table

struct primary_servo_sync_parameter servo;

void printHex(int num) {
  char hex[8];
  sprintf(hex, "%02X", num);
  PRINTF(" 0x");
  PRINTF(hex);
}

void setup() {
  // put your setup code here, to run once:
#if defined(ESP32_BOARD)
  Serial.begin(115200);
  SERVO_SERIAL.begin(1000000, SERIAL_8N1, 16, 17);  
#else
  SERVO_SERIAL.begin(1000000);  
#endif
  pinMode(4,OUTPUT);    //It is used to control the uart transmission direction
}

void loop() {
  servo.id_counts = 2;            //Sync write two servos
  servo.id[0] = 1;                //Set the ID of the first servo to 1
  servo.id[1] = 2;                //Set the ID of the second servo to 2

  //Change the torque switch of the servo ID1, ID2 to OFF respectively.
  servo.torque_switch[0] = 0;
  servo.torque_switch[1] = 0;

  primary_servo_sync_write_torque_switch(servo, order_buffer,&order_len);
  digitalWrite(4,HIGH);
  SERVO_SERIAL.write(order_buffer, order_len);
  PRINTF("\r\nsync write torque switch complete\r\n");
  delay(1000);

  //Change the control mode of the servo ID1, ID2 to time base position control mode respectively.
  servo.control_mode[0] = 0;
  servo.control_mode[1] = 0;
  primary_servo_sync_write_control_mode(servo, order_buffer,&order_len);
  digitalWrite(4,HIGH);
  SERVO_SERIAL.write(order_buffer, order_len);
  PRINTF("\r\nsync write control mode complete\r\n");
  delay(1000);

  //Change the time base target position, and moving time of servo ID1 to 300°, and 500ms, respectively.
  primary_servo_set_time_base_target_position_and_moving_time(1, 3000, 500, order_buffer, &order_len);
  digitalWrite(4,HIGH);
  SERVO_SERIAL.write(order_buffer, order_len);
  SERVO_SERIAL.flush();
  digitalWrite(4,LOW);
  delay(10);

  if (SERVO_SERIAL.available()>0)
  {
      pack_len = SERVO_SERIAL.available();
      readFunction(pack, pack_len);
      ret = primary_servo_set_time_base_target_position_and_moving_time_analysis(pack);
      if(ret == PRIMARY_SUCCESS)
          PRINTF("write time base target position and moving time complete\r\n");
  }
  else
  {
      PRINTF("failed to read data\r\n");
  }
  delay(1000);

  //Change the time base target ACC, position, and moving time of servo ID1 to 0°, 300°, and 1s, respectively.
  write_buffer[0] = 0;
  write_buffer[1] = 3000 & 0xff;
  write_buffer[2] = (3000 >> 8) & 0xff;
  write_buffer[3] = 1000 & 0xff;
  write_buffer[4] = (1000 >> 8) & 0xff;

  primary_servo_write(1, 0x3B, 5, write_buffer, order_buffer, &order_len);
  digitalWrite(4,HIGH);
  SERVO_SERIAL.write(order_buffer, order_len);
  SERVO_SERIAL.flush();
  digitalWrite(4,LOW);
  delay(10);

  if (SERVO_SERIAL.available()>0)
  {
      pack_len = SERVO_SERIAL.available();
      readFunction(pack, pack_len);
      PRINTF("\r\nwrite time base target acc, position and moving time status packet: ");
      for(uint8_t i = 0; i < pack_len; i++)
      {
          printHex(pack[i]);
      }
      PRINTF("\r\n");
  }
  else
  {
      PRINTF("Failed to read data.\r\n");
  }
  delay(1000);

  //In time base position control mode, let servo ID1 move to the 150° position at a velocity of 500ms,
  //and let servo ID2 move to the 0° position at a constant velocity of 1s.
  servo.position[0] = 1500;
  servo.position[1] = 0;
  servo.time[0] = 500;
  servo.time[1] = 1000;

  primary_servo_sync_write_time_base_target_position_and_moving_time(servo, order_buffer, &order_len);
  digitalWrite(4,HIGH);
  SERVO_SERIAL.write(order_buffer, order_len);
  PRINTF("\r\nsync write time base target position and moving time complete\r\n");
  delay(1000);

  //In time base position control mode, let servo ID1 move to the 0° position at a velocity of 1s,
  //and let servo ID2 move to the 3000° position at a constant velocity of 500ms.
  servo.position[0] = 0;
  servo.position[1] = 3000;
  servo.time[0] = 1000;
  servo.time[1] = 500;

  primary_servo_sync_write_time_base_target_position_and_moving_time(servo, order_buffer, &order_len);
  digitalWrite(4,HIGH);
  SERVO_SERIAL.write(order_buffer, order_len);
  PRINTF("\r\nsync write time base target position and moving time complete\r\n");
  delay(1000);
}
