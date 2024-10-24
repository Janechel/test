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
uint32_t analysis_data;                   //Data parsed from the status packet

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
  //Change the torque switch of servo ID1 to OFF.
  primary_servo_set_torque_switch(1, 0, order_buffer,&order_len);

  digitalWrite(4,HIGH);
  SERVO_SERIAL.write(order_buffer, order_len);
  SERVO_SERIAL.flush();
  digitalWrite(4,LOW);
  delay(10);

  if (SERVO_SERIAL.available()>0)
  {
      pack_len = SERVO_SERIAL.available();
      readFunction(pack, pack_len);
      ret = primary_servo_set_torque_switch_analysis(pack);
      if(ret == PRIMARY_SUCCESS)
          PRINTF("write torque switch complete\r\n");
  }
  else
  {
      PRINTF("failed to read data\r\n");
  }
  delay(1000);

  //Change the control mode of servo ID1 to the current control mode.
  primary_servo_set_control_mode(1, 2, order_buffer,&order_len);

  digitalWrite(4,HIGH);
  SERVO_SERIAL.write(order_buffer, order_len);
  SERVO_SERIAL.flush();
  digitalWrite(4,LOW);
  delay(1);

  if (SERVO_SERIAL.available()>0)
  {
      pack_len = SERVO_SERIAL.available();
      readFunction(pack, pack_len);
      ret = primary_servo_set_control_mode_analysis(pack);
      if(ret == PRIMARY_SUCCESS)
          PRINTF("write control mode complete\r\n");
  }
  else
  {
      PRINTF("failed to read data\r\n");
  }
  delay(1000);

  //Change the torque switch of servo ID1 to ON.
  primary_servo_set_torque_switch(1, 1, order_buffer,&order_len);

  digitalWrite(4,HIGH);
  SERVO_SERIAL.write(order_buffer, order_len);
  SERVO_SERIAL.flush();
  digitalWrite(4,LOW);
  delay(1);

  if (SERVO_SERIAL.available()>0)
  {
      pack_len = SERVO_SERIAL.available();
      readFunction(pack, pack_len);
      ret = primary_servo_set_torque_switch_analysis(pack);
      if(ret == PRIMARY_SUCCESS)
          PRINTF("write torque switch complete\r\n");
  }
  else
  {
      PRINTF("failed to read data\r\n");
  }
  delay(1000);

  //Change the target PWM of servo ID1 to 100mA.
  primary_servo_set_target_current(1, 100, order_buffer,&order_len);

  digitalWrite(4,HIGH);
  SERVO_SERIAL.write(order_buffer, order_len);
  SERVO_SERIAL.flush();
  digitalWrite(4,LOW);
  delay(1);

  if (SERVO_SERIAL.available()>0)
  {
      pack_len = SERVO_SERIAL.available();
      readFunction(pack, pack_len);
      ret = primary_servo_set_target_current_analysis(pack);
      if(ret == PRIMARY_SUCCESS)
          PRINTF("write target current complete\r\n");
  }
  else
  {
      PRINTF("failed to read data\r\n");
  }
  delay(3000);
}
