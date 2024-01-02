
#define GET_LOW_BYTE(A) (uint8_t)((A))
//Macro functions get low byte of A
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
//Macro functions get high byte of A
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))
//Macro functions define A as the high byte and B as the low byte, which unit to 16-bit. 
#define LOBOT_SERVO_FRAME_HEADER 0x55
#define LOBOT_SERVO_MOVE_TIME_WRITE 1
const int Activate = 12;
const int buttonPin1 = 7;  // The number of the pin where the first button is connected
const int buttonPin2 = 8;
int buttonState = 0;  // 
byte LobotCheckSum(byte buf[])
{
  byte i;
uint16_t temp = 0;
for (i = 2; i < buf[3] + 2; i++) {
temp += buf[i];
}
temp = ~temp;
i = (byte)temp;
return i;
}

void LobotSerialServoMove(HardwareSerial &SerialX, uint8_t id, int16_t position, uint16_t
time)
{
byte buf[10];
if(position < 0)
  position = 0;
if(position > 1000)
  position = 1000;
buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
buf[2] = id;
buf[3] = 7;
buf[4] = LOBOT_SERVO_MOVE_TIME_WRITE;
buf[5] = GET_LOW_BYTE(position);
buf[6] = GET_HIGH_BYTE(position);
buf[7] = GET_LOW_BYTE(time);
buf[8] = GET_HIGH_BYTE(time);
buf[9] = LobotCheckSum(buf);
SerialX.write(buf, 10);
}
void setup() {
// put your setup code here, to run once:
Serial.begin(115200);
delay(1000);
}
#define ID1 1
#define ID2 2
#define ID3 3
#define ID4 4
#define ID5 5
#define ID6 6


void loop() {
  if (digitalRead(buttonPin1) == HIGH) {
    // Sequence for picking up food and turning towards the person
    Serial.print("Going for Pick up food!");
    LobotSerialServoMove(Serial, ID1, 1000, 1000);
    LobotSerialServoMove(Serial, ID2, 235, 1000);
    LobotSerialServoMove(Serial, ID3, 530, 1000);
    LobotSerialServoMove(Serial, ID4, 745, 1000);
    LobotSerialServoMove(Serial, ID5, 220, 1000);
    LobotSerialServoMove(Serial, ID6, 500, 1000);
    delay(3000);

    Serial.print("Turning towards person");
    LobotSerialServoMove(Serial, ID1, 1000, 1000);
    LobotSerialServoMove(Serial, ID2, 460, 1000);
    LobotSerialServoMove(Serial, ID3, 530, 1000);
    LobotSerialServoMove(Serial, ID4, 745, 1000);
    LobotSerialServoMove(Serial, ID5, 220, 1000);
    LobotSerialServoMove(Serial, ID6, 500, 1000);
    delay(5000);
  } 
  else if (digitalRead(buttonPin2) == HIGH) {
    // Sequence for feeding the person
    Serial.print("Feeding the person");
    LobotSerialServoMove(Serial, ID1, 1000, 1000);
    LobotSerialServoMove(Serial, ID2, 485, 1000);
    LobotSerialServoMove(Serial, ID3, 530, 1000);
    LobotSerialServoMove(Serial, ID4, 745, 1000);
    LobotSerialServoMove(Serial, ID5, 310, 1000);
    LobotSerialServoMove(Serial, ID6, 325, 1000);
    delay(5000);
  } else {
    // Idle state when no buttons are pressed
    Serial.print("In Reset/Idle Position");
    LobotSerialServoMove(Serial, ID1, 500, 1000);
    LobotSerialServoMove(Serial, ID2, 500, 1000);
    LobotSerialServoMove(Serial, ID3, 500, 1000);
    LobotSerialServoMove(Serial, ID4, 500, 1000);
    LobotSerialServoMove(Serial, ID5, 500, 1000);
    LobotSerialServoMove(Serial, ID6, 500, 1000);
    delay(1000);
  }
}
  