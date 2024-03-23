#include <Arduino.h>
#include <HardwareSerial.h>

HardwareSerial SerialPort(2);
typedef struct DataStructure{

	int data1;

	char name[10];

} DataStructure;

DataStructure ds1 = {2500, "Apple"};
DataStructure recv;
//uint8_t data_buffer[sizeof(DataStructure)];
uint8_t data_recv[sizeof(DataStructure)];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  SerialPort.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(SerialPort.write((const char*)&ds1, sizeof(ds1)) == sizeof(DataStructure)){
    Serial.println("Success transmiting ESP2");
  }
  
  if(SerialPort.available()>0){
  int read_bytes = SerialPort.readBytes((char *)data_recv, sizeof(data_recv));
  if(read_bytes != sizeof(data_recv)){
    Serial.println("Bytes misread");
  }
  else
  {
    memcpy(&recv, data_recv, sizeof(DataStructure));
    Serial.println("Received Data in ESP2 ");
    Serial.println(recv.data1);
    Serial.println(recv.name);
  }
  }

  delay(2000);
}

