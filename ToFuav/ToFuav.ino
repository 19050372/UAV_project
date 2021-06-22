#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;
VL53L0X sensor2;
VL53L0X sensor3;

void setup()
{

  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);

  delay(500);
  Wire.begin();


  Serial.begin (9600);

  //SENSOR
  pinMode(5, INPUT);
  delay(150);
  Serial.println("00");
  sensor.init(true);
  Serial.println("01");
  delay(100);
  sensor.setAddress((uint8_t)22);
  Serial.println("02");

  //SENSOR 2
  pinMode(6, INPUT);
  delay(150);
  sensor2.init(true);
  Serial.println("03");
  delay(100);
  sensor2.setAddress((uint8_t)25);
  Serial.println("04");

  //SENSOR 3
  pinMode(7, INPUT);
  delay(150);
  sensor3.init(true);
  Serial.println("05");
  delay(100);
  sensor3.setAddress((uint8_t)28);
  Serial.println("06");

  Serial.println("");
  Serial.println("addresses set");
  Serial.println("");
  Serial.println("");



  sensor.setTimeout(500);
  sensor2.setTimeout(500);
  sensor3.setTimeout(500);


}

void loop()
{
  Serial.println("__________________________________________________________________");
  Serial.println("");
  Serial.println("=================================");
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;


  //for (byte i = 1; i < 120; i++)
  for (byte i = 1; i < 30; i++)
  {

    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      
    } // end of good response
  } // end of for loop
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");
  Serial.println("=================================");


  //CHECK DISTANCES
  long DISTANCE_FWD = (sensor.readRangeSingleMillimeters());
  long DISTANCE_FLT = (sensor2.readRangeSingleMillimeters());
  long DISTANCE_FRT = (sensor3.readRangeSingleMillimeters());

  //FWD OR SENSOR
  if (sensor.timeoutOccurred())
  {
    Serial.print("Distance 5 (READING): ");
    Serial.println(" TIMEOUT");
    Serial.println("");
  }
  else
  {
    Serial.print("Distance 5 : ");
    Serial.println(DISTANCE_FWD);
    Serial.println("");
  }

  //FLT OR SENSOR2
  if (sensor2.timeoutOccurred())
  {
    Serial.print("Distance 6 (READING): ");
    Serial.println(" TIMEOUT");
    Serial.println("");
  }
  else
  {
    Serial.print("Distance 6: ");
    Serial.println(DISTANCE_FLT);
    Serial.println("");
  }
  
  //FRT OR SENSOR3
  if (sensor3.timeoutOccurred())
  {
    Serial.print("Distance 7 (READING): ");
    Serial.println(" TIMEOUT");
    Serial.println("");
  }
  else
  {
    Serial.print("Distance 7: ");
    Serial.println(DISTANCE_FRT);
    Serial.println("");
  }


  
  Serial.println("__________________________________________________________________");
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();

}
