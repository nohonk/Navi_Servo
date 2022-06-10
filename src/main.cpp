#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <CompassHeading.h>

/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
*/

#define   dataPin   13   //DS Pin of 74HC595(Pin14)
#define   latchPin  12   //ST_CP Pin of 74HC595(Pin12)
#define   clockPin 11    //CH_CP Pin of 74HC595(Pin11)
#define NORD      0x80
#define NORD_OST  0x01
#define OST       0x02
#define SUED_OST  0x04
#define SUED      0x08
#define SUED_WEST 0x10
#define WEST      0x20
#define NORD_WEST 0x40


void _shiftOut(int dPin,int cPin,int order,int val){   
	int i;  
    for(i = 0; i < 8; i++){
        digitalWrite(cPin,LOW);
        if(order == LSBFIRST){
            digitalWrite(dPin,((0x01&(val>>i)) == 0x01) ? HIGH : LOW);
            delayMicroseconds(10);
		}
        else {//if(order == MSBFIRST){
            digitalWrite(dPin,((0x80&(val<<i)) == 0x80) ? HIGH : LOW);
            delayMicroseconds(10);
		}
        digitalWrite(cPin,HIGH);
        delayMicroseconds(10);
	}
}

#define DECLINATION_ANGLE 0.22
CompassHeading compass = CompassHeading(DECLINATION_ANGLE);


// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(9600);
	Serial.println("Program is starting ...\n");
	
	pinMode(dataPin,OUTPUT);
	pinMode(latchPin,OUTPUT);
	pinMode(clockPin,OUTPUT);

 
  Serial.println("Compass Heading Test"); Serial.println("");

  /* Initialise the sensor. Blocks if not found. */
  compass.start();

  /* Display some basic information on this sensor */
  compass.displaySensorDetails();
  delay(500);


}

// the loop function runs over and over again forever
void loop() {
  int i;
	unsigned char x;




  float heading = compass.headingDegrees();
  Serial.print("Heading (degrees): "); Serial.println(heading);
  if ( (heading > 337.5) || (heading <= 22.5) )
  {
    x = NORD;
  }
  if ( (heading > 22.5) && (heading <= 67.5) )
  {
    x = NORD_OST;
  }
  if ( (heading > 67.5) && (heading <= 112.5) )
  {
    x = OST;
  }
  if ( (heading > 112.5) && (heading <= 157.5) )
  {
    x = SUED_OST;
  }
  if ( (heading > 157.5) && (heading <= 202.5) )
  {
    x = SUED;
  }
  if ( (heading > 202.5) && (heading <= 247.5) )
  {
    x = SUED_WEST;
  }
  if ( (heading > 247.5) && (heading <= 292.5) )
  {
    x = WEST;
  }
  if ( (heading > 292.5) && (heading <= 337.5) )
  {
    x = NORD_WEST;
  }
  delay(500);

			digitalWrite(latchPin,LOW);		// Output low level to latchPin
			_shiftOut(dataPin,clockPin,LSBFIRST,x);// Send serial data to 74HC595
			digitalWrite(latchPin,HIGH);   //Output high level to latchPin, and 74HC595 will update the data to the parallel output port.
}

