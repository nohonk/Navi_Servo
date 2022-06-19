#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <CompassHeading.h>

/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-gps
 */

#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

const int RXPin = 3, TXPin = 4;
const uint32_t GPSBaud = 9600; //Default baud of NEO-6M is 9600
#define   dataPin   13   //DS Pin of 74HC595(Pin14)
#define   latchPin  12   //ST_CP Pin of 74HC595(Pin12)
#define   clockPin 11    //CH_CP Pin of 74HC595(Pin11)
#define GPS_AVAIL_LED 5
static const double LONDON_LAT = 52.227068, LONDON_LON = 10.575161;
#define NORD      0x80
#define NORD_OST  0x01
#define OST       0x02
#define SUED_OST  0x04
#define SUED      0x08
#define SUED_WEST 0x10
#define WEST      0x20
#define NORD_WEST 0x40

TinyGPSPlus gps; // the TinyGPS++ object
SoftwareSerial gpsSerial(RXPin, TXPin); // the serial interface to the GPS device
#define DECLINATION_ANGLE 0.22
CompassHeading compass = CompassHeading(DECLINATION_ANGLE);
float heading, oldheading;
float ledDirection;
unsigned char x;
uint8_t ledState = LOW;
uint16_t ledCounter = 0;
uint32_t millisCounter = 0;
const uint16_t NoGpsLed = 500;
const uint16_t InValidGpsLed = 250;


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






void setup() {
  Serial.begin(9600);
  gpsSerial.begin(GPSBaud);
  compass.start();
  Serial.println(F("Arduino - GPS module"));
  pinMode(dataPin,OUTPUT);
	pinMode(latchPin,OUTPUT);
	pinMode(clockPin,OUTPUT);
  pinMode(GPS_AVAIL_LED, OUTPUT);

}

void loop() {
  if (gpsSerial.available() > 0) {
   // Serial.println("Gps Serial available");
    if (gps.encode(gpsSerial.read())) {
     // Serial.println("Read true");
      if (gps.location.isValid()) {
        Serial.print(F("- latitude: "));
        Serial.println(gps.location.lat(),6);
        Serial.print(F("- longitude: "));
        Serial.println(gps.location.lng(),6); 
        ledState = HIGH;       
      } else {
          if (millis() > millisCounter)
          {
            ledState = ledState == LOW ? HIGH : LOW;
            millisCounter += InValidGpsLed; 
          }    
       // Serial.println(F("- location: INVALID"));
      }
//      Serial.println();
    }
  } else 
  {
    if (millis() > millisCounter)
    {
      ledState = ledState == LOW ? HIGH : LOW;
      millisCounter += NoGpsLed; 

    }
  }

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));


  double courseToLondon =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
    //  HERE_LAT,
    //  HERE_LON,
      LONDON_LAT, 
      LONDON_LON);

  


  oldheading = heading;
  heading = compass.headingDegrees();

  ledDirection = courseToLondon - heading;
  if (ledDirection < 0.0) 
  {
    ledDirection += 360.0;
  }

  if (abs(oldheading - heading) > 5 )
  {
    Serial.print("Heading (degrees): "); Serial.println(heading);
    Serial.print("Kurs nach London ");
    Serial.println(courseToLondon);
    Serial.print("Kurs angepasst ");
    Serial.println(ledDirection);


  }

  if ( (ledDirection > 337.5) || (ledDirection <= 22.5) )
  {
    x = OST;
    //Serial.println("OST");
  }
  if ( (ledDirection > 22.5) && (ledDirection <= 67.5) )
  {
    x = NORD_OST;
    //Serial.println("NORD_OST");
  }
  if ( (ledDirection > 67.5) && (ledDirection <= 112.5) )
  {
    x = NORD;
    //Serial.println("NORD");
  }
  if ( (ledDirection > 112.5) && (ledDirection <= 157.5) )
  {
    x = NORD_WEST;
    //Serial.println("NORDWEST");
  }
  if ( (ledDirection > 157.5) && (ledDirection <= 202.5) )
  {
    x = WEST;
    //Serial.println("WEST");
  }
  if ( (ledDirection > 202.5) && (ledDirection <= 247.5) )
  {
    x = SUED_WEST;
    //Serial.println("SUED_WEST");
  }
  if ( (ledDirection > 247.5) && (ledDirection <= 292.5) )
  {
    x = SUED;
    //Serial.println("SUED");
  }
  if ( (ledDirection > 292.5) && (ledDirection <= 337.5) )
  {
    x = SUED_OST;
    //Serial.println("SUED_OST");
  }


	digitalWrite(latchPin,LOW);		// Output low level to latchPin
	_shiftOut(dataPin,clockPin,LSBFIRST,x);// Send serial data to 74HC595
	digitalWrite(latchPin,HIGH);   //Output high level to latchPin, and 74HC595 will update the data to the parallel output port.
  digitalWrite(GPS_AVAIL_LED, ledState);

}

#if 0
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

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

/*
Was soll das Programm leisten
unabhängig von der Ausrichtung soll die Richtung zu einer Kooridnate angezeigt werden.
Es sollen mehrere Koordinaten abgelegt werden können.
AUf knopfdruck soll dir nächste Koordinate angezeigt werden.
Alternativ soll ein Knopfdruck code eingegeben werden. 
Dafür muss :
Das gerät muss seinen Ort kennen.#
Der nächste Zielort muss bekannt sein.
Die Direction zu diesem Ort muss  berechnet werden können.
Das Gerät muss seine Ausrichtung kennen. also Kompass lage um die Korrekte Richtung anzeigen zu können.
Grobe Richtung reicht. Mit 8 Led kann Nord Nord-ost ost sued ost sued sued west west nord west angezeigt werden. 
*/







float bearing;

static void smartDelay(unsigned long ms);


// the setup function runs once when you press reset or power the board
void setup() {


 
  Serial.println("Compass Heading Test"); Serial.println("");

  /* Initialise the sensor. Blocks if not found. */
//  

  /* Display some basic information on this sensor */
//  compass.displaySensorDetails();
  delay(500);


}

static const double HERE_LAT = 52.225143, HERE_LON = 10.571486;
bool ValidLocation = false;

// the loop function runs over and over again forever
void loop() {

   if (ValidLocation) {

  float heading = compass.headingDegrees();
  //Serial.print("Heading (degrees): "); Serial.println(heading);
  bearing = courseToLondon;
  
  Serial.println("Heading   Orig Lond   Angep Lond");
  Serial.print(heading);
  Serial.print("    ");
  Serial.print(bearing);
  Serial.print("     ");
  Serial.print(ledDirection);
  Serial.println("");
  
  delay(50);

 // smartDelay(1000);

  }


  
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());
  } while (millis() - start < ms);
}


#endif