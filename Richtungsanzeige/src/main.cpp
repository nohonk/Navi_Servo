#include <Arduino.h>

//#include <Wire.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <PWMServo.h>


PWMServo myservo;
/*
   This sample sketch demonstrates the normal use of a TinyGPSPlus (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);



enum class Direction : uint8_t
{
  RECHTS = 0,
  VORNE,
  LINKS
};

class Position
{
public:
  Position(double lat, double lon)
  {
    lat_ = lat;
    lon_ = lon;
  }

  void setPosition(double lat, double lon)
  {
    lat_ = lat;
    lon_ = lon;
  }

  void setPosition(Position pos)
  {
    lat_ = pos.lat();
    lon_ = pos.lon();
  }

  double lat()
  {
    return lat_;
  }

  double lon()
  {
    return lon_;
  }

private:
  double lat_;
  double lon_;
};

class Waypoint : public Position
{
public:
  Waypoint(double lat, double lon, Direction direction) : Position(lat, lon)
  {
    direction_ = direction;
  }

  Direction getDirection()
  {
    return direction_;
  }

  void setWayPoint(Waypoint waypoint)
  {
    direction_ = waypoint.getDirection();
    setPosition(waypoint.lat(), waypoint.lon());
  }
 
private:
  Direction direction_;
};

Waypoint lastTarget(0.0,0.0,Direction::VORNE);
Waypoint nextTarget(0.0,0.0,Direction::VORNE);
Position currentPosition(0.0,0.0);

const int numberOfWaypoints = 3;

Waypoint WegPunkt[numberOfWaypoints] =
{
  {52.225321, 10.571335, Direction::LINKS},
  {52.225265,10.57084, Direction::RECHTS},
  {52.226474, 10.570037, Direction::LINKS}
};

int targetIdx = 1;
const int numberOfWege = 11;
// TESTDATA
Position weg[12] =
{
  {52.225321, 10.571335},
  {52.225281, 10.571107},
  {52.225278, 10.570899},
  {52.225372, 10.570768},
  {52.225575, 10.570672},
  {52.225855, 10.570548},
  {52.226154, 10.570399},
  {52.226357, 10.57024},
  {52.226462, 10.57009},
  {52.226432, 10.569987},
  {52.226349, 10.569916},
  {52.226227, 10.569791} 

};

int testidx = 0;

enum class State : uint8_t
{
  AUF_DEM_WEG = 0,
  AM_WEGPUNKT,
  HINTERM_WEGPUNKT
};

State state = State::AUF_DEM_WEG; 
double lastDistanceToNext;
double lastDistanceToLast;

void setup()
{
  Serial.begin(9600);
  ss.begin(GPSBaud);
  myservo.attach(9);

  myservo.write(90);
  nextTarget.setWayPoint(WegPunkt[1]);
  lastTarget.setWayPoint(WegPunkt[0]);
Serial.println("STARTING");
}


void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0)
  {
    if (gps.encode(ss.read()))
    {
      Serial.print(F("Location: ")); 
      if (gps.location.isValid())
      {
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(","));
        Serial.print(gps.location.lng(), 6);
        currentPosition.setPosition(gps.location.lat(), gps.location.lng());
      }
      else
      {
        Serial.print(F("INVALID"));
      }
    }
  }
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }

  double distanceToNext =
    TinyGPSPlus::distanceBetween(
      currentPosition.lat(),
      currentPosition.lon(),
      nextTarget.lat(), 
      nextTarget.lon());

    double distanceToLast =
    TinyGPSPlus::distanceBetween(
      currentPosition.lat(),
      currentPosition.lon(),
      lastTarget.lat(), 
      lastTarget.lon());  

  Serial.print("Abstand in Metern zu Wegpunkt Nr. ");
  Serial.print(targetIdx);
  Serial.print(" : ");
  Serial.println(distanceToNext);  

  switch (state)
  {
    case State::AUF_DEM_WEG:
      if (distanceToNext < 18.0)
      { 
        state = State::AM_WEGPUNKT;
          
      }
      break;
    case State::AM_WEGPUNKT:
      Serial.println((uint8_t)nextTarget.getDirection());
      myservo.write((uint8_t)nextTarget.getDirection() * 90);
      
      if (targetIdx < numberOfWaypoints-1)
      {
        lastTarget.setWayPoint(WegPunkt[targetIdx]);
        targetIdx++;
        nextTarget.setWayPoint(WegPunkt[targetIdx]);
      }
      state = State::HINTERM_WEGPUNKT;
      break;
    case State::HINTERM_WEGPUNKT:

      //TODO Absichern, dass ich in der richtigen Richtung vom Wegpunkt erntfernt wird.
      // GGf. Wegpunkte zurücksetzen
      // ggf. zwischenwegpunkt mit geradeausrichtung einführen um abstände eindeutiger zu machen.
      // Evtl. den Abstand zu den Wegpunkten als INformation dazugeben, damit man nicht weiter weg muss als der nächste wegpunkt weg ist.

      if (distanceToLast > 10.0) 
      {
        myservo.write((uint8_t)Direction::VORNE * 90);
        state = State::AUF_DEM_WEG;
      }
      break;
    default:
      break;
  }

  if ( testidx < numberOfWege)
  {
    testidx++;
  }

}


#if 0

void setup() {
//  myservo.attach(9);

//  myservo.write(90);

 
}


int gpsSearchPos = 0;
uint8_t gpsSearchDir = 0;

void loop() {
  
          myservo.write(gpsSearchPos);
          gpsSearchPos = gpsSearchDir == 0 ? gpsSearchPos+1 : gpsSearchPos-1;
          if (gpsSearchPos > 180) gpsSearchDir = 1;
          if (gpsSearchPos < 0) gpsSearchDir = 0;
          myservo.write(gpsSearchPos); 
              
        Serial.println(F("- location: INVALID"));
      }

#endif
#if 0
  currentPosition.setPosition(weg[testidx]);

  double distanceToNext =
    TinyGPSPlus::distanceBetween(
      currentPosition.lat(),
      currentPosition.lon(),
      nextTarget.lat(), 
      nextTarget.lon());

    double distanceToLast =
    TinyGPSPlus::distanceBetween(
      currentPosition.lat(),
      currentPosition.lon(),
      lastTarget.lat(), 
      lastTarget.lon());  

    Serial.print("Abstand in Metern zu Wegpunkt Nr. ");
    Serial.print(targetIdx);
    Serial.print(" : ");
    Serial.println(distanceToNext);  

    switch (state)
    {
      case State::AUF_DEM_WEG:
        if (distanceToNext < 10.0)
        { 
          state = State::AM_WEGPUNKT;
           
        }
      break;
      case State::AM_WEGPUNKT:
          Serial.println((uint8_t)nextTarget.getDirection());
          myservo.write((uint8_t)nextTarget.getDirection() * 90);
          
          if (targetIdx < numberOfWaypoints-1)
          {
            lastTarget.setWayPoint(WegPunkt[targetIdx]);
            targetIdx++;
            nextTarget.setWayPoint(WegPunkt[targetIdx]);
          }
          state = State::HINTERM_WEGPUNKT;
      break;
      case State::HINTERM_WEGPUNKT:

      //TODO Absichern, dass ich in der richtigen Richtung vom Wegpunkt erntfernt wird.
      // GGf. Wegpunkte zurücksetzen
      // ggf. zwischenwegpunkt mit geradeausrichtung einführen um abstände eindeutiger zu machen.
      // Evtl. den Abstand zu den Wegpunkten als INformation dazugeben, damit man nicht weiter weg muss als der nächste wegpunkt weg ist.


         // if ((distanceToNext < lastDistanceToNext) && (distanceToLast > lastDistanceToLast))
         // {
            if (distanceToLast > 5.0) {
              myservo.write((uint8_t)Direction::VORNE * 90);
              state = State::AUF_DEM_WEG;
            }
         // }

      break;
      default:
      break;

    }

    lastDistanceToLast = distanceToLast;
    lastDistanceToNext = distanceToNext;

    delay(2000);
    if ( testidx < numberOfWege)
    {
      testidx++;
    }

}
 #endif
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