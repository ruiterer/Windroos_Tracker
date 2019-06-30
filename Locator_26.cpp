// This #include statement was automatically added by the Particle IDE.
#include "ThingSpeak.h"

// This #include statement was automatically added by the Particle IDE.
#include <ThingSpeak.h>



/*
  *****************************************************************************************
  **** Libraries to be included
  *****************************************************************************************/

// ThingSpeak.com library
#include <ThingSpeak.h>

// AssetTracker libary from Particle.io
// #include "AssetTracker.h"


#include "Adafruit_GPS.h"
#include "Adafruit_LIS3DH.h"
#include "GPS_Math.h"

#include <math.h>
#include "math.h"
#include <ctype.h>




// Define the GPS, Accelerometer and battery fuel gauge
#define mySerial Serial1
Adafruit_GPS GPS(&mySerial);
Adafruit_LIS3DH accel = Adafruit_LIS3DH(A2, A5, A4, A3);


// A FuelGauge named 'fuel' for checking on the battery state
FuelGauge fuel;

// Name of the AssetTracker
#define MY_NAME "AssetTracker"

// What is this threshhold for??
#define CLICKTHRESHHOLD 100


// lets keep the radio off until we get a fix, or 2 minutes go by.
SYSTEM_MODE(SEMI_AUTOMATIC);


STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));






/*
  *****************************************************************************************
  **** Global parameters
  *****************************************************************************************/

// Not sure what this is for?
int lastSecond = 0;

// State of the blue led on the Electron
bool ledState = false;


unsigned long lastMotion = 0;
unsigned long lastPublish2 = 0;
time_t lastIdleCheckin = 0;

#define PUBLISH_DELAY (60 * 1000)

// if no motion for 3 minutes, sleep! (milliseconds)
#define NO_MOTION_IDLE_SLEEP_DELAY (3 * 60 * 1000)

// lets wakeup every 6 hours and check in (seconds)
#define HOW_LONG_SHOULD_WE_SLEEP (6 * 60 * 60)

// when we wakeup from deep-sleep not as a result of motion,
// how long should we wait before we publish our location?
// lets set this to less than our sleep time, so we always idle check in.
// (seconds)
#define MAX_IDLE_CHECKIN_DELAY (HOW_LONG_SHOULD_WE_SLEEP - 60)


// Set whether you want the device to publish data to the internet by default here.
// 1 will Particle.publish AND Serial.print, 0 will just Serial.print
// Extremely useful for saving data while developing close enough to have a cable plugged in.
// You can also change this remotely using the Particle.function "tmode" defined in setup()
int transmittingData = 1;

// Used to keep track of the last time we published data
long lastPublish = 0;

// Used to measure period for avarage calculation
long lastAverage = 0;

// Used as an array index counter for the average calculation
int averageIndex = 0;

// How many minutes between publishes? 10+ recommended for long-time continuous publishing!
int delayMinutes = 0; // Delay to be used. Will be adjusted depending on the circumstances
int delayMinutesDirect = 0;
int delayMinutesFast = 5;
int delayMinutesMedium = 30;
int delayMinutesSlow = 180;

// Time period in minutes to calculate average and max speed and acceleration
int periodCalcAverage = 1;

// Above this threshhold the asset it assumed to be moving
float speedThreshhold = 1.5;
bool movingStatus = false;

// Above this threshhold the asset it assumed to have undergone an accelleration
float accellThreshhold = 1.5;
bool accellStatus = false;

// Above this threshhold the asset it assumed to be outside of its geofencing area
float geoThreshhold = 100;
bool inGeoFenceStatus = true;

// The radius of the geofence.
float geoFenceDistance = 50;
   
   
float assetSpeed = 0;
float assetElevation = 0;
float assetAccellXYZ = 0;


// Arrays to store the speed over time at different levels
float arraySpeedLevel1[10];
float arraySpeedLevel2[10];
float arraySpeedLevel3[10];
int a = 0;
int b = 0;
int c = 0;
float averageSpeedLevel1 = 0;
float averageSpeedLevel2 = 0;
float averageSpeedLevel3 = 0;
int speedArrayLength = 10;      //Length of 15 is roughly 30 seconds

// Max value parameters
float maxAssetSpeed = 0;
float maxAssetAccell = 0;

// Parameter to store the results of a split string. Lat Lon
String sParams[2];

// A TCPClient for communication
TCPClient client;


// Instantiate the harbour lat and lon for geofencing purposes
// Fill harbour lat lon with the Kampen harbour location: Kattewaardweg 4, 8267 AJ, Kampen
float latGeoFence = 52.584173;
float lonGeoFence = 5.869283;

// Almere Kornetstraat 32
// float latGeoFence = 52.374294;
// float lonGeoFence = 5.177971;



//  https://www.thingspeak.com parameters
unsigned long myChannelNumber = 282816;
const char * myWriteAPIKey = "7WIBVZ83XIZOO22V";    // When you share the code, don't share the API key!!!!!!!


/*
  *****************************************************************************************
  **** setup() runs once when the device starts and is used for registering functions
  **** and variables and initializing things
  *****************************************************************************************/
void setup() {


    lastMotion = 0;
    lastPublish = 0;

    // electron asset tracker shield needs this to enable the power to the gps module.
    pinMode(D6, OUTPUT);
    digitalWrite(D6, LOW);

    // for blinking.
    pinMode(D7, OUTPUT);
    digitalWrite(D7, LOW);

    // wait a little for the GPS to wakeup
    delay(250);

    GPS.begin(9600);
    mySerial.begin(9600);
    Serial.begin(9600);


    //# request a HOT RESTART, in case we were in standby mode before.
    GPS.sendCommand("$PMTK101*32");
    delay(250);

    // request everything!
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
    delay(250);

    // turn off antenna updates
    GPS.sendCommand(PGCMD_NOANTENNA);
    delay(250);
    
    // Start the accelerometer
    initAccel();

    
    // Start the ThingSpeak process
    ThingSpeak.begin(client);

    // These four functions are useful for remote diagnostics and setting. Read more below.
    Particle.function("tmode", transmitMode);
    Particle.function("battData", batteryStatus);
    Particle.function("gpsData", gpsPublish);
    Particle.function("setAccThrHld", setAccelThreshhold);
    Particle.function("setFenceCrd", setGeoFenceCoord);
    Particle.function("setFenceDist", setGeoFenceDistance);
    
    
    Particle.function("setSpdThrHld", setSpeedThreshhold);
    Particle.function("setGeoThrHld", setGeoThreshhold); 
    
    Particle.function("setDelayFast", setDelayMinutesFast);
    Particle.function("setDelayMed", setDelayMinutesMedium);
    Particle.function("setDelaySlow", setDelayMinutesSlow);
    
    Particle.function("reset", cloudResetFunction);
    
};


/*
  *****************************************************************************************
  **** loop() runs continuously
  *****************************************************************************************/

void loop() {


    // Store the last time that the idle state was checked
    if (lastIdleCheckin == 0) {
        lastIdleCheckin = Time.now();
    }

    unsigned long now = millis();
    
    // Not clear why this is done
    if (lastMotion > now) { lastMotion = now; }
    if (lastPublish2 > now) { lastPublish2 = now; }

    checkGPS();
    
    // Serial.println(String(GPS.fix));

    // we'll be woken by motion, lets keep listening for more motion.
    // if we get two in a row, then we'll connect to the internet and start reporting in.
    bool hasMotion = digitalRead(WKP);
    digitalWrite(D7, (hasMotion) ? HIGH : LOW);
    if (hasMotion) {
        Serial.println("BUMP!");
        lastMotion = now;

        if (Particle.connected() == false) {
            Serial.println("CONNECTING DUE TO MOTION!");
            Particle.connect();
        }
    }

    // use the real-time-clock here, instead of millis.
    if ((Time.now() - lastIdleCheckin) >= MAX_IDLE_CHECKIN_DELAY) {

        // it's been too long!  Lets say hey!
        if (Particle.connected() == false) {
            Serial.println("CONNECTING DUE TO IDLE!");
            Particle.connect();
        }

        //Particle.publish(MY_NAME + String("_status"), "miss you <3");
        Serial.println("Miss you");
        lastIdleCheckin = Time.now();
    }


    // have we published recently?
    //Serial.println("lastPublish is " + String(lastPublish));
    if (((millis() - lastPublish) > PUBLISH_DELAY) || (lastPublish2 == 0)) {
        lastPublish2 = millis();


    
        unsigned int msSinceLastMotion = (millis() - lastMotion);
        int motionInTheLastMinute = (msSinceLastMotion < 60000);
    


    }


    // Check if there is a GPS fix
    if (GPS.fix) {
        
        // Get the speed from the GPS
        assetSpeed = 1.88 * GPS.speed;  // 1.88 is a speed correction factor to get to km/h
        
        // Get the elecation from the GPS
        assetElevation = GPS.altitude;
        
        // When there is a GPS fix calculate the distance between the actual position and the geo fence
        // String strCoordinates = t.readLatLon();
        // StringSplit(strCoordinates, ',',sParams, 2);
        sParams[0]=String(convertDegMinToDecDeg(GPS.latitude));
        sParams[1]=String(convertDegMinToDecDeg(GPS.longitude));
        geoFenceDistance = distanceCoordinates(sParams[0].toFloat(),sParams[1].toFloat(), latGeoFence, lonGeoFence);
        
        // Get the XYZ Acceleration
        sensors_event_t event;
        accel.getEvent(&event);
        assetAccellXYZ = sqrt(pow(event.acceleration.x,2)+pow(event.acceleration.y,2)+pow(event.acceleration.z,2));

    }
    // If there is no GPS fix yet, use default values
    else {
        geoFenceDistance = -1;
        assetSpeed = 0;
        assetElevation = 0;
        sParams[0] = 0;
        sParams[1] = 0;
        assetAccellXYZ = 0;
        
    }
    

    // Calculate average speed which will be communicated
    // The following nested if statements calculate the the average speed at different levels
    // averageSpeedLevel1, averageSpeedLevel2 and averageSpeedLevel3
    arraySpeedLevel1[a]=assetSpeed;
    //Check if speed level 1 array is at last position
    if(a==(speedArrayLength-1)){
        // Calculate average at level 1
        averageSpeedLevel1 = average(arraySpeedLevel1, speedArrayLength);
        
        // Store the level 1 average in level 2
        arraySpeedLevel2[b] = averageSpeedLevel1;

        // Reset parameter a to 0
        a=0;
        
        //Check if speed level 2 array is at last position
        if(b==(speedArrayLength-1)){

            // Calculate average at level 2
            averageSpeedLevel2 = average(arraySpeedLevel2, speedArrayLength);
            
            // Store the level 1 average in level 2
            arraySpeedLevel3[c] = averageSpeedLevel2;
    
            // Reset parameter a to 0
            b=0;
            
            
            if(c==(speedArrayLength-1)){

                // Calculate average at level 3
                averageSpeedLevel3 = average(arraySpeedLevel3, speedArrayLength);
                
                // Reset parameter a to 0
                c=0;
                
                // No next if statement is required for further level 4 averaging
                
            }    
          
            // If c is smaller than 9 increase c with 1
            else if (c<(speedArrayLength-1)){
                c++;
            }  
            
        }    
        // If b is smaller than 9 increase b with 1
        else if (b<(speedArrayLength-1)){
            b++;
        }
    
    }
    // If a is smaller than 9 increase a with 1
    else if (a<(speedArrayLength-1)){
        a++;
    }
   
        //Serial.println("geoFenceDistance= "+String(geoFenceDistance)); 
        //Serial.println("a="+String(a)+" b="+String(b)+" c="+String(c)); 
        //Serial.println("Level 0="+String(assetSpeed)+" level 1="+String(averageSpeedLevel1)+" level 2="+String(averageSpeedLevel3)+" level 3="+String(averageSpeedLevel3));
    
    // Set a new max asset speed when it exceeds the previous max speed
    // This is required to remember the highest asset speed between communication moments
    if (assetSpeed > maxAssetSpeed) {
        maxAssetSpeed = assetSpeed;
    }
    
    // Set a new max asset accelaration when it exceeds the previous max accell
    // This is required to remember the highest asset acceleration between communication moments
    if (assetAccellXYZ > maxAssetAccell) {
        maxAssetAccell = assetAccellXYZ;
    }

    // Store the previous statuses for moving, acceleration and geofence to compare with new values
    bool previousMovingStatus = movingStatus;
    bool previousAccellStatus = accellStatus;
    bool previousInGeoFenceStatus = inGeoFenceStatus;
    
    
    // If the average speed3 is at or above the threshhold speed then it is assumed that the asset is moving
    // If the accelleration is at or above the threshhold then it is assumed that the asset accellerated
    
    
    // + - + + - - + -
    // + + - + - + - -
    // - - - + - + + +
    
    // +Speed
    // +Accel
    // -Geo
    if ((averageSpeedLevel3 >= speedThreshhold) && (maxAssetAccell >= accellThreshhold) && (geoFenceDistance >= geoThreshhold)){ 
        // No alarmThingSpeak.setField(2,m
        // Varen in ruwe omstandigheden buiten de geofence
        
        // Set the statuses
        movingStatus = true;
        accellStatus = true;
        inGeoFenceStatus = false;
        
        // Set standard communication frequency based on the circumstances
        delayMinutes = delayMinutesMedium;
    }

    // -Speed
    // +Accel
    // -Geo
    else if ((averageSpeedLevel3 < speedThreshhold) && (maxAssetAccell >= accellThreshhold) && (geoFenceDistance >= geoThreshhold)){ 
        // Alarm - immediate communication required
        // Schok tijdens stilliggen buiten de geofence
        movingStatus = false;
        accellStatus = true;
        inGeoFenceStatus = false;
        
        // Set standard communication frequency based on the circumstances
        delayMinutes = delayMinutes;
    }
    
    // +Speed
    // -Accel
    // -Geo
    else if ((averageSpeedLevel3 >= speedThreshhold) && (maxAssetAccell < accellThreshhold) && (geoFenceDistance >= geoThreshhold)){ 
        // No alarm - short term and frequent communication required
        // Varen in rustige omstandigheden buiten de geofence
        movingStatus = true;
        accellStatus = false;
        inGeoFenceStatus = false;

        // Set standard communication frequency based on the circumstances
        delayMinutes = delayMinutesMedium;
    }
    
    // +Speed
    // +Accel
    // +Geo
    else if ((averageSpeedLevel3 >= speedThreshhold) && (maxAssetAccell >= accellThreshhold) && (geoFenceDistance < geoThreshhold)){ 
        // Alarm - short term communication required - unlikely to happen
        // Varen in ruwe omstandigheden binnen de geofence. Dit kan eigenlijk niet
        movingStatus = true;
        accellStatus = true;
        inGeoFenceStatus = true;
        
        // Set standard communication frequency based on the circumstances
        delayMinutes = delayMinutesSlow;
    }
    
    // -Speed
    // -Accel
    // -Geo
    else if ((averageSpeedLevel3 < speedThreshhold) && (maxAssetAccell < accellThreshhold) && (geoFenceDistance >= geoThreshhold)){ 
        // No alarm - infrequent communication required
        // Stilliggen in rustige omstandigheden buiten de geofence
        movingStatus = false;
        accellStatus = false;
        inGeoFenceStatus = false; 

        // Set standard communication frequency based on the circumstances
        delayMinutes = delayMinutesSlow;
    }

    // -Speed
    // +Accel
    // +Geo
    else if ((averageSpeedLevel3 < speedThreshhold) && (maxAssetAccell >= accellThreshhold) && (geoFenceDistance < geoThreshhold)){ 
        // Alarm - frequent communication required
        // Schok tijdens stilliggen binnen de geofence
        movingStatus = false;
        accellStatus = true;
        inGeoFenceStatus = true;

        // Set standard communication frequency based on the circumstances
        delayMinutes = delayMinutesDirect;    
    }
    
    // +Speed
    // -Accel
    // +Geo
    else if ((averageSpeedLevel3 >= speedThreshhold) && (maxAssetAccell < accellThreshhold) && (geoFenceDistance < geoThreshhold)){ 
        // No alarm - short term and frequent communication required
        // Varen in rustige omstandigheden binnen de geofence - Dit kan maar heel even b.v in de haven
        movingStatus = true;
        accellStatus = false;
        inGeoFenceStatus = true;
        
        // Set standard communication frequency based on the circumstances
        delayMinutes = delayMinutesMedium;
    }
    
    // -Speed
    // -Accel
    // +Geo
    else if ((averageSpeedLevel3 < speedThreshhold) && (maxAssetAccell < accellThreshhold) && (geoFenceDistance < geoThreshhold)){ 
        // No alarm - short term and frequent communication required
        // Stilliggen in rustige omstandigheden binnen de geofence
        movingStatus = false;
        accellStatus = false;
        inGeoFenceStatus = true; 
        
        // Set standard communication frequency based on the circumstances
        delayMinutes = delayMinutesSlow;
    }
    
    // If a status change has occurred compared to the previous measurement then the communication frequency that has been set above might need adjustment
    // Change the communication frequency when asset starts or stops moving
    // Change the communication frequency when asset moves out or in the geofence
    if (movingStatus != previousMovingStatus || inGeoFenceStatus != previousInGeoFenceStatus) {
        // Make sure coordinates are logged when assets stops or starts moving or when the asset moves out or in the geofence
        delayMinutes = delayMinutesFast;
    }
    
     // Check if the asset just left the geofence area
    if (inGeoFenceStatus == 0  && inGeoFenceStatus != previousInGeoFenceStatus) {
        // Publish an event when asset leaves the geofence
        Particle.publish("Windroos","Verlaat geofence");
        
    }
    
        Serial.println(
            "lat:"          + String::format("%.2f",sParams[0].toFloat())   +
            ", lon:"        + String::format("%.2f",sParams[1].toFloat())   +
            ", alt:"        + String::format("%.2f",assetElevation)         +
            ", speed:"      + String::format("%.2f",assetSpeed)             +
            ", avspeed:"    + String::format("%.2f",averageSpeedLevel3)     +
            ", maxspeed:"   + String::format("%.2f",maxAssetSpeed)          +
            ", accell:"     + String::format("%.2f",assetAccellXYZ)         +
            ", maxaccell:"  + String::format("%.2f",maxAssetAccell)         +
            ", fencedist:"  + String::format("%.2f",geoFenceDistance)       +
            ", gpsfix:"     + String(GPS.fix)                               +
            ", gpssat:"     + String(GPS.satellites)                        +
            ", movsts:"     + String(movingStatus)                          +
            ", accellsts:"  + String(accellStatus)                          +
            ", fencests:"   + String(inGeoFenceStatus)                      +
            ", batvol:"     + String(fuel.getVCell())                       +
            ", soc:"        + String(fuel.getSoC())                         +
            ", delay:"      + String(delayMinutes)                          +
            ", s_m_f:"      + String(delayMinutesSlow) + "_" + String(delayMinutesMedium) + "_" + String(delayMinutesFast) 
        );
        


    // if the current time - the last time we published is greater than your set delay...
     //Serial.println(millis()-lastPublish);
    if (millis()-lastPublish > delayMinutes*60*1000) {

        // Dumps the full NMEA sentence to serial in case you're curious
        //Serial.println(t.preNMEA());

        // GPS requires a "fix" on the satellites to give good data,
        // if (GPS.fix) {   // Optional if statement to only transmit data when there is a gps fix
            
            // Only publish if we're in transmittingData mode 1;
            if (transmittingData) {

                // Set the ThingSpeak fields
                ThingSpeak.setField(1,assetSpeed);
                ThingSpeak.setField(2,averageSpeedLevel3);
                ThingSpeak.setField(3,maxAssetSpeed);
                ThingSpeak.setField(4,maxAssetAccell);
                ThingSpeak.setField(5,geoFenceDistance);
                ThingSpeak.setField(6,movingStatus);
                ThingSpeak.setField(7,accellStatus);
                ThingSpeak.setField(8,inGeoFenceStatus);
                
                ThingSpeak.setLatitude(sParams[0].toFloat());       // Latitude field should be of type Float
                ThingSpeak.setLongitude(sParams[1].toFloat());     // Longitude field should be of type Float
                ThingSpeak.setElevation(assetElevation);
                // Write the data to thing speak
                ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);  
            

                        
                
                // Reset the MaxSpeed and MaxAccell variables to 0 because after communication a new period starts until the next communication
                maxAssetSpeed = 0;
                maxAssetAccell = 0;
                
                // Reset the publish frequency
                delayMinutes = delayMinutesSlow;
                
                // Remember when we published
                lastPublish = millis();
                
                // After communication due to an event, the event aftermath should not immediately trigger a new communication
                delay(5000);
                
            }
        // } // Closing bracket for gps.fix if statement
    }
};



// Allows you to remotely change whether a device is publishing to the cloud
// or is only reporting data over Serial. Saves data when using only Serial!
// Change the default at the top of the code.
int transmitMode(String command) {
    transmittingData = atoi(command);
    return 1;
};

// Allows you to remotely change the acceleration threshhold
// Change the default at the top of the code.
int setAccelThreshhold(String command) {
    accellThreshhold = atoi(command);
    return 1;
};

// Allows you to remotely change the speed threshhold which indicates whether the asset is moving
// Change the default at the top of the code.
int setSpeedThreshhold(String command) {
    speedThreshhold = atoi(command);
    return 1;
};

// Allows you to remotely change the geo fence threshhold which indicates the size of the fence radius
// Change the default at the top of the code.
int setGeoThreshhold(String command) {
    geoThreshhold = atoi(command);
    return 1;
};

// Allows you to remotely change the number of delay minutes under Fast conditions
// Change the default at the top of the code.
int setDelayMinutesFast(String command) {
    delayMinutesFast = atoi(command);
    return 1;
};

// Allows you to remotely change the number of delay minutes under Medium conditions
// Change the default at the top of the code.
int setDelayMinutesMedium(String command) {
    delayMinutesMedium = atoi(command);
    return 1;
};

// Allows you to remotely change the number of delay minutes under Slow conditions
// Change the default at the top of the code.
int setDelayMinutesSlow(String command) {
    delayMinutesSlow = atoi(command);
    return 1;
};



// Allows you to remotely set the geofence location
// Change the default at the top of the code.
int setGeoFenceCoord(String command) {
    latGeoFence = sParams[0].toFloat();
    lonGeoFence = sParams[1].toFloat();
    return 1;
};

// Allows you to remotely change the size of the geofence
// Change the default at the top of the code.
int setGeoFenceDistance(String command) {
    geoFenceDistance = atoi(command);
    return 1;
};

// A function to reset the Particle Electron via the Cloud
int cloudResetFunction(String command) {
    // Reset the system
    System.reset();
    return 1;
};


// Actively ask for a GPS reading if you're impatient. Only publishes if there's
// a GPS fix, otherwise returns '0'
int gpsPublish(String command) {
    
   // Set the ThingSpeak fields
    ThingSpeak.setField(1,assetSpeed);
    ThingSpeak.setField(2,averageSpeedLevel3);
    ThingSpeak.setField(3,maxAssetSpeed);
    ThingSpeak.setField(4,maxAssetAccell);
    ThingSpeak.setField(5,geoFenceDistance);
    ThingSpeak.setField(6,movingStatus);
    ThingSpeak.setField(7,accellStatus);
    ThingSpeak.setField(8,inGeoFenceStatus);
    
    ThingSpeak.setLatitude(sParams[0].toFloat());       // Latitude field should be of type Float
    ThingSpeak.setLongitude(sParams[1].toFloat());     // Longitude field should be of type Float
    ThingSpeak.setElevation(assetElevation);
    // Write the data to thing speak
    ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);  


    Serial.println(
        "lat:"          + String::format("%.2f",sParams[0].toFloat())   +
        ", lon:"        + String::format("%.2f",sParams[1].toFloat())   +
        ", alt:"        + String::format("%.2f",assetElevation)         +
        ", speed:"      + String::format("%.2f",assetSpeed)             +
        ", avspeed:"    + String::format("%.2f",averageSpeedLevel3)     +
        ", maxspeed:"   + String::format("%.2f",maxAssetSpeed)          +
        ", accell:"     + String::format("%.2f",assetAccellXYZ)         +
        ", maxaccell:"  + String::format("%.2f",maxAssetAccell)         +
        ", fencedist:"  + String::format("%.2f",geoFenceDistance)       +
        ", gpsfix:"     + String(GPS.fix)                               +
        ", gpssat:"     + String(GPS.satellites)                        +
        ", movsts:"     + String(movingStatus)                          +
        ", accellsts:"  + String(accellStatus)                          +
        ", fencests:"   + String(inGeoFenceStatus)                      +
        ", batvol:"     + String(fuel.getVCell())                       +
        ", soc:"        + String(fuel.getSoC())                         +
        ", delay:"      + String(delayMinutes)                          +
        ", s_m_f:"      + String(delayMinutesSlow) + "_" + String(delayMinutesMedium) + "_" + String(delayMinutesFast) 
    );


    Particle.publish("GPS",
        "lat:"          + String::format("%.2f",sParams[0].toFloat())   +
        ", lon:"        + String::format("%.2f",sParams[1].toFloat())   +
        ", alt:"        + String::format("%.2f",assetElevation)         +
        ", speed:"      + String::format("%.2f",assetSpeed)             +
        ", avspeed:"    + String::format("%.2f",averageSpeedLevel3)     +
        ", maxspeed:"   + String::format("%.2f",maxAssetSpeed)          +
        ", accell:"     + String::format("%.2f",assetAccellXYZ)         +
        ", maxaccell:"  + String::format("%.2f",maxAssetAccell)         +
        ", fencedist:"  + String::format("%.2f",geoFenceDistance)       +
        ", gpsfix:"     + String(GPS.fix)                               +
        ", gpssat:"     + String(GPS.satellites)                        +
        ", movsts:"     + String(movingStatus)                          +
        ", accellsts:"  + String(accellStatus)                          +
        ", fencests:"   + String(inGeoFenceStatus)                      +
        ", batvol:"     + String(fuel.getVCell())                       +
        ", soc:"        + String(fuel.getSoC())                         +
        ", delay:"      + String(delayMinutes)                          +
        ", s_m_f:"      + String(delayMinutesSlow) + "_" + String(delayMinutesMedium) + "_" + String(delayMinutesFast) ,
          60, PRIVATE
    );


    // Reset the MaxSpeed and MaxAccell variables to 0 because after communication a new period starts until the next communication
    maxAssetSpeed = 0;
    maxAssetAccell = 0;
    
    // Reset the publish frequency
    delayMinutes = delayMinutesSlow;

    // Remember when we published
    lastPublish = millis();

    if(GPS.fix) {
        return 1;
    }
    else {
        return 0;
    }
    
};


// Lets you remotely check the battery status by calling the function "batt"
// Triggers a publish with the info (so subscribe or watch the dashboard)
// and also returns a '1' if there's >10% battery left and a '0' if below
int batteryStatus(String command){
    // Publish the battery voltage and percentage of battery remaining
    // if you want to be really efficient, just report one of these
    // the String::format("%f.2") part gives us a string to publish,
    // but with only 2 decimal points to save space
    Particle.publish("B",
          "v:" + String::format("%.2f",fuel.getVCell()) +
          ",c:" + String::format("%.2f",fuel.getSoC()),
          60, PRIVATE
    );
    // if there's more than 10% of the battery left, then return 1
    if (fuel.getSoC()>10){ return 1;}
    // if you're running out of battery, return 0
    else { return 0;}
};


int StringSplit(String sInput, char cDelim, String sParams[], int iMaxParams)
{
    int iParamCount = 0;
    int iPosDelim, iPosStart = 0;

    do {
        // Searching the delimiter using indexOf()
        iPosDelim = sInput.indexOf(cDelim,iPosStart);
        if (iPosDelim > (iPosStart+1)) {
            // Adding a new parameter using substring() 
            sParams[iParamCount] = sInput.substring(iPosStart,iPosDelim-1);
            iParamCount++;
            // Checking the number of parameters
            if (iParamCount >= iMaxParams) {
                return (iParamCount);
            }
            iPosStart = iPosDelim + 1;
        }
    } while (iPosDelim >= 0);
    if (iParamCount < iMaxParams) {
        // Adding the last parameter as the end of the line
        sParams[iParamCount] = sInput.substring(iPosStart);
        iParamCount++;
    }

    return (iParamCount);
};



// Calculate distance between two points
float distanceCoordinates(float flat1, float flon1, float flat2, float flon2) {

  // Variables
  float dist_calc=0;
  float dist_calc2=0;
  float diflat=0;
  float diflon=0;

  // Calculations
  diflat  = radians(flat2-flat1);
  flat1 = radians(flat1);
  flat2 = radians(flat2);
  diflon = radians((flon2)-(flon1));

  dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
  dist_calc2 = cos(flat1);
  dist_calc2*=cos(flat2);
  dist_calc2*=sin(diflon/2.0);
  dist_calc2*=sin(diflon/2.0);
  dist_calc +=dist_calc2;

  dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));
  
  dist_calc*=6371000.0; //Converting to meters

  return dist_calc;
};


// Convert degrees to radians
float radians(float degrees) {

    // Variables
    float rad=0;

    // Calculations
    rad  = (degrees*3.14159265359)/180;
    
    // return the radians
    return rad;
};


//Calculate average in array
float average (float * array, int len)  // assuming array is float.
{
  long sum = 0L ;  // sum will be larger than an item, long for safety.
  for (int i = 0 ; i < len ; i++)
    sum += array [i] ;
  return  ((float) sum) / len ;  // average will be fractional, so float may be appropriate.
};


void checkGPS() {
    // process and dump everything from the module through the library.
    while (mySerial.available()) {
        char c = GPS.read();

        // lets echo the GPS output until we get a good clock reading, then lets calm things down.
        //if (!hasGPSTime) {
        //   Serial.print(c);
        //}

        if (GPS.newNMEAreceived()) {
            GPS.parse(GPS.lastNMEA());
        }
    }
}

// Start the acceleration sensor function
void initAccel() {
    accel.begin(LIS3DH_DEFAULT_ADDRESS);

    // Default to 5kHz low-power sampling
    accel.setDataRate(LIS3DH_DATARATE_LOWPOWER_5KHZ);

    // Default to 4 gravities range
    accel.setRange(LIS3DH_RANGE_4_G);

    // listen for single-tap events at the threshold
    // keep the pin high for 1s, wait 1s between clicks

    //uint8_t c, uint8_t clickthresh, uint8_t timelimit, uint8_t timelatency, uint8_t timewindow
    accel.setClick(1, CLICKTHRESHHOLD);//, 0, 100, 50);
}

