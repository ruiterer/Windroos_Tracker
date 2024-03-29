/*
  ThingSpeak(TM) Communication Library For Particle

  Enables Particle hardware to write or read data to or from ThingSpeak,
  an open data platform for the Internet of Things with MATLAB analytics and visualization. 

  ThingSpeak ( https://www.thingspeak.com ) is an analytic IoT platform service that allows you to aggregate, visualize and analyze live data streams in the cloud.
  
  Copyright 2017, The MathWorks, Inc.
 
  See the accompaning licence file for licensing information.
*/

/**
  @mainpage
 * 
 * \ref ThingSpeakClass "For technical documentation, visit this page"
 * 
 * ThingSpeak offers free data storage and analysis of time-stamped numeric or alphanumeric data.
 * Users can access ThingSpeak by visiting http://thingspeak.com and creating a ThingSpeak user account.
 *
 * ThingSpeak stores data in channels.  Channels support an unlimited number of timestamped observations (think of these as rows in a spreadsheet).  
 * Each channel has up to 8 fields (think of these as columns in a speadsheet).  Check out this <a href="http://www.mathworks.com/videos/introduction-to-thingspeak-107749.html">video</a> for an overview.
 * 
 * Channels may be public, where anyone can see the data, or private, where only the owner and select users can read the data.
 * Each channel has an associated Write API Key that is used to control who can write to a channel.  
 * In addition, private channels have one or more Read API Keys to control who can read from private channel.  
 * An API Key is not required to read from public channels.  Each channel can have up to 8 fields. One field is created by default.
 *
 * You can visualize and do online analytics of your data on ThingSpeak using the built in version of MATLAB, or use the desktop version of MATLAB to get
 * deeper historical insight.  Visit https://www.mathworks.com/hardware-support/thingspeak.html to learn more.
 * 
 * <h3>Compatible Hardware</h3>
 * * Particle (Formally Spark) Core, <a href="https://www.particle.io/prototype#photon">Photon</a>, <a href="https://www.particle.io/prototype#electron">Electron</a> and <a href="https://www.particle.io/prototype#p0-and-p1">P1</a>
 * 
 * <h3>Examples</h3>
 * The library includes several examples to help you get started.  These are accessible in ThingSpeak library section of the Particle Web IDE.
 * * <b>CheerLights:</b> Reads the latest <a href="http://www.cheerlights.com">CheerLights</a> color on ThingSpeak, and sets an RGB LED.
 * * <b>ReadLastTemperature:</b> Reads the latest temperature from the public <a href="https://thingspeak.com/channels/12397">MathWorks weather station</a> in Natick, MA on ThingSpeak.
 * * <b>ReadPrivateChannel:</b> Reads the latest voltage value from a private channel on ThingSpeak.
 * * <b>ReadWeatherStation:</b> Reads the latest weather data from the public <a href="https://thingspeak.com/channels/12397">MathWorks weather station</a> in Natick, MA on ThingSpeak.
 * * <b>WriteMultipleVoltages:</b> Reads analog voltages from pins 0-7 and writes them to the 8 fields of a channel on ThingSpeak.
 * * <b>WriteVoltage:</b> Reads an analog voltage from pin 0, converts to a voltage, and writes it to a channel on ThingSpeak.
 */

#ifndef ThingSpeak_h
#define ThingSpeak_h

//#define PRINT_DEBUG_MESSAGES
//#define PRINT_HTTP


// Create platform defines for Particle devices
#if PLATFORM_ID == 0
	#define PARTICLE_CORE
#elif PLATFORM_ID == 6
	#define PARTICLE_PHOTON
	#define PARTICLE_PHOTONELECTRON
#elif PLATFORM_ID == 8
	#define PARTICLE_P1
	#define PARTICLE_PHOTONELECTRON
#elif PLATFORM_ID == 10
	#define PARTICLE_ELECTRON
	#define PARTICLE_PHOTONELECTRON
#else
	#error Only Spark Core/Photon/Electron/P1 are supported.
#endif


#include "math.h"
#include "application.h"
#ifdef PARTICLE_PHOTONELECTRON
	extern char* dtoa(double val, unsigned char prec, char *sout);
	// On spark photon, There is no itoa, so map to ltoa.
	#include "string_convert.h"
	#define itoa ltoa
#else
	// On spark core, a long and an int are equivalent, and so there's no "ltoa" function defined.  Map it to itoa.
	extern char * itoa(int a, char* buffer, unsigned char radix);
	#define ltoa itoa
	extern char *dtostrf (double val, signed char width, unsigned char prec, char *sout);
#endif


#define THINGSPEAK_URL "api.thingspeak.com"
#define THINGSPEAK_IPADDRESS IPAddress(184,106,153,149)
#define THINGSPEAK_PORT_NUMBER 80


#ifdef PARTICLE_CORE
	#define TS_USER_AGENT "tslib-arduino/1.3 (particle core)"
#elif defined(PARTICLE_PHOTON)
	#define TS_USER_AGENT "tslib-arduino/1.3 (particle photon)"
#elif defined(PARTICLE_ELECTRON)
	#define TS_USER_AGENT "tslib-arduino/1.3 (particle electron)"
#elif defined(PARTICLE_P1)
	#define TS_USER_AGENT "tslib-arduino/1.3 (particle p1)"
#endif
#define SPARK_PUBLISH_TTL 60 // Spark "time to live" for published messages
#define SPARK_PUBLISH_TOPIC "thingspeak-debug"


#define FIELDNUM_MIN 1
#define FIELDNUM_MAX 8
#define FIELDLENGTH_MAX 255  // Max length for a field in ThingSpeak is 255 bytes (UTF-8)

#define TIMEOUT_MS_SERVERRESPONSE 5000  // Wait up to five seconds for server to respond

#define OK_SUCCESS              200     // OK / Success
#define ERR_BADAPIKEY           400     // Incorrect API key (or invalid ThingSpeak server address)
#define ERR_BADURL              404     // Incorrect API key (or invalid ThingSpeak server address)
#define ERR_OUT_OF_RANGE        -101    // Value is out of range or string is too long (> 255 bytes)
#define ERR_INVALID_FIELD_NUM   -201    // Invalid field number specified
#define ERR_SETFIELD_NOT_CALLED -210    // setField() was not called before writeFields()
#define ERR_CONNECT_FAILED      -301    // Failed to connect to ThingSpeak
#define ERR_UNEXPECTED_FAIL     -302    // Unexpected failure during write to ThingSpeak
#define ERR_BAD_RESPONSE        -303    // Unable to parse response
#define ERR_TIMEOUT             -304    // Timeout waiting for server to respond
#define ERR_NOT_INSERTED        -401    // Point was not inserted (most probable cause is the rate limit of once every 15 seconds)

/**
 * @brief Particle hardware to write or read data to or from ThingSpeak, an open data platform for the Internet of Things with MATLAB analytics and visualization. 
 */
class ThingSpeakClass
{
  public:
	ThingSpeakClass()
	{
		resetWriteFields();
	    this->lastReadStatus = OK_SUCCESS;
	};

	/**
	 * @brief Initializes the ThingSpeak library and network settings using a custom installation of ThingSpeak.
	 * @param client TCPClient created earlier in the sketch
	 * @param customHostName Host name of a custom install of ThingSpeak
	 * @param port Port number to use with a custom install of ThingSpeak
	 * @return Always returns true
     * @comment This does not validate the information passed in, or generate any calls to ThingSpeak.
	 * @code

		#include "ThingSpeak.h"
		TCPClient client;

		void setup() {
		  ThingSpeak.begin(client,"api.thingspeak.com", 80);
		}
	 * @endcode
	 */
    bool begin(Client & client, const char * customHostName, unsigned int port)
	{
#ifdef PRINT_DEBUG_MESSAGES
		Particle.publish(SPARK_PUBLISH_TOPIC, "ts::tsBegin    (client: Client URL: " + String(customHostName) + ")", SPARK_PUBLISH_TTL, PRIVATE);
#endif
	this->setClient(&client);
	this->setServer(customHostName, port);
	resetWriteFields();
	this->lastReadStatus = OK_SUCCESS;
	return true;
	};

	/**
	 * @brief Initializes the ThingSpeak library and network settings using a custom installation of ThingSpeak.
	 * @param client TCPClient created earlier in the sketch
	 * @param customIP IP address of a custom install of ThingSpeak
	 * @param port Port number to use with a custom install of ThingSpeak
	 * @return Always returns true
     * @comment This does not validate the information passed in, or generate any calls to ThingSpeak.
	 * @code

		#include "ThingSpeak.h"
		TCPClient client;

		void setup() {
		  ThingSpeak.begin(client,IPAddress(184,106,153,149), 80);
		}
	 * @endcode
	 */
	bool begin(Client & client, IPAddress customIP, unsigned int port)
	{
#ifdef PRINT_DEBUG_MESSAGES
	Particle.publish(SPARK_PUBLISH_TOPIC, "ts::tsBegin    (client: Client IP: " + String(customIP) + ")" , SPARK_PUBLISH_TTL, PRIVATE);
#endif
		this->setClient(&client);
		this->setServer(customIP, port);
		resetWriteFields();
		this->lastReadStatus = OK_SUCCESS;
		return true;
	};

	/**
	 * @brief Initializes the ThingSpeak library and network settings using the ThingSpeak.com service.
	 * @param client TCPClient created earlier in the sketch
	 * @return Always returns true
     * @comment This does not validate the information passed in, or generate any calls to ThingSpeak.
	 * @code

		#include "ThingSpeak.h"
		TCPClient client;

		void setup() {
		  ThingSpeak.begin(client);
		}
	 * @endcode
	 */
	bool begin(Client & client)
	{
#ifdef PRINT_DEBUG_MESSAGES
		Particle.publish(SPARK_PUBLISH_TOPIC, "ts::tsBegin" , SPARK_PUBLISH_TTL, PRIVATE);
#endif
		this->setClient(&client);
		this->setServer();
		resetWriteFields();
		this->lastReadStatus = OK_SUCCESS;
		return true;
	};
	
	/**
	 * @brief Write an integer value to a single field in a ThingSpeak channel
	 * @param channelNumber Channel number
	 * @param field Field number (1-8) within the channel to write to.
	 * @param value Integer value (from -32,768 to 32,767) to write.
	 * @param writeAPIKey Write API key associated with the channel.  *If you share code with others, do _not_ share this key*
	 * @return HTTP status code of 200 if successful.  See getLastReadStatus() for other possible return values.
	 * @remark Visit https://thingspeak.com/docs/channels for more information about channels, API keys, and fields.  ThingSpeak limits the number of writes to a channel to once every 15 seconds.
	 * @code
		void loop() {
			int sensorValue = analogRead(A0);
			ThingSpeak.writeField(myChannelNumber, 1, sensorValue, myWriteAPIKey);
			delay(20000);
		}
	 * @endcode
	 */
	int writeField(unsigned long channelNumber, unsigned int field, int value, const char * writeAPIKey)
	{
		// On Spark, int and long are the same, so map to the long version
		return writeField(channelNumber, field, (long)value, writeAPIKey);
	};

	/**
	 * @brief Write a long value to a single field in a ThingSpeak channel
	 * @param channelNumber Channel number
	 * @param field Field number (1-8) within the channel to write to.
	 * @param value Long value (from -2,147,483,648 to 2,147,483,647) to write.
	 * @param writeAPIKey Write API key associated with the channel.  *If you share code with others, do _not_ share this key*
	 * @return HTTP status code of 200 if successful.  See getLastReadStatus() for other possible return values.
	 * @remark Visit https://thingspeak.com/docs/channels for more information about channels, API keys, and fields.  ThingSpeak limits the number of writes to a channel to once every 15 seconds.
	 * @code
		void loop() {
			int sensorValue = analogRead(A0);
			ThingSpeak.writeField(myChannelNumber, 1, sensorValue, myWriteAPIKey);
			delay(20000);
		}
	 * @endcode
	 */
	int writeField(unsigned long channelNumber, unsigned int field, long value, const char * writeAPIKey)
	{
		char valueString[15];  // long range is -2147483648 to 2147483647, so 12 bytes including terminator
   		ltoa(value, valueString, 10);
		return writeField(channelNumber, field, valueString, writeAPIKey);
	};

	/**
	 * @brief Write a floating point value to a single field in a ThingSpeak channel
	 * @param channelNumber Channel number
	 * @param field Field number (1-8) within the channel to write to.
	 * @param value Floating point value (from -999999000000 to 999999000000) to write.  If you need more accuracy, or a wider range, you should format the number using <tt>dtostrf</tt> and writeField().
	 * @param writeAPIKey Write API key associated with the channel.  *If you share code with others, do _not_ share this key*
	 * @return HTTP status code of 200 if successful.  See getLastReadStatus() for other possible return values.
	 * @remark Visit https://thingspeak.com/docs/channels for more information about channels, API keys, and fields.  ThingSpeak limits the number of writes to a channel to once every 15 seconds.
	 * @code
		void loop() {
			int sensorValue = analogRead(A0);
			float voltage = sensorValue * (3.3 / 4095.0);
			ThingSpeak.writeField(myChannelNumber, 1, voltage, myWriteAPIKey);
			delay(20000);
		}
	 * @endcode
	 */
	int writeField(unsigned long channelNumber, unsigned int field, float value, const char * writeAPIKey)
	{
		#ifdef PRINT_DEBUG_MESSAGES
		Particle.publish(SPARK_PUBLISH_TOPIC, "ts::writeField (channelNumber: " + String(channelNumber) + " writeAPIKey: " + String(writeAPIKey) + " field: " + String(field) + " value: " + String(value,5) + ")" , SPARK_PUBLISH_TTL, PRIVATE);
		#endif
		char valueString[20]; // range is -999999000000.00000 to 999999000000.00000, so 19 + 1 for the terminator
		int status = convertFloatToChar(value, valueString);
		if(status != OK_SUCCESS) return status;

		return writeField(channelNumber, field, valueString, writeAPIKey);
	};

	/**
	 * @brief Write a string to a single field in a ThingSpeak channel
	 * @param channelNumber Channel number
	 * @param field Field number (1-8) within the channel to write to.
	 * @param value String to write (UTF8 string).  ThingSpeak limits this field to 255 bytes.
	 * @param writeAPIKey Write API key associated with the channel.  *If you share code with others, do _not_ share this key*
	 * @return HTTP status code of 200 if successful.  See getLastReadStatus() for other possible return values.
	 * @remark Visit https://thingspeak.com/docs/channels for more information about channels, API keys, and fields.  ThingSpeak limits the number of writes to a channel to once every 15 seconds.
	 * @code
		void loop() {
			int sensorValue = analogRead(A0);
			if (sensorValue > 512) {
				ThingSpeak.writeField(myChannelNumber, 1, "High", myWriteAPIKey);
			}
			else {
				ThingSpeak.writeField(myChannelNumber, 1, "Low", myWriteAPIKey);
			}
			delay(20000);
		}
	 * @endcode
	 */
	int writeField(unsigned long channelNumber, unsigned int field, const char * value, const char * writeAPIKey)
	{
		return writeField(channelNumber, field, String(value), writeAPIKey);
	};

	/**
	 * @brief Write a String to a single field in a ThingSpeak channel
	 * @param channelNumber Channel number
	 * @param field Field number (1-8) within the channel to write to.
	 * @param value Character array (zero terminated) to write (UTF8).  ThingSpeak limits this field to 255 bytes.
	 * @param writeAPIKey Write API key associated with the channel.  *If you share code with others, do _not_ share this key*
	 * @return HTTP status code of 200 if successful.  See getLastReadStatus() for other possible return values.
	 * @remark Visit https://thingspeak.com/docs/channels for more information about channels, API keys, and fields.  ThingSpeak limits the number of writes to a channel to once every 15 seconds.
	 * @code
		void loop() {
			int sensorValue = analogRead(A0);
			String meaning;
			if (sensorValue < 400) {
				meaning = String("Too Cold!");
			} else if (sensorValue > 600) {
				meaning = String("Too Hot!");
			} else {
				meaning = String("Just Right");
			}
			ThingSpeak.writeField(myChannelNumber, 1, meaning, myWriteAPIKey);
			delay(20000);
		}
	 * @endcode
	 */
	int writeField(unsigned long channelNumber, unsigned int field, String value, const char * writeAPIKey)
	{
		// Invalid field number specified
		if(field < FIELDNUM_MIN || field > FIELDNUM_MAX) return ERR_INVALID_FIELD_NUM;
		// Max # bytes for ThingSpeak field is 255
		if(value.length() > FIELDLENGTH_MAX) return ERR_OUT_OF_RANGE;
		
		#ifdef PRINT_DEBUG_MESSAGES
			Particle.publish(SPARK_PUBLISH_TOPIC, "writeField (" + String(channelNumber) + ", " + String(writeAPIKey) + ", " + String(field) + ", " + String(value) + ")", SPARK_PUBLISH_TTL, PRIVATE);
		#endif
		String postMessage = String("field") + String(field) + "=" + value;
		return writeRaw(channelNumber, postMessage, writeAPIKey);
 	};

    
	/**
	 * @brief Set the value of a single field that will be part of a multi-field update.
	 * To write multiple fields at once, call setField() for each of the fields you want to write, and then call writeFields()
	 * @param field Field number (1-8) within the channel to set
	 * @param value Integer value (from -32,768 to 32,767) to set.
	 * @return HTTP status code of 200 if successful.  See getLastReadStatus() for other possible return values.
	 * @see setLatitude(), setLongitude(), setElevation(), writeFields()
	 * @code
		void loop() {
			int sensor1Value = analogRead(A0);
			float sensor2Voltage = analogRead(A1) * (3.3 / 4095.0);
			String sensor3Meaning;
			int sensor3Value = analogRead(A2);
			if (sensor3Value < 400) {
				sensor3Meaning = String("Too Cold!");
			} else if (sensor3Value > 600) {
				sensor3Meaning = String("Too Hot!");
			} else {
				sensor3Meaning = String("Just Right");
			}
			long timeRead = millis();

			ThingSpeak.setField(1, sensor1Value);
			ThingSpeak.setField(2, sensor2Voltage);
			ThingSpeak.setField(3, sensor3Meaning);
			ThingSpeak.setField(4, timeRead);
			ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
			delay(20000);
		}
	 * @endcode
	 */
	int setField(unsigned int field, int value)
	{
        // On Spark, int and long are the same, so map to the long version
        return setField(field, (long)value);
	};

	/**
	 * @brief Set the value of a single field that will be part of a multi-field update.
	 * To write multiple fields at once, call setField() for each of the fields you want to write, and then call writeFields()
	 * @param field Field number (1-8) within the channel to set
	 * @param value Long value (from -2,147,483,648 to 2,147,483,647) to write.
	 * @return HTTP status code of 200 if successful.  See getLastReadStatus() for other possible return values.
	 * @see setLatitude(), setLongitude(), setElevation(), writeFields()
	 * @code
		void loop() {
			int sensor1Value = analogRead(A0);
			float sensor2Voltage = analogRead(A1) * (3.3 / 4095.0);
			String sensor3Meaning;
			int sensor3Value = analogRead(A2);
			if (sensor3Value < 400) {
				sensor3Meaning = String("Too Cold!");
			} else if (sensor3Value > 600) {
				sensor3Meaning = String("Too Hot!");
			} else {
				sensor3Meaning = String("Just Right");
			}
			long timeRead = millis();

			ThingSpeak.setField(1, sensor1Value);
			ThingSpeak.setField(2, sensor2Voltage);
			ThingSpeak.setField(3, sensor3Meaning);
			ThingSpeak.setField(4, timeRead);
			ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
			delay(20000);
		}
	 * @endcode
	 */
	int setField(unsigned int field, long value)
	{
		char valueString[15];  // long range is -2147483648 to 2147483647, so 12 bytes including terminator
	    ltoa(value, valueString, 10);
		return setField(field, valueString);
	};


	/**
	 * @brief Set the value of a single field that will be part of a multi-field update.
	 * To write multiple fields at once, call setField() for each of the fields you want to write, and then call writeFields()
	 * @param field Field number (1-8) within the channel to set
	 * @param value Floating point value (from -999999000000 to 999999000000) to write.  If you need more accuracy, or a wider range, you should format the number yourself (using <tt>dtostrf</tt>) and setField() using the resulting string.
	 * @return HTTP status code of 200 if successful.  See getLastReadStatus() for other possible return values.
	 * @see setLatitude(), setLongitude(), setElevation(), writeFields()
	 * @code
		void loop() {
			int sensor1Value = analogRead(A0);
			float sensor2Voltage = analogRead(A1) * (3.3 / 4095.0);
			String sensor3Meaning;
			int sensor3Value = analogRead(A2);
			if (sensor3Value < 400) {
				sensor3Meaning = String("Too Cold!");
			} else if (sensor3Value > 600) {
				sensor3Meaning = String("Too Hot!");
			} else {
				sensor3Meaning = String("Just Right");
			}
			long timeRead = millis();

			ThingSpeak.setField(1, sensor1Value);
			ThingSpeak.setField(2, sensor2Voltage);
			ThingSpeak.setField(3, sensor3Meaning);
			ThingSpeak.setField(4, timeRead);
			ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
			delay(20000);
		}
	 * @endcode
	 */
    int setField(unsigned int field, float value)
	{
		char valueString[20]; // range is -999999000000.00000 to 999999000000.00000, so 19 + 1 for the terminator
		int status = convertFloatToChar(value, valueString);
		if(status != OK_SUCCESS) return status;

		return setField(field, valueString);
	};

	/**
	 * @brief Set the value of a single field that will be part of a multi-field update.
	 * To write multiple fields at once, call setField() for each of the fields you want to write, and then call writeFields()
	 * @param field Field number (1-8) within the channel to set
	 * @param value String to write (UTF8).  ThingSpeak limits this to 255 bytes.  
	 * @return HTTP status code of 200 if successful.  See getLastReadStatus() for other possible return values.
	 * @see setLatitude(), setLongitude(), setElevation(), writeFields()
	 * @code
		void loop() {
			int sensor1Value = analogRead(A0);
			float sensor2Voltage = analogRead(A1) * (3.3 / 4095.0);
			String sensor3Meaning;
			int sensor3Value = analogRead(A2);
			if (sensor3Value < 400) {
				sensor3Meaning = String("Too Cold!");
			} else if (sensor3Value > 600) {
				sensor3Meaning = String("Too Hot!");
			} else {
				sensor3Meaning = String("Just Right");
			}
			long timeRead = millis();

			ThingSpeak.setField(1, sensor1Value);
			ThingSpeak.setField(2, sensor2Voltage);
			ThingSpeak.setField(3, sensor3Meaning);
			ThingSpeak.setField(4, timeRead);
			ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
			delay(20000);
		}
	 * @endcode
	 */
    int setField(unsigned int field, const char * value)
	{
		return setField(field, String(value));
	};

	/**
	 * @brief Set the value of a single field that will be part of a multi-field update.
	 * To write multiple fields at once, call setField() for each of the fields you want to write, and then call writeFields()
	 * @param field Field number (1-8) within the channel to set
	 * @param value String to write (UTF8).  ThingSpeak limits this to 255 bytes.
	 * @return HTTP status code of 200 if successful.  See getLastReadStatus() for other possible return values.
	 * @see setLatitude(), setLongitude(), setElevation(), writeFields()
	 * @code
		void loop() {
			int sensor1Value = analogRead(A0);
			float sensor2Voltage = analogRead(A1) * (3.3 / 4095.0);
			String sensor3Meaning;
			int sensor3Value = analogRead(A2);
			if (sensor3Value < 400) {
				sensor3Meaning = String("Too Cold!");
			} else if (sensor3Value > 600) {
				sensor3Meaning = String("Too Hot!");
			} else {
				sensor3Meaning = String("Just Right");
			}
			long timeRead = millis();

			ThingSpeak.setField(1, sensor1Value);
			ThingSpeak.setField(2, sensor2Voltage);
			ThingSpeak.setField(3, sensor3Meaning);
			ThingSpeak.setField(4, timeRead);
			ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
			delay(20000);
		}
	 * @endcode
	 */
    int setField(unsigned int field, String value)
	{
		#ifdef PRINT_DEBUG_MESSAGES
            Particle.publish(SPARK_PUBLISH_TOPIC, "setField " + String(field) + " to " + String(value), SPARK_PUBLISH_TTL, PRIVATE);
		#endif
		if(field < FIELDNUM_MIN || field > FIELDNUM_MAX) return ERR_INVALID_FIELD_NUM;
		// Max # bytes for ThingSpeak field is 255 (UTF-8)
		if(value.length() > FIELDLENGTH_MAX) return ERR_OUT_OF_RANGE;
		this->nextWriteField[field - 1] = value;
		return OK_SUCCESS;
	};


	/**
	 * @brief Set the latitude of a multi-field update.
	 * To record latitude, longitude and elevation of a write, call setField() for each of the fields you want to write, setLatitude() / setLongitude() / setElevation(), and then call writeFields()
	 * @param latitude Latitude of the measurement (degrees N, use negative values for degrees S)
	 * @return HTTP status code of 200 if successful.  See getLastReadStatus() for other possible return values.
	 * @see setField(), setLongitude(), setElevation(), writeFields()
	 * @code
		void loop() {
			int sensor1Value = analogRead(A0);
			float sensor2Voltage = analogRead(A1) * (3.3 / 4095.0);
			String sensor3Meaning;
			int sensor3Value = analogRead(A2);
			if (sensor3Value < 400) {
				sensor3Meaning = String("Too Cold!");
			} else if (sensor3Value > 600) {
				sensor3Meaning = String("Too Hot!");
			} else {
				sensor3Meaning = String("Just Right");
			}
			long timeRead = millis();

			ThingSpeak.setField(1, sensor1Value);
			ThingSpeak.setField(2, sensor2Voltage);
			ThingSpeak.setField(3, sensor3Meaning);
			ThingSpeak.setField(4, timeRead);
			ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
			setLatitude(42.2833);
			setLongitude(-71.3500);
			setElevation(100);
			delay(20000);
		}
	 * @endcode
	 */
	int setLatitude(float latitude)
	{
		#ifdef PRINT_DEBUG_MESSAGES
			Particle.publish(SPARK_PUBLISH_TOPIC, "ts::setLatitude(latitude: " + String(latitude,3) + "\")" , SPARK_PUBLISH_TTL, PRIVATE);
		#endif
		this->nextWriteLatitude = latitude;
		return OK_SUCCESS;
	};


	/**
	 * @brief Set the longitude of a multi-field update.
	 * To record latitude, longitude and elevation of a write, call setField() for each of the fields you want to write, setLatitude() / setLongitude() / setElevation(), and then call writeFields()
	 * @param longitude Longitude of the measurement (degrees E, use negative values for degrees W)
	 * @return HTTP status code of 200 if successful.  See getLastReadStatus() for other possible return values.
	 * @see setField(), setLatitude(), setElevation(), writeFields()
	 * @code
		void loop() {
			int sensor1Value = analogRead(A0);
			float sensor2Voltage = analogRead(A1) * (3.3 / 4095.0);
			String sensor3Meaning;
			int sensor3Value = analogRead(A2);
			if (sensor3Value < 400) {
				sensor3Meaning = String("Too Cold!");
			} else if (sensor3Value > 600) {
				sensor3Meaning = String("Too Hot!");
			} else {
				sensor3Meaning = String("Just Right");
			}
			long timeRead = millis();

			ThingSpeak.setField(1, sensor1Value);
			ThingSpeak.setField(2, sensor2Voltage);
			ThingSpeak.setField(3, sensor3Meaning);
			ThingSpeak.setField(4, timeRead);
			setLatitude(42.2833);
			setLongitude(-71.3500);
			setElevation(100);
			ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
			delay(20000);
		}
	 * @endcode
	 */
	int setLongitude(float longitude)
	{
		#ifdef PRINT_DEBUG_MESSAGES
			Particle.publish(SPARK_PUBLISH_TOPIC, "ts::setLongitude(longitude: " + String(longitude,3) + "\")" , SPARK_PUBLISH_TTL, PRIVATE);
		#endif
		this->nextWriteLongitude = longitude;
		return OK_SUCCESS;
	};


	/**
	 * @brief Set the elevation of a multi-field update.
	 * To record latitude, longitude and elevation of a write, call setField() for each of the fields you want to write, setLatitude() / setLongitude() / setElevation(), and then call writeFields()
	 * @param elevation Elevation of the measurement (meters above sea level)
	 * @return HTTP status code of 200 if successful.  See getLastReadStatus() for other possible return values.
	 * @see setField(), setLatitude(), setLongitude(), writeFields()
	 * @code
		void loop() {
			int sensor1Value = analogRead(A0);
			float sensor2Voltage = analogRead(A1) * (3.3 / 4095.0);
			String sensor3Meaning;
			int sensor3Value = analogRead(A2);
			if (sensor3Value < 400) {
				sensor3Meaning = String("Too Cold!");
			} else if (sensor3Value > 600) {
				sensor3Meaning = String("Too Hot!");
			} else {
				sensor3Meaning = String("Just Right");
			}
			long timeRead = millis();

			ThingSpeak.setField(1, sensor1Value);
			ThingSpeak.setField(2, sensor2Voltage);
			ThingSpeak.setField(3, sensor3Meaning);
			ThingSpeak.setField(4, timeRead);
			setLatitude(42.2833);
			setLongitude(-71.3500);
			setElevation(100);
			ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
			delay(20000);
		}
	 * @endcode
	 */
	int setElevation(float elevation)
	{
		#ifdef PRINT_DEBUG_MESSAGES
			Particle.publish(SPARK_PUBLISH_TOPIC, "ts::setElevation(elevation: " + String(elevation,3) + "\")" , SPARK_PUBLISH_TTL, PRIVATE);
		#endif
		this->nextWriteElevation = elevation;
		return OK_SUCCESS;
	};
	
	/**
	 * @brief Set the status of a multi-field update.
	 * To record a status message on a write, call setStatus() then call writeFields(). Use status to provide additonal 
	 * details when writing a channel update.  Additonally, status can be used by the ThingTweet App to send a message to
	 * Twitter.
	 * @param status String to write (UTF8).  ThingSpeak limits this to 255 bytes.
	 * @return HTTP status code of 200 if successful.  See getLastReadStatus() for other possible return values.
	 * @see writeFields()
	 * @code
		void loop() {
			int sensor1Value = analogRead(A0);
			float sensor2Voltage = analogRead(A1) * (3.3 / 4095.0);
			String sensor3Meaning;
			int sensor3Value = analogRead(A2);
			if (sensor3Value < 400) {
				sensor3Meaning = String("Too Cold!");
			} else if (sensor3Value > 600) {
				sensor3Meaning = String("Too Hot!");
			} else {
				sensor3Meaning = String("Just Right");
			}
			long timeRead = millis();

			ThingSpeak.setField(1, sensor1Value);
			ThingSpeak.setField(2, sensor2Voltage);
			ThingSpeak.setField(3, timeRead);
			ThingSpeak.setStatus(sensor3Meaning);
			ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
			delay(20000);
		}
	 * @endcode
	 */
	
    int setStatus(const char * status)
	{
		return setStatus(String(status));
	};

	/**
	 * @brief Set the status of a multi-field update.
	 * To record a status message on a write, call setStatus() then call writeFields(). Use status to provide additonal 
	 * details when writing a channel update.  Additonally, status can be used by the ThingTweet App to send a message to
	 * Twitter.
	 * @param status String to write (UTF8).  ThingSpeak limits this to 255 bytes.
	 * @return HTTP status code of 200 if successful.  See getLastReadStatus() for other possible return values.
	 * @see writeFields()
	 * @code
		void loop() {
			int sensor1Value = analogRead(A0);
			float sensor2Voltage = analogRead(A1) * (3.3 / 4095.0);
			String sensor3Meaning;
			int sensor3Value = analogRead(A2);
			if (sensor3Value < 400) {
				sensor3Meaning = String("Too Cold!");
			} else if (sensor3Value > 600) {
				sensor3Meaning = String("Too Hot!");
			} else {
				sensor3Meaning = String("Just Right");
			}
			long timeRead = millis();

			ThingSpeak.setField(1, sensor1Value);
			ThingSpeak.setField(2, sensor2Voltage);
			ThingSpeak.setField(3, timeRead);
			ThingSpeak.setStatus(sensor3Meaning);
			ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
			delay(20000);
		}
	 * @endcode
	 */
    int setStatus(String status)
	{
		#ifdef PRINT_DEBUG_MESSAGES
			Particle.publish(SPARK_PUBLISH_TOPIC, "ts::setStatus(status: " + status + "\")" , SPARK_PUBLISH_TTL, PRIVATE);
		#endif
		// Max # bytes for ThingSpeak field is 255 (UTF-8)
		if(status.length() > FIELDLENGTH_MAX) return ERR_OUT_OF_RANGE;
		this->nextWriteStatus = status;
		return OK_SUCCESS;
	};

	/**
	 * @brief Set the Twitter account and message to use for an update to be tweeted.
	 * To send a message to twitter call setTwitterTweet() then call writeFields()
	 * @param twitter Twitter account name as a String.
	 * @param tweet Twitter message as a String (UTF-8) limited to 140 character.
	 * @return HTTP status code of 200 if successful.  See getLastReadStatus() for other possible return values.
	 * @remark Prior to using this feature, a twitter account must be linked to your ThingSpeak account. Do this by logging into ThingSpeak and going to Apps, then ThingTweet and clicking Link Twitter Account.
	 * @see writeFields(),getLastReadStatus()
	 * @code
		void loop() {
			int sensor1Value = analogRead(A0);
			float sensor2Voltage = analogRead(A1) * (5.0 / 1023.0);
			String sensor3Meaning;
			int sensor3Value = analogRead(A2);
			if (sensor3Value < 400) {
				sensor3Meaning = String("Too Cold!");
			} else if (sensor3Value > 600) {
				sensor3Meaning = String("Too Hot!");
			} else {
				sensor3Meaning = String("Just Right");
			}
			long timeRead = millis();

			ThingSpeak.setField(1, sensor1Value);
			ThingSpeak.setField(2, sensor2Voltage);
			ThingSpeak.setField(3, timeRead);
			ThingSpeak.setTwitterTweet("YourTwitterAccountName",sensor3Meaning);
			ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
			delay(20000);	
		}
	 * @endcode
	 */	
	int setTwitterTweet(const char * twitter, const char * tweet)
	{
		return setTwitterTweet(String(twitter), String(tweet));
	};

	/**
	 * @brief Set the Twitter account and message to use for an update to be tweeted.
	 * To send a message to twitter call setTwitterTweet() then call writeFields()
	 * @param twitter Twitter account name as a String.
	 * @param tweet Twitter message as a String (UTF-8) limited to 140 character.
	 * @return HTTP status code of 200 if successful.  See getLastReadStatus() for other possible return values.
	 * @remark Prior to using this feature, a twitter account must be linked to your ThingSpeak account. Do this by logging into ThingSpeak and going to Apps, then ThingTweet and clicking Link Twitter Account.
	 * @see writeFields(),getLastReadStatus()
	 * @code
		void loop() {
			int sensor1Value = analogRead(A0);
			float sensor2Voltage = analogRead(A1) * (5.0 / 1023.0);
			String sensor3Meaning;
			int sensor3Value = analogRead(A2);
			if (sensor3Value < 400) {
				sensor3Meaning = String("Too Cold!");
			} else if (sensor3Value > 600) {
				sensor3Meaning = String("Too Hot!");
			} else {
				sensor3Meaning = String("Just Right");
			}
			long timeRead = millis();

			ThingSpeak.setField(1, sensor1Value);
			ThingSpeak.setField(2, sensor2Voltage);
			ThingSpeak.setField(3, timeRead);
			ThingSpeak.setTwitterTweet("YourTwitterAccountName",sensor3Meaning);
			ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
			delay(20000);	
		}
	 * @endcode
	 */	
	int setTwitterTweet(String twitter, const char * tweet)
	{
		return setTwitterTweet(twitter, String(tweet));
	};

	/**
	 * @brief Set the Twitter account and message to use for an update to be tweeted.
	 * To send a message to twitter call setTwitterTweet() then call writeFields()
	 * @param twitter Twitter account name as a String.
	 * @param tweet Twitter message as a String (UTF-8) limited to 140 character.
	 * @return HTTP status code of 200 if successful.  See getLastReadStatus() for other possible return values.
	 * @remark Prior to using this feature, a twitter account must be linked to your ThingSpeak account. Do this by logging into ThingSpeak and going to Apps, then ThingTweet and clicking Link Twitter Account.
	 * @see writeFields(),getLastReadStatus()
	 * @code
		void loop() {
			int sensor1Value = analogRead(A0);
			float sensor2Voltage = analogRead(A1) * (5.0 / 1023.0);
			String sensor3Meaning;
			int sensor3Value = analogRead(A2);
			if (sensor3Value < 400) {
				sensor3Meaning = String("Too Cold!");
			} else if (sensor3Value > 600) {
				sensor3Meaning = String("Too Hot!");
			} else {
				sensor3Meaning = String("Just Right");
			}
			long timeRead = millis();

			ThingSpeak.setField(1, sensor1Value);
			ThingSpeak.setField(2, sensor2Voltage);
			ThingSpeak.setField(3, timeRead);
			ThingSpeak.setTwitterTweet("YourTwitterAccountName",sensor3Meaning);
			ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
			delay(20000);	
		}
	 * @endcode
	 */	
	int setTwitterTweet(const char * twitter, String tweet)
	{
		return setTwitterTweet(String(twitter), tweet);
	};

	/**
	 * @brief Set the Twitter account and message to use for an update to be tweeted.
	 * To send a message to twitter call setTwitterTweet() then call writeFields()
	 * @param twitter Twitter account name as a String.
	 * @param tweet Twitter message as a String (UTF-8) limited to 140 character.
	 * @return HTTP status code of 200 if successful.  See getLastReadStatus() for other possible return values.
	 * @remark Prior to using this feature, a twitter account must be linked to your ThingSpeak account. Do this by logging into ThingSpeak and going to Apps, then ThingTweet and clicking Link Twitter Account.
	 * @see writeFields(),getLastReadStatus()
	 * @code
		void loop() {
			int sensor1Value = analogRead(A0);
			float sensor2Voltage = analogRead(A1) * (5.0 / 1023.0);
			String sensor3Meaning;
			int sensor3Value = analogRead(A2);
			if (sensor3Value < 400) {
				sensor3Meaning = String("Too Cold!");
			} else if (sensor3Value > 600) {
				sensor3Meaning = String("Too Hot!");
			} else {
				sensor3Meaning = String("Just Right");
			}
			long timeRead = millis();

			ThingSpeak.setField(1, sensor1Value);
			ThingSpeak.setField(2, sensor2Voltage);
			ThingSpeak.setField(3, timeRead);
			ThingSpeak.setTwitterTweet("YourTwitterAccountName",sensor3Meaning);
			ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
			delay(20000);	
		}
	 * @endcode
	 */	
	int setTwitterTweet(String twitter, String tweet){
		#ifdef PRINT_DEBUG_MESSAGES
			Particle.publish(SPARK_PUBLISH_TOPIC, "ts::setTwitter(twitter: " + twitter + ", tweet: " + tweet + ")" , SPARK_PUBLISH_TTL, PRIVATE);
		#endif
		// Max # bytes for ThingSpeak field is 255 (UTF-8)
		if((twitter.length() > FIELDLENGTH_MAX) || (tweet.length() > FIELDLENGTH_MAX)) return ERR_OUT_OF_RANGE;
		
		this->nextWriteTwitter = twitter;
		this->nextWriteTweet = tweet;
		
		return OK_SUCCESS;	
	};	
	
	/**
	 * @brief Set the created-at date of a multi-field update.
	 * To record created-at of a write, call setField() for each of the fields you want to write, setCreatedAt(), and then call writeFields()
	 * @param createdAt Desired timestamp to be included with the channel update as a String.  The timestamp string must be in the ISO 8601 format. Example "2017-01-12 13:22:54"
	 * @return HTTP status code of 200 if successful.  See getLastReadStatus() for other possible return values.
	 * @remark Timezones can be set using the timezone hour offset parameter. For example, a timestamp for Eastern Standard Time is: "2017-01-12 13:22:54-05".  If no timezone hour offset parameter is used, UTC time is assumed.
	 * @see setField(), writeFields()
	 * @code
		void loop() {
			int sensor1Value = analogRead(A0);
			float sensor2Voltage = analogRead(A1) * (3.3 / 4095.0);
			String sensor3Meaning;
			int sensor3Value = analogRead(A2);
			if (sensor3Value < 400) {
				sensor3Meaning = String("Too Cold!");
			} else if (sensor3Value > 600) {
				sensor3Meaning = String("Too Hot!");
			} else {
				sensor3Meaning = String("Just Right");
			}
			long timeRead = millis();

			ThingSpeak.setField(1, sensor1Value);
			ThingSpeak.setField(2, sensor2Voltage);
			ThingSpeak.setField(3, sensor3Meaning);
			ThingSpeak.setField(4, timeRead);
			ThingSpeak.setCreatedAt("2017-01-06T13:56:28");
			ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
			delay(20000);
		}
	 * @endcode
	 */

	 int setCreatedAt(const char * createdAt)
	{
		return setCreatedAt(String(createdAt));
	}
	
    /**
	 * @brief Set the created-at date of a multi-field update.
	 * To record created-at of a write, call setField() for each of the fields you want to write, setCreatedAt(), and then call writeFields()
	 * @param createdAt Desired timestamp to be included with the channel update as a String.  The timestamp string must be in the ISO 8601 format. Example "2017-01-12 13:22:54"
	 * @return HTTP status code of 200 if successful.  See getLastReadStatus() for other possible return values.
	 * @remark Timezones can be set using the timezone hour offset parameter. For example, a timestamp for Eastern Standard Time is: "2017-01-12 13:22:54-05".  If no timezone hour offset parameter is used, UTC time is assumed.
	 * @see setField(), writeFields()
	 * @code
		void loop() {
			int sensor1Value = analogRead(A0);
			float sensor2Voltage = analogRead(A1) * (3.3 / 4095.0);
			String sensor3Meaning;
			int sensor3Value = analogRead(A2);
			if (sensor3Value < 400) {
				sensor3Meaning = String("Too Cold!");
			} else if (sensor3Value > 600) {
				sensor3Meaning = String("Too Hot!");
			} else {
				sensor3Meaning = String("Just Right");
			}
			long timeRead = millis();

			ThingSpeak.setField(1, sensor1Value);
			ThingSpeak.setField(2, sensor2Voltage);
			ThingSpeak.setField(3, sensor3Meaning);
			ThingSpeak.setField(4, timeRead);
			ThingSpeak.setCreatedAt("2017-01-06T13:56:28");
			ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
			delay(20000);
		}
	 * @endcode
	 */	
	int setCreatedAt(String createdAt)
	{
		#ifdef PRINT_DEBUG_MESSAGES
			Particle.publish(SPARK_PUBLISH_TOPIC, "ts::setCreatedAt(createdAt: " + createdAt + "\")" , SPARK_PUBLISH_TTL, PRIVATE);
		#endif
		
		// the ISO 8601 format is too complicated to check for valid timestamps here
		// we'll need to reply on the api to tell us if there is a problem
		// Max # bytes for ThingSpeak field is 255 (UTF-8)
		if(createdAt.length() > FIELDLENGTH_MAX) return ERR_OUT_OF_RANGE;
		this->nextWriteCreatedAt = createdAt;
		
		return OK_SUCCESS;
	}
	


	/**
	 * @brief Write a multi-field update.
	 * Call setField() for each of the fields you want to write, setLatitude() / setLongitude() / setElevation(), and then call writeFields()
	 * @param channelNumber Channel number
	 * @param writeAPIKey Write API key associated with the channel.  *If you share code with others, do _not_ share this key*
	 * @return HTTP status code of 200 if successful.  See getLastReadStatus() for other possible return values.
	 * @see setField(), setLatitude(), setLongitude(), setElevation()
	 * @code
		void loop() {
			int sensor1Value = analogRead(A0);
			float sensor2Voltage = analogRead(A1) * (3.3 / 4095.0);
			String sensor3Meaning;
			int sensor3Value = analogRead(A2);
			if (sensor3Value < 400) {
				sensor3Meaning = String("Too Cold!");
			} else if (sensor3Value > 600) {
				sensor3Meaning = String("Too Hot!");
			} else {
				sensor3Meaning = String("Just Right");
			}
			long timeRead = millis();

			ThingSpeak.setField(1, sensor1Value);
			ThingSpeak.setField(2, sensor2Voltage);
			ThingSpeak.setField(3, sensor3Meaning);
			ThingSpeak.setField(4, timeRead);
			setLatitude(42.2833);
			setLongitude(-71.3500);
			setElevation(100);
			ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
			delay(20000);
		}
	 * @endcode
	 */
	int writeFields(unsigned long channelNumber, const char * writeAPIKey)
	{
		String postMessage = String("");
		bool fFirstItem = true;
		for(size_t iField = 0; iField < 8; iField++)
		{
			if(this->nextWriteField[iField].length() > 0)
			{
				if(!fFirstItem)
				{
					postMessage = postMessage + String("&");
				}
				postMessage = postMessage + String("field") + String(iField + 1) + String("=") + this->nextWriteField[iField];
				fFirstItem = false;
				this->nextWriteField[iField] = "";
			}
		}

		if(!isnan(nextWriteLatitude))
		{
			if(!fFirstItem)
			{
				postMessage = postMessage + String("&");
			}
			postMessage = postMessage + String("lat=") + String(this->nextWriteLatitude);
			fFirstItem = false;
			this->nextWriteLatitude = NAN;
		}

		if(!isnan(this->nextWriteLongitude))
		{
			if(!fFirstItem)
			{
				postMessage = postMessage + String("&");
			}
			postMessage = postMessage + String("long=") + String(this->nextWriteLongitude);
			fFirstItem = false;
			this->nextWriteLongitude = NAN;
		}


		if(!isnan(this->nextWriteElevation))
		{
			if(!fFirstItem)
			{
				postMessage = postMessage + String("&");
			}
			postMessage = postMessage + String("elevation=") + String(this->nextWriteElevation);
			fFirstItem = false;
			this->nextWriteElevation = NAN;
		}
		
		if(this->nextWriteStatus.length() > 0)
		{
			if(!fFirstItem)
			{
				postMessage = postMessage + String("&");
			}
			postMessage = postMessage + String("status=") + String(this->nextWriteStatus);
			fFirstItem = false;
			this->nextWriteStatus = "";
		}
		
		if(this->nextWriteTwitter.length() > 0)
		{
			if(!fFirstItem)
			{
				postMessage = postMessage + String("&");
			}
			postMessage = postMessage + String("twitter=") + String(this->nextWriteTwitter);
			fFirstItem = false;
			this->nextWriteTwitter = "";
		}
		
		if(this->nextWriteTweet.length() > 0)
		{
			if(!fFirstItem)
			{
				postMessage = postMessage + String("&");
			}
			postMessage = postMessage + String("tweet=") + String(this->nextWriteTweet);
			fFirstItem = false;
			this->nextWriteTweet = "";
		}
		
		if(this->nextWriteCreatedAt.length() > 0)
		{
			if(!fFirstItem)
			{
				postMessage = postMessage + String("&");
			}
			postMessage = postMessage + String("created_at=") + String(this->nextWriteCreatedAt);
			fFirstItem = false;
			this->nextWriteCreatedAt = "";
		}
		
		
		if(fFirstItem)
		{
			// setField was not called before writeFields
			return ERR_SETFIELD_NOT_CALLED;
		}

		return writeRaw(channelNumber, postMessage, writeAPIKey);
	};


	/**
	 * @brief Write a raw POST to a ThingSpeak channel
	 * @param channelNumber Channel number
	 * @param postMessage Raw URL to write to ThingSpeak as a string.  See the documentation at https://thingspeak.com/docs/channels#update_feed.
	 * @param writeAPIKey Write API key associated with the channel.  *If you share code with others, do _not_ share this key*
	 * @return HTTP status code of 200 if successful.  See getLastReadStatus() for other possible return values.
	 * @remark This is low level functionality that will not be required by most users.
	 * @code
		void loop() {
			const char postMessage[] = "field1=23&created_at=2014-12-31%2023:59:59";

			ThingSpeak.setField(1, sensor1Value);
			ThingSpeak.setField(2, sensor2Voltage);
			ThingSpeak.setField(3, sensor3Meaning);
			ThingSpeak.setField(4, timeRead);
			ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
			delay(20000);
		}
	 * @endcode
	 */
	int writeRaw(unsigned long channelNumber, const char * postMessage, const char * writeAPIKey)
	{
		return writeRaw(channelNumber, String(postMessage), writeAPIKey);
	};


	/**
	 * @brief Write a raw POST to a ThingSpeak channel
	 * @param channelNumber Channel number
	 * @param postMessage Raw URL to write to ThingSpeak as a String.  See the documentation at https://thingspeak.com/docs/channels#update_feed.
	 * @param writeAPIKey Write API key associated with the channel.  *If you share code with others, do _not_ share this key*
	 * @return HTTP status code of 200 if successful.  See getLastReadStatus() for other possible return values.
	 * @remark This is low level functionality that will not be required by most users.
	 * @code
		void loop() {
			String postMessage = String("field1=23&created_at=2014-12-31%2023:59:59");
			ThingSpeak.writeRaw(myChannelNumber, postMessage, myWriteAPIKey);
			delay(20000);
		}
	 * @endcode
	 */
	int writeRaw(unsigned long channelNumber, String postMessage, const char * writeAPIKey)
	{
		#ifdef PRINT_DEBUG_MESSAGES
			Particle.publish(SPARK_PUBLISH_TOPIC, "ts::writeRaw   (channelNumber: " + String(channelNumber) + " writeAPIKey: " + String(writeAPIKey) + " postMessage: \"" + postMessage + "\")" , SPARK_PUBLISH_TTL, PRIVATE);
		#endif

		if(!connectThingSpeak())
		{
			// Failed to connect to ThingSpeak
			return ERR_CONNECT_FAILED;
		}

		postMessage = postMessage + String("&headers=false");

		#ifdef PRINT_DEBUG_MESSAGES
            Particle.publish(SPARK_PUBLISH_TOPIC, "Post " + postMessage, SPARK_PUBLISH_TTL, PRIVATE);
		#endif

		postMessage = postMessage + String("\n");

		// Post data to thingspeak
		if(!this->client->print("POST /update HTTP/1.1\r\n")) return abortWriteRaw();
		if(!writeHTTPHeader(writeAPIKey)) return abortWriteRaw();
		if(!this->client->print("Content-Type: application/x-www-form-urlencoded\r\n")) return abortWriteRaw();
		if(!this->client->print("Content-Length: ")) return abortWriteRaw();
		if(!this->client->print(postMessage.length())) return abortWriteRaw();
		if(!this->client->print("\r\n\r\n")) return abortWriteRaw();
		if(!this->client->print(postMessage)) return abortWriteRaw();
  
		String entryIDText = String();
		int status = getHTTPResponse(entryIDText);
		if(status != OK_SUCCESS)
		{
			client->stop();
			return status;
		}
		long entryID = entryIDText.toInt();

		#ifdef PRINT_DEBUG_MESSAGES
		Particle.publish(SPARK_PUBLISH_TOPIC, "               Entry ID \"" + entryIDText + "\" (" + String(entryID) + ")" , SPARK_PUBLISH_TTL, PRIVATE);
		#endif

		client->stop();
		
		#ifdef PRINT_DEBUG_MESSAGES
			Particle.publish(SPARK_PUBLISH_TOPIC, "disconnected.", SPARK_PUBLISH_TTL, PRIVATE);
		#endif
		if(entryID == 0)
		{
			// ThingSpeak did not accept the write
			status = ERR_NOT_INSERTED;
		}
		return status;
	};
	
	/**
	 * @brief Read the latest string from a private ThingSpeak channel
	 * @param channelNumber Channel number
	 * @param field Field number (1-8) within the channel to read from.
	 * @param readAPIKey Read API key associated with the channel.  *If you share code with others, do _not_ share this key*
	 * @return Value read (UTF8 string), or empty string if there is an error.  Use getLastReadStatus() to get more specific information.
	 * @code
		void loop() {
		  String message = ThingSpeak.readStringField(myChannelNumber, 1, myReadAPIKey);
		  Particle.publish("Latest message is: " + message); 
		  delay(30000);
		}
	 * @endcode
	 */
    String readStringField(unsigned long channelNumber, unsigned int field, const char * readAPIKey)
	{
		if(field < FIELDNUM_MIN || field > FIELDNUM_MAX)
		{
			this->lastReadStatus = ERR_INVALID_FIELD_NUM;
			return("");
		}
		#ifdef PRINT_DEBUG_MESSAGES
			
			if(NULL != readAPIKey)
			{
				Particle.publish(SPARK_PUBLISH_TOPIC, "ts::readStringField(channelNumber: " + String(channelNumber) + " readAPIKey: " + String(readAPIKey) + " field: " + String(field) +")", SPARK_PUBLISH_TTL, PRIVATE);
			}
			else{
				Particle.publish(SPARK_PUBLISH_TOPIC, "ts::readStringField(channelNumber: " + String(channelNumber) + " field: " + String(field) + ")", SPARK_PUBLISH_TTL, PRIVATE);	
			}
		#endif
		return readRaw(channelNumber, String(String("/fields/") + String(field) + String("/last")), readAPIKey);
	}


	/**
	 * @brief Read the latest string from a public ThingSpeak channel
	 * @param channelNumber Channel number
	 * @param field Field number (1-8) within the channel to read from.
	 * @return Value read (UTF8), or empty string if there is an error.  Use getLastReadStatus() to get more specific information.
	 * @code
		void loop() {
		  String message = ThingSpeak.readStringField(myChannelNumber, 1);
		  Particle.publish("Latest message is: " + message); 
		  delay(30000);
		}
	 * @endcode
	 */
	String readStringField(unsigned long channelNumber, unsigned int field)
	{
		return readStringField(channelNumber, field, NULL);
	};


	/**
	 * @brief Read the latest float from a private ThingSpeak channel
	 * @param channelNumber Channel number
	 * @param field Field number (1-8) within the channel to read from.
	 * @param readAPIKey Read API key associated with the channel.  *If you share code with others, do _not_ share this key*
	 * @return Value read, or 0 if the field is text or there is an error.  Use getLastReadStatus() to get more specific information.  Note that NAN, INFINITY, and -INFINITY are valid results.
	 * @code
		void loop() {
		  float voltage = ThingSpeak.readFloatField(myChannelNumber, 1, myReadAPIKey);
		  Particle.publish("Latest voltage is: " + String(voltage) + "V"); 
		  delay(30000);
		}
	 * @endcode
	 */
    float readFloatField(unsigned long channelNumber, unsigned int field, const char * readAPIKey)
	{
		return convertStringToFloat(readStringField(channelNumber, field, readAPIKey));
	};


	/**
	 * @brief Read the latest float from a public ThingSpeak channel
	 * @param channelNumber Channel number
	 * @param field Field number (1-8) within the channel to read from.
	 * @return Value read, or 0 if the field is text or there is an error.  Use getLastReadStatus() to get more specific information.  Note that NAN, INFINITY, and -INFINITY are valid results.
	 * @code
		void loop() {
		  float voltage = ThingSpeak.readFloatField(myChannelNumber, 1);
		  Particle.publish("Latest voltage is: " + String(voltage) + "V");  
		  delay(30000);
		}
	 * @endcode
	 */
    float readFloatField(unsigned long channelNumber, unsigned int field)
	{
		return readFloatField(channelNumber, field, NULL);
	};


	/**
	 * @brief Read the latest long from a private ThingSpeak channel
	 * @param channelNumber Channel number
	 * @param field Field number (1-8) within the channel to read from.
	 * @param readAPIKey Read API key associated with the channel.  *If you share code with others, do _not_ share this key*
	 * @return Value read, or 0 if the field is text or there is an error.  Use getLastReadStatus() to get more specific information.
	 * @code
		void loop() {
		  long value = ThingSpeak.readLongField(myChannelNumber, 1, myReadAPIKey);
		  Particle.publish("Latest value is: " + String(value)); 
		  delay(30000);
		}
	 * @endcode
	 */
    long readLongField(unsigned long channelNumber, unsigned int field, const char * readAPIKey)
	{
        // Note that although the function is called "toInt" it really returns a long.
		return readStringField(channelNumber, field, readAPIKey).toInt();
	}


	/**
	 * @brief Read the latest long from a public ThingSpeak channel
	 * @param channelNumber Channel number
	 * @param field Field number (1-8) within the channel to read from.
	 * @return Value read, or 0 if the field is text or there is an error.  Use getLastReadStatus() to get more specific information.
	 * @code
		void loop() {
		  long value = ThingSpeak.readLongField(myChannelNumber, 1);
		  Particle.publish("Latest value is: " + String(value)); 
		  delay(30000);
		}
	 * @endcode
	 */
	long readLongField(unsigned long channelNumber, unsigned int field)
	{
		return readLongField(channelNumber, field, NULL);
	};


	/**
	 * @brief Read the latest int from a private ThingSpeak channel
	 * @param channelNumber Channel number
	 * @param field Field number (1-8) within the channel to read from.
	 * @param readAPIKey Read API key associated with the channel.  *If you share code with others, do _not_ share this key*
	 * @return Value read, or 0 if the field is text or there is an error.  Use getLastReadStatus() to get more specific information.
	 * @remark If the value returned is out of range for an int, the result is undefined.
	 * @code
		void loop() {
		  int value = ThingSpeak.readIntField(myChannelNumber, 1, myReadAPIKey);
		  Particle.publish("Latest value is: " + String(value)); 
		  delay(30000);
		}
	 * @endcode
	 */
    int readIntField(unsigned long channelNumber, unsigned int field, const char * readAPIKey)
	{
		return readLongField(channelNumber, field, readAPIKey);
	}


	/**
	 * @brief Read the latest int from a public ThingSpeak channel
	 * @param channelNumber Channel number
	 * @param field Field number (1-8) within the channel to read from.
	 * @return Value read, or 0 if the field is text or there is an error.  Use getLastReadStatus() to get more specific information.
	 * @remark If the value returned is out of range for an int, the result is undefined.
	 * @code
		void loop() {
		  int value = ThingSpeak.readIntField(myChannelNumber, 1);
		  Particle.publish("Latest value is: " + String(value)); 
		  delay(30000);
		}
	 * @endcode
	 */
    int readIntField(unsigned long channelNumber, unsigned int field)
	{
		return readLongField(channelNumber, field, NULL);
	};

	/**
	 * @brief Read the latest status from a private ThingSpeak channel
	 * @param channelNumber Channel number
	 * @param readAPIKey Read API key associated with the channel.  *If you share code with others, do _not_ share this key*
	 * @return Value read (UTF8 string). An empty string is returned if there was no status written to the channel or in case of an error.  Use getLastReadStatus() to get more specific information.
	 * @code
		void loop() {
		  String value = ThingSpeak.readStatus(myChannelNumber, myReadAPIKey);
		  Particle.publish("Latest status is: " + value); 
		  delay(30000);
		}
	 * @endcode
	 */	
	String readStatus(unsigned long channelNumber, const char * readAPIKey)
	{
		String content = readRaw(channelNumber, "/feeds/last.txt?status=true", readAPIKey);
		
		if(getLastReadStatus() != OK_SUCCESS){
			return String("");
		}
		
		return getJSONValueByKey(content, "status");
	};
	
		/**
	 * @brief Read the latest status from a public ThingSpeak channel
	 * @param channelNumber Channel number	
	 * @return Value read (UTF8 string). An empty string is returned if there was no status written to the channel or in case of an error.  Use getLastReadStatus() to get more specific information.
	 * @code
		void loop() {
		  String value = ThingSpeak.readStatus(myChannelNumber, myReadAPIKey);
		  Particle.publish("Latest status is: " + value); 
		  delay(30000);
		}
	 * @endcode
	 */
	String readStatus(unsigned long channelNumber)
	{
		return readStatus(channelNumber, NULL);
	};
	
	/**
	 * @brief Read the created-at timestamp associated with the latest update to a private ThingSpeak channel
	 * @param channelNumber Channel number
	 * @param readAPIKey Read API key associated with the channel.  *If you share code with others, do _not_ share this key*
	 * @return Value read (UTF8 string). An empty string is returned if there was no created-at timestamp written to the channel or in case of an error.  Use getLastReadStatus() to get more specific information.
	 * @code
		void loop() {
		  String value = ThingSpeak.readCreatedAt(myChannelNumber);
		  Particle.publish("Latest update timestamp is: " + value); 
		  delay(30000);
		}
	 * @endcode
	 */	
	String readCreatedAt(unsigned long channelNumber, const char * readAPIKey)
	{
		String content = readRaw(channelNumber, "/feeds/last.txt", readAPIKey);
		
		if(getLastReadStatus() != OK_SUCCESS){
			return String("");
		}
		
		return getJSONValueByKey(content, "created_at");
	};

	/**
	 * @brief Read the created-at timestamp associated with the latest update to a private ThingSpeak channel
	 * @param channelNumber Channel number
	 * @return Value read (UTF8 string). An empty string is returned if there was no created-at timestamp written to the channel or in case of an error.  Use getLastReadStatus() to get more specific information.
	 * @code
		void loop() {
		  String value = ThingSpeak.readCreatedAt(myChannelNumber);
		  Particle.publish("Latest update timestamp is: " + value); 
		  delay(30000);
		}
	 * @endcode
	 */	
	String readCreatedAt(unsigned long channelNumber)
	{
		return readCreatedAt(channelNumber, NULL);
	};
	
	/**
	 * @brief Read a raw response from a public ThingSpeak channel
	 * @param channelNumber Channel number
	 * @param URLSuffix Raw URL to write to ThingSpeak as a String.  See the documentation at https://thingspeak.com/docs/channels#get_feed
	 * @return Response if successful, or empty string.	 Use getLastReadStatus() to get more specific information.
	 * @remark This is low level functionality that will not be required by most users.
	 * @code
		void loop() {
		  String response = ThingSpeak.readRaw(myChannelNumber, String("feeds/days=1"));
		  Particle.publish("Response: " + response); 
		  delay(30000);
		}
	 * @endcode
	 */
	String readRaw(unsigned long channelNumber, String URLSuffix)
	{
		return readRaw(channelNumber, URLSuffix, NULL);
	}
	
	/**
	 * @brief Read a raw response from a private ThingSpeak channel
	 * @param channelNumber Channel number
	 * @param URLSuffix Raw URL to write to ThingSpeak as a String.  See the documentation at https://thingspeak.com/docs/channels#get_feed
	 * @param readAPIKey Read API key associated with the channel.  *If you share code with others, do _not_ share this key*
	 * @return Response if successful, or empty string.	 Use getLastReadStatus() to get more specific information.
	 * @remark This is low level functionality that will not be required by most users.
	 * @code
		void loop() {
		  String response = ThingSpeak.readRaw(myChannelNumber, String("feeds/days=1"), myReadAPIKey);
		  Particle.publish("Response: " + response); 
		  delay(30000);
		}
	 * @endcode
	 */
	String readRaw(unsigned long channelNumber, String URLSuffix, const char * readAPIKey)
	{
		#ifdef PRINT_DEBUG_MESSAGES
			if(NULL != readAPIKey)
			{
				Particle.publish(SPARK_PUBLISH_TOPIC, "ts::readRaw   (channelNumber: " + String(channelNumber) + " readAPIKey: " + String(readAPIKey) + " URLSuffix: \"" + URLSuffix + "\")" , SPARK_PUBLISH_TTL, PRIVATE);	
			}
			else
			{
				Particle.publish(SPARK_PUBLISH_TOPIC, "ts::readRaw   (channelNumber: " + String(channelNumber) + " URLSuffix: \"" + URLSuffix + "\")" , SPARK_PUBLISH_TTL, PRIVATE);	
			}
		#endif

		if(!connectThingSpeak())
		{
			this->lastReadStatus = ERR_CONNECT_FAILED;
			return String("");
		}

		String URL = String("/channels/") + String(channelNumber) + URLSuffix;

		#ifdef PRINT_DEBUG_MESSAGES
		Particle.publish(SPARK_PUBLISH_TOPIC,"               GET \"" + URL + "\"" , SPARK_PUBLISH_TTL, PRIVATE);
		#endif

		// Post data to thingspeak
		if(!this->client->print("GET ")) return abortReadRaw();
		if(!this->client->print(URL)) return abortReadRaw();
		if(!this->client->print(" HTTP/1.1\r\n")) return abortReadRaw();
		if(!writeHTTPHeader(readAPIKey)) return abortReadRaw();
		if(!this->client->print("\r\n")) return abortReadRaw();
 
		String content = String();
		int status = getHTTPResponse(content);
		this->lastReadStatus = status;


		#ifdef PRINT_DEBUG_MESSAGES
			if(status == OK_SUCCESS)
			{
				Particle.publish(SPARK_PUBLISH_TOPIC, "Read: \"" + content + "\"" , SPARK_PUBLISH_TTL, PRIVATE);
			}
		#endif

		client->stop();
		#ifdef PRINT_DEBUG_MESSAGES
			Particle.publish(SPARK_PUBLISH_TOPIC, "disconnected." , SPARK_PUBLISH_TTL, PRIVATE);
		#endif

		if(status != OK_SUCCESS)
		{
			// return status;
			return String("");
		}

        // This is a workaround to a bug in the Spark implementation of String
    	return String("") + content;
	};
	
	/**
	 * @brief Get the status of the previous read.
	 * @return Generally, these are HTTP status codes.  Negative values indicate an error generated by the library.
	 * Possible response codes:
	 *  * 200: OK / Success
	 *  * 404: Incorrect API key (or invalid ThingSpeak server address)
	 *  * -101: Value is out of range or string is too long (> 255 characters)
	 *  * -201: Invalid field number specified
	 *  * -210: setField() was not called before writeFields()
	 *  * -301: Failed to connect to ThingSpeak
	 *  * -302: Unexpected failure during write to ThingSpeak
	 *  * -303: Unable to parse response
     *  * -304: Timeout waiting for server to respond
	 *  * -401: Point was not inserted (most probable cause is the rate limit of once every 15 seconds)
	 * @remark The read functions will return zero or empty if there is an error.  Use this function to retrieve the details.
	 * @code
		void loop() {
		  String message = ThingSpeak.readStringField(myChannelNumber, 1);
		  int resultCode = ThingSpeak.getLastReadStatus();
		  if(resultCode == 200)
		  {
			  Particle.publish("Latest message is: " + message); 
		  }
		  else
		  {
			  Particle.publish("Error reading message.  Status was: " + String(resultCode)); 
		  }
		  delay(30000);
		}
	 * @endcode
	 */
	int getLastReadStatus()
	{
		return this->lastReadStatus;
	};
private:

	String getJSONValueByKey(String textToSearch, String key)
	{	
		if(textToSearch.length() == 0){
			return String("");
		} 
		
		String searchPhrase = String("\"") + key + String("\":\"");
		
		int fromPosition = textToSearch.indexOf(searchPhrase,0);
		
		if(fromPosition == -1){
			// return because there is no status or it's null
			return String("");
		}
		
		fromPosition = fromPosition + searchPhrase.length();
				
		int toPosition = textToSearch.indexOf("\"", fromPosition);
		
		
		if(toPosition == -1){
			// return because there is no end quote
			return String("");
		}
		
		textToSearch.remove(toPosition);
		
		return textToSearch.substring(fromPosition);	
	}
	
    int abortWriteRaw()
    {
        this->client->stop();
        return ERR_UNEXPECTED_FAIL;
    }

    String abortReadRaw()
    {
		this->client->stop();
		#ifdef PRINT_DEBUG_MESSAGES
			Particle.publish(SPARK_PUBLISH_TOPIC, "ReadRaw abort - disconnected." , SPARK_PUBLISH_TTL, PRIVATE);
		#endif
		this->lastReadStatus = ERR_UNEXPECTED_FAIL;
		return String("");
    }

	void setServer(const char * customHostName, unsigned int port)
	{
		#ifdef PRINT_DEBUG_MESSAGES
			Particle.publish(SPARK_PUBLISH_TOPIC, "ts::setServer  (URL: \"" + String(customHostName) + "\")" , SPARK_PUBLISH_TTL, PRIVATE);
		#endif
		this->customIP = INADDR_NONE;
		this->customHostName = customHostName;
        this->port = port;
	};

	void setServer(IPAddress customIP, unsigned int port)
	{
		#ifdef PRINT_DEBUG_MESSAGES
		Particle.publish(SPARK_PUBLISH_TOPIC, "ts::setServer  (IP: \"" + String(customIP) + "\")" , SPARK_PUBLISH_TTL, PRIVATE);
		#endif
		this->customIP = customIP;
		this->customHostName = NULL;
        this->port = port;
	};

	void setServer()
	{
		#ifdef PRINT_DEBUG_MESSAGES
		Particle.publish(SPARK_PUBLISH_TOPIC, "ts::setServer  (default)" , SPARK_PUBLISH_TTL, PRIVATE);
		#endif
		this->customIP = INADDR_NONE;
		this->customHostName = NULL;
        this->port = THINGSPEAK_PORT_NUMBER;
	};

	void setClient(Client * client) {this->client = client;};

	Client * client = NULL;
	const char * customHostName = NULL;
	IPAddress customIP = INADDR_NONE;
    unsigned int port = THINGSPEAK_PORT_NUMBER;
	String nextWriteField[8];
	float nextWriteLatitude;
	float nextWriteLongitude;
	float nextWriteElevation;
	int lastReadStatus;
	String nextWriteStatus;
	String nextWriteTwitter;
	String nextWriteTweet;
	String nextWriteCreatedAt;

	bool connectThingSpeak()
	{
		bool connectSuccess = false;
		if(this->customIP == INADDR_NONE && NULL == this->customHostName)
        {
 			#ifdef PRINT_DEBUG_MESSAGES
			Particle.publish(SPARK_PUBLISH_TOPIC, "               Connect to default ThingSpeak URL..." , SPARK_PUBLISH_TTL, PRIVATE);
			#endif
			connectSuccess = client->connect(THINGSPEAK_URL,THINGSPEAK_PORT_NUMBER);
            if(!connectSuccess)
            {
 			    #ifdef PRINT_DEBUG_MESSAGES
				Particle.publish(SPARK_PUBLISH_TOPIC, "Failed. Try default IP..." , SPARK_PUBLISH_TTL, PRIVATE);
			    #endif
			    connectSuccess = client->connect(THINGSPEAK_IPADDRESS,THINGSPEAK_PORT_NUMBER);
            }
       }
       else
       {
		    if(!(this->customIP == INADDR_NONE))
		    {
			    // Connect to the server on port 80 (HTTP) at the customIP address
			    #ifdef PRINT_DEBUG_MESSAGES
				Particle.publish(SPARK_PUBLISH_TOPIC, "               Connect to " + String(this->customIP) + "..." , SPARK_PUBLISH_TTL, PRIVATE);  			     
			    #endif
			    connectSuccess = client->connect(this->customIP,this->port);
		    }
		    if(NULL != this->customHostName)
		    {
			    // Connect to the server on port 80 (HTTP) at the URL address
			    #ifdef PRINT_DEBUG_MESSAGES
                    Particle.publish(SPARK_PUBLISH_TOPIC, "Attempt Connect to URL " + String(this->customHostName), SPARK_PUBLISH_TTL, PRIVATE);
			    #endif
			    connectSuccess = client->connect(customHostName,this->port);
		    }
        }

		#ifdef PRINT_DEBUG_MESSAGES
		if (connectSuccess)
		{
			Particle.publish(SPARK_PUBLISH_TOPIC, "Connection Success", SPARK_PUBLISH_TTL, PRIVATE);
		}
		else
		{
			Particle.publish(SPARK_PUBLISH_TOPIC, "Connection Failure", SPARK_PUBLISH_TTL, PRIVATE);
		}
		#endif
		return connectSuccess;
	};

	bool writeHTTPHeader(const char * APIKey)
	{
        if(NULL != this->customHostName)
        {
		    if (!this->client->print("Host: ")) return false;
		    if (!this->client->print(this->customHostName)) return false;
		    if (!this->client->print("\r\n")) return false;
        }
        else
        {
		    if (!this->client->print("Host: api.thingspeak.com\r\n")) return false;
        }
		if (!this->client->print("Connection: close\r\n")) return false;
		if (!this->client->print("User-Agent: ")) return false;
		if (!this->client->print(TS_USER_AGENT)) return false;
		if (!this->client->print("\r\n")) return false;
		if(NULL != APIKey)
		{
			if (!this->client->print("X-THINGSPEAKAPIKEY: ")) return false;
			if (!this->client->print(APIKey)) return false;
			if (!this->client->print("\r\n")) return false;
		}
		return true;
	};

	int getHTTPResponse(String & response)
	{
        long startWaitForResponseAt = millis();
        while(client->available() == 0 && millis() - startWaitForResponseAt < TIMEOUT_MS_SERVERRESPONSE)
        {
            delay(100);
        }
        if(client->available() == 0)
        {
			return ERR_TIMEOUT; // Didn't get server response in time
        }

		if(!client->find(const_cast<char *>("HTTP/1.1")))
		{
			#ifdef PRINT_HTTP
				Particle.publish(SPARK_PUBLISH_TOPIC, "ERROR: Didn't find HTTP/1.1", SPARK_PUBLISH_TTL, PRIVATE);
    		#endif
			return ERR_BAD_RESPONSE; // Couldn't parse response (didn't find HTTP/1.1)
		}
		int status = client->parseInt();
		#ifdef PRINT_HTTP
			Particle.publish(SPARK_PUBLISH_TOPIC, "Got Status of " + String(status), SPARK_PUBLISH_TTL, PRIVATE);
		#endif
		if(status != OK_SUCCESS)
		{
			return status;
		}

		if(!client->find(const_cast<char *>("\r\n")))
		{
			#ifdef PRINT_HTTP
				Particle.publish(SPARK_PUBLISH_TOPIC, "ERROR: Didn't find end of status line", SPARK_PUBLISH_TTL, PRIVATE);
			#endif
			return ERR_BAD_RESPONSE;
		}
		#ifdef PRINT_HTTP
			Particle.publish(SPARK_PUBLISH_TOPIC, "Found end of status line", SPARK_PUBLISH_TTL, PRIVATE);
		#endif

		if(!client->find(const_cast<char *>("\n\r\n")))
		{
			#ifdef PRINT_HTTP
				Particle.publish(SPARK_PUBLISH_TOPIC, "ERROR: Didn't find end of header", SPARK_PUBLISH_TTL, PRIVATE);
			#endif
			return ERR_BAD_RESPONSE;
		}
		#ifdef PRINT_HTTP
			Particle.publish(SPARK_PUBLISH_TOPIC, "Found end of header", SPARK_PUBLISH_TTL, PRIVATE);
		#endif
		// This is a workaround to a bug in the Spark implementation of String
		String tempString = client->readString();
		response = tempString;
		#ifdef PRINT_HTTP
			Particle.publish(SPARK_PUBLISH_TOPIC, "Response: \"" + tempString + "\"", SPARK_PUBLISH_TTL, PRIVATE);
		#endif
		return status;
	};

	int convertFloatToChar(float value, char *valueString)
	{
		// Supported range is -999999000000 to 999999000000
		if(0 == isinf(value) && (value > 999999000000 || value < -999999000000))
		{
			// Out of range
			return ERR_OUT_OF_RANGE;
		}
		// Given that the resolution of Spark is 1 / 2^12, or ~0.00024 volts, assume that 5 places right of decimal should be sufficient for most applications
        sprintf(valueString, "%.5f", value);

		return OK_SUCCESS;
	};

	float convertStringToFloat(String value)
	{
		// There's a bug in the AVR function strtod that it doesn't decode -INF correctly (it maps it to INF)
		float result = value.toFloat();
		if(1 == isinf(result) && *value.c_str() == '-')
		{
			result = (float)-INFINITY;
		}
		return result;
	};

	void resetWriteFields()
	{
		for(size_t iField = 0; iField < 8; iField++)
		{
			this->nextWriteField[iField] = "";
		}
		this->nextWriteLatitude = NAN;
		this->nextWriteLongitude = NAN;
		this->nextWriteElevation = NAN;
		this->nextWriteStatus = "";
		this->nextWriteTwitter = "";
		this->nextWriteTweet = "";
		this->nextWriteCreatedAt = "";
	};
};

extern ThingSpeakClass ThingSpeak;

#endif //ThingSpeak_h


