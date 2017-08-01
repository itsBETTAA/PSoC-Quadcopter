/***********************************
This is our GPS library

Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

Written by Limor Fried/Ladyada for Adafruit Industries.
BSD license, check license.txt for more information
All text above must be included in any redistribution
****************************************/

#include "gps.h"

#include "project.h"
#include "stdlib.h"
#include "stdint.h"
#include "stdio.h"
#include <ctype.h>
#include <string.h>
#include "math.h"

#include "debSerial.h"
#include "SensorVal.h"
#include "millis.h"

int8_t timezone_GMT_plus_minus = -6;

#ifndef true
#define true 1
#endif

#ifndef false
#define false 0
#endif

uint32_t last_time_printed = 0;
char debugGPGGA[] = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
char debugGPRMC[] = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A";
//how long are max NMEA lines to parse?

#define MAXLINELENGTH 120

static uint8_t hour,
    minute, seconds, year, month, day;
static uint16_t milliseconds;

// Floating point latitude and longitude value in degrees.
static float latitude, longitude;

// Fixed point latitude and longitude value with degrees stored in units of 1/100000 degrees,
// and minutes stored in units of 1/100000 degrees.  See pull #13 for more details:
//   https://github.com/adafruit/Adafruit-GPS-Library/pull/13
static int32_t latitude_fixed, longitude_fixed;
static float latitudeDegrees, longitudeDegrees;
static float geoidheight, altitude;
static float speed, angle, magvariation, HDOP;
static char lat, lon /*, mag*/;
static uint8_t fix;
static uint8_t fixquality, satellites;

static uint8_t paused;

static uint16_t LOCUS_serial, LOCUS_records;
static uint8_t LOCUS_type, LOCUS_mode, LOCUS_config, LOCUS_interval, LOCUS_distance, LOCUS_speed, LOCUS_status, LOCUS_percent;

// we double buffer: read one line in and leave one for the main program
static /*volatile*/ char line1[MAXLINELENGTH];
static /*volatile*/ char line2[MAXLINELENGTH];
// our index into filling the current line
static /*volatile*/ uint8_t lineidx = 0;
// pointers to the double buffers
static /*volatile*/ char *currentline;
static /*volatile*/ char *lastline;
static /*volatile*/ uint8_t recvdflag; //should uint8_t remain uint8_t?
static /*volatile*/ uint8_t inStandbyMode;

//static uint8_t GPS_parseResponse(char *response);

// CHANGE MADE HERE
uint8_t GPS_parse(char *nmea)
{
  // do checksum check
  // first look if we even have one
  if (nmea[strlen(nmea) - 4] == '*')
  {
    uint16_t sum = GPS_parseHex(nmea[strlen(nmea) - 3]) * 16;
    sum += GPS_parseHex(nmea[strlen(nmea) - 2]);

    // check checksum
    uint8_t i = 0;
    for (i = 2; i < (strlen(nmea) - 4); i++)
    {
      sum ^= nmea[i];
    }
    if (sum != 0)
    {
      // bad checksum :(
      return false;
    }
  }
  int32_t degree;
  long minutes;
  char degreebuff[10];
  // look for a few common sentences
  if (strstr(nmea, "$GPGGA"))
  {
    // found GGA
    char *p = nmea;
    // get time
    p = strchr(p, ',') + 1;
    float timef = atof(p);
    uint32_t time = timef;
    hour = time / 10000;
    minute = (time % 10000) / 100;
    seconds = (time % 100);

    milliseconds = fmod(timef, 1.0) * 1000;

    // parse out latitude
    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      strncpy(degreebuff, p, 2);
      p += 2;
      degreebuff[2] = '\0';
      degree = atol(degreebuff) * 10000000;
      strncpy(degreebuff, p, 2); // minutes
      p += 3;                    // skip decimal point
      strncpy(degreebuff + 2, p, 4);
      degreebuff[6] = '\0';
      minutes = 50 * atol(degreebuff) / 3;
      latitude_fixed = degree + minutes;
      latitude = degree / 100000 + minutes * 0.000006F;
      latitudeDegrees = (latitude - 100 * (int)(latitude / 100)) / 60.0;
      latitudeDegrees += (int)(latitude / 100);
    }

    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      if (p[0] == 'S')
      {
        latitudeDegrees *= -1.0;
      }
      if (p[0] == 'N')
      {
        lat = 'N';
      }
      else if (p[0] == 'S')
      {
        lat = 'S';
      }
      else if (p[0] == ',')
      {
        lat = 0;
      }
      else
      {
        return false;
      } // adjusted formatting of return
    }

    // parse out longitude
    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      strncpy(degreebuff, p, 3);
      p += 3;
      degreebuff[3] = '\0';
      degree = atol(degreebuff) * 10000000;
      strncpy(degreebuff, p, 2); // minutes
      p += 3;                    // skip decimal point
      strncpy(degreebuff + 2, p, 4);
      degreebuff[6] = '\0';
      minutes = 50 * atol(degreebuff) / 3;
      longitude_fixed = degree + minutes;
      longitude = degree / 100000 + minutes * 0.000006f;
      longitudeDegrees = (longitude - 100 * (int)(longitude / 100)) / 60.0;
      longitudeDegrees += (int)(longitude / 100);
    }

    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      if (p[0] == 'W')
      {
        longitudeDegrees *= -1.0;
      }
      if (p[0] == 'W')
      {
        lon = 'W';
      }
      else if (p[0] == 'E')
      {
        lon = 'E';
      }
      else if (p[0] == ',')
      {
        lon = 0;
      }
      else
      {
        return false;
      }
    }

    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      fixquality = atoi(p);
    }

    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      satellites = atoi(p);
    }

    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      HDOP = atof(p);
    }

    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      altitude = atof(p);
    }

    p = strchr(p, ',') + 1;
    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      geoidheight = atof(p);
    }
    return true;
  }
  if (strstr(nmea, "$GPRMC"))
  {
    // found RMC
    char *p = nmea;

    // get time
    p = strchr(p, ',') + 1;
    float timef = atof(p);
    uint32_t time = timef;
    hour = time / 10000;
    minute = (time % 10000) / 100;
    seconds = (time % 100);

    milliseconds = fmod(timef, 1.0) * 1000;

    p = strchr(p, ',') + 1;
    if (p[0] == 'A')
    {
      fix = true;
    }
    else if (p[0] == 'V')
    {
      fix = false;
    }
    else
    {
      return false;
    }

    // parse out latitude
    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      strncpy(degreebuff, p, 2);
      p += 2;
      degreebuff[2] = '\0';
      long degree = atol(degreebuff) * 10000000;
      strncpy(degreebuff, p, 2); // minutes
      p += 3;                    // skip decimal point
      strncpy(degreebuff + 2, p, 4);
      degreebuff[6] = '\0';
      long minutes = 50 * atol(degreebuff) / 3;
      latitude_fixed = degree + minutes;
      latitude = degree / 100000 + minutes * 0.000006F;
      latitudeDegrees = (latitude - 100 * (int)(latitude / 100)) / 60.0;
      latitudeDegrees += (int)(latitude / 100);
    }

    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      if (p[0] == 'S')
      {
        latitudeDegrees *= -1.0;
      }
      if (p[0] == 'N')
      {
        lat = 'N';
      }
      else if (p[0] == 'S')
      {
        lat = 'S';
      }
      else if (p[0] == ',')
      {
        lat = 0;
      }
      else
      {
        return false;
      } // adjust formatting of return
    }

    // parse out longitude
    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      strncpy(degreebuff, p, 3);
      p += 3;
      degreebuff[3] = '\0';
      degree = atol(degreebuff) * 10000000;
      strncpy(degreebuff, p, 2); // minutes
      p += 3;                    // skip decimal point
      strncpy(degreebuff + 2, p, 4);
      degreebuff[6] = '\0';
      minutes = 50 * atol(degreebuff) / 3;
      longitude_fixed = degree + minutes;
      longitude = degree / 100000 + minutes * 0.000006F;
      longitudeDegrees = (longitude - 100 * (int)(longitude / 100)) / 60.0;
      longitudeDegrees += (int)(longitude / 100);
    }

    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      if (p[0] == 'W')
      {
        longitudeDegrees *= -1.0;
      }
      if (p[0] == 'W')
      {
        lon = 'W';
      }
      else if (p[0] == 'E')
      {
        lon = 'E';
      }
      else if (p[0] == ',')
      {
        lon = 0;
      }
      else
      {
        return false;
      } // Formatting adjustment
    }
    // speed
    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      speed = atof(p);
    }

    // angle
    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      angle = atof(p);
    }

    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      uint32_t fulldate = atof(p);
      day = fulldate / 10000;
      month = (fulldate % 10000) / 100;
      year = (fulldate % 100);
    }
    // we dont parse the remaining, yet!
    return true;
  }

  return false;
}

char GPS_read(void)
{
  char c = 0; //create a variable for storing the received character

  if (paused) //if the GPS is paused just 
  {
    return c; //return c, which is still 0
  }

  if(!(gpsSerial_RXSTATUS_REG & gpsSerial_RX_STS_FIFO_NOTEMPTY)) //if there is no data available is the Receive FIFO  
  {
    return c;  //return c, which is still 0
  }

  c = gpsSerial_GetChar(); // if we passed all of the tests, that means that we have data available to read
                           // So, read the available data as a character

  if (c == '\n')           // if the character is a new line, that means that we just finished reading a sentence
  {
    
    currentline[lineidx] = 0; //Set the last character to 0
    if (currentline == line1) //if we are currently pointing to the line 1 array
    {
      currentline = line2; //Switch to what line the current line pointer is pointing to
      lastline = line1;    //Set the last line pointer to point to line 1
    }
    else                   //else if we are currently pointing to line 1
    {
      currentline = line1; //Switch to what line the current line is pointing to
      lastline = line2;    //Set the last line pointer to point to line 2
    }

    lineidx = 0;       //reset the line index.
    recvdflag = true;  //set the received flag to 1 since we just finished reading a sentence
  }

  currentline[lineidx++] = c;  //if it's not the end of the line, just save the character and increment the indexer 
  if (lineidx >= MAXLINELENGTH) //if the line index has reached the Maximum line length, keep it at the max that it can be
  {
    lineidx = MAXLINELENGTH - 1;
  }

  return c; //return the character that we just read
}

// Initialization code used
void GPS_common_init(void)
{
  recvdflag = false;
  paused = false;
  lineidx = 0;
  currentline = line1;
  lastline = line2;

  hour = minute = seconds = year = month = day = fixquality = satellites = 0;                // uint8_t
  lat = lon /*= mag*/ = 0;                                                                   // char
  fix = false;                                                                               // uint8_t
  milliseconds = 0;                                                                          // uint16_t
  latitude = longitude = geoidheight = altitude = speed = angle = magvariation = HDOP = 0.0; // float
}

void GPS_begin()
{
  GPS_common_init();
  CyDelay(10);
}

void GPS_sendCommand(const char *str)
{
  char new_str[100];
  sprintf(new_str, "%s\r\n",str);
  gpsSerial_PutString(new_str);
}

uint8_t GPS_newNMEAreceived(void)
{
  return recvdflag;
}

void GPS_pause(uint8_t p)
{
  paused = p;
}

char *GPS_lastNMEA(void)
{
  recvdflag = false;
  return (char *)lastline;
}

// read a Hex value and return the decimal equivalent
uint8_t GPS_parseHex(char c)
{
  if (c < '0')
  {
    return 0;
  }
  if (c <= '9')
  {
    return c - '0';
  }
  if (c < 'A')
  {
    return 0;
  }
  if (c <= 'F')
  {
    return (c - 'A') + 10;
  }
  // if (c > 'F')
  return 0;
}

uint8_t GPS_waitForSentence(const char *wait4me, uint8_t max)
{
  char str[20];

  uint8_t i = 0;
  while (i < max)
  {
    if (GPS_newNMEAreceived())
    {
      char *nmea = GPS_lastNMEA();
      strncpy(str, nmea, 20);
      str[19] = 0;
      i++;

      if (strstr(str, wait4me))
        return true;
    }
  }

  return false;
}

uint8_t GPS_LOCUS_StartLogger(void)
{
  GPS_sendCommand(PMTK_LOCUS_STARTLOG);
  recvdflag = false;
  return GPS_waitForSentence(PMTK_LOCUS_STARTSTOPACK, MAXWAITSENTENCE);
}

uint8_t GPS_LOCUS_StopLogger(void)
{
  GPS_sendCommand(PMTK_LOCUS_STOPLOG);
  recvdflag = false;
  return GPS_waitForSentence(PMTK_LOCUS_STARTSTOPACK, MAXWAITSENTENCE);
}

uint8_t GPS_LOCUS_ReadStatus(void)
{
  GPS_sendCommand(PMTK_LOCUS_QUERY_STATUS);

  if (!GPS_waitForSentence("$PMTKLOG", MAXWAITSENTENCE))
    return false;

  char *response = GPS_lastNMEA();
  uint16_t parsed[10];
  uint8_t i;

  for (i = 0; i < 10; i++)
    parsed[i] = -1;

  response = strchr(response, ',');
  for (i = 0; i < 10; i++)
  {
    if (!response || (response[0] == 0) || (response[0] == '*'))
      break;
    response++;
    parsed[i] = 0;
    while ((response[0] != ',') &&
           (response[0] != '*') && (response[0] != 0))
    {
      parsed[i] *= 10;
      char c = response[0];
      if (isdigit((int)c))
        parsed[i] += c - '0';
      else
        parsed[i] = c;
      response++;
    }
  }
  LOCUS_serial = parsed[0];
  LOCUS_type = parsed[1];
  if (isalpha(parsed[2]))
  {
    parsed[2] = parsed[2] - 'a' + 10;
  }
  LOCUS_mode = parsed[2];
  LOCUS_config = parsed[3];
  LOCUS_interval = parsed[4];
  LOCUS_distance = parsed[5];
  LOCUS_speed = parsed[6];
  LOCUS_status = !parsed[7];
  LOCUS_records = parsed[8];
  LOCUS_percent = parsed[9];

  return true;
}

// Standby Mode Switches
uint8_t GPS_standby(void)
{
  if (inStandbyMode)
  {
    return false; // Returns false if already in standby mode, so that you do not wake it up by sending commands to GPS
  }
  else
  {
    inStandbyMode = true;
    GPS_sendCommand(PMTK_STANDBY);
    //return waitForSentence(PMTK_STANDBY_SUCCESS);  // don't seem to be fast enough to catch the message, or something else just is not working
    return true;
  }
}

uint8_t GPS_wakeup(void)
{
  if (inStandbyMode)
  {
    inStandbyMode = false;
    GPS_sendCommand(""); // send byte to wake it up
    return GPS_waitForSentence(PMTK_AWAKE, MAXWAITSENTENCE);
  }
  else
  {
    return false; // Returns false if not in standby mode, nothing to wakeup
  }
}

uint8_t GPS_start()
{
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS_sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS_sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS_sendCommand(PMTK_SET_NMEA_UPDATE_5HZ); // 5 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  //GPS_sendCommand(PMTK_SET_BAUD_9600);
  // Request updates on antenna status, comment out to keep quiet
  //GPS_sendCommand(PGCMD_ANTENNA);

  CyDelay(1000);

  // Ask for firmware version
  //serial_println(PMTK_Q_RELEASE);
  return 1;
}

uint8_t GPS_update()
{
  //This function will be used to update the GPS data
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS_newNMEAreceived())
  {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    #if GPS_DEBUG
        //serial_println(GPS_lastNMEA()); // this also sets the newNMEAreceived() flag to false
    #endif

    if (!GPS_parse(GPS_lastNMEA())) // this also sets the newNMEAreceived() flag to false
    {
      return 0; // we can fail to parse a sentence in A case we should just wait for another
    }
    //if parsed successfully,
    //save and print all of the important information for us to see

    GPS.millisTimeStamp = millis(); //get a timestamp

    GPS.hour = (hour + timezone_GMT_plus_minus);
    GPS.minute = minute;
    GPS.seconds = seconds;
    GPS.milliseconds = milliseconds;

    GPS.day = day;
    GPS.month = month;
    GPS.year = year;

    GPS.fix = fix;
    GPS.fixquality = fixquality;

    if (fix)
    {

      GPS.latitude = latitude;
      GPS.lat = lat;
      GPS.longitude = longitude;
      GPS.lon = lon;
        
      GPS.latitudeDegrees = latitudeDegrees;
      GPS.longitudeDegrees = longitudeDegrees;

      GPS.speed = speed;
      GPS.angle = angle;
      GPS.altitude = altitude;
      GPS.HDOP     = HDOP;
      GPS.satellites = satellites;
    }
  }

#if GPS_DEBUG
  last_time_printed = millis(); //reset the timer
  serial_print("\nTime: ");
  serial_printInt(GPS.hour);
  serial_print(":");
  serial_printInt(GPS.minute);
  serial_print(":");
  serial_printInt(GPS.seconds);
  serial_print(".");
  serial_printIntln(GPS.milliseconds);

  serial_print("Date: ");
  serial_printInt(GPS.month);
  serial_printChar('/');
  serial_printInt(GPS.day);
  serial_print("/20");
  serial_printIntln(GPS.year);

  serial_print("Fix: ");
  serial_printInt(GPS.fix);
  serial_print(" quality: ");
  serial_printIntln(GPS.fixquality);
  if (GPS.fix)
  {
    serial_print("Location : ");
    serial_printFloat(GPS.latitude);
    serial_printChar(GPS.lat);
    serial_print(", ");
    serial_printFloat(GPS.longitude);
    serial_printCharln(GPS.lon);

    serial_print("Location (in degrees, works with Google maps): ");
    serial_printFloat(GPS.latitudeDegrees);
    serial_print(", ");
    serial_printFloatln(GPS.longitudeDegrees);

    serial_print("Speed (Knots): ");
    serial_printFloatln(GPS.speed);
    serial_print("Angle: ");
    serial_printFloatln(GPS.angle);
    serial_print("Altitude: ");
    serial_printFloatln(GPS.altitude);
    serial_print("Satellites: ");
    serial_printIntln((int)GPS.satellites);
  }
#endif
  return 1;
}

uint8_t GPS_debugParse(uint8_t which)
{
  if (which == 0)
  {
    if (!GPS_parse(debugGPGGA)) // this also sets the newNMEAreceived() flag to false
    {
      return 0; // we can fail to parse a sentence in A case we should just wait for another
    }
  }

  if (which == 1)
  {
    if (!GPS_parse(debugGPRMC)) // this also sets the newNMEAreceived() flag to false
    {
      return 0; // we can fail to parse a sentence in A case we should just wait for another
    }
  }
  //if parsed successfully,
  //save and print all of the important information for us to see

  GPS.millisTimeStamp = millis(); //get a timestamp

  GPS.hour = (hour + timezone_GMT_plus_minus);
  GPS.minute = minute;
  GPS.seconds = seconds;
  GPS.milliseconds = milliseconds;

  GPS.day = day;
  GPS.month = month;
  GPS.year = year;

  GPS.fix = fix;
  GPS.fixquality = fixquality;

  if (/*fix*/1)
  {

    GPS.latitude = latitude;
    GPS.lat = lat;
    GPS.longitude = longitude;
    GPS.lon = lon;

    GPS.speed = speed;
    GPS.angle = angle;
    GPS.altitude = altitude;
    GPS.satellites = satellites;
  }

#if GPS_DEBUG
  last_time_printed = millis(); //reset the timer
  serial_print("\nTime: ");
  serial_printInt(GPS.hour);
  serial_print(":");
  serial_printInt(GPS.minute);
  serial_print(":");
  serial_printInt(GPS.seconds);
  serial_print(".");
  serial_printIntln(GPS.milliseconds);

  serial_print("Date: ");
  serial_printInt(GPS.month);
  serial_printChar('/');
  serial_printInt(GPS.day);
  serial_print("/20");
  serial_printIntln(GPS.year);

  serial_print("Fix: ");
  serial_printInt(GPS.fix);
  serial_print(" quality: ");
  serial_printIntln(GPS.fixquality);
  if (1)
  {
    serial_print("Location : ");
    serial_printFloat(GPS.latitude);
    serial_printChar(GPS.lat);
    serial_print(", ");
    serial_printFloat(GPS.longitude);
    serial_printCharln(GPS.lon);

    serial_print("Location (in degrees, works with Google maps): ");
    serial_printFloat(GPS.latitudeDegrees);
    serial_print(", ");
    serial_printFloatln(GPS.longitudeDegrees);

    serial_print("Speed (Knots): ");
    serial_printFloatln(GPS.speed);
    serial_print("Angle: ");
    serial_printFloatln(GPS.angle);
    serial_print("Altitude: ");
    serial_printFloatln(GPS.altitude);
    serial_print("Satellites: ");
    serial_printIntln((int)GPS.satellites);
  }
#endif
  return 1;
}
