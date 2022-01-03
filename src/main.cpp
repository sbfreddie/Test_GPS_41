#include <Arduino.h>
#include <ublox3.h>            // This library is used to control the uBlox GPS chip attached to the hardware serial port.


enum gpsCompletionCodes
{
    NO_FIX                  = 0,
    DEAD_RECKONING_ONLY     = 1,
    TWO_D_FIX               = 2,
    THREE_D_FIX             = 3,
    GNSS_DR_COMBINED        = 4,
    TIME_FIX_ONLY           = 5,
    BAD_UBLOX_PACKET        = 6,
    NOT_FULL_PACKET         = 7
};


// Set this to the GPS hardware serial port you wish to use
    #define GPShwSERIAL 1 // 1 = Serial1, 2 = Serial2, 3 = Serial3, 4 = Serial4 ....

    uint8_t const GPSSerialPort = GPShwSERIAL; // default settings

    uint32_t const BaudDefault = 9600; // default settings.
    #define SERIAL1_RX_BUFFER_SIZE = 512;  // Need large incoming buffer for GPS Packet (100 bytes).

    #define DEBUG1    true              // Turn on/off main functions debug messages.
    #define DEBUG2    true              // Turn on/off main functions debug messages.

    /*******************************************************************************************************************
    *   This Class is the uBlox GPS receiver Class.
    *   Needs:
    *       Hardware serial port the uBlox GPS Receiver is connected to (1->3) on Teensy 3.2.
    * fix Type: 0: no fix, 1: dead reckoning only, 2: 2D-fix, 3: 3D-fix, 4: GNSS + dead reckoning combined, 5: time only fix
    *******************************************************************************************************************/
    UBLOX gps(GPShwSERIAL);

    /*******************************************************************************************************************
    * This Variable is set to the GPS Completon Code to indicate whether or not the GPS data has been properly received
    * or not.
    *******************************************************************************************************************/
    gpsCompletionCodes gpsCodes;  // These are the Return codes from the GPS device to determine errors or good data.

    /*******************************************************************************************************************
    * This struct is for the uBlox data structure.
    *******************************************************************************************************************/
    gpsData uBloxData;


    /*******************************************************************************************************************
    * This Function clears the local uint8_t(8 bit) buffer provided of data (Set to zero).
    *******************************************************************************************************************/
    void clearBuffer(uint8_t *LocalBufferAddress, int16_t LocalBufferSize)
        {
            #if defined (DEBUG2)
                Serial.print(F("\nClearing Buffer, its Size is: "));
                Serial.println(LocalBufferSize, DEC);
            #endif
    
            memset(LocalBufferAddress, 0, LocalBufferSize);
 
        }


/************************************************************************************************************************
* This Code starts up the GPS on Serial Port 1 either from Startup or from an error occuring needing restarting.
************************************************************************************************************************/
void RestartGPS(bool initial_Startup)
{
    #if defined (DEBUG1)
        Serial.println(F("Restarting GPS"));
        Serial.println(F("Ending Previous GPS\n"));
    #endif

    if(initial_Startup == false)
        {
            gps.end();
            delay(100);
        }
    
    #if defined (DEBUG1)
        Serial.println(F("Beginning the GPS and Finding suitable Baud Rate."));
    #endif

    #if defined (DEBUG1)
        Serial.println(F("Trying Baud Rate 4800."));
        gps.begin(4800);
        gps.SetGPSbaud(230400, true);
        gps.end();
    #else
        gps.begin(4800);
        gps.SetGPSbaud(460800, false);
        gps.end();
    #endif

    delay(100);

    #if defined (DEBUG1)
        Serial.println(F("Trying Baud Rate 9600."));
        gps.begin(9600);
        gps.SetGPSbaud(230400, true);
        gps.end();
    #else
        gps.begin(9600);
        gps.SetGPSbaud(460800, false);
        gps.end();
    #endif
 
    delay(100);

    #if defined (DEBUG1)
        Serial.println(F("Trying Baud Rate 19200."));
        gps.begin(19200);
        gps.SetGPSbaud(230400, true);
        gps.end();
    #else
        gps.begin(19200);
        gps.SetGPSbaud(460800, false);
        gps.end();
    #endif

    delay(100);

    #if defined (DEBUG1)
        Serial.println(F("Trying Baud Rate 38400."));
        gps.begin(38400);
        gps.SetGPSbaud(230400, true);
        gps.end();
    #else
        gps.begin(38400);
        gps.SetGPSbaud(460800, false);
        gps.end();
    #endif

    delay(100);

    #if defined (DEBUG1)
        Serial.println(F("Trying Baud Rate 57600."));
        gps.begin(57600);
        gps.SetGPSbaud(230400, true);
        gps.end();
    #else
        gps.begin(57600);
        gps.SetGPSbaud(460800, false);
        gps.end();
    #endif

    delay(100);

    #if defined (DEBUG1)
        Serial.println(F("Trying Baud Rate 115200."));
        gps.begin(115200);
        gps.SetGPSbaud(230400, true);
        gps.end();
    #else
        gps.begin(115200);
        gps.SetGPSbaud(460800, false);
        gps.end();
    #endif 

    delay(100);          
            
    #if defined (DEBUG1)
        Serial.println(F("Trying Baud Rate 230400."));
        gps.begin(230400);
        gps.SetGPSbaud(230400, true);
        gps.end();
    #else
        gps.begin(230400);
        gps.SetGPSbaud(460800, false);
        gps.end();

    #endif

    delay(100);        
            
    #if defined (DEBUG1)
        Serial.println(F("Trying Baud Rate 460800."));
        gps.begin(460800);
        gps.SetGPSbaud(230400, true);
        gps.end();
    #else
        gps.begin(460800);
        gps.SetGPSbaud(460800, false);
        gps.end();
    #endif

    delay(100);        
            
    #if defined (DEBUG1)
        Serial.println(F("Trying Baud Rate 921600."));
        gps.begin(921600);
        gps.SetGPSbaud(230400, true);
        gps.end();
    #else
        gps.begin(921600);
        gps.SetGPSbaud(460800, false);
        gps.end();
    #endif

    delay(100);        
            
    #if defined (DEBUG1)
        Serial.println(F("Beginning the GPS at 230400 Baud Rate."));
        // now start communication with the GPS receiver at 230400 baud,
        gps.begin(230400);  // Enable Teensy serial communication @ given baud rate
        gps.Poll_GPSbaud_Port1(true);  // Polls the GPS baud configuration for one I/O Port, I/O Target 0x01=UART1
    #else
        // now start communication with the GPS receiver at 230400 baud,
        gps.begin(460800);  // Enable Teensy serial communication @ given baud rate
        gps.Poll_GPSbaud_Port1(false);  // Polls the GPS baud configuration for one I/O Port, I/O Target 0x01=UART1
    #endif

    delay(50);

    #if defined (DEBUG1)
        //gps.Poll_NAV_PVT();  // Polls UBX-NAV-PVT    (0x01 0x07) Navigation Position Velocity Time Solution
    #else
        //gps.Poll_NAV_PVT();  // Polls UBX-NAV-PVT    (0x01 0x07) Navigation Position Velocity Time Solution
    #endif

    delay(50);

    #if defined (DEBUG1)
        gps.SetRATE(1000, 1000, true);  // Set Navigation/Measurement Rate Settings, e.g. 100ms => 10Hz, 200 => 5.00Hz, 1000ms => 1Hz, 10000ms => 0.1Hz
        // Possible Configurations:
        // 60=>16.67Hz, 64=>15.63Hz, 72=>13.89Hz, 80=>12.50Hz, 100=>10.00Hz, 125=>8.00Hz, 200=>5.00Hz, 250=>4.00Hz, 500=>2.00Hz
        // 800=>1.25Hz, 1000=>1.00Hz, 2000=>0.50Hz, 4000=>0.25Hz, 10000=>0.10Hz, 20000=>0.05Hz, 50000=>0.02Hz       
    #else
        gps.SetRATE(200, 200, false);  // Set Navigation/Measurement Rate Settings, e.g. 100ms => 10Hz, 200 => 5.00Hz, 1000ms => 1Hz, 10000ms => 0.1Hz
        // Possible Configurations:
        // 60=>16.67Hz, 64=>15.63Hz, 72=>13.89Hz, 80=>12.50Hz, 100=>10.00Hz, 125=>8.00Hz, 200=>5.00Hz, 250=>4.00Hz, 500=>2.00Hz
        // 800=>1.25Hz, 1000=>1.00Hz, 2000=>0.50Hz, 4000=>0.25Hz, 10000=>0.10Hz, 20000=>0.05Hz, 50000=>0.02Hz       
    #endif

    delay(50);
    
    #if defined (DEBUG1)
        // NOTE: Dis_all_NMEA -strongly suggest changing RX buffer to 255 or more,*otherwise you will miss ACKs*on serial monitor
        gps.Dis_all_NMEA_Child_MSGs(true);  // Disable All NMEA Child Messages Command    
    #else
        // NOTE: Dis_all_NMEA -strongly suggest changing RX buffer to 255 or more,*otherwise you will miss ACKs*on serial monitor
        gps.Dis_all_NMEA_Child_MSGs(false);  // Disable All NMEA Child Messages Command    
    #endif 

    delay(50);  
    
    #if defined (DEBUG1)
        gps.SetNAV5(3, true);  // Set Dynamic platform model Navigation Engine Settings (0:portable, 2: stationary, 3:pedestrian, Etc)
        // Possible Configurations
        // 0: portable, 2: stationary, 3: pedestrian, 4: automotive, 5: sea, 6: airborne with <1g, 7: airborne with <2g
        // 8: airborne with <4g, 9: wrist worn watch (not supported in protocol v.less than 18)    
    #else
        gps.SetNAV5(3, false);  // Set Dynamic platform model Navigation Engine Settings (0:portable, 2: stationary, 3:pedestrian, Etc)
        // Possible Configurations
        // 0: portable, 2: stationary, 3: pedestrian, 4: automotive, 5: sea, 6: airborne with <1g, 7: airborne with <2g
        // 8: airborne with <4g, 9: wrist worn watch (not supported in protocol v.less than 18)    
    #endif  

    delay(50); 

    #if defined (DEBUG1)
        // ### Periodic auto update ON,OFF Command ###
        //gps.Ena_NAV_PVT(false);  // Enable periodic auto update NAV_PVT
        gps.Dis_NAV_PVT(true);  // Disable periodic auto update NAV_PVT    
    #else
        // ### Periodic auto update ON,OFF Command ###
        //gps.Ena_NAV_PVT(false);  // Enable periodic auto update NAV_PVT
        gps.Dis_NAV_PVT(false);  // Disable periodic auto update NAV_PVT    
    #endif   
    
    delay(50);

    #if defined (DEBUG1)
        //gps.Ena_NAV_ATT(true);  // Enable periodic auto update NAV_ATT ~ U-blox M8 from protocol version 19
        gps.Dis_NAV_ATT(true);  // Disable periodic auto update NAV_ATT ~ ---^    
    #else
        //gps.Ena_NAV_ATT(true);  // Enable periodic auto update NAV_ATT ~ U-blox M8 from protocol version 19
        gps.Dis_NAV_ATT(false);  // Disable periodic auto update NAV_ATT ~ ---^    
    #endif   
    
    delay(50);

    #if defined (DEBUG1)
        //gps.Ena_NAV_POSLLH(true);  // Enable periodic auto update NAV_POSLLH
        gps.Dis_NAV_POSLLH(true);  // Disable periodic auto update NAV_POSLLH    
    #else
        //gps.Ena_NAV_POSLLH(true);  // Enable periodic auto update NAV_POSLLH
        gps.Dis_NAV_POSLLH(false);  // Disable periodic auto update NAV_POSLLH    
    #endif   

    delay(500);  // Give the GPS time to reset.

    #if defined (DEBUG2)
        while (Serial.available() == 0)
        {
            unsigned int rd;
            uint8_t incomingByte, k = 1, buffer[1024];
            uint32_t output_start, current_micros, millis_start, incomingByteTransferTime[512], bytecount = 0, waitForMoreDataTimeout = 2000;

            // Clearing the input buffer.
            clearBuffer(buffer, sizeof(buffer));

            Serial1.clear();  // Clear the Serial1 buffer of all data, does not set data to zero.
            gps.Poll_NAV_PVT();  // Polls UBX-NAV-PVT, this requests the data from the GPS Module.

            // Looking for UBX_HEADER which is: 0xB5 and 0x62 in first 2 bytes.

            Serial.println(F(""));

            millis_start = millis();
            output_start = micros();

            while ( (millis() - millis_start) < (waitForMoreDataTimeout))
                {
                if (Serial1.available())    // Wait for Serial1 Data to be available.
                        {
                            buffer[bytecount] = Serial1.read();  // Store incoming byte in buffer.
                            incomingByteTransferTime[bytecount] = (micros() - output_start);
                            bytecount++;  // Increase byte count after storing in data and time in buffers.
                        } 
                }

            Serial.print(F("This is the total number of bytes received from Serial1: "));
            Serial.println(bytecount);

            Serial.println(F("These are the data bytes Received: "));
            k = 1;
            for (size_t i = 0; i < bytecount; i++)
                {
                    Serial.print(buffer[i],HEX);
                    if ( k == 8 )
                        {
                            Serial.println("");
                            k = 1;
                        }
                    else
                        {
                            Serial.print(" : ");
                            k++;
                        }
                }
            Serial.println(F("\n"));

            uint32_t temp;
            k = 1;
            size_t j = 0;

            for (j = 0; j < 2; j++)
                {
                    if (j == 0)
                        {
                            Serial.print(F("This is the time it took for the GPS to Return the first byte: "));
                            Serial.print(incomingByteTransferTime[j]);
                            Serial.println(F(" microseconds.\n"));
                        }
                    else if (j == 1)
                        {
                            Serial.println(F("These are the times (microseconds) it took to receive data bytes 2 thru x : "));
                            temp = incomingByteTransferTime[j] - incomingByteTransferTime[j - 1];
                            Serial.print(temp);
                        }
                }
            j = 2;
            for (j = 2; j < bytecount; j++)
                {
                    temp = incomingByteTransferTime[j] - incomingByteTransferTime[j - 1];
                    Serial.print(temp);

                    if ( k == 8 )
                        {
                            Serial.println("");
                            k = 1;
                        }
                    else
                        {
                            Serial.print(" : ");
                            k++;
                        }
                }

            Serial.println(F("\nCompleted printing all incoming data from Serial 1 Buffer.\n"));
            delay(2000);
        }
    #endif
    

}


/*******************************************************************************************************************
* This Function prints the Lat, Long, etc on the monitor.
*
* Needs:
*   validity Code
* Returns:
*   Nothing.
*******************************************************************************************************************/
void printGPSPositionOnMoniter(uint8_t validityCode)
    {
        if ( validityCode == 2 )
            {
                Serial.print(F("FixType: "));  // Print the Heading.
                Serial.println(F("2D-Fix"));  // Print the Fix Type.
            }
        else if ( validityCode == 3 )
            {
                Serial.print(F("FixType: "));  // Print the Heading.
                Serial.println(F("3D-Fix"));  // Print the Fix Type.
            }
        else if ( validityCode == 4 )
            {
                Serial.print(F("FixType: "));  // Print the Heading.
                Serial.println(F("GNSS_DR_COMBINED"));  // Print the Fix Type.
            }
        else
            {
                return;
            }

        Serial.print(F("Sats: "));  // Print the Heading.
        Serial.println(uBloxData.numSV,DEC);  // Print the Number of Satellites Used.
    
        Serial.print(F("Lat: "));  // Print the Heading.
        Serial.println(uBloxData.lat,3);  // Print the Latitude.
    
        Serial.print(F("Lon: "));  // Print the Heading.
        Serial.println(uBloxData.lon,3);  // Print the Longitude.
    
        Serial.print(F("Alt: "));  // Print the Altitude    .
        Serial.println( ( 3.281 * (uBloxData.hMSL) ),1 );  // Print the Altitude, for an approximate result, multiply the length value by 3.281.
    
        Serial.print(F("Spd: "));  // Print the Speed in MPH.
        Serial.println( ( 2.237 * (uBloxData.gSpeed) ),1 );  // Print the speed, multiply the speed value by 2.237 to get miles per hour.

        Serial.print(F("Baud Rate: "));  // Print the baud rate.
        Serial.println( uBloxData.GpsUart1Baud);  // Print the baud rate.
    }


/***************************************************************************************************************
* This function updates the Location Data on the TFT from the GPS.
*
*******************************************************************************************************
*               High level Commands, for the user ~ UBLOX lib. v1.0.2 2018-03-20 *
*******************************************************************************************************
*     NOTE: gps.command(Boolean) == (true)Print Message Acknowledged on USB Serial Monitor
* end();                            // Disables Teensy serial communication, to re-enable, call begin(Baud)
* Poll_CFG_Port1(bool);             // Polls the configuration for one I/O Port, I/O Target 0x01=UART1
* Poll_NAV_PVT();                   // Polls UBX-NAV-PVT    (0x01 0x07) Navigation Position Velocity Time Solution
* Poll_NAV_POSLLH();                // Polls UBX-NAV-POSLLH (0x01 0x02) Geodetic Position Solution
* Poll_NAV_ATT();                   // Polls UBX-NAV-ATT    (0x01 0x05) Attitude Solution
*
* ### Periodic Auto Update ON,OFF Command ###
* Ena_NAV_PVT(bool);                // Enable periodic auto update NAV_PVT
* Dis_NAV_PVT(bool);                // Disable periodic auto update NAV_PVT
* Ena_NAV_ATT(bool);                // Enable periodic auto update NAV_ATT
* Dis_NAV_ATT(bool);                // Disable periodic auto update NAV_ATT
* Ena_NAV_POSLLH(bool);             // Enable periodic auto update NAV_POSLLH
* Dis_NAV_POSLLH(bool);             // Disable periodic auto update NAV_POSLLH
*
* #### u-blox Switch off all NMEA MSGs ####
* Dis_all_NMEA_Child_MSGs(bool);    // Disable All NMEA Child Messages Command
*
* ### High level Command Generator ###
* SetGPSbaud(uint32_t baud, bool)   // Set UBLOX GPS Port Configuration Baud rate
* SetNAV5(uint8_t dynModel, bool)   // Set Dynamic platform model Navigation Engine Settings (0:portable, 3:pedestrian, Etc)
* SetRATE(uint16_t measRate, bool)  // Set Navigation/Measurement Rate Settings (100ms=10.00Hz, 200ms=5.00Hz, 1000ms=1.00Hz, Etc)
*
*   UBX-NAV-PVT --  Navigation Position Velocity Time Solution
*   ### UBX Protocol, Class NAV 0x01, ID 0x07 ###
*   UBX-NAV-PVT (0x01 0x07)    (Payload U-blox-M8=92, M7&M6=84)
*   iTOW                       ///< [ms], GPS time of the navigation epoch
*   utcYear                    ///< [year], Year (UTC)
*   utcMonth                   ///< [month], Month, range 1..12 (UTC)
*   utcDay                     ///< [day], Day of month, range 1..31 (UTC)
*   utcHour                    ///< [hour], Hour of day, range 0..23 (UTC)
*   utcMin                     ///< [min], Minute of hour, range 0..59 (UTC)
*   utcSec                     ///< [s], Seconds of minute, range 0..60 (UTC)
*   valid                      ///< [ND], Validity flags
*   tAcc                       ///< [ns], Time accuracy estimate (UTC)
*   utcNano                    ///< [ns], Fraction of second, range -1e9 .. 1e9 (UTC)
*   fixType                    ///< [ND], GNSSfix Type: 0: no fix, 1: dead reckoning only, 2: 2D-fix, 3: 3D-fix, 4: GNSS + dead reckoning combined, 5: time only fix
*   flags                      ///< [ND], Fix status flags
*   flags2                     ///< [ND], Additional flags
*   numSV                      ///< [ND], Number of satellites used in Nav Solution
*   lon                        ///< [deg], Longitude
*   lat                        ///< [deg], Latitude
*   height                     ///< [m], Height above ellipsoid
*   hMSL                       ///< [m], Height above mean sea level
*   hAcc                       ///< [m], Horizontal accuracy estimate
*   vAcc                       ///< [m], Vertical accuracy estimate
*   velN                       ///< [m/s], NED north velocity
*   velE                       ///< [m/s], NED east velocity
*   velD                       ///< [m/s], NED down velocity
*   gSpeed                     ///< [m/s], Ground Speed (2-D)
*   heading                    ///< [deg], Heading of motion (2-D)
*   sAcc                       ///< [m/s], Speed accuracy estimate
*   headingAcc                 ///< [deg], Heading accuracy estimate (both motion and vehicle)
*   pDOP                       ///< [ND], Position DOP
*   headVeh                    ///< [deg], Heading of vehicle (2-D)             #### NOTE: u-blox8 only ####
*   --- magDec, magAcc --- TODO TEST
*   magDec                     ///< [deg], Magnetic declination                 #### NOTE: u-blox8 only ####
*   magAcc                     ///< [deg], Magnetic declination accuracy        #### NOTE: u-blox8 only ####
*
* Needs:
*   Nothing
* Returns:
*    GNSSfix Type:
*       0: No fix
*       1: Dead reckoning only
*       2: 2D-fix
*       3: 3D-fix
*       4: GNSS + dead reckoning combined
*       5: Time only fix
*       6: Bad Ublox Packet
***************************************************************************************************************/
uint8_t updatePositionFromGPS(void)
{
    uint8_t fixType;

    // GNSSfix Type:
    //  0: no fix
    //  1: dead reckoning only
    //  2: 2D-fix
    //  3: 3D-fix
    //  4: GNSS + dead reckoning combined
    //  5: time only fix
    //  6: Bad Ublox Packet
    
    if ( gps.read( &uBloxData ) )  // Returns false if a full packet is not received.
        {
            fixType = ( uBloxData.fixType );  // Strip off unneeded bits.

            switch (fixType)
                {
                    case NO_FIX:
                        #if defined (DEBUG2)
                            Serial.println(F("NO FIX"));  // Print the Heading.
                        #endif
                        return NO_FIX;
                        
                    case DEAD_RECKONING_ONLY:
                        #if defined (DEBUG2)
                            Serial.println(F("DR ONLY"));  // Print the Heading.
                        #endif
                        return DEAD_RECKONING_ONLY;
                        
                    case TWO_D_FIX:
                        printGPSPositionOnMoniter( fixType );  // Print the Lat, Long, etc on the TFT.
                        return TWO_D_FIX;
                        
                    case THREE_D_FIX:
                        printGPSPositionOnMoniter( fixType  );  // Print the Lat, Long, etc on the TFT.
                        return THREE_D_FIX;
                        
                    case GNSS_DR_COMBINED:
                        printGPSPositionOnMoniter( fixType );  // Print the Lat, Long, etc on the TFT.
                        return GNSS_DR_COMBINED;
                        
                    case TIME_FIX_ONLY:
                        #if defined (DEBUG2)
                            Serial.println(F("TIME FIX ONLY"));  // Print the Heading.
                        #endif
                        return TIME_FIX_ONLY;

                    case BAD_UBLOX_PACKET:
                        #if defined (DEBUG2)
                            Serial.println(F("BAD PACKET"));  // Print the Heading.
                        #endif
                        return BAD_UBLOX_PACKET;

                    default:
                        return NO_FIX;
                }
        }
    else
        {
            Serial.println(F("Full Packet Not Received"));  // Print the Heading.
            return NOT_FULL_PACKET;
        }
    
}


/************************************************************************************************************************
* This is the Setup Routine, it only runs once.
************************************************************************************************************************/
void setup()
{
    bool initial_Startup;
    initial_Startup = true;  // The first time set true.

    Serial.begin(115200);

    unsigned long debug_start = millis();
    while (!Serial && ((millis() - debug_start) <= 3000)) ;  // Wait for the GPS to startup and the Serial Interface.

    RestartGPS(initial_Startup);
    initial_Startup = false;
        
    Serial1.clear();  // Clear the Serial1 buffer of all data
    gps.Poll_NAV_PVT();  // Polls UBX-NAV-PVT, this requests the data from the GPS Module.
    delay(1000);  // Wait 310mSecs for GPS response + 10mSecs for 100 bytes @ 230400 baud.


}

/************************************************************************************************************************
* This is the Loop code, it runs continuosly.
*   GNSSfix Type:
*   0: no fix
*   1: dead reckoning only
*   2: 2D-fix
*   3: 3D-fix
*   4: GNSS + dead reckoning combined
*   5: time only fix
*   6: Bad Ublox Packet
************************************************************************************************************************/
void loop()
{
    uint8_t fixType;
    int badPacketCount = 0;
    long unsigned timer_start;

    timer_start = millis();

    fixType = updatePositionFromGPS();
    if (fixType == NOT_FULL_PACKET)
        {
            badPacketCount++;    
        }

    if (badPacketCount == 150)  // Wait five minutes before we restart the GPS
        {
            RestartGPS(false);
        }
    
    
    #if defined (DEBUG2)
        switch (fixType)
            {
                case NO_FIX:
                 Serial.println(F("NO FIX"));  // Print the Heading.
                    Serial.println(F(""));
                case DEAD_RECKONING_ONLY:
                    Serial.println(F("DR ONLY"));  // Print the Heading.
                    Serial.println(F(""));
                case TWO_D_FIX:
                    Serial.println(F("TWO_D_FIX"));  // Print the Heading.
                    Serial.println(F(""));
                case THREE_D_FIX:
                    Serial.println(F("THREE_D_FIX"));  // Print the Heading.
                    Serial.println(F(""));
                 case GNSS_DR_COMBINED:
                    Serial.println(F("GNSS_DR_COMBINED"));  // Print the Heading.
                    Serial.println(F(""));
                case TIME_FIX_ONLY:
                    Serial.println(F("TIME_FIX_ONLY"));  // Print the Heading.
                    Serial.println(F(""));
                case BAD_UBLOX_PACKET:
                    Serial.println(F("BAD PACKET"));  // Print the Heading.
                    Serial.println(F(""));
                case NOT_FULL_PACKET:
                    Serial.println(F("NOT_FULL_PACKET"));  // Print the Heading.
                    Serial.println(F(""));
                    RestartGPS(false);

                default:
                    Serial.println(F("BAD PACKET"));  // Print the Heading.
                    Serial.println(F(""));


        }
        #endif
        
    Serial1.clear();  // Clear the Serial1 buffer of all data
    gps.Poll_NAV_PVT();  // Polls UBX-NAV-PVT, this requests the data from the GPS Module.

    Serial.println(F(""));
    while ((millis() - timer_start) <= 2000) ;  // Wait for 2 sec, this should also be plenty of time for the GPS to update.

}