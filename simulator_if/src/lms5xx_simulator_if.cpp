#include <cstring>    // sizeof()
#include <iostream>
#include <string>   
#include <iomanip>
#include <sstream>

// headers for socket(), getaddrinfo() and friends
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <unistd.h>    // close()

#include "sick_scan/tcp/colaa.hpp"
#include "sick_scan/tcp/toolbox.hpp"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define TRUE   1  
#define FALSE  0  

UINT8 scannerStatus = 6;
UINT8 sendMeasurementData = 0;
int scannerSocket = 0;

UINT8 scanDataTxBuf[10000];

void sendMeasurementMessage(int sock, bool periodic, const sensor_msgs::LaserScan::ConstPtr& msg);

template< typename T >
std::string int_to_hex( T i )
{
  std::stringstream sstream;
//  sstream << "0x" 
//         << std::setfill ('0') << std::setw(sizeof(T)*2) 
//         << std::hex << i;
  sstream << std::hex << i;
  return sstream.str();
}

void rosLaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int rangeCount = msg->ranges.size();
    int intensityCount = msg->intensities.size();

    //ROS_INFO("Received scan with: ts: %f, seq: %d", msg->header.stamp, msg->header.seq);
    if ( sendMeasurementData && scannerSocket )
    {
        sendMeasurementMessage(scannerSocket, true, msg);
    }   

}

void sendMeasurementMessage(int sock, bool periodic, const sensor_msgs::LaserScan::ConstPtr& msg)
{
            
    static UINT16 sendCount = 0;
    int channelCount = 1;
    int pointCount = 191;
    
    UINT32 bufLen = 0;

    if( ( pointCount != msg->intensities.size() ) || ( pointCount != msg->ranges.size()) ) 
    {
        ROS_ERROR("Invalid amount of measurements in ros topic");
        return;
    }

    // Beginne mit dem 23-Frame-Header
    scanDataTxBuf[bufLen++] = 0x02;
    scanDataTxBuf[bufLen++] = 's';

    if ( periodic )
    {
        bufLen += colaa::addStringToBuffer(&(scanDataTxBuf[bufLen]), colaa::COMMAND_Send_Event_ByName);
    }
    else
    {
        bufLen += colaa::addStringToBuffer(&(scanDataTxBuf[bufLen]), colaa::COMMAND_Register_Event_Answer);
    }
    
    scanDataTxBuf[bufLen++] = ' ';
    bufLen += colaa::addStringToBuffer(&(scanDataTxBuf[bufLen]), colaa::EVENTNAME_SUBSCRIBE_SCANS);
    scanDataTxBuf[bufLen++] = ' ';

    bufLen += colaa::addUINT16ToBuffer(&(scanDataTxBuf[bufLen]), 1); // version
    scanDataTxBuf[bufLen++] = ' ';

    /* Device Info */
    bufLen += colaa::addUINT16ToBuffer(&(scanDataTxBuf[bufLen]), 1); // device number
    scanDataTxBuf[bufLen++] = ' ';
    bufLen += colaa::addStringToBuffer(&(scanDataTxBuf[bufLen]), int_to_hex(42)); // S/N
    scanDataTxBuf[bufLen++] = ' ';
    bufLen += colaa::addUINT8ToBuffer(&(scanDataTxBuf[bufLen]), 0); // device status high
    scanDataTxBuf[bufLen++] = ' ';
    bufLen += colaa::addUINT8ToBuffer(&(scanDataTxBuf[bufLen]), 0); // device status low
    scanDataTxBuf[bufLen++] = ' ';

    /* Status Info */
    bufLen += colaa::addStringToBuffer(&(scanDataTxBuf[bufLen]), int_to_hex(++sendCount)); // telegram counter
    scanDataTxBuf[bufLen++] = ' ';
    bufLen += colaa::addStringToBuffer(&(scanDataTxBuf[bufLen]), int_to_hex(sendCount*8)); // scan counter
    scanDataTxBuf[bufLen++] = ' ';
    // TODO: get timestamps from real system
    bufLen += colaa::addStringToBuffer(&(scanDataTxBuf[bufLen]), int_to_hex(12345)); // Time from start us
    scanDataTxBuf[bufLen++] = ' ';
    bufLen += colaa::addStringToBuffer(&(scanDataTxBuf[bufLen]), int_to_hex(1234679)); // time of scan us
    scanDataTxBuf[bufLen++] = ' ';
    bufLen += colaa::addUINT8ToBuffer(&(scanDataTxBuf[bufLen]), 0); // din status high
    scanDataTxBuf[bufLen++] = ' ';
    bufLen += colaa::addUINT8ToBuffer(&(scanDataTxBuf[bufLen]), 0); // din status low
    scanDataTxBuf[bufLen++] = ' ';
    bufLen += colaa::addUINT8ToBuffer(&(scanDataTxBuf[bufLen]), 0); // dout status high
    scanDataTxBuf[bufLen++] = ' ';
    bufLen += colaa::addUINT8ToBuffer(&(scanDataTxBuf[bufLen]), 0); // dout status low
    scanDataTxBuf[bufLen++] = ' ';
    bufLen += colaa::addUINT16ToBuffer(&(scanDataTxBuf[bufLen]), 0); // reserved
    scanDataTxBuf[bufLen++] = ' ';

    /* Frequencies */
    bufLen += colaa::addStringToBuffer(&(scanDataTxBuf[bufLen]), int_to_hex(7500)); // Scan frequency 75Hz
    scanDataTxBuf[bufLen++] = ' ';
    /*
    Inverse of the time
    between two
    measurement
    shots (in 100Hz)
    example: 50Hz,
    0,5° Resolution 
    720 shots/20ms 
    36 kHz
    */
    bufLen += colaa::addStringToBuffer(&(scanDataTxBuf[bufLen]), int_to_hex(270)); // Measurement Frequency
    scanDataTxBuf[bufLen++] = ' ';


    /* Encoder Position */
    bufLen += colaa::addUINT16ToBuffer(&(scanDataTxBuf[bufLen]), 0); // Amount of encoder data
    scanDataTxBuf[bufLen++] = ' ';
    // TODO, if encoder data required, this needs to be filled in here

    UINT32 scaleFactor = 0x40000000; //0x3F800000
    UINT32 scaleFactorOffset = 0x00000000;
    UINT32 startAngle = 0xFFFF3CB0;
    UINT16 step = 10000; 

    /* 16-bit Output channels */
    bufLen += colaa::addUINT16ToBuffer(&(scanDataTxBuf[bufLen]), channelCount); // Amount of 16-bit channels (dist1-dist5)
    scanDataTxBuf[bufLen++] = ' ';
    for( int i = 1; i <= channelCount; i++ )
    {
        bufLen += colaa::addStringToBuffer(&(scanDataTxBuf[bufLen]), "DIST"); // Content
        bufLen += colaa::addINT32ToBuffer(&(scanDataTxBuf[bufLen]), i); // Contentnumber
        scanDataTxBuf[bufLen++] = ' ';
        bufLen += colaa::addStringToBuffer(&(scanDataTxBuf[bufLen]), toHexString(scaleFactor)); // Scale factor x2
        scanDataTxBuf[bufLen++] = ' ';
        bufLen += colaa::addStringToBuffer(&(scanDataTxBuf[bufLen]), toHexString(scaleFactorOffset)); // Scale factor offset 0
        scanDataTxBuf[bufLen++] = ' ';
        bufLen += colaa::addStringToBuffer(&(scanDataTxBuf[bufLen]), int_to_hex(startAngle)); // start angle (-5deg)
        scanDataTxBuf[bufLen++] = ' ';
        bufLen += colaa::addStringToBuffer(&(scanDataTxBuf[bufLen]), int_to_hex(step)); // step (1deg)
        scanDataTxBuf[bufLen++] = ' ';
        bufLen += colaa::addStringToBuffer(&(scanDataTxBuf[bufLen]), int_to_hex(pointCount)); // point count
        scanDataTxBuf[bufLen++] = ' ';

        for( int j = 0; j < pointCount; j++ )
        {
            int mval = 0;
            bufLen += colaa::addStringToBuffer(&(scanDataTxBuf[bufLen]), int_to_hex( (UINT16)(msg->ranges.at(j)*2000.0) )); // dummy
            scanDataTxBuf[bufLen++] = ' ';
        }
    }

    /* 8-bit Output channels */
    scaleFactor = 0x3F800000;
    bufLen += colaa::addUINT16ToBuffer(&(scanDataTxBuf[bufLen]), channelCount); // Amount of 16-bit channels (dist1-dist5)
    scanDataTxBuf[bufLen++] = ' ';
    for(int i = 1; i <= channelCount; i++)
    {
        bufLen += colaa::addStringToBuffer(&(scanDataTxBuf[bufLen]), "RSSI"); // Content
        bufLen += colaa::addINT32ToBuffer(&(scanDataTxBuf[bufLen]), i); // Contentnumber
        scanDataTxBuf[bufLen++] = ' ';
        bufLen += colaa::addStringToBuffer(&(scanDataTxBuf[bufLen]), toHexString(scaleFactor)); // Scale factor x1
        scanDataTxBuf[bufLen++] = ' ';
        bufLen += colaa::addStringToBuffer(&(scanDataTxBuf[bufLen]), toHexString(scaleFactorOffset)); // Scale factor offset 0
        scanDataTxBuf[bufLen++] = ' ';
        bufLen += colaa::addStringToBuffer(&(scanDataTxBuf[bufLen]), int_to_hex(startAngle)); // start angle (-5deg)
        scanDataTxBuf[bufLen++] = ' ';
        bufLen += colaa::addStringToBuffer(&(scanDataTxBuf[bufLen]), int_to_hex(step)); // step (1deg)
        scanDataTxBuf[bufLen++] = ' ';
        bufLen += colaa::addStringToBuffer(&(scanDataTxBuf[bufLen]), int_to_hex(pointCount)); // point count
        scanDataTxBuf[bufLen++] = ' ';

        for( int j = 0; j < pointCount; j++ )
        {
            int mval = 0;
            bufLen += colaa::addStringToBuffer(&(scanDataTxBuf[bufLen]), int_to_hex((UINT16)(msg->intensities.at(j)))); // dummy
            scanDataTxBuf[bufLen++] = ' ';
        }
    }

    /* Position */
    bufLen += colaa::addUINT16ToBuffer(&(scanDataTxBuf[bufLen]), 0); // Amount of position data
    scanDataTxBuf[bufLen++] = ' ';
    /* Position */
    bufLen += colaa::addUINT16ToBuffer(&(scanDataTxBuf[bufLen]), 0); // Amount of position data
    scanDataTxBuf[bufLen++] = ' ';
    /* Position */
    bufLen += colaa::addUINT16ToBuffer(&(scanDataTxBuf[bufLen]), 0); // Amount of position data
    scanDataTxBuf[bufLen++] = ' ';
    /* Position */
    bufLen += colaa::addUINT16ToBuffer(&(scanDataTxBuf[bufLen]), 0); // Amount of position data
    scanDataTxBuf[bufLen++] = ' ';
    /* Position */
    bufLen += colaa::addUINT16ToBuffer(&(scanDataTxBuf[bufLen]), 0); // Amount of position data

    // Schliesse den 23-Frame ab
    scanDataTxBuf[bufLen++] = 0x03;


    send(scannerSocket, scanDataTxBuf, bufLen, 0);
}

bool responseReadVariable(std::string * variableName, int socket)
{
    if ( *variableName == colaa::VARIABLENAME_DEVICESTATUS)
    {
        // Build command
        BYTE msgBuffer[128];
        BYTE replyBuffer[128];
        UINT16 replyBufferLen = 0;

        std::time_t t = std::time(nullptr);
        char timestr[100];
        char datestr[100];
        size_t timelen = std::strftime(timestr, sizeof(timestr), "%H:%M:%S", std::localtime(&t));
        size_t datelen = std::strftime(datestr, sizeof(datestr), "%d.%m.%Y", std::localtime(&t));

//        std::cout << "Time: " << timestr << std::endl;

        replyBufferLen += colaa::addStringToBuffer(&(replyBuffer[0]), colaa::COMMAND_Read_Variable_Answer);
		replyBuffer[replyBufferLen++] = ' ';
		replyBufferLen += colaa::addStringToBuffer(&(replyBuffer[replyBufferLen]), *variableName);
        /* status 
        0 = undefined
        1 = initialization
        2 = configuration
        3 = lower case
        4 = rotating
        5 = in preparation
        6 = ready
        7 = measurement active
        8 .. 11 = reserved
        */
		replyBuffer[replyBufferLen++] = ' ';
		replyBufferLen += colaa::addUINT16ToBuffer(&(replyBuffer[replyBufferLen]), scannerStatus);
        // Op. Temp. Range 00h..FFh
		replyBuffer[replyBufferLen++] = ' ';
		replyBufferLen += colaa::addUINT8ToBuffer(&(replyBuffer[replyBufferLen]), 0);
        // Unknown 16
		replyBuffer[replyBufferLen++] = ' ';
		replyBufferLen += colaa::addUINT16ToBuffer(&(replyBuffer[replyBufferLen]), 8);
        // Time HH:MM:SS
		replyBuffer[replyBufferLen++] = ' ';
        replyBufferLen += colaa::addStringToBuffer(&(replyBuffer[replyBufferLen]), timestr);
        /*
		replyBufferLen += colaa::addUINT16ToBuffer(&(replyBuffer[replyBufferLen]), 0);
		replyBuffer[replyBufferLen++] = ':';
		replyBufferLen += colaa::addUINT16ToBuffer(&(replyBuffer[replyBufferLen]), 0);
		replyBuffer[replyBufferLen++] = ':';
		replyBufferLen += colaa::addUINT16ToBuffer(&(replyBuffer[replyBufferLen]), 0);
        */
        // DATE DD.MM.YYYY
		replyBuffer[replyBufferLen++] = ' ';
        replyBufferLen += colaa::addStringToBuffer(&(replyBuffer[replyBufferLen]), datestr);
        /*
		replyBufferLen += colaa::addUINT16ToBuffer(&(replyBuffer[replyBufferLen]), 1);
		replyBuffer[replyBufferLen++] = '.';
		replyBufferLen += colaa::addUINT16ToBuffer(&(replyBuffer[replyBufferLen]), 1);
		replyBuffer[replyBufferLen++] = '.';
		replyBufferLen += colaa::addUINT32ToBuffer(&(replyBuffer[replyBufferLen]), 2020);
        */
        // Unknown 16
		replyBuffer[replyBufferLen++] = ' ';
		replyBufferLen += colaa::addUINT16ToBuffer(&(replyBuffer[replyBufferLen]), 10);
        // LED 1 - 0: inactive, 1 active
		replyBuffer[replyBufferLen++] = ' ';
		replyBufferLen += colaa::addUINT16ToBuffer(&(replyBuffer[replyBufferLen]), 0);
        // LED 2 - 0: inactive, 1 active
		replyBuffer[replyBufferLen++] = ' ';
		replyBufferLen += colaa::addUINT16ToBuffer(&(replyBuffer[replyBufferLen]), 0);
        // LED 3 - 0: inactive, 1 active
		replyBuffer[replyBufferLen++] = ' ';
		replyBufferLen += colaa::addUINT16ToBuffer(&(replyBuffer[replyBufferLen]), 0);

        colaa::addFrameToBuffer(msgBuffer, replyBuffer, &replyBufferLen);
        send(socket, msgBuffer, replyBufferLen, 0);
        std::cout << "Replying for status request" << std::endl;

        return true;
    }

    return false;
}

bool responseInvokeMethod(std::string * methodName, std::string * methodData, int socket)
{
    if ( *methodName == colaa::METHODNAME_STOP_MEASURE || *methodName == colaa::METHODNAME_START_MEASURE)
    {
        // TODO: Handle internal state, now accepts the request always

        // Build command
        BYTE msgBuffer[128];
        BYTE replyBuffer[128];
        UINT16 replyBufferLen = 0;

        replyBufferLen += colaa::addStringToBuffer(&(replyBuffer[0]), colaa::COMMAND_Method_Result_ByName);
		replyBuffer[replyBufferLen++] = ' ';
		replyBufferLen += colaa::addStringToBuffer(&(replyBuffer[replyBufferLen]), *methodName);
        /* Change user level - 0: noError, 1: not allowed
        */
		replyBuffer[replyBufferLen++] = ' ';
		replyBufferLen += colaa::addUINT8ToBuffer(&(replyBuffer[replyBufferLen]), 0);
    
        colaa::addFrameToBuffer(msgBuffer, replyBuffer, &replyBufferLen);
    
        send(socket, msgBuffer, replyBufferLen, 0);
        std::cout << "Replying for start/stop measurement request" << std::endl;

        return true;
    }
    else if ( *methodName == colaa::METHODNAME_LOGIN)
    {
        // TODO: Handle methodData, now allows access with eny data

        scannerStatus = 6;
        
        // Build command
        BYTE msgBuffer[128];
        BYTE replyBuffer[128];
        UINT16 replyBufferLen = 0;

        replyBufferLen += colaa::addStringToBuffer(&(replyBuffer[0]), colaa::COMMAND_Method_Result_ByName);
		replyBuffer[replyBufferLen++] = ' ';
		replyBufferLen += colaa::addStringToBuffer(&(replyBuffer[replyBufferLen]), *methodName);
        /* Change user level - 0: error, 1: success
        */
		replyBuffer[replyBufferLen++] = ' ';
		replyBufferLen += colaa::addUINT8ToBuffer(&(replyBuffer[replyBufferLen]), 1);
    
        colaa::addFrameToBuffer(msgBuffer, replyBuffer, &replyBufferLen);
    
        send(socket, msgBuffer, replyBufferLen, 0);
        std::cout << "Replying for login request" << std::endl;

        return true;
    }
    else if ( *methodName == colaa::METHODNAME_LOGOUT)
    {
        // TODO: Handle methodData, now allows access with eny data

        // Build command
        BYTE msgBuffer[128];
        BYTE replyBuffer[128];
        UINT16 replyBufferLen = 0;

        replyBufferLen += colaa::addStringToBuffer(&(replyBuffer[0]), colaa::COMMAND_Method_Result_ByName);
		replyBuffer[replyBufferLen++] = ' ';
		replyBufferLen += colaa::addStringToBuffer(&(replyBuffer[replyBufferLen]), *methodName);
        /* Change user level - 0: error, 1: success
        */
		replyBuffer[replyBufferLen++] = ' ';
		replyBufferLen += colaa::addUINT8ToBuffer(&(replyBuffer[replyBufferLen]), 1);
    
        colaa::addFrameToBuffer(msgBuffer, replyBuffer, &replyBufferLen);
    
        send(socket, msgBuffer, replyBufferLen, 0);
        std::cout << "Replying for logout request" << std::endl;
        
        scannerStatus = 7;

        return true;
    }
    else if ( *methodName == colaa::METHODNAME_STORE_PARAMETERS_PERMANENT)
    {
        // TODO: Handle methodData, now allows access with eny data

        // Build command
        BYTE msgBuffer[128];
        BYTE replyBuffer[128];
        UINT16 replyBufferLen = 0;

        replyBufferLen += colaa::addStringToBuffer(&(replyBuffer[0]), colaa::COMMAND_Method_Result_ByName);
		replyBuffer[replyBufferLen++] = ' ';
		replyBufferLen += colaa::addStringToBuffer(&(replyBuffer[replyBufferLen]), *methodName);
        /* Change user level - 0: error, 1: success
        */
		replyBuffer[replyBufferLen++] = ' ';
		replyBufferLen += colaa::addUINT8ToBuffer(&(replyBuffer[replyBufferLen]), 1);
    
        colaa::addFrameToBuffer(msgBuffer, replyBuffer, &replyBufferLen);
    
        send(socket, msgBuffer, replyBufferLen, 0);
        std::cout << "Replying for store parameters request" << std::endl;

        return true;
    }
    else if ( *methodName == colaa::METHODNAME_SET_SCANCONFIG)
    {
        // TODO: Handle methodData with simulation backend, now accepts anything
        // <STX>sMN{SPC}mLMPsetscancfg{SPC}+5000{SPC}+1{SPC}+5000{SPC}-450000{SPC}+2250000<ETX>
        // <STX>sAN{SPC}mLMPsetscancfg{SPC}0{SPC}1388{SPC}1{SPC}1388{SPC}FFF92230{SPC}225510<ETX>
        // Build command
        BYTE msgBuffer[128];
        BYTE replyBuffer[128];
        UINT16 replyBufferLen = 0;

        UINT32 freq = colaa::decodeUINT32(methodData);
        std::string dummy = colaa::getNextStringToken(methodData);
        UINT32 angRes = colaa::decodeUINT32(methodData);
        UINT32 start = colaa::decodeUINT32(methodData);
        UINT32 stop = colaa::decodeUINT32(methodData);
        
        std::cout   << "freq: " << freq
                    << "angRes: " << angRes
                    << "start: " << start
                    << "stop: " << stop << std::endl;

        replyBufferLen += colaa::addStringToBuffer(&(replyBuffer[0]), colaa::COMMAND_Method_Result_ByName);
		replyBuffer[replyBufferLen++] = ' ';
		replyBufferLen += colaa::addStringToBuffer(&(replyBuffer[replyBufferLen]), *methodName);
        /* Status Code 8-bit
        0 no Error
        1 Frequency Error
        2 Resolution Error
        3 Res. and Scn. Error
        4 Scan area Error
        5 other Errors
        */
		replyBuffer[replyBufferLen++] = ' ';
		replyBufferLen += colaa::addUINT8ToBuffer(&(replyBuffer[replyBufferLen]), 0);
    
        /* Scan Frequency [1/100Hz] 32-bit
        LMS5xx:
        25Hz: 9C4h (2500d)
        35Hz: DACh (3500d)
        50Hz: 1388h (5000d)
        75Hz: 1A0Bh (7500d)
        100Hz: 2710h (10000d)
        */
		replyBuffer[replyBufferLen++] = ' ';
		replyBufferLen += colaa::addStringToBuffer(&(replyBuffer[replyBufferLen]), int_to_hex(freq));

        /* Value Reserved 16-bit must be 1*/
		replyBuffer[replyBufferLen++] = ' ';
		replyBufferLen += colaa::addUINT16ToBuffer(&(replyBuffer[replyBufferLen]), 1);

        /* Angular resolution Angle Uint_32 Resolution[1/10000°] 
        LMS5xx:
        0,1667°: 683h (1667d)
        0,25°: 9C4h (2500d)
        0,333°: D05h (3333d)
        0,5°: : 1388h (5000d)
        0,667°: 1A0Bh (6670d)
        1°:
        2710h (10000d)
        */
		replyBuffer[replyBufferLen++] = ' ';
		replyBufferLen += colaa::addStringToBuffer(&(replyBuffer[replyBufferLen]), int_to_hex(angRes));

        /* Start angle [1/10000°] UINT32
        LMS5xx:
        FFFF3CB0h..1C3A90h
        (–50000d..+1850000d)
        */
		replyBuffer[replyBufferLen++] = ' ';
		replyBufferLen += colaa::addStringToBuffer(&(replyBuffer[replyBufferLen]), int_to_hex(start));

        /* Stop angle [1/10000°] UINT32
        LMS5xx:
        FFFF3CB0h..1C3A90h
        (–50000d..+1850000d)
        */
		replyBuffer[replyBufferLen++] = ' ';
		replyBufferLen += colaa::addStringToBuffer(&(replyBuffer[replyBufferLen]), int_to_hex(stop));

        colaa::addFrameToBuffer(msgBuffer, replyBuffer, &replyBufferLen);
    
        send(socket, msgBuffer, replyBufferLen, 0);
        std::cout << "Replying for scan config request" << std::endl;

        return true;
    }
    else if ( *methodName == colaa::EVENTNAME_SUBSCRIBE_SCANS)
    {
        // TODO: Handle methodData, now allows access with eny data
        sendMeasurementData = colaa::decodeUINT8(methodData);


        // Build command
        BYTE msgBuffer[128];
        BYTE replyBuffer[128];
        UINT16 replyBufferLen = 0;

        replyBufferLen += colaa::addStringToBuffer(&(replyBuffer[0]), colaa::COMMAND_Register_Event_Answer);
		replyBuffer[replyBufferLen++] = ' ';
		replyBufferLen += colaa::addStringToBuffer(&(replyBuffer[replyBufferLen]), *methodName);
        /* Change user level - 0: error, 1: success
        */
		replyBuffer[replyBufferLen++] = ' ';
		replyBufferLen += colaa::addUINT8ToBuffer(&(replyBuffer[replyBufferLen]), 1);
    
        colaa::addFrameToBuffer(msgBuffer, replyBuffer, &replyBufferLen);
    
        send(socket, msgBuffer, replyBufferLen, 0);
        std::cout << "Replying for data subscription event: " << sendMeasurementData << std::endl;
        //sendMeasurementMessage(socket, false);
        return true;
    }


    return false;
}

bool responseWriteVariable(std::string * varName, std::string * varData, int socket)
{
    // TODO: Var specific behaviour needed
    //if ( *varName == colaa::METHODNAME_STOP_MEASURE)
    {
        // TODO: Handle internal state, now accepts the request always

        // Build command
        BYTE msgBuffer[128];
        BYTE replyBuffer[128];
        UINT16 replyBufferLen = 0;

        replyBufferLen += colaa::addStringToBuffer(&(replyBuffer[0]), colaa::COMMAND_Write_Variable_Answer);
		replyBuffer[replyBufferLen++] = ' ';
		replyBufferLen += colaa::addStringToBuffer(&(replyBuffer[replyBufferLen]), *varName);
    
        colaa::addFrameToBuffer(msgBuffer, replyBuffer, &replyBufferLen);
    
        send(socket, msgBuffer, replyBufferLen, 0);
        std::cout << "Replying for variable writing request" << std::endl;

        return true;
    }
    return false;
}

bool handleCommand(std::string * command, int socket)
{
    std::string token = colaa::getNextStringToken(command);

    std::cout << "Received command: " << token << " - remainder is " << *command << std::endl;

    if ( token == colaa::COMMAND_Read_Variable_ByName)
    {
        token = colaa::getNextStringToken(command);
        std::cout << "read variable by name - var " << token << std::endl;
        return responseReadVariable(&token, socket);
    }
    else if ( token == colaa::COMMAND_Invoke_Method_ByName || token == colaa::COMMAND_Register_Event_ByName)
    {
        token = colaa::getNextStringToken(command);
        std::cout << "invoke method by name - var " << token << std::endl;
        return responseInvokeMethod(&token, command, socket);

    }
    else if ( token == colaa::COMMAND_Write_Variable_ByName)
    {
        token = colaa::getNextStringToken(command);
        std::cout << "write variable by name - var " << token << std::endl;
        return responseWriteVariable(&token, command, socket);

    }

    return false;
}

int main(int argc, char *argv[])
{
    int opt = TRUE;   
    int master_socket , addrlen , new_socket , client_socket[30] ,  
          max_clients = 30 , activity, i , valread , sd;   
    int max_sd;   
    struct sockaddr_in address;   
    UINT16 port;

    char buffer[1025];  //data buffer of 1K  

	// Response buffer
	UINT32 m_numberOfBytesInCommandBuffer = 0; ///< Number of bytes in buffer
	UINT8 m_commandBuffer[1024]; ///< Receive buffer for everything except scan data and eval case data.

	// Receive buffer
	UINT32 m_numberOfBytesInReceiveBuffer = 0; ///< Number of bytes in buffer
	UINT8 m_receiveBuffer[25000]; ///< Low-Level receive buffer for all data (25000 should be enough for NAV300 Events)

    //set of socket descriptors  
    fd_set readfds;   
        // Let's check if port number is supplied or not..
    if (argc != 3) {
        std::cerr << "Run program as 'program <port> <ros_topic>'\n";
        return -1;
    }
/* ros arguments added to command line, take care of those.
[rearscan_jyka_M51BC_13524_8401147609633836867-10] process has died [pid 13585, exit code 255, cmd /home/jyka/sandvik/am_ros_simulator/devel/lib/sick_scan/lms5xx_tcp_server 12346 /truck/rear_laser/scan __name:=rearscan_jyka_M51BC_13524_8401147609633836867 __log:=/home/jyka/.ros/log/2918322c-6893-11ea-9ca9-d850e65b70c0/rearscan_jyka_M51BC_13524_8401147609633836867-10.log].
log file: /home/jyka/.ros/log/2918322c-6893-11ea-9ca9-d850e65b70c0/rearscan_jyka_M51BC_13524_8401147609633836867-10*.log
*/

    std::istringstream iss( argv[1] );
    int val;

    if (not(iss >> port))
    {
        // Conversion unsuccessful
        return -1;
    }   
    auto &portNum = argv[1];
    const unsigned int backLog = 8;  // number of connections allowed on the incoming queue

    ros::init(argc, argv, "lms5xx_tcp_server", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe(argv[2], 1000, rosLaserCallback);


    //initialise all client_socket[] to 0 so not checked  
    for (i = 0; i < max_clients; i++)   
    {   
        client_socket[i] = 0;   
    }   
         
/*
    addrinfo hints, *res, *p;    // we need 2 pointers, res to hold and p to iterate over
    memset(&hints, 0, sizeof(hints));

    // for more explanation, man socket
    hints.ai_family   = AF_UNSPEC;    // don't specify which IP version to use yet
    hints.ai_socktype = SOCK_STREAM;  // SOCK_STREAM refers to TCP, SOCK_DGRAM will be?
    hints.ai_flags    = AI_PASSIVE;


    // man getaddrinfo
    int gAddRes = getaddrinfo(NULL, portNum, &hints, &res);
    if (gAddRes != 0) {
        std::cerr << gai_strerror(gAddRes) << "\n";
        return -2;
    }

    std::cout << "Detecting addresses" << std::endl;

    unsigned int numOfAddr = 0;
    char ipStr[INET6_ADDRSTRLEN];    // ipv6 length makes sure both ipv4/6 addresses can be stored in this variable


    // Now since getaddrinfo() has given us a list of addresses
    // we're going to iterate over them and ask user to choose one
    // address for program to bind to
    for (p = res; p != NULL; p = p->ai_next) {
        void *addr;
        std::string ipVer;

        // if address is ipv4 address
        if (p->ai_family == AF_INET) {
            ipVer             = "IPv4";
            sockaddr_in *ipv4 = reinterpret_cast<sockaddr_in *>(p->ai_addr);
            addr              = &(ipv4->sin_addr);
            ++numOfAddr;
        }

        // if address is ipv6 address
        else {
            ipVer              = "IPv6";
            sockaddr_in6 *ipv6 = reinterpret_cast<sockaddr_in6 *>(p->ai_addr);
            addr               = &(ipv6->sin6_addr);
            ++numOfAddr;
        }

        // convert IPv4 and IPv6 addresses from binary to text form
        inet_ntop(p->ai_family, addr, ipStr, sizeof(ipStr));
        std::cout << "(" << numOfAddr << ") " << ipVer << " : " << ipStr
                  << std::endl;
    }

    // if no addresses found :(
    if (!numOfAddr) {
        std::cerr << "Found no host address to use\n";
        return -3;
    }

    // ask user to choose an address
    std::cout << "Enter the number of host address to bind with: ";
    unsigned int choice = 0;
    bool madeChoice     = false;
    do {
        std::cin >> choice;
        if (choice > (numOfAddr + 1) || choice < 1) {
            madeChoice = false;
            std::cout << "Wrong choice, try again!" << std::endl;
        } else
            madeChoice = true;
    } while (!madeChoice);



    p = res;
*/

    // let's create a new socket, socketFD is returned as descriptor
    // man socket for more information
    // these calls usually return -1 as result of some error
//    master_socket = socket(p->ai_family, p->ai_socktype, p->ai_protocol);
    master_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (master_socket == -1) {
        std::cerr << "Error while creating socket\n";
        //freeaddrinfo(res);
        return -4;
    }

    //set master socket to allow multiple connections ,  
    //this is just a good habit, it will work without this  
    if( setsockopt(master_socket, SOL_SOCKET, SO_REUSEADDR, (char *)&opt,  
          sizeof(opt)) < 0 )   
    {   
        std::cerr << "Error while setting socket options\n";
        //freeaddrinfo(res);
        return -7;
    }   
     
    //struct sockaddr_in address; 
    addrlen = sizeof(address);
    address.sin_family = AF_INET; 
    address.sin_addr.s_addr = INADDR_ANY; 
    address.sin_port = htons( port ); 

    // Let's bind address to our socket we've just created
//    int bindR = bind(master_socket, p->ai_addr, p->ai_addrlen);
    int bindR = bind(master_socket, (struct sockaddr *)&address, sizeof(address));
    if (bindR == -1) {
        std::cerr << "Error while binding socket\n";
        
        // if some error occurs, make sure to close socket and free resources
        close(master_socket);
        //freeaddrinfo(res);
        return -5;
    }


    // finally start listening for connections on our socket
    int listenR = listen(master_socket, backLog);
    if (listenR == -1) {
        std::cerr << "Error while Listening on socket\n";

        // if some error occurs, make sure to close socket and free resources
        close(master_socket);
        //freeaddrinfo(res);
        return -6;
    }

    
    // structure large enough to hold client's address
    sockaddr_storage client_addr;
    socklen_t client_addr_size = sizeof(client_addr);


    const std::string response = "Hello World";
    const std::string message = "Sick LMS5xx simulation interface v0.1";


    // a fresh infinite loop to communicate with incoming connections
    // this will take client connections one at a time
    // in further examples, we're going to use fork() call for each client connection
    ros::Rate r(1000); // 1000 hz
    while (ros::ok()) {

        //clear the socket set  
        FD_ZERO(&readfds);   
     
        //add master socket to set  
        FD_SET(master_socket, &readfds);   
        max_sd = master_socket;   
             
        //add child sockets to set  
        for ( i = 0 ; i < max_clients ; i++)   
        {   
            //socket descriptor  
            sd = client_socket[i];   
                 
            //if valid socket descriptor then add to read list  
            if(sd > 0)   
                FD_SET( sd , &readfds);   
                 
            //highest file descriptor number, need it for the select function  
            if(sd > max_sd)   
                max_sd = sd;   
        }   
     
        //wait for an activity on one of the sockets , timeout is NULL ,  
        //so wait indefinitely  
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 1000;
        activity = select( max_sd + 1 , &readfds , NULL , NULL , &timeout);   
       
        if ((activity < 0) && (errno!=EINTR))   
        {   
            std::cerr << "Error in socket select\n";  
        }   
             
        //If something happened on the master socket ,  
        //then its an incoming connection  
        if (FD_ISSET(master_socket, &readfds))   
        {   
            if ((new_socket = accept(master_socket,  
                    (struct sockaddr *)&address, (socklen_t*)&addrlen))<0)   
            {   
                std::cerr << "Error while Accepting on socket\n";   
                exit(EXIT_FAILURE);   
            }   
             
            //inform user of socket number - used in send and receive commands  
            std::cout   << "New connection, socket fd is " << new_socket
                        << ", ip: " << inet_ntoa(address.sin_addr)
                        << ", port: " << ntohs(address.sin_port)
                        << std::endl;

            //send new connection greeting message  
            // send call sends the data you specify as second param and it's length as 3rd param, also returns how many bytes were actually sent
            auto bytes_sent = send(new_socket, message.data(), message.length(), 0);
                 
            //add new socket to array of sockets  
            for (i = 0; i < max_clients; i++)   
            {   
                //if position is empty  
                if( client_socket[i] == 0 )   
                {   
                    client_socket[i] = new_socket;  
                    scannerSocket = new_socket; 
                    std::cout << "Adding to list of sockets as " << i << std::endl;
                    break;   
                }   
            }   
        }   
             
        //else its some IO operation on some other socket 
        for (i = 0; i < max_clients; i++)   
        {   
            sd = client_socket[i];   
                 
            if (FD_ISSET( sd , &readfds))   
            {   
                //Check if it was for closing , and also read the  
                //incoming message  
                if ((valread = read( sd , buffer, 1024)) == 0)   
                {   
                    //Somebody disconnected , get his details and print  
                    getpeername(sd , (struct sockaddr*)&address , (socklen_t*)&addrlen);   
                    std::cout   << "Host disconnected , ip " << inet_ntoa(address.sin_addr)
                                << "port " << ntohs(address.sin_port)
                                << std::endl;
                         
                    //Close the socket and mark as 0 in list for reuse  
                    close( sd );   
                    client_socket[i] = 0;   
                    scannerStatus = 6;
                    sendMeasurementData = 0;
                    scannerSocket = 0;
                }   
                     
                //Echo back the message that came in  
                else 
                {   
                    //set the string terminating NULL byte on the end  
                    //of the data read  
                    buffer[valread] = '\0';   
                    std::cout << buffer << std::endl;
                    //send(sd , buffer , strlen(buffer) , 0 );   

                    // <STX> = 0x02
                    // {SPC} = 0x20
                    // <ETX> = 0x03

                    // Status request
                    //<STX>sRN{SPC}STlms<ETX>
                    //02 73 52 4E 20 53 54 6C 6D 73 03

                    // Status reply - ready = 6
                    //<STX>sRA{SPC}STlms{SPC}6{SPC}0{SPC}8{SPC}16:36:54{SPC}8{SPC}17.03.2030{SPC}0{SPC}0{SPC}0<ETX>
                    // Status reply - measuring = 7
                    //<STX>sRA{SPC}STlms{SPC}7{SPC}0{SPC}8{SPC}16:36:54{SPC}8{SPC}17.03.2030{SPC}0{SPC}0{SPC}0<ETX>
                    UINT32 remainingSpace = sizeof(m_receiveBuffer) - m_numberOfBytesInReceiveBuffer;
                    UINT32 bytesToBeTransferred = valread;
                    if (remainingSpace < valread)
                    {
                        bytesToBeTransferred = remainingSpace;
//                        printWarning("SopasBase::readCallbackFunction(): Input buffer space is to small, transferring only " +
//                                                        ::toString(bytesToBeTransferred) + " of " + ::toString(numOfBytes) + " bytes.");
                    }
                    else
                    {
//                        printInfoMessage("SopasBase::readCallbackFunction(): Transferring " + ::toString(bytesToBeTransferred) +
//                                                        " bytes from TCP to input buffer.", beVerboseHere);
                    }

                    if (bytesToBeTransferred > 0)
                    {
                        // Data can be transferred into our input buffer
                        memcpy(&(m_receiveBuffer[m_numberOfBytesInReceiveBuffer]), buffer, bytesToBeTransferred);
                        m_numberOfBytesInReceiveBuffer += bytesToBeTransferred;

                        UINT32 frameLen = 0;

                        while (1)
                        {
                            // Now work on the input buffer until all received datasets are processed
                            bool frameFound = colaa::findFrameInReceiveBuffer(m_receiveBuffer, &m_numberOfBytesInReceiveBuffer, &frameLen);

                            if (!frameFound)
                            {
                                // Framesize = 0: There is no valid frame in the buffer. The buffer is either empty or the frame
                                // is incomplete, so leave the loop
//                                printInfoMessage("SopasBase::readCallbackFunction(): No complete frame in input buffer, we are done.", beVerboseHere);

                                // Leave the loop
                                break;
                            }
                            else
                            {
                                // A frame was found in the buffer, so process it now.
//                                printInfoMessage("SopasBase::readCallbackFunction(): Processing a frame of length " + ::toString(frame.size()) + " bytes.", beVerboseHere);
//                                processFrame(TBD);
                                std::string rxdata = colaa::convertRxBufferToString(m_receiveBuffer, frameLen);
                                std::cout << "Received: \"" << rxdata << "\"" << std::endl;
                                handleCommand(&rxdata, sd);

                                m_numberOfBytesInReceiveBuffer = m_numberOfBytesInReceiveBuffer-frameLen;
                                memmove(&(m_receiveBuffer[0]), &(m_receiveBuffer[frameLen]), m_numberOfBytesInReceiveBuffer);
                            }
                        }
                    }
                    else
                    {
                        // There was input data from the TCP interface, but our input buffer was unable to hold a single byte.
                        // Either we have not read data from our buffer for a long time, or something has gone wrong. To re-sync,
                        // we clear the input buffer here.
                        m_numberOfBytesInReceiveBuffer = 0;
                    }

                }   
            }   
        }

        ros::spinOnce();
        //r.sleep();
    }

    return 0;
}
