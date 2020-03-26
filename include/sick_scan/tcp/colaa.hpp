//
// colaa.hpp
//
// (c) 2011 SICK AG, Hamburg, Germany
//

#ifndef COLAA_HPP
#define COLAA_HPP

#include "sick_scan/tcp/BasicDatatypes.hpp"
// #include <boost/tokenizer.hpp>

/**
 * Parser functions for a partly implementation of the CoLa-A
 * protocol, needed for communication with SICK sensors.
 */
namespace colaa
{

// typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
// typedef boost::char_separator<char> separator_type;

const std::string EVENTNAME_SUBSCRIBE_EVALCASES("LFErec");
const std::string EVENTNAME_SUBSCRIBE_SCANS("LMDscandata");
const std::string METHODNAME_LOGIN("SetAccessMode");
const std::string METHODNAME_LOGOUT("Run");
const std::string METHODNAME_SET_SCANCONFIG("mLMPsetscancfg");
const std::string METHODNAME_START_MEASURE("LMCstartmeas");
const std::string METHODNAME_STOP_MEASURE("LMCstopmeas");
const std::string METHODNAME_STORE_PARAMETERS_PERMANENT("mEEwriteall");
const std::string VARIABLENAME_SCANCONFIG("LMPscancfg");
const std::string VARIABLENAME_DATAOUTPUTRANGE("LMPoutputRange");
const std::string VARIABLENAME_SCANDATACONFIG("LMDscandatacfg");
const std::string VARIABLENAME_DEVICEIDENT("DeviceIdent");
const std::string VARIABLENAME_DEVICESTATUS("STlms");
// sopas commands
const std::string COMMAND_Read_Variable_ByIndex("RI");
const std::string COMMAND_Write_Variable_ByIndex("WI");
const std::string COMMAND_Invoke_Method_ByIndex("MI");
const std::string COMMAND_Method_Result_ByIndex("AI");
const std::string COMMAND_Register_Event_ByIndex("EI");
const std::string COMMAND_Send_Event_ByIndex("SI"); // receive data event

const std::string COMMAND_Read_Variable_Answer("RA");
const std::string COMMAND_Write_Variable_Answer("WA");
const std::string COMMAND_Invoke_Method_Answer("MA");
const std::string COMMAND_Method_Result_Answer("AA");
const std::string COMMAND_Register_Event_Answer("EA");
const std::string COMMAND_Event_Acknowledge("SA");

const std::string COMMAND_Read_Variable_ByName("RN");
const std::string COMMAND_Write_Variable_ByName("WN");
const std::string COMMAND_Invoke_Method_ByName("MN");
const std::string COMMAND_Method_Result_ByName("AN");
const std::string COMMAND_Register_Event_ByName("EN");
const std::string COMMAND_Send_Event_ByName("SN"); // receive data event

UINT16 getValueOfChar(UINT8 c);
UINT8 nibbleToAscii(UINT8 value);

void addFrameToBuffer(UINT8* sendBuffer, UINT8* cmdBuffer, UINT16* len);
UINT16 addUINT8ToBuffer(UINT8* buffer, UINT8 value);
UINT16 addUINT16ToBuffer(UINT8* buffer, UINT16 value);
UINT16 addINT8ToBuffer(UINT8* buffer, INT8 value);
UINT16 addINT32ToBuffer(UINT8* buffer, INT32 value);
UINT16 addUINT32ToBuffer(UINT8* buffer, UINT32 value);
UINT16 addStringToBuffer(UINT8* buffer, const std::string& text);
std::string getNextStringToken(std::string* rxData);

namespace detail
{
UINT16 writeToBuffer(BYTE* buffer, double value);
inline UINT16 writeToBuffer(BYTE* buffer, UINT8 value) { return addUINT8ToBuffer(buffer, value); }
inline UINT16 writeToBuffer(BYTE* buffer, INT8 value) { return addINT8ToBuffer(buffer, value); }
inline UINT16 writeToBuffer(BYTE* buffer, UINT16 value) { return addUINT16ToBuffer(buffer, value); }
inline UINT16 writeToBuffer(BYTE* buffer, INT16 value) { return addUINT16ToBuffer(buffer, value); }
inline UINT16 writeToBuffer(BYTE* buffer, UINT32 value) { return addUINT32ToBuffer(buffer, value); }
inline UINT16 writeToBuffer(BYTE* buffer, INT32 value) { return addINT32ToBuffer(buffer, value); }
inline UINT16 writeToBuffer(BYTE* buffer, const std::string& value) { return addStringToBuffer(buffer, value); }
}

double decodeReal(std::string* rxData);
INT16 decodeINT16(std::string* rxData);
INT32 decodeINT32(std::string* rxData);
UINT32 decodeUINT32(std::string* rxData);
UINT16 decodeUINT16(std::string* rxData);
UINT8 decodeUINT8(std::string* rxData);
UINT32 decodeXByte(std::string* rxData, UINT16 len);
std::string decodeString(std::string* rxData, UINT16 len = 0);
std::string convertRxBufferToString(UINT8* buffer, UINT16 bufferLen);
bool findFrameInReceiveBuffer(UINT8* recBuf, UINT32* bytesInRecBuf, UINT32* frameLen);

/// set of more efficient functions that do not copy strings (should be prefered in use together with the colaa::tokenizer)
double decodeReal(const std::string& rxData);
INT16 decodeINT16(const std::string& rxData);
INT32 decodeINT32(const std::string& rxData);
UINT8 decodeUINT8(const std::string& rxData);
UINT16 decodeUINT16(const std::string& rxData);
UINT32 decodeUINT32(const std::string& rxData);

//
UINT16 decodeUINT16(BYTE* buffer);

namespace detail
{
/// General template which is unimplemented; implemented specializations follow below
template<typename T>
inline T read (const std::string& str)
{
//	BOOST_STATIC_ASSERT(sizeof(T) == 0); // must not be instantiated
	return T(); // to avoid additional compiler errors
}
template<> inline double read<double>(const std::string& rxData) { return decodeReal(rxData); }
template<> inline INT16 read<INT16>(const std::string& rxData) { return decodeINT16(rxData); }
template<> inline INT32 read<INT32>(const std::string& rxData) { return decodeINT32(rxData); }
template<> inline INT8 read<INT8>(const std::string& rxData) { return decodeUINT8(rxData); }
template<> inline UINT8 read<UINT8>(const std::string& rxData) { return decodeUINT8(rxData); }
template<> inline UINT32 read<UINT32>(const std::string& rxData) { return decodeUINT32(rxData); }
template<> inline UINT16 read<UINT16>(const std::string& rxData) { return decodeUINT16(rxData); }
template<> inline std::string read<std::string>(const std::string& rxData) { return rxData; }
}

//
// Lese ein XByte-Array bekannter Laenge (1..4 Bytes) und verpacke es als UINT32-Wert.
// Das 1. empfangene Byte steht in den unteren 8 Bit des Ergebniswerts, usw.
//
// HINWEIS: der Iterator wird weitergeschoben len-1 mal. Um das naechste Element zu lesen
//          muss vorher ++tok aufgerufen werden.
//
// @param begin = Startpunkt, von wo aus einzelne Stringtokens in Zahlen verwandelt werden
// @param end = Ende des containers, ueber den iteriert wird
// @param len = Anzahl der Bytes (= Array-Elemente)
//
//UINT32 decodeXByte(tokenizer::const_iterator& tok, const tokenizer::const_iterator& end, UINT16 len);

} // END namespace colaa
#endif
