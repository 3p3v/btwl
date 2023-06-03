#include "include/Sim800l.hpp"
#include "Sim800l.hpp"
#include "string.h"
#include "stdio.h"
#include <esp_log.h>

Sim800l::Sim800l()
   : receivedLen(0) {
      memset(receivedData, '\0', SIM800L_DEF_BUF_SIZE);
   }

Sim800l::~Sim800l() {}

Sim800lError Sim800l::execAT()
{
    return sendAT();
}

Sim800lError Sim800l::execCCID()
{
    return sendAT("CCID");
}

Sim800lError Sim800l::readCREG()
{
    return sendAT("CREG?");
}

/*  AT+CSQ Signal Quality Report;
*   returns "rrsi" and "ber"
*/
// Sim800lError execCSQ(int &rssi, int &ber) {
//    // sendAT("CSQ");
// }

/* AT+CCID Show ICCID */
// Sim800lError execCCID(char *ccid);

/* AT+CREG Network Registration */
// Sim800lError readCREG(unsigned char &n, int &stat);

/* AT+SAPBR Bearer Settings for Applications Based on IP */
Sim800lError Sim800l::writeSAPBR(const unsigned char cmd_type, const unsigned char cid, const char *ConParamTag, const char *ConParamValue) {
   char cmd_type_str[11] = {};
   char cid_str[11] = {};

   sprintf(cmd_type_str, "%i", (int)cmd_type);
   sprintf(cid_str, "%i", (int)cid);

   if(ConParamTag != nullptr || ConParamValue != nullptr)
      return sendAT("SAPBR", {cmd_type_str, cid_str, ConParamTag, ConParamValue});
   else
      return sendAT("SAPBR", {cmd_type_str, cid_str});
}

Sim800lError Sim800l::writeSAPBR(unsigned char cmd_type, unsigned char cid) {
   return writeSAPBR(cmd_type, cid, nullptr, nullptr);
}

/* AT+HTTPINIT Initialize HTTP Service  */
Sim800lError Sim800l::execHTTPINIT() {
   return sendAT("HTTPINIT");
}

/* AT+HTTPPARA Set HTTP Parameters Value */
Sim800lError Sim800l::writeHTTPPARA(const char *HTTPParamTag, const int HTTPParamValue) {
   char HTTPParamValue_str[11] = {};

   sprintf(HTTPParamValue_str, "%i", (int)HTTPParamValue);

   return writeHTTPPARA(HTTPParamTag, HTTPParamValue_str);
}

Sim800lError Sim800l::writeHTTPPARA(const char *HTTPParamTag, const char *HTTPParamValue) {
   return sendAT("HTTPPARA", {HTTPParamTag, HTTPParamValue});
}

/* AT+HTTPDATA Input HTTP Data *///TODO
Sim800lError Sim800l::writeHTTPDATA(const int size, const int time, const char *inputData) {
   char size_str[11] = {};
   char time_str[11] = {};

   sprintf(size_str, "%i", (int)size);
   sprintf(time_str, "%i", (int)time);

   if(sendAT("HTTPDATA", {size_str, time_str}) == Sim800lOk) {
      ESP_LOGI("SIM", "TEST");
      simDelay(100);
      if(sendData(inputData) != Sim800lOk)
         return Sim800lErr;

      ESP_LOGI("SIM", "TEST2");
      if(receiveData() <= 0)
         return Sim800lErr;

      ESP_LOGI("SIM", "TEST3");
      return Sim800lOk;
   }

   return Sim800lErr;
}

/*  AT+HTTPACTION HTTP Method Action
 *   method =:
 *   - 0 - GET,
 *   - 1 - POST,
 *   - 2 - HEAD.
 */
Sim800lError Sim800l::writeHTTPACTION(const int method) {
   char method_str[10] = {};

   sprintf(method_str, "%i", (int)method);

   if(sendAT("HTTPACTION", {method_str}) == Sim800lOk) {
      /* Check if got responce from server */
      char resp[] = "+HTTPACTION:";
      char * ptr = strstr(receivedData, resp);
      
      if(ptr != NULL) {
         ptr += strlen(resp) + 4;

         if(strstr(ptr, "200") == NULL)
            return sim800lServerErr;

         return Sim800lOk;
      }

      /* Redundand cycle */
      if(receiveData() <= 0)
         return Sim800lRecErr;

      ptr = strstr(receivedData, resp);

      if(ptr == NULL)
         return Sim800lErr;

      ptr += strlen(resp) + 4;

      if(strstr(ptr, "200") == NULL)
         return sim800lServerErr;

      return Sim800lOk;
   }

   return Sim800lErr;
}

/* AT+HTTPREAD Read the HTTP Server Response */
// int writeHTTPREAD(const int start_address, const int byte_size, char *receivedData) {

// }

Sim800lError Sim800l::execHTTPREAD() {
   return sendAT("HTTPREAD");
}

/* AT+HTTPTERM Terminate HTTP Service */
Sim800lError Sim800l::execHTTPTERM() {
   return sendAT("HTTPTERM");
}

Sim800lError Sim800l::writeCSCLK(const unsigned char n) {
   char nStr[10] = {};

   sprintf(nStr, "%i", (int)n);

   return sendAT("CSCLK", {nStr});
}

Sim800lError Sim800l::writeCFUN(const unsigned char fun)
{
   char funStr[10] = {};

   sprintf(funStr, "%i", (int)fun);
   
   return sendAT("CFUN", {funStr});
}

Sim800lError Sim800l::setDRT(Sim800lPin set)
{
   return Sim800lError();
}

Sim800lError Sim800l::setRST(Sim800lPin set)
{
   return Sim800lError();
}

Sim800lError Sim800l::writeCPOWD(const unsigned char n)
{
   char nStr[10] = {};

   sprintf(nStr, "%i", (int)n);

   return sendAT("CPOWD", {nStr});
}

// Sim800lError sendData(const char *data, int len);
Sim800lError Sim800l::sendData(const char *data) {
   return Sim800lOk;
}

int Sim800l::receiveData() {
   return (int)Sim800lOk;
}

// Sim800lError receiveErr();

Sim800lError Sim800l::sendAT() {
   return sendReceiveRetriveData("AT\r\n");
}

Sim800lError Sim800l::retrieveErr()
{
   if(strstr(receivedData, "OK") != NULL || strstr(receivedData, "DOWNLOAD") != NULL)
      return Sim800lOk;
   else if (strstr(receivedData, "ERROR"))
      return Sim800lRecErr;
   else 
      return Sim800lErr;
}

Sim800lError Sim800l::sendReceiveRetriveData(const char * data)
{
   Sim800lError err = sendData(data);
   if(err != Sim800lOk)
      return err;

   int errInt = receiveData();
   if(errInt <= 0)
      return (Sim800lError)errInt;

   err = retrieveErr();

   ESP_LOGI("SIM","%i, %i, %s", err, receivedLen, receivedData);

   if(err == Sim800lRecErr)
      return err;
   else if (err == Sim800lOk)
      return err;
   else {
      errInt = receiveData();
      if(errInt <= 0)
         return (Sim800lError)errInt;

      err = retrieveErr();

      return err;
   }
}

void Sim800l::simDelay(int ms) {}

Sim800lError Sim800l::sendAT(const char *type) {
   char beg[] = "AT+";
   char newLine[] = "\r\n";

   int len = strlen(type) + strlen(beg) + strlen(newLine);
   char temp[len + 1] = {};

   memcpy(temp, beg, strlen(beg));
   int len2 = strlen(beg);

   memcpy(temp + len2, type, strlen(type));
   len2 += strlen(type);

   memcpy(temp + len2, newLine, strlen(newLine));
   len2 += strlen(newLine);

   return sendReceiveRetriveData(temp);
}

Sim800lError Sim800l::sendAT(const char *type, std::initializer_list<const char *> params)
{
   char beg[] = "AT+";
   char end[] = "=";
   char newLine[] = "\r\n";

   int len = strlen(type) + strlen(beg) + strlen(end);

   for(const char * i : params) {
      len += strlen(i) + 1;
   }
   len--;

   char temp[len + strlen(newLine) + 1] = {};
   memcpy(temp, beg, strlen(beg));
   int len2 = strlen(beg);

   memcpy(temp + len2, type, strlen(type));
   len2 += strlen(type);

   memcpy(temp + len2, end, strlen(end));
   len2 += strlen(end);

   for(const char * i : params) {
      memcpy(temp + len2, i, strlen(i));
      len2 += strlen(i);
      if(len2 < len) {
         *(temp + len2) = ',';
         len2 += 1;
      }
   }

   memcpy(temp + len2, newLine, strlen(newLine));
   len2 += strlen(newLine);

   return sendReceiveRetriveData(temp);
}

// Sim800lError retriveParams(std::initializer_list<const char *> params);