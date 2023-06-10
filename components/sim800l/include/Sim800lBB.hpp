#include "Sim800l.hpp"

#define SIM800L_DEF_URL_LEN 100

typedef enum {
    Sim800lNoConnection = 0,
    Sim800lHTTPPOSTSent = 1,
    Sim800lHTTPRespReceived = 2,
    Sim800lUARTInitialised = 4
} Sim800lStatus;

/* Wrapper; virtual class specyfying commands */
class Sim800lBB : protected Sim800l {
public:
    Sim800lBB(const char * url);
    virtual ~Sim800lBB() = 0;

    virtual bool getSleep() final;
    virtual void setSleep(bool sleep) final;

    /* Set defualt server */
    virtual void setDefualtUrl(const char * url);

    /* Initialization of UART */
    virtual Sim800lError init();
    virtual Sim800lError deinit();
    virtual Sim800lError reinit();

    /* AT */
    /* Check if Sim800l responds */
    virtual Sim800lError handshake() final;
    /* Check x times until the module responds correctly */
    virtual Sim800lError handshake(int x) final;

    /* AT+CSQ */
    /* Check signal quality */
    // virtual Sim800lError checkSignal(int & rssi, int & ber) final;
    
    /* AT+CCID */
    /* Get SIM info / Test SIM card */
    virtual Sim800lError getSIMInfo() final;

    /* AT+CREG */
    /* Check if registered */
    virtual Sim800lError checkIfRegistered() final;

    /* AT+SAPBR */
    /* "CONTYPE" Type of Internet connection */
    virtual Sim800lError setConnectionType(const char * type) final;
    /* "APN" Access point name string: maximum 64 characters */
    virtual Sim800lError setAccessPoint(const char * ap);

    /* POST */
    virtual Sim800lError sendHTTPPOST(const char * url, const char * data, char * output) final;
    virtual Sim800lError sendHTTPPOST(const char * data, char * output) final;
    /* PUT */
    // virtual Sim800lError sendHTTPPUT(const char * url, const char * data, char * output) final;
    // virtual Sim800lError sendHTTPPUT(const char * data) final;

    virtual Sim800lError sendHTTPGET(const char * url, char * output) final;
    // virtual Sim800lError sendHTTPPUT(const char * data) final;

    /* Enable sleep mode 1 */
    virtual Sim800lError sleepModeEnable() final;
    /* Disable sleep mode 1 */
    virtual Sim800lError sleepModeDisable() final;

    /* Airplane mode ?? */
    virtual Sim800lError airplaneModeEnable() final;
    virtual Sim800lError airplaneModeDisable() final;
    /* Reset */
    virtual Sim800lError resetNormal() final;
    virtual Sim800lError resetUrgent() final;
    virtual Sim800lError resetForce() final;

    /* Read status */
    virtual Sim800lStatus getGPRSStatus() final;
    virtual Sim800lStatus getHTTPStatus() final;
    virtual Sim800lStatus getInitStatus() final;

protected:
    char * defaultUrl[SIM800L_DEF_URL_LEN]; 
    uint8_t conFlags;
    bool sleep;

    /* Set status */
    virtual void setStatus(const Sim800lStatus status) final;
    /* Reset status */
    virtual void resetStatus(const Sim800lStatus status) final;



};