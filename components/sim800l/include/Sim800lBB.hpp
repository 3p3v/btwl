#include "Sim800l.hpp"

/* Wrapper; virtual class specyfying commands */
class Sim800lBB : protected Sim800l {
public:
    Sim800lBB(const char * url);
    virtual ~Sim800lBB() = 0;

    /* Set defualt server */
    virtual void setDefualtUrl(const char * url);

    /* Initialization of UART */
    virtual Sim800lError init();

    /* AT */
    /* Check if Sim800l responds */
    virtual Sim800lError handshake() final;

    /* AT+CSQ */
    /* Check signal quality */
    // virtual Sim800lError checkSignal(int & rssi, int & ber) final;
    
    /* AT+CCID */
    /* Get SIM info / Test SIM card */
    // virtual Sim800lError getSIMInfo(char * ccid) final;

    /* AT+CREG */
    /* Check if registered */
    // virtual Sim800lError checkIfRegistered() final;

    /* AT+SAPBR */
    /* "CONTYPE" Type of Internet connection */
    virtual Sim800lError setConnectionType(const char * type) final;
    /* "APN" Access point name string: maximum 64 characters */
    virtual Sim800lError setAccessPoint(const char * ap);

    virtual Sim800lError sendHTTPPOST(const char * url, const char * data) final;
    virtual Sim800lError sendHTTPPOST(const char * data) final;

protected:
    char * defaultUrl[50]; 





};