#pragma once

class Message {
public:
    Message(bool protect, bool open, bool ack)
        : protect(protect), open(open), ack(ack) {}

    bool getProtect() {
        return protect;
    }
	bool getOpen() {
        return open;
    }
	bool getAck() {
        return ack;
    }
    void setProtect(bool newV) {
        protect = newV;
    }
	void setOpen(bool newV) {
        open = newV;
    }
	void setAck(bool newV) {
        ack = newV;
    }

    bool isSame(Message message) {
        if(message.getProtect() == protect && message.getOpen() == open)
            return true;
        else
            return false;
    }

    bool isSameWithAck(Message message) {
        if(message.getProtect() == protect && message.getOpen() == open && message.getAck() == ack)
            return true;
        else 
            return false;
    }

protected:
    bool protect;
	bool open;
	bool ack;
};

class ServerMessage : public Message {
public:
    ServerMessage(bool protect, bool open, bool ack, bool valid)
        : Message(protect, open, ack), valid(valid) {}

    void setValid(bool valid) {
        this->valid = valid;
    }

    bool getValid() {
        return valid;
    }

private:
    bool valid;
};