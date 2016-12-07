#define tipManageData (byte)13
#define tipManageClock (byte)14

// -----------------------------------------------------------------------------

class extruder
{
public:
    extruder() {}
    
    void setTemp(int temp);
    byte sendTemp(int temp);
    int requestTemp();
    int recieveTemp();
    byte timeoutHigh();
    byte timeoutLow();
};

// -----------------------------------------------------------------------------

void extruder::setTemp(int temp)
{
    for(int x=0;x<200;x++)
    {
        noInterrupts();
        if(sendTemp(temp)==1)
        {
            interrupts();
            break;
        }
        
        interrupts();
    }
}

byte extruder::sendTemp(int temp)
{
    pinMode(tipManageData,INPUT);
    digitalWrite(tipManageClock,0);
    byte checkTimeout=1;
    for(int timeout=0;timeout<=600;timeout++)
    {
        if(digitalRead(tipManageData)!=1)
        {
            checkTimeout=0;
            break;
        }
    }
    if(checkTimeout==1)
    {
        return 0;
    }
    digitalWrite(tipManageClock,1);
    delayMicroseconds(50);
    if(timeoutHigh()==1)
    {
        return 0;
    }
    digitalWrite(tipManageClock,0);
    pinMode(tipManageData,OUTPUT);
    digitalWrite(tipManageData,0);
    delayMicroseconds(50);
    digitalWrite(tipManageClock,1);
    delayMicroseconds(50);
    
    for(int mask=0x0001;mask;mask<<=1)
    {
        digitalWrite(tipManageClock,0);
        if(temp & mask)
        {
            pinMode(tipManageData,INPUT);
        } else {
            pinMode(tipManageData,OUTPUT);
            digitalWrite(tipManageData,0);
        }
        delayMicroseconds(50);
        digitalWrite(tipManageClock,1);
        delayMicroseconds(50);
    }
    digitalWrite(tipManageClock,0);
    pinMode(tipManageData,INPUT);
    delayMicroseconds(50);
    digitalWrite(tipManageClock,1);
    if(timeoutLow()==1)
    {
        return 0;
    }
    digitalWrite(tipManageClock,0);
    delayMicroseconds(50);
    digitalWrite(tipManageClock,1);
    if(timeoutHigh()==1)
    {
        return 0;
    }
    digitalWrite(tipManageClock,0);
    delayMicroseconds(50);
    digitalWrite(tipManageClock,1);
    if(timeoutLow()==1)
    {
        return 0;
    }
    digitalWrite(tipManageClock,0);
    delayMicroseconds(50);
    digitalWrite(tipManageClock,1);
    if(timeoutHigh()==1)
    {
        return 0;
    }
    if(recieveTemp()==temp)
    {
        return 1;
    }
    return 0;
}

int extruder::requestTemp()
{
    pinMode(tipManageClock,OUTPUT);
    pinMode(tipManageData,INPUT);
    digitalWrite(tipManageClock,0);
    byte checkTimeout=1;
    int timeout;
    for(timeout=0;timeout<=2500;timeout++)
    {
        if(digitalRead(tipManageData)!=1)
        {
            checkTimeout=0;
            break;
        }
    }
    if(checkTimeout==1)
    {
        return timeout+10;
    }
    pinMode(tipManageData,INPUT);
    digitalWrite(tipManageClock,1);
    if(timeoutHigh()==1)
    {
        return 1989;
    }
    digitalWrite(tipManageClock,0);
    delayMicroseconds(50);
    digitalWrite(tipManageClock,1);
    delayMicroseconds(50);
    return recieveTemp();
}

int extruder::recieveTemp()
{
    pinMode(tipManageData,INPUT);
    int recieve=0;
    for(int mask=0x0001;mask;mask<<=1)
    {
        digitalWrite(tipManageClock,0);
        delayMicroseconds(50);
        digitalWrite(tipManageClock,1);
        delayMicroseconds(50);
        if(digitalRead(tipManageData)==1)
        {
            recieve=recieve | mask;
        }
    }
    recieve=recieve & 0b0111111111111111;
    return recieve;
}

byte extruder::timeoutHigh()
{
    for(int timeout=0; timeout<=200; timeout++)
    {
        if(digitalRead(tipManageData)==1)
        {
            return 0;
        }
    }
    return 1;
}

byte extruder::timeoutLow()
{
    for(int timeout=0; timeout<=200; timeout++)
    {
        if(digitalRead(tipManageData)==0)
        {
            return 0;
        }
    } 
    return 1;
}

// -----------------------------------------------------------------------------

extruder myExtruder;

void setup() {
    Serial.begin(9600);
    pinMode(0, OUTPUT);
}

void loop() {
    static byte onOff = 0;
    onOff = !onOff;
    digitalWrite(0, onOff);
    Serial.println(myExtruder.requestTemp());
    delay(500);
}

