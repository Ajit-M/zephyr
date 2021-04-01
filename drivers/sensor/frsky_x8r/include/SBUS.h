#ifndef SBUS_h
#define SBUS_h

#define SBUS_FAILSAFE_INACTIVE 0
#define SBUS_FAILSAFE_ACTIVE   1
#define SBUS_STARTBYTE         0x0f
#define SBUS_ENDBYTE           0x00
#define FRSKY_SBUS             "FRSKY" 

struct frsky_data {
    uint8_t _channels[18];
    uint8_t _failsafe;
    long int _goodFrames;
    long int _lostFrames;
    long int _decoderErrorFrames;
    long long int _lastGoodFrame;
}

struct frsky_config


SBUS(HardwareSerial & serial) : _serial (serial) {} // Need to remove this constructor
void begin();
void begin(bool useTimer);
void process();
int getChannel(int channel);
int getNormalizedChannel(int channel);
int getFailsafeStatus();
int getFrameLoss();
long getGoodFrames();
long getLostFrames();
long getDecoderErrorFrames();
long long getLastTime();
HardwareSerial & _serial;


#endif
