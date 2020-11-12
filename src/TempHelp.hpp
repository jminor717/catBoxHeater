#include <OneWire.h>
class Tempsensor
{
public:
    OneWire *Buss = NULL;
    uint8_t line = 0;
    OneWire sensor;
    byte addr[8]={0};
    bool connected = false;
    byte type_s;
    Tempsensor(uint8_t);
    int16_t read();
};