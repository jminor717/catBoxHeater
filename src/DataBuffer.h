#include "ADCBord.h"

#define BUFFER_SIZE 20

struct Intbuffer
{
    Intbuffer(uint32_t);
    uint16_t GetRunningAverage();
    uint32_t maxsize = BUFFER_SIZE;
    uint32_t buffer[BUFFER_SIZE];
    uint32_t size; //number of valid elements in the buffer
    uint32_t head; //most recently written index
    TickType_t lastUpdated;
    void push(uint32_t);
    void push(uint32_t, TickType_t );
    bool Valid();
};

struct Tempbuffer
{
    Tempbuffer(int32_t);
    int32_t GetRunningAverage();
    int32_t maxsize = BUFFER_SIZE;
    int32_t buffer[BUFFER_SIZE];
    int32_t size; //number of valid elements in the buffer
    int32_t head; //most recently written index
    TickType_t lastUpdated;
    void push(int32_t);
    void push(int32_t, TickType_t);
    bool Valid();
};