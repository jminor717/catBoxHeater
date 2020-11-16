#include "DataBuffer.h"
const TickType_t measureValidFor = 15000 / portTICK_PERIOD_MS;

Intbuffer::Intbuffer(uint32_t len)
{
    this->maxsize = len;
}

uint16_t Intbuffer::GetRunningAverage()
{
    if (this->size > 0)
    {
        int64_t sum = 0;
        for (uint32_t i = 0; i < this->size; i++)
        {
            sum += this->buffer[i];
        }
        return (uint16_t)(((sum / this->size) & 0x00ffff00) >> 8); // return (uint16_t)(((sum / this->size) & 0x00ffff00) > 8);
    }
    else
        return 0;
}



void Intbuffer::push(uint32_t val)
{
    if ((this->head + 1) >= this->maxsize)
    {
        this->head = 0;
    }
    else
    {
        this->head++;
    }
    if ((this->size + 1) <= this->maxsize)
    {
        this->size++;
    }
    this->buffer[this->head] = val;
}

void Intbuffer::push(uint32_t val, TickType_t now)
{
    this->push(val);
    this->lastUpdated = now;
}

bool Intbuffer::Valid()
{
    TickType_t now = xTaskGetTickCount();
    return (this->size > 0 && now < (this->lastUpdated + measureValidFor));
}


/** 
 * 
 * 
 * 
 * 
 * 
 * 
*/

Tempbuffer::Tempbuffer(int32_t len)
{
    this->maxsize = len;
}


int32_t Tempbuffer::GetRunningAverage()
{
    if (this->size > 0)
    {
        int64_t sum = 0;
        for (uint32_t i = 0; i < this->size; i++)
        {
            //Serial.print(", ");
           // Serial.print(buffer[i]);
            sum += this->buffer[i];
        }
       // Serial.print("    , ");
       // Serial.println((int32_t)(sum / this->size));
        return (int32_t)(sum / this->size);
    }
    else
        return 0;
}

void Tempbuffer::push(int32_t val)
{
    if ((this->head + 1) >= this->maxsize)
    {
        this->head = 0;
    }
    else
    {
        this->head++;
    }
    if ((this->size + 1) <= this->maxsize)
    {
        this->size++;
    }
    this->buffer[this->head] = val;
}

void Tempbuffer::push(int32_t val, TickType_t now)
{
    this->push(val);
    this->lastUpdated = now;
}

bool Tempbuffer::Valid()
{
    TickType_t now = xTaskGetTickCount();
    return (this->size > 0 && now < (this->lastUpdated + measureValidFor));
}