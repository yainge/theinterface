#ifndef INTERFACE_SENSOR_H
#define INTERFACE_SENSOR_H

#include <Arduino.h>
#include <SoftWire.h>

class InterfaceSensor
{
public:
    byte getFIFOWritePointer();
    byte getFIFOReadPointer();
    byte getFIFOCount();

    InterfaceSensor(uint8_t sdaPin, uint8_t sclPin);

    bool begin();

    byte readRegister(byte reg);
    bool writeRegister(byte reg, byte value);

    bool reset();

    bool setupSensor();
    bool setIRLedAmplitude(byte amplitude);


    bool readFIFO(uint32_t &red, uint32_t &ir);
    long getIR();
    void dumpConfig();
private:

    SoftWire i2c;
    byte txBuffer[2];
    byte rxBuffer[6];

    static const byte ADDRESS = 0x57;
};

#endif
