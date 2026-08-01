#include "InterfaceSensor.h"

InterfaceSensor::InterfaceSensor(uint8_t sdaPin, uint8_t sclPin)
    : i2c(sdaPin, sclPin)
{
}

bool InterfaceSensor::begin()
{
    i2c.setDelay_us(5);
    i2c.setTimeout_ms(200);

    i2c.begin();

    delay(50);

    byte id = readRegister(0xFF);

    return (id == 0x15);
}
byte InterfaceSensor::getFIFOCount()
{
    byte write = readRegister(0x04);
    byte read = readRegister(0x06);

    return (write - read) & 0x1F;
}
byte InterfaceSensor::readRegister(byte reg)
{
    byte value = 0;
    byte errors = 0;

    errors += i2c.startWait(ADDRESS, SoftWire::writeMode);

    errors += i2c.write(reg);

    errors += i2c.repeatedStart(ADDRESS, SoftWire::readMode);

    errors += i2c.readThenNack(value);

    i2c.stop();

    if(errors)
        return 0;

    return value;
}

bool InterfaceSensor::writeRegister(byte reg, byte value)
{
    byte errors = 0;

    errors += i2c.startWait(ADDRESS, SoftWire::writeMode);

    errors += i2c.write(reg);

    errors += i2c.write(value);

    i2c.stop();

    return (errors == 0);
}

bool InterfaceSensor::reset()
{
    // Send reset command
    if (!writeRegister(0x09, 0x40))
        return false;


    // Wait for reset bit to clear
    unsigned long start = millis();

    while (millis() - start < 100)
    {
        byte mode = readRegister(0x09);

        if ((mode & 0x40) == 0)
            return true;

        delay(1);
    }


    return false;
}

bool InterfaceSensor::setupSensor()
{
    if(!reset())
        return false;

    delay(50);


    // Clear FIFO pointers
    writeRegister(0x04, 0x00); // FIFO_WR_PTR
    writeRegister(0x05, 0x00); // OVF_COUNTER
    writeRegister(0x06, 0x00); // FIFO_RD_PTR


    // FIFO configuration: sample averaging 4, FIFO rollover enabled, FIFO_A_FULL 15
    writeRegister(0x08, 0x5F);


    // Mode configuration
    // SpO2 mode (Red + IR)
    writeRegister(0x09, 0x03);


    // SpO2 configuration:
    // ADC range: 4096
    // Sample rate: 100Hz
    // Pulse width: 411us
    writeRegister(0x0A, 0x27);


    // LED pulse amplitude
    // Red LED
    writeRegister(0x0C, 0x3F);

    // IR LED
    writeRegister(0x0D, 0x3F);


    // Clear FIFO again after configuration
    writeRegister(0x04, 0x00);
    writeRegister(0x05, 0x00);
    writeRegister(0x06, 0x00);


    delay(100);


    return true;
}

bool InterfaceSensor::readFIFO(uint32_t &red, uint32_t &ir)
{
    byte data[6];
    byte errors = 0;


    // Point to FIFO_DATA register
    errors += i2c.startWait(ADDRESS, SoftWire::writeMode);

    errors += i2c.write(0x07);


    // Switch to reading
    errors += i2c.repeatedStart(ADDRESS, SoftWire::readMode);


    for(int i = 0; i < 5; i++)
    {
        errors += i2c.readThenAck(data[i]);
    }

    errors += i2c.readThenNack(data[5]);


    i2c.stop();


    if(errors)
        return false;


    // MAX30102 uses 18-bit values
    red =
        ((uint32_t)data[0] << 16) |
        ((uint32_t)data[1] << 8) |
        data[2];


    ir =
        ((uint32_t)data[3] << 16) |
        ((uint32_t)data[4] << 8) |
        data[5];


    red &= 0x03FFFF;
    ir &= 0x03FFFF;


    return true;
}
long InterfaceSensor::getIR()
{
    uint32_t red;
    uint32_t ir;

    if(readFIFO(red, ir))
        return ir;

    return -1;
}
byte InterfaceSensor::getFIFOWritePointer()
{
    return readRegister(0x04);
}


byte InterfaceSensor::getFIFOReadPointer()
{
    return readRegister(0x06);
}

void InterfaceSensor::dumpConfig()
{
    const byte regs[] = { 0x08, 0x09, 0x0A, 0x0C, 0x0D };
    const char* names[] = { "FIFO_CFG", "MODE_CFG", "SPO2_CFG", "LED1_PA", "LED2_PA" };

    for(int i = 0; i < 5; i++)
    {
        Serial.print(names[i]);
        Serial.print(" (0x");
        Serial.print(regs[i], HEX);
        Serial.print(") = 0x");
        Serial.println(readRegister(regs[i]), HEX);
    }
}