#include "InterfaceSensor.h"

InterfaceSensor::InterfaceSensor(uint8_t sdaPin, uint8_t sclPin)
    : i2c(sdaPin, sclPin)
{
    // SoftWire's Wire-compatible API needs caller-owned buffers.  The
    // low-level API happened to work on AVR, but its repeated-start reads
    // are unreliable on the UNO R4's Renesas core.
    i2c.setTxBuffer(txBuffer, sizeof(txBuffer));
    i2c.setRxBuffer(rxBuffer, sizeof(rxBuffer));
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
    i2c.beginTransmission(ADDRESS);
    if (i2c.write(reg) != 1 || i2c.endTransmission(false) != 0)
    {
        i2c.stop();
        return 0;
    }

    if (i2c.requestFrom(ADDRESS, (uint8_t)1, true) != 1)
        return 0;

    return (byte)i2c.read();
}

bool InterfaceSensor::writeRegister(byte reg, byte value)
{
    i2c.beginTransmission(ADDRESS);
    if (i2c.write(reg) != 1 || i2c.write(value) != 1)
    {
        i2c.stop();
        return false;
    }

    return i2c.endTransmission(true) == 0;
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
    writeRegister(0x0C, 0x30);

    // IR LED
    writeRegister(0x0D, 0x30);


    // Clear FIFO again after configuration
    writeRegister(0x04, 0x00);
    writeRegister(0x05, 0x00);
    writeRegister(0x06, 0x00);


    delay(100);


    return true;
}

bool InterfaceSensor::setIRLedAmplitude(byte amplitude)
{
    return writeRegister(0x0D, amplitude);
}

bool InterfaceSensor::setRedLedAmplitude(byte amplitude)
{
    return writeRegister(0x0C, amplitude);
}

byte InterfaceSensor::getOverflowCount()
{
    byte count = readRegister(0x05);

    // Explicitly clear rather than relying on the chip to auto-reset this on
    // its own -- if it doesn't, a single overflow event latches this
    // register nonzero forever and the caller's overflow handling never
    // stops firing.
    if (count > 0)
        writeRegister(0x05, 0x00);

    return count;
}

bool InterfaceSensor::readFIFO(uint32_t &red, uint32_t &ir)
{
    byte data[6];
    i2c.beginTransmission(ADDRESS);
    if (i2c.write(0x07) != 1 || i2c.endTransmission(false) != 0)
    {
        i2c.stop();
        return false;
    }

    if (i2c.requestFrom(ADDRESS, (uint8_t)6, true) != 6)
        return false;

    for (byte i = 0; i < 6; ++i)
        data[i] = (byte)i2c.read();


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
