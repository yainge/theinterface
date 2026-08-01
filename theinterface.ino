#include "InterfaceSensor.h"

InterfaceSensor sensor(8, 9);

void setup()
{
    Serial.begin(115200);

    delay(2000);

    Serial.println("Starting Interface Sensor Test");

    if (sensor.begin())
    {
        Serial.println("Sensor Connected!");

        if (sensor.setupSensor())
        {
            Serial.println("Sensor Setup OK");
            sensor.dumpConfig();
        }
        else
        {
            Serial.println("Sensor Setup Failed");
            while (1)
                ;
        }
    }
    else
    {
        Serial.println("Sensor NOT Found");
        while (1)
            ;
    }

    delay(1000);

    Serial.println("Beginning readings...");
}

void loop()
{
    byte count = sensor.getFIFOCount();

    for (byte i = 0; i < count; i++)
    {
        long ir = sensor.getIR();
        if (ir >= 0)
        {
            Serial.print("IR: ");
            Serial.print(ir);
            Serial.print("  OVF: ");
            Serial.println(sensor.readRegister(0x05));
        }
    }

    delay(20);
}
