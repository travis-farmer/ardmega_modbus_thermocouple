/*
Address Allocation
------------------
Modbus = 3
Baud = 9600
dePin = 2
UART = Serial1

Coils
-----
0x00/D22 = Vacuum SSR
0x01/D24 = Laser In-Use Warning Sign SSR

DiscreteInputs
--------------
0x00 = MAX31855_FAULT_OPEN
0x01 = MAX31855_FAULT_SHORT_GND
0x02 = MAX31855_FAULT_SHORT_VCC
0x03/D23 = Dust Bin Full (capacitive proximity sensor, NPN NO)

HoldingRegisters
----------------
0x00/D03 = 
0x01/D04 = 
0x02/D05 = 
0x03/D06 = 
0x04/D07 = 
0x05/D08 = 
0x06/D09 = 
0x07/D10 = 
0x08/D11 = 
0x09/D12 = 
0x10/D13 = 
0x11/D44 = 
0x12/D45 = 
0x13/D46 = 

InputRegisters
--------------
0x00 = Spindle Temp F * 100
0x01 = Spindle Temp C * 100
0x02/A0 = Air Compressor PSI * 100
0x03/A1 = Tool PSI * 100
*/

#include <SPI.h>
#include "Adafruit_MAX31855.h"
//#include <SoftwareSerial.h>
#include <ModbusRTUSlave.h>

const uint8_t dePin = 2;
Adafruit_MAX31855 thermocouple(10);

ModbusRTUSlave modbus(Serial1, dePin); // serial port, driver enable pin for rs-485 (optional)

int coilPins[2] = {22, 24};
bool coils[sizeof(coilPins)];
int discreteInputPins[4] = {100, 101, 102, 23};
bool discreteInputs[sizeof(discreteInputPins)];
int holdingRegisterPins[14] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 44, 45, 46};
uint16_t holdingRegisters[sizeof(holdingRegisterPins)];
int inputRegisterPins[18] = {100, 101, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
int inputRegisterMode[sizeof(inputRegisterPins)][3] = {{1,0,0},{1,0,0},{2,0,0},{2,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
uint16_t inputRegisters[sizeof(inputRegisterPins)];

void setup()
{
    Serial.begin(9600);
    thermocouple.begin();
    modbus.configureCoils(coils, sizeof(coilPins));                       // bool array of coil values, number of coils
    modbus.configureDiscreteInputs(discreteInputs, sizeof(discreteInputPins));     // bool array of discrete input values, number of discrete inputs
    modbus.configureHoldingRegisters(holdingRegisters, sizeof(holdingRegisterPins)); // unsigned 16 bit integer array of holding register values, number of holding registers
    modbus.configureInputRegisters(inputRegisters, sizeof(inputRegisterPins));     // unsigned 16 bit integer array of input register values, number of input registers
    modbus.begin(3, 9600);                                // slave id, baud rate, config (optional)

    for (int i = 0; i<sizeof(coilPins);i++) {
        if (coilPins[i] < 100) {
            pinMode(coilPins[i],OUTPUT);
            digitalWrite(coilPins[i],LOW);
        }
    }
    for (int i = 0; i<sizeof(discreteInputPins);i++) {
        if (discreteInputPins[i] < 100) {
            pinMode(discreteInputPins[i],INPUT_PULLUP);
        }
    }
    for (int i = 0; i<sizeof(holdingRegisterPins);i++) {
        if (holdingRegisterPins[i] < 100) {
            pinMode(holdingRegisterPins[i],OUTPUT);
            analogWrite(holdingRegisterPins[i],0);
        }
    }
}

void loop()
{
    double c = (thermocouple.readCelsius() * 100);
    double f = (thermocouple.readFahrenheit() * 100);
    bool faultA = false;
    bool faultB = false;
    bool faultC = false;
    uint16_t tempF = 0;
    uint16_t tempC = 0;

    if (isnan(c))
    {
        uint8_t e = thermocouple.readError();
        if (e & MAX31855_FAULT_OPEN) faultA = true;
        if (e & MAX31855_FAULT_SHORT_GND) faultB = true;
        if (e & MAX31855_FAULT_SHORT_VCC) faultC = true;
    }
    else
    {
        tempF = (uint16_t)(f);
        tempC = (uint16_t)(c);
    }

    for (int i = 0; i<sizeof(discreteInputPins);i++) {
        if (discreteInputPins[i] < 100) {
            discreteInputs[i] = digitalRead(discreteInputPins[i]);
        } else {
            switch (discreteInputPins[i]) {
                case 100:
                    discreteInputs[i] = faultA;
                    break;
                case 101:
                    discreteInputs[i] = faultB;
                    break;
                case 102:
                    discreteInputs[i] = faultC;
                    break;
            }
        }
    }
    for (int i = 0; i<sizeof(inputRegisterPins);i++) {
        switch (inputRegisterMode[i][0]) {
            case 0: // direct send, adc value output
                inputRegisters[i] = analogRead(inputRegisterPins[i]);
                break;
            case 1: // temperature send, temperature*100 output
                if (inputRegisterPins[i] == 100) inputRegisters[i] = tempF;
                if (inputRegisterPins[i] == 101) inputRegisters[i] = tempC;
                break;
            case 2: // 0-5v 1-150psi pressure sensor, psi*100 output
                inputRegisters[i] = (uint16_t) map(analogRead(inputRegisterPins[i]), 0.50, 4.5, 0.00, 15000.00);
                break;
            case 3: // voltage 0-5v, volt*100 output
                inputRegisters[i] = (uint16_t) ((5.00/ 1024.00) * analogRead(inputRegisterPins[i]));
                break;
            case 4: // voltage divider, volt*100 output
                inputRegisters[i] = (uint16_t) ((((5.0/ 1024.00) * analogRead(inputRegisterPins[i])) / inputRegisterMode[i][1]) * (inputRegisterMode[i][2] + inputRegisterMode[i][1]));
        }
    }
    modbus.poll();
    for (int i = 0; i<sizeof(coilPins);i++) {
        if (coilPins[i] < 100) {
            digitalWrite(coilPins[i],coils[i]);
        }
    }
    for (int i = 0; i<sizeof(holdingRegisterPins); i++) {
        if (holdingRegisterPins[i] < 100) {
            analogWrite(holdingRegisterPins[i], holdingRegisters[i]);
        }
    }

}
