#include "Actron485.h"
#include "Utilities.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

namespace Actron485 {

namespace {

void printHex8(Stream *out, uint8_t value) {
    char text[5];
    snprintf(text, sizeof(text), "0x%02X", value);
    out->print(text);
}

void printHex16(Stream *out, uint16_t value) {
    char text[7];
    snprintf(text, sizeof(text), "0x%04X", value);
    out->print(text);
}

void printFloatValue(Stream *out, float value) {
    if (!isfinite(value) || value > 1000000.0f || value < -1000000.0f) {
        out->print("ovf");
        return;
    }
    out->print(value, 4);
}

float decodeFloatABCD(uint16_t regA, uint16_t regB) {
    uint32_t raw = ((uint32_t) regA << 16) | regB;
    float value = 0.0f;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

float decodeFloatCDAB(uint16_t regA, uint16_t regB) {
    uint32_t raw = ((uint32_t) regB << 16) | regA;
    float value = 0.0f;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

double absDouble(double value) {
    return value >= 0 ? value : -value;
}

}  // namespace

void Controller::printCaptureStateSnapshot(uint8_t *data, uint8_t length, const char *sourceTag) {
    if (!printOut) {
        return;
    }

    printOut->print("CAP ACTRON ");
    printOut->print(sourceTag);
    printOut->print(": on=");
    printOut->print(getSystemOn() ? 1 : 0);
    printOut->print(", op=");
    printOut->print((int) getOperatingMode());
    printOut->print(", fan=");
    printOut->print((int) getFanSpeed());
    printOut->print(", run_fan=");
    printOut->print((int) getRunningFanSpeed());
    printOut->print(", cont_fan=");
    printOut->print(getContinuousFanMode() ? 1 : 0);
    printOut->print(", comp=");
    printOut->print((int) getCompressorMode());
    printOut->print(", setpoint=");
    printOut->print(getMasterSetpoint(), 1);
    printOut->print(", temp=");
    printOut->print(getMasterCurrentTemperature(), 1);
    printOut->print(", zones=");
    for (int z = 1; z <= 8; z++) {
        printOut->print(getZoneOn(z) ? "1" : "0");
    }
    printOut->print(", raw=");
    printBytes(data, length);
    printOut->println();
}

void Controller::printCaptureZoneWallMessage(uint8_t zone, uint8_t *data, uint8_t length) {
    if (!printOut || zone < 1 || zone > 8) {
        return;
    }

    printOut->print("CAP ACTRON C: zone=");
    printOut->print(zone);
    printOut->print(", mode=");
    printOut->print((int) zoneMessage[zindex(zone)].mode);
    printOut->print(", setpoint=");
    printOut->print(zoneMessage[zindex(zone)].setpoint, 1);
    printOut->print(", temp=");
    printOut->print(zoneMessage[zindex(zone)].temperature, 1);
    printOut->print(", temp_pre=");
    printOut->print(zoneMessage[zindex(zone)].temperaturePreAdjustment, 1);
    printOut->print(", raw=");
    printBytes(data, length);
    printOut->println();
}

void Controller::printCaptureZoneMasterMessage(uint8_t zone, uint8_t *data, uint8_t length) {
    if (!printOut || zone < 1 || zone > 8) {
        return;
    }

    printOut->print("CAP ACTRON M: zone=");
    printOut->print(zone);
    printOut->print(", on=");
    printOut->print(masterToZoneMessage[zindex(zone)].on ? 1 : 0);
    printOut->print(", comp_mode=");
    printOut->print(masterToZoneMessage[zindex(zone)].compressorMode ? 1 : 0);
    printOut->print(", setpoint=");
    printOut->print(masterToZoneMessage[zindex(zone)].setpoint, 1);
    printOut->print(", temp=");
    printOut->print(masterToZoneMessage[zindex(zone)].temperature, 1);
    printOut->print(", min=");
    printOut->print(masterToZoneMessage[zindex(zone)].minSetpoint, 1);
    printOut->print(", max=");
    printOut->print(masterToZoneMessage[zindex(zone)].maxSetpoint, 1);
    printOut->print(", damper=");
    printOut->print(masterToZoneMessage[zindex(zone)].damperPosition);
    printOut->print(", raw=");
    printBytes(data, length);
    printOut->println();
}

void Controller::printModbusMessage() {
    if (!printOut || _serialBufferIndex < 4) {
        return;
    }

    bool capture = (printOutMode == PrintOutMode::CorrelationCapture);

    auto printRegisterValue = [&](uint16_t regAddress, int regIndex, uint16_t regValue, bool hasAddress) {
        if (hasAddress) {
            printOut->print("Modbus Reg ");
            printHex16(printOut, regAddress);
            printOut->print(": ");
        } else {
            printOut->print("Modbus Reg R");
            printOut->print(regIndex);
            printOut->print(": ");
        }
        printOut->print("u16=");
        printOut->print(regValue);
        printOut->print(" (");
        printHex16(printOut, regValue);
        printOut->print("), i16=");
        printOut->print((int16_t) regValue);
        printOut->println();
    };

    auto printFloatPair = [&](uint16_t addressLow, int pairIndex, uint16_t regA, uint16_t regB, bool hasAddress) {
        float floatABCD = decodeFloatABCD(regA, regB);
        float floatCDAB = decodeFloatCDAB(regA, regB);

        if (hasAddress) {
            printOut->print("Modbus F32 ");
            printHex16(printOut, addressLow);
            printOut->print("/");
            printHex16(printOut, (uint16_t) (addressLow + 1));
            printOut->print(": ");
        } else {
            printOut->print("Modbus F32 P");
            printOut->print(pairIndex);
            printOut->print(": ");
        }

        printOut->print("abcd=");
        printFloatValue(printOut, floatABCD);
        printOut->print(", cdab=");
        printFloatValue(printOut, floatCDAB);
        printOut->println();
    };

    uint8_t slave = _serialBuffer[0];
    uint8_t functionCode = _serialBuffer[1];
    bool isException = (functionCode & 0x80) == 0x80;

    printOut->print("Modbus Message Ignored: ");
    printBytes(_serialBuffer, _serialBufferIndex);
    printOut->println();

    if (isException) {
        if (_serialBufferIndex >= 5) {
            printOut->print("Modbus Exception: Slave ");
            printOut->print(slave);
            printOut->print(", Function 0x");
            printOut->print(functionCode & 0x7F, HEX);
            printOut->print(", Code 0x");
            printOut->print(_serialBuffer[2], HEX);
            printOut->println();
        }
        return;
    }

    if (functionCode == 0x03 || functionCode == 0x04) {
        if (_serialBufferIndex == 8) {
            uint16_t startAddress = ((uint16_t) _serialBuffer[2] << 8) | _serialBuffer[3];
            uint16_t registerCount = ((uint16_t) _serialBuffer[4] << 8) | _serialBuffer[5];
            _modbusLastReadSlave = slave;
            _modbusLastReadFunction = functionCode;
            _modbusLastReadStartAddress = startAddress;
            _modbusLastReadCount = registerCount;
            _modbusLastReadTimestamp = millis();

            printOut->print("Modbus Read Request: Slave ");
            printOut->print(slave);
            printOut->print(", Function 0x");
            printOut->print(functionCode, HEX);
            printOut->print(", Start ");
            printOut->print(startAddress);
            printOut->print(", Count ");
            printOut->print(registerCount);
            printOut->println();
            return;
        }

        uint8_t byteCount = _serialBuffer[2];
        uint16_t registerCount = byteCount / 2;
        bool hasAddress = false;
        uint16_t startAddress = 0;

        if ((millis() - _modbusLastReadTimestamp) < 5000 &&
            _modbusLastReadSlave == slave &&
            _modbusLastReadFunction == functionCode) {
            hasAddress = true;
            startAddress = _modbusLastReadStartAddress;
        }

        printOut->print("Modbus Read Response: Slave ");
        printOut->print(slave);
        printOut->print(", Function 0x");
        printOut->print(functionCode, HEX);
        printOut->print(", Bytes ");
        printOut->print(byteCount);
        printOut->print(", Registers ");
        printOut->print(registerCount);
        if (hasAddress) {
            printOut->print(", Start ");
            printHex16(printOut, startAddress);
        }
        printOut->println();

        if ((byteCount % 2) != 0) {
            return;
        }

        const uint8_t maxDecodedRegisters = 64;
        uint16_t registerValues[maxDecodedRegisters];
        uint16_t decodedCount = registerCount;
        if (decodedCount > maxDecodedRegisters) {
            decodedCount = maxDecodedRegisters;
        }

        for (uint16_t reg = 0; reg < decodedCount; reg++) {
            uint16_t offset = (uint16_t) (3 + (reg * 2));
            if (offset + 1 >= _serialBufferIndex - 2) {
                decodedCount = reg;
                break;
            }
            uint16_t value = ((uint16_t) _serialBuffer[offset] << 8) | _serialBuffer[offset + 1];
            registerValues[reg] = value;
            uint16_t regAddress = startAddress + reg;
            printRegisterValue(regAddress, reg, value, hasAddress);
        }

        for (uint16_t reg = 0; reg + 1 < decodedCount; reg += 2) {
            uint16_t regAddress = startAddress + reg;
            printFloatPair(regAddress, reg / 2, registerValues[reg], registerValues[reg + 1], hasAddress);
        }

        if (capture && hasAddress) {
            if (startAddress == 0x000F && decodedCount >= 1) {
                printOut->print("CAP MODBUS CAND: 0x000F status=");
                printHex16(printOut, registerValues[0]);
                printOut->print(", bit0=");
                printOut->print((registerValues[0] & 0x0001) ? 1 : 0);
                printOut->print(", bit13=");
                printOut->print((registerValues[0] & 0x2000) ? 1 : 0);
                printOut->println();
            } else if (startAddress == 0x0023 && decodedCount >= 13) {
                printOut->print("CAP MODBUS CAND: 0x0023 profile r0-r4=");
                printOut->print(registerValues[0]);
                printOut->print("/");
                printOut->print(registerValues[1]);
                printOut->print("/");
                printOut->print(registerValues[2]);
                printOut->print("/");
                printOut->print(registerValues[3]);
                printOut->print("/");
                printOut->print(registerValues[4]);
                printOut->print(", r5=");
                printOut->print(registerValues[5]);
                printOut->print(", r7=");
                printOut->print(registerValues[7]);
                printOut->print(", r10=");
                printOut->print(registerValues[10]);
                printOut->print(", r11=");
                printOut->print(registerValues[11]);
                printOut->print(", r12=");
                printOut->print(registerValues[12]);
                printOut->println();
            } else if (startAddress == 0x0078 && decodedCount >= 12) {
                printOut->print("CAP MODBUS CAND: 0x0078 f32=");
                for (uint8_t pair = 0; pair < 6; pair++) {
                    if (pair > 0) {
                        printOut->print(",");
                    }
                    float f = decodeFloatABCD(registerValues[pair * 2], registerValues[pair * 2 + 1]);
                    printFloatValue(printOut, f);
                }
                printOut->println();
            } else if (startAddress == 0x00B4 && decodedCount >= 9) {
                printOut->print("CAP MODBUS CAND: 0x00B4 signed10=");
                printOut->print(((int16_t) registerValues[0]) / 10.0f, 1);
                printOut->print(",");
                printOut->print(((int16_t) registerValues[1]) / 10.0f, 1);
                printOut->print(",");
                printOut->print(((int16_t) registerValues[5]) / 10.0f, 1);
                printOut->print(", raw=");
                printOut->print(registerValues[6]);
                printOut->print("/");
                printOut->print(registerValues[7]);
                printOut->print("/");
                printOut->print(registerValues[8]);
                printOut->println();
            }
        }
        return;
    }

    if (functionCode == 0x06) {
        if (_serialBufferIndex == 8) {
            uint16_t registerAddress = ((uint16_t) _serialBuffer[2] << 8) | _serialBuffer[3];
            uint16_t registerValue = ((uint16_t) _serialBuffer[4] << 8) | _serialBuffer[5];
            printOut->print("Modbus Write Single Register: Slave ");
            printOut->print(slave);
            printOut->print(", Address ");
            printOut->print(registerAddress);
            printOut->print(", Value ");
            printOut->print(registerValue);
            printOut->print(" (");
            printHex16(printOut, registerValue);
            printOut->print(")");
            printOut->println();
        }
        return;
    }

    if (functionCode == 0x10) {
        if (_serialBufferIndex == 8) {
            uint16_t startAddress = ((uint16_t) _serialBuffer[2] << 8) | _serialBuffer[3];
            uint16_t registerCount = ((uint16_t) _serialBuffer[4] << 8) | _serialBuffer[5];
            printOut->print("Modbus Write Multiple Registers Response: Slave ");
            printOut->print(slave);
            printOut->print(", Start ");
            printOut->print(startAddress);
            printOut->print(", Count ");
            printOut->print(registerCount);
            printOut->println();
            return;
        }

        if (_serialBufferIndex < 9) {
            return;
        }

        uint16_t startAddress = ((uint16_t) _serialBuffer[2] << 8) | _serialBuffer[3];
        uint16_t registerCount = ((uint16_t) _serialBuffer[4] << 8) | _serialBuffer[5];
        uint8_t byteCount = _serialBuffer[6];
        printOut->print("Modbus Write Multiple Registers Request: Slave ");
        printOut->print(slave);
        printOut->print(", Start ");
        printOut->print(startAddress);
        printOut->print(", Count ");
        printOut->print(registerCount);
        printOut->print(", Bytes ");
        printOut->print(byteCount);
        printOut->println();

        const uint8_t maxDecodedRegisters = 64;
        uint16_t registerValues[maxDecodedRegisters];
        uint16_t decodedCount = byteCount / 2;
        if (decodedCount > registerCount) {
            decodedCount = registerCount;
        }
        if (decodedCount > maxDecodedRegisters) {
            decodedCount = maxDecodedRegisters;
        }

        for (uint16_t reg = 0; reg < decodedCount; reg++) {
            uint16_t offset = (uint16_t) (7 + (reg * 2));
            if (offset + 1 >= _serialBufferIndex - 2) {
                decodedCount = reg;
                break;
            }
            uint16_t value = ((uint16_t) _serialBuffer[offset] << 8) | _serialBuffer[offset + 1];
            registerValues[reg] = value;
            printRegisterValue(startAddress + reg, reg, value, true);
        }

        for (uint16_t reg = 0; reg + 1 < decodedCount; reg += 2) {
            printFloatPair(startAddress + reg, reg / 2, registerValues[reg], registerValues[reg + 1], true);
        }

        if (capture && startAddress == 0x0122 && decodedCount >= 3) {
            double likelySetpoint = registerValues[0] / 2.0;
            double likelyTemperature = ((int16_t) registerValues[1]) / 10.0;
            uint16_t flags = registerValues[2];
            uint8_t modbusFlagsLow = (uint8_t) (flags & 0x00FF);
            double actronSetpoint = getMasterSetpoint();
            double actronTemperature = getMasterCurrentTemperature();
            double deltaSetpoint = likelySetpoint - actronSetpoint;
            double deltaTemperature = likelyTemperature - actronTemperature;
            uint8_t actronZoneBits = stateMessageRaw[11];
            uint8_t actronStat1Flags = stateMessageRaw[18];
            uint8_t flagsXor = (uint8_t) (modbusFlagsLow ^ actronStat1Flags);
            const char *match = (absDouble(deltaSetpoint) <= 0.2 && absDouble(deltaTemperature) <= 0.2) ? "yes" : "no";

            printOut->print("CAP MODBUS MAP: Likely Master Setpoint=");
            printOut->print(likelySetpoint, 1);
            printOut->print(", Likely Master Temp=");
            printOut->print(likelyTemperature, 1);
            printOut->print(", Flags=");
            printHex16(printOut, flags);
            printOut->print(", ModbusFlagsLow=");
            printHex8(printOut, modbusFlagsLow);
            printOut->print(", Actron SP=");
            printOut->print(actronSetpoint, 1);
            printOut->print(", Actron Temp=");
            printOut->print(actronTemperature, 1);
            printOut->print(", dSP=");
            printOut->print(deltaSetpoint, 1);
            printOut->print(", dTemp=");
            printOut->print(deltaTemperature, 1);
            printOut->print(", ActronZoneBits=");
            printHex8(printOut, actronZoneBits);
            printOut->print(", ActronStat1Flags=");
            printHex8(printOut, actronStat1Flags);
            printOut->print(", FlagsXor=");
            printHex8(printOut, flagsXor);
            printOut->print(", Match=");
            printOut->print(match);
            printOut->println();
        }
        return;
    }

    printOut->print("Modbus Function 0x");
    printOut->print(functionCode, HEX);
    printOut->print(" from Slave ");
    printOut->print(slave);
    printOut->println();
}

}  // namespace Actron485
