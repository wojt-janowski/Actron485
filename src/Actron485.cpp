#include "Actron485.h"
#include "Utilities.h"
#include <cmath>
#include <cstring>

namespace Actron485 {
 
    void Controller::serialWrite(bool enable) {
        if (enable) {
            platformSwitchSharedUartToTx(_rxPin, _txPin);

            if (_writeEnablePin > 0) {
                platformDigitalWrite(_writeEnablePin, true);
            }

        } else {
            if (_serial != nullptr) {
                _serial->flush();
            }

            if (_writeEnablePin > 0) {
                platformDigitalWrite(_writeEnablePin, false);
            }

            platformSwitchSharedUartToRx(_rxPin, _txPin);
        }
    }

    void Controller::sendZoneMessage(int zone) {
        if (_serial == nullptr) {
            return;
        }

        if (printOut) {
            printOut->println("Send Zone Message");
        }
        if (zone <= 0 || zone > 8) {
            // Out of bounds
            return;
        }

        // If we have mode change (off/on/open) request pending, send it now
        if (_requestZoneMode[zindex(zone)] != ZoneMode::Ignore) {
            zoneMessage[zindex(zone)].mode = _requestZoneMode[zindex(zone)];
        }

        zoneMessage[zindex(zone)].type = ZoneMessageType::Normal;
        zoneMessage[zindex(zone)].temperature = zoneTemperature[zindex(zone)];

        // Enforce, and set based on set point range limit, if we aren't currently adjusting the master set point
        if (!sendSetpointCommand) {
            zoneSetpoint[zindex(zone)] = clampDouble(
                zoneSetpoint[zindex(zone)],
                masterToZoneMessage[zindex(zone)].minSetpoint,
                masterToZoneMessage[zindex(zone)].maxSetpoint
            );
        }
        zoneMessage[zindex(zone)].setpoint = zoneSetpoint[zindex(zone)];
        
        uint8_t data[zoneMessage[zindex(zone)].messageLength];
        zoneMessage[zindex(zone)].generate(data);

        serialWrite(true); 
        
        for (int i=0; i<5; i++) {
            _serial->write(data[i]);
        }

        serialWrite(false);

        zoneMessage[zindex(zone)].print();
        if (printOut) {
            printOut->println();
            printOut->println();
        }
    }

    void Controller::sendZoneConfigMessage(int zone) {
        if (_serial == nullptr) {
            return;
        }

        if (zone <= 0 || zone > 8) {
            // Out of bounds
            return;
        }
        if (printOut) {
            printOut->println("Send Zone Config");
        }
        ZoneToMasterMessage configMessage;
        configMessage.type = ZoneMessageType::Config;
        configMessage.zone = zoneMessage[zindex(zone)].zone;
        configMessage.mode = zoneMessage[zindex(zone)].mode;
        configMessage.setpoint = zoneMessage[zindex(zone)].setpoint;
        configMessage.temperature = 0;
        uint8_t data[configMessage.messageLength];
        configMessage.generate(data);
        serialWrite(true); 
        
        for (int i=0; i<configMessage.messageLength; i++) {
            _serial->write(data[i]);
        }

        serialWrite(false);
    }

    void Controller::sendZoneInitMessage(int zone) {
        if (_serial == nullptr) {
            return;
        }

        if (printOut) {
            printOut->println("Send Zone Init");
        }
        serialWrite(true);  
        _serial->write((uint8_t) 0x00);
        _serial->write((uint8_t) 0xCC);
        serialWrite(false);
    }

    void Controller::processMasterMessage(MasterToZoneMessage masterMessage) {
        uint8_t zone = masterMessage.zone;
        if (zone <= 0 || zone > 8) {
            // Out of bounds
            return;
        }
        if (zoneControlled[zindex(zone)] == false) {
            // We don't care about this, not us
            return;
        }

        // Confirm our request to turn on/off the zone has been processed
        switch (_requestZoneMode[zindex(zone)]) {
            case ZoneMode::Ignore:
                break;
            case ZoneMode::Off:
                if (masterMessage.on == false) {
                    _requestZoneMode[zindex(zone)] = ZoneMode::Ignore;
                }
                break;
            case ZoneMode::On:
            case ZoneMode::Open: // Master message doesn't show this case, so at this stage we can't tell we switch to open to on and vice versa, so assume it was successful anyway if on
                if (masterMessage.on == true) {
                    _requestZoneMode[zindex(zone)] = ZoneMode::Ignore;
            }
        }

        // If zone is set to 0, we need to copy some values from master
        bool copyZoneSate = (zoneMessage[zindex(zone)].zone == 0);

        // Update Zone on/off state based on this message (open is same as on, but the master message doesn't know the difference)
        if (masterMessage.on == true && zoneMessage[zindex(zone)].mode == ZoneMode::Off || copyZoneSate) {
            zoneMessage[zindex(zone)].mode = ZoneMode::On;
        } else if (masterMessage.on == false && zoneMessage[zindex(zone)].mode != ZoneMode::Off || copyZoneSate) {
            zoneMessage[zindex(zone)].mode = ZoneMode::Off;
        }
        
        if (copyZoneSate) {
            zoneMessage[zindex(zone)].zone = zone;
            zoneMessage[zindex(zone)].temperature = masterMessage.temperature;
            zoneMessage[zindex(zone)].setpoint = masterMessage.setpoint;
        }

        ////////////////////////
        // Send our awaited status report/request/config

        // Do we want to send a config message this round?
        if (_sendZoneConfig[zindex(zone)]) {
            sendZoneConfigMessage(zindex(zone));
            _sendZoneConfig[zindex(zone)] = false;
        } else {
            sendZoneMessage(zone);
        }
    }
    
    void Controller::processZoneMessage(ZoneToMasterMessage zoneMessage) {
        if (zoneControlled[zindex(zoneMessage.zone)] == false) {
            // We don't care about this, not us
            return;
        }

        // If we got a zone message from not us, its probably the master controller
        // asking us to respond
        if (zoneMessage.type == ZoneMessageType::InitZone) {
            sendZoneInitMessage(zoneMessage.zone);
            _sendZoneConfig[zindex(zoneMessage.zone)] = true;
        }
    }

    Controller::Controller(uint8_t rxPin, uint8_t txPin, uint8_t writeEnablePin) {
        _rxPin = rxPin;
        _txPin = txPin;
        _writeEnablePin = writeEnablePin;
        _serial = platformInitDefaultSerial(rxPin, txPin);

        if (writeEnablePin > 0) {
            platformPinModeOutput(writeEnablePin);
            serialWrite(false);
        }
        
        setup();
    }

    Controller::Controller(SerialStream &stream, uint8_t writeEnablePin) {
        _serial = &stream;
        _writeEnablePin = writeEnablePin;
        setup();
    }

    Controller::Controller() {
        setup();
    }

    void Controller::configure(SerialStream &stream, uint8_t writeEnablePin) {
        _serial = &stream;
        _writeEnablePin = writeEnablePin;
    }

    void Controller::configureLogging(LogSink *stream) {
        printOut = stream;
    }

    void Controller::setup() {
        printOutMode = PrintOutMode::ChangedMessages;

        dataLastReceivedTime = 99999;
        outdoorTemperature = NAN;
        outdoorTemperatureTimestamp = 0;

        // Set to ignore
        for (int i=0; i<8; i++) {
            _requestZoneMode[i] = ZoneMode::Ignore;
        }
    }

    uint8_t Controller::totalPendingCommands() {
        uint8_t pending = totalPendingMainCommands();
        for (int i=0; i<8; i++) {
            pending += sendMasterToZoneMessage[i];
        }
        return pending;
    }

    uint8_t Controller::totalPendingMainCommands() {
        uint8_t pending = 0;
        pending += sendOperatingModeCommand;
        pending += sendZoneStateCommand;
        pending += sendFanModeCommand;
        pending += sendSetpointCommand;
        pending += sendZoneSetpointCustomCommand;
        return pending;
    }

    bool Controller::isPendingZoneCommand(int zone) {
        return sendMasterToZoneMessage[zindex(zone)];
    }

    bool Controller::sendQueuedCommand() {
        if (_serial == nullptr) {
            return false;
        }

        uint8_t data[7];
        int send = 0;

        // We can only send one command at a time, per sequence
        // start with the most important ones and work our way down
        if (sendOperatingModeCommand) {
            if (printOut) {
                printOut->print("Send: ");
            }
            sendOperatingModeCommand = false;
            nextOperatingModeCommand.generate(data);
            nextOperatingModeCommand.print();
            send = nextOperatingModeCommand.messageLength;
            
        } else if (sendZoneStateCommand) {
            if (printOut) {
                printOut->print("Send: ");
            }
            sendZoneStateCommand = false;
            nextZoneStateCommand.generate(data);
            nextZoneStateCommand.print();
            send = nextZoneStateCommand.messageLength;
            
        } else if (sendFanModeCommand) {
            if (printOut) {
                printOut->print("Send: ");
            }
            sendFanModeCommand = false;
            nextFanModeCommand.generate(data);
            nextFanModeCommand.print();
            send = nextFanModeCommand.messageLength;
            
        } else if (sendSetpointCommand) {
            if (printOut) {
                printOut->print("Send: ");
            }
            sendSetpointCommand = false;
            nextSetpointCommand.generate(data);
            nextSetpointCommand.print();
            send = nextSetpointCommand.messageLength;
            
        } else if (sendZoneSetpointCustomCommand) {
            if (printOut) {
                printOut->print("Send: ");
            }
            sendZoneSetpointCustomCommand = false;
            nextZoneSetpointCustomCommand.generate(data);
            nextZoneSetpointCustomCommand.print();
            send = nextZoneSetpointCustomCommand.messageLength;

        } else {
            for (int i=0; i<8; i++) {
                if (!sendMasterToZoneMessage[i]) {
                    continue;
                }
                
                if (printOut) {
                    printOut->print("Send: ");
                }
                sendMasterToZoneMessage[i] = false;
                nextMasterToZoneMessage[i].generate(data);
                nextMasterToZoneMessage[i].print();
                send = nextMasterToZoneMessage[i].messageLength;
                break;
            }
        }

        if (send > 0) {
            serialWrite(true); 
            
            for (int i=0; i<send; i++) {
                _serial->write(data[i]);
            }

            serialWrite(false);
            dataLastSentTime = platformMillis();
        }
        
        return send > 0;
    }

    bool Controller::messageLengthCheck(int received, int expected, const char *name, uint8_t *data) {
        if (received == expected) {
            return true;
        }
        if (printOut) {
            printOut->print(name);
            printOut->print(": Invalid Length of ");
            printOut->print(received);
            printOut->print(" received, expected ");
            printOut->print(expected);
            printOut->println();
            printBytes(data, received);
            printOut->println();
            printOut->println();
        }
        return false;
    }

    void Controller::flushSerialBuffer() {
        if (_serialBufferIndex == 0) {
            return;
        }

        // 2020+ Que masters speak pure Modbus RTU. Validated frames go to the
        // register decoder; everything else is dropped. The legacy custom-byte
        // protocol path (processMessage) is intentionally not invoked here:
        // its handlers produce coincidental garbage decodes when fed misframed
        // bytes from a Modbus stream.
        if (isModbusMessage()) {
            processModbusFrame(_serialBuffer, _serialBufferIndex);
            if (printOut && (printOutMode == PrintOutMode::AllMessages
                           || printOutMode == PrintOutMode::CorrelationCapture
                           || printOutMode == PrintOutMode::RegisterDelta)) {
                printModbusMessage();
            }
        }

        _serialBufferIndex = 0;
        _serialBufferExpectedLength = 0;
    }

    void Controller::processModbusFrame(const uint8_t *frame, uint8_t length) {
        if (length < 4) {
            return;
        }
        uint8_t slave = frame[0];
        uint8_t function = frame[1];

        unsigned long now = platformMillis();
        dataLastReceivedTime = now;

        // Function 0x10: Write Multiple Registers Request
        // Layout: [slave][0x10][addrHi][addrLo][cntHi][cntLo][byteCount][data...][crcLo][crcHi]
        if (function == 0x10 && length >= 9) {
            uint16_t startAddr = (uint16_t(frame[2]) << 8) | frame[3];
            uint16_t regCount = (uint16_t(frame[4]) << 8) | frame[5];
            uint8_t byteCount = frame[6];
            if (length < uint8_t(9 + byteCount)) {
                return;
            }
            const uint8_t *data = frame + 7;

            if (slave == 11) {
                applySlave11StateBroadcast(startAddr, regCount, data, byteCount);
            }
            return;
        }

        // Function 0x03: Read Holding Registers
        if (function == 0x03) {
            // Request: 8 bytes total
            if (length == 8) {
                _modbusLastReadSlave = slave;
                _modbusLastReadFunction = function;
                _modbusLastReadStartAddress = (uint16_t(frame[2]) << 8) | frame[3];
                _modbusLastReadCount = (uint16_t(frame[4]) << 8) | frame[5];
                _modbusLastReadTimestamp = now;
                return;
            }
            // Response: [slave][0x03][byteCount][data...][crcLo][crcHi]
            // Match the response back to the most recent request from the same
            // slave so we know which addresses the payload covers.
            if (length < 5 || (now - _modbusLastReadTimestamp) > 5000 ||
                _modbusLastReadSlave != slave || _modbusLastReadFunction != 0x03) {
                return;
            }
            uint8_t byteCount = frame[2];
            if ((byteCount & 1) || length < uint8_t(byteCount + 5)) {
                return;
            }
            uint16_t startAddr = _modbusLastReadStartAddress;
            uint16_t regCount = byteCount / 2;
            const uint8_t *data = frame + 3;

            // Phase 2 #DIAG (2026-04-30): log EVERY read response regardless
            // of slave address so we can see if humidity lives on a slave we
            // don't currently dispatch (2, 4, etc.). Remove once decoded.
            if (printOut != nullptr && slave != 1 && slave != 3) {
                printOut->print("[s");
                printOut->print(slave);
                printOut->print("-data] start=");
                printOut->print(startAddr);
                printOut->print(" count=");
                printOut->print(regCount);
                printOut->print(" :");
                for (uint16_t i = 0; i < regCount; i++) {
                    uint16_t word = (uint16_t(data[i * 2]) << 8) | data[i * 2 + 1];
                    printOut->print(" 0x");
                    if (word < 0x1000) printOut->print("0");
                    if (word < 0x0100) printOut->print("0");
                    if (word < 0x0010) printOut->print("0");
                    printOut->print(word, HEX);
                }
                printOut->println();
            }

            if (slave == 1) {
                applySlave1ReadResponse(startAddr, regCount, data);
            } else if (slave == 3) {
                applySlave3ReadResponse(startAddr, regCount, data);
            }
            return;
        }
    }

    void Controller::applySlave3ReadResponse(uint16_t startAddress, uint16_t regCount, const uint8_t *data) {
        // Phase 2 #DIAG (2026-04-30): humidity probe. Dump every slave 3 read
        // response so we can spot polls outside the usual reg 2-0x18 window
        // (humidity sensor probably lives on the wall controller). Format:
        // `[s3-data] start=N count=M : 0xAABB 0xCCDD ...` Raw 16-bit words.
        if (printOut != nullptr) {
            printOut->print("[s3-data] start=");
            printOut->print(startAddress);
            printOut->print(" count=");
            printOut->print(regCount);
            printOut->print(" :");
            for (uint16_t i = 0; i < regCount; i++) {
                uint16_t word = (uint16_t(data[i * 2]) << 8) | data[i * 2 + 1];
                printOut->print(" 0x");
                if (word < 0x1000) printOut->print("0");
                if (word < 0x0100) printOut->print("0");
                if (word < 0x0010) printOut->print("0");
                printOut->print(word, HEX);
            }
            printOut->println();
        }

        // Slave 3 read at start=2 count=11 carries the AC head's authoritative
        // operating-mode + compressor state. Slave 11's broadcast lags behind
        // for compressor-active transitions (it stays at 0x80 even when the
        // compressor is cooling), so prefer slave 3 here when available.
        //
        // Reg 2 high byte (bits 0-3 = mode, bit 6 = compressor cooling,
        //                  bit 7 = quiet mode):
        //   0x02 = Cool/Auto, compressor idle
        //   0x42 (bits 1+6) = Cool, compressor cooling   (also seen 0x47/0x4A —
        //                     the low nibble seems to be a small counter)
        //   0x01 = Heat, compressor idle
        //   0x01 + low byte 0x64 = Heat, compressor heating
        //   0x08 = Fan only
        //   +0x80 overlaid on any of the above = quiet mode flag (Phase 2
        //                                        probe 2026-04-30: 0x01 → 0x81)
        //
        // Reg 2 low byte:
        //   0x00 = Cool        (when reg 2 hi == 0x02)
        //   0x23 = Auto        (when reg 2 hi == 0x02)
        //   ... low-nibble counters when actively cooling (0x42-0x4A range)
        //
        // Reg 3 high byte: bit 0 = system armed (any zone enabled),
        //                  bit 1 = fan running (transient at spin-up),
        //                  bit 2 = continuous-fan flag.
        if (startAddress > 2 || startAddress + regCount <= 2) {
            return;
        }
        uint16_t reg2Index = 2 - startAddress;
        uint8_t reg2HiRaw = data[reg2Index * 2];
        uint8_t reg2Lo = data[reg2Index * 2 + 1];
        // Strip quiet bit before mode matching; record it on the state.
        stateMessage2.quietMode = (reg2HiRaw & 0x80) != 0;
        uint8_t reg2Hi = reg2HiRaw & 0x7F;

        if ((reg2Hi & 0x40) != 0) {
            // Compressor actively cooling.
            stateMessage2.operatingMode = OperatingMode::Cool;
            stateMessage2.compressorMode = CompressorMode::Cooling;
        } else if (reg2Hi == 0x02) {
            // Cool or Auto, compressor idle. Distinguish by low byte.
            stateMessage2.operatingMode = (reg2Lo == 0x23)
                ? OperatingMode::Auto
                : OperatingMode::Cool;
            stateMessage2.compressorMode = CompressorMode::Idle;
        } else if (reg2Hi == 0x01) {
            // Heat. Low byte non-zero = compressor actually heating
            // (0x64 observed). 0x00 = standby.
            stateMessage2.operatingMode = OperatingMode::Heat;
            stateMessage2.compressorMode = (reg2Lo != 0x00)
                ? CompressorMode::Heating
                : CompressorMode::Idle;
        } else if (reg2Hi == 0x08) {
            stateMessage2.operatingMode = OperatingMode::FanOnly;
            stateMessage2.compressorMode = CompressorMode::Idle;
        } else if (reg2Hi == 0x00 && reg2Lo == 0x00) {
            stateMessage2.operatingMode = OperatingMode::Off;
            stateMessage2.compressorMode = CompressorMode::Idle;
        }
        // Unknown patterns: leave previous values untouched.

        // Reg 3 carries fan setting + status flags (Phase 2 probe 2026-04-30):
        //
        //   High byte:
        //     bit 0 = system armed
        //     bit 1 = fan running (transient at spin-up; also set in non-fan-
        //             only modes when fan is auto-running)
        //     bit 2 = continuous fan flag
        //     (in Fan-only mode, bits 3-7 ALSO reflect the user-selected fan
        //      speed: bit 3 = Low, bit 4 = Medium, bit 6 = High; we read the
        //      low byte instead since it's mode-independent)
        //
        //   Low byte = explicit fan speed value:
        //     0x00 = Auto / Esp (let the system pick)
        //     0x28 (40) = Low
        //     0x3D (61) = Medium
        //     0x59 (89) = High
        //   Holds across Heat/Cool/Fan-only — in thermal modes, 0x00 (Auto)
        //   is the typical default.
        if (startAddress + regCount > 3) {
            uint16_t reg3Index = 3 - startAddress;
            uint8_t reg3Hi = data[reg3Index * 2];
            uint8_t reg3Lo = data[reg3Index * 2 + 1];
            stateMessage2.continuousFan = (reg3Hi & 0x04) != 0;

            FanMode decodedFan;
            switch (reg3Lo) {
                case 0x00: decodedFan = FanMode::Esp;    break;  // "Auto" on the LCD
                case 0x28: decodedFan = FanMode::Low;    break;
                case 0x3D: decodedFan = FanMode::Medium; break;
                case 0x59: decodedFan = FanMode::High;   break;
                default:
                    // Unknown — leave previous values rather than silently
                    // misreporting. Probe more values if they crop up.
                    decodedFan = stateMessage2.fanMode;
                    break;
            }
            stateMessage2.fanMode = decodedFan;
            // Running fan is what's actually moving air right now. In
            // Fan-only mode the requested speed IS what's running; in
            // thermal modes it's only running when the compressor calls for
            // it (bit 1 of reg 3 high byte indicates that).
            stateMessage2.runningFanMode = ((reg3Hi & 0x02) != 0)
                ? decodedFan
                : FanMode::Off;
            stateMessage2.fanActive = (reg3Hi & 0x02) != 0;
        }
    }

    void Controller::applySlave1ReadResponse(uint16_t startAddress, uint16_t regCount, const uint8_t *data) {
        // Slave 1 (AC head) returns a 9-float block at reg 120 in response to a
        // count=18 read. Float index 5 (regs 130/131) is the outdoor temperature
        // sensor — confirmed against the wall LCD's "Outdoor" reading (~18.7°C).
        // Encoding is big-endian IEEE-754 with the high half-word first (ABCD).
        // Phase 2 #DIAG (2026-04-30): humidity probe. Slave 1 only responds
        // when the compressor is active. Dump all decoded floats from EVERY
        // slave 1 read response so we can identify which register block
        // carries the indoor humidity reading. Remove once humidity is
        // decoded. Placed BEFORE the reg-120 filter so we see any block.
        if (printOut != nullptr) {
            printOut->print("[s1-floats] start=");
            printOut->print(startAddress);
            printOut->print(" count=");
            printOut->print(regCount);
            printOut->print(" :");
            for (uint16_t i = 0; i + 1 < regCount; i += 2) {
                uint16_t regAddr = startAddress + i;
                uint16_t a = (uint16_t(data[i * 2]) << 8) | data[i * 2 + 1];
                uint16_t b = (uint16_t(data[i * 2 + 2]) << 8) | data[i * 2 + 3];
                uint32_t raw = (uint32_t(a) << 16) | b;
                float val;
                memcpy(&val, &raw, sizeof(val));
                printOut->print(" r");
                printOut->print(regAddr);
                printOut->print("=");
                if (std::isfinite(val)) {
                    printOut->print(double(val), 3);
                } else {
                    printOut->print("nan");
                }
            }
            printOut->println();
        }

        const uint16_t outdoorRegHigh = 130;
        const uint16_t outdoorRegLow = 131;
        if (startAddress > outdoorRegHigh || startAddress + regCount <= outdoorRegLow) {
            return;
        }

        uint16_t offsetHigh = (outdoorRegHigh - startAddress) * 2;
        uint16_t regA = (uint16_t(data[offsetHigh]) << 8) | data[offsetHigh + 1];
        uint16_t regB = (uint16_t(data[offsetHigh + 2]) << 8) | data[offsetHigh + 3];
        uint32_t raw = (uint32_t(regA) << 16) | regB;
        float value;
        memcpy(&value, &raw, sizeof(value));
        if (!std::isfinite(value) || value < -60.0f || value > 80.0f) {
            return;
        }
        outdoorTemperature = double(value);
        outdoorTemperatureTimestamp = platformMillis();
    }

    double Controller::getOutdoorTemperature() {
        return outdoorTemperature;
    }

    void Controller::applySlave11StateBroadcast(uint16_t startAddress, uint16_t regCount, const uint8_t *data, uint8_t byteCount) {
        // The master writes a 10-register status block to slave 11 every ~2-15s.
        // Layout (validated against a 2020 Que with mixed per-zone setpoints):
        //   reg 2  : status word — high byte 0x02 when system active, 0x00 off
        //   reg 3  : mode flags — high byte (see decode table below).
        //   reg 4-7  : 8 per-zone setpoints packed 2 per register,
        //              each byte = (zone_setpoint × 2). Phase 1 mistook these
        //              for "master setpoint markers" because all zones in the
        //              probe sample shared the same setpoint; with mixed
        //              setpoints the bytes diverge — see Phase 2 capture in
        //              PROTOCOL_NOTES.md.
        //   reg 8-11 : 8 zone temp offsets packed 2 per register, each byte
        //              is signed int8 tenths of °C offset from THAT zone's
        //              setpoint (not from the master setpoint).
        if (startAddress != 2 || regCount < 10 || byteCount < 20) {
            return;
        }

        // Operating mode lives in reg 2 high byte (data[0]) — Phase 2 captured:
        //   0x00 = system off
        //   0x01 = Heat
        //   0x02 = Cool or Auto (slave 11 alone can't disambiguate; slave 3
        //          reg 2 low byte 0x00=Cool / 0x23=Auto — see PROTOCOL_NOTES)
        //   0x08 = Fan only
        //
        // Reg 2 high byte (data[0]) carries the operating mode in bits 0-3
        // and the **quiet-mode flag in bit 7** (Phase 2 probe 2026-04-30:
        // 0x01 → 0x81 when quiet enabled in Heat). Mask bit 7 before mode
        // matching so the switch still picks the right operating mode.
        //
        // Reg 3 high byte (data[2]) is the **active-zone bitmap** — bit N set
        // means zone N+1 is enabled by the wall controller. Confirmed Phase 2
        // probe (2026-04-30): all-off → 0x00, zone 1 only → 0x01, zones 1+4 →
        // 0x09, zones 1+4+8 → 0x89. Earlier "Heat active = 0xFF" / "Cool
        // active = 0x82" notes were a misread of this bitmap (0xFF = all 8
        // zones on during heat). Compressor state now comes exclusively from
        // slave 3 reg 2 (applySlave3ReadResponse — authoritative).
        //
        // Reg 3 low byte (data[3]) high nibble carries system status flags;
        // bit 2 of that low byte = continuous-fan flag (Phase 2 probe
        // 2026-04-30: data[3] high byte on slave 3 reg 3 went 0x01 → 0x07
        // when continuous fan enabled). Slave 11 broadcast doesn't seem to
        // mirror this — only slave 3 read response sees it — so continuous
        // fan is decoded in applySlave3ReadResponse, not here.
        uint8_t modeRaw = data[0];
        uint8_t modeWord = modeRaw & 0x0F;
        stateMessage2.quietMode = (modeRaw & 0x80) != 0;

        uint8_t zoneBitmap = data[2];
        for (int z = 0; z < 8; z++) {
            stateMessage2.zoneOn[z] = (zoneBitmap & (1 << z)) != 0;
        }
        if (modeWord == 0x00 && zoneBitmap == 0x00) {
            stateMessage2.operatingMode = OperatingMode::Off;
            stateMessage2.compressorMode = CompressorMode::Idle;
        } else {
            switch (modeWord) {
                case 0x01:
                    stateMessage2.operatingMode = OperatingMode::Heat;
                    break;
                case 0x02:
                    // Default to Cool — Auto vs Cool disambiguation comes
                    // from slave 3 read response (separate Phase 2 task).
                    stateMessage2.operatingMode = OperatingMode::Cool;
                    break;
                case 0x08:
                    stateMessage2.operatingMode = OperatingMode::FanOnly;
                    break;
                default:
                    // Unknown — leave previous values rather than spuriously
                    // pinning the UI to Off.
                    break;
            }
        }

        // 8 per-zone setpoints (regs 4-7, bytes data[4..11]).
        // Defensive: a 0 byte means "no value" — skip rather than report 0°C.
        bool anyValid = false;
        double firstValidSetpoint = 0.0;
        for (int z = 0; z < 8; z++) {
            uint8_t b = data[4 + z];
            if (b == 0) {
                continue;
            }
            zoneSetpoint[z] = double(b) / 2.0;
            if (!anyValid) {
                firstValidSetpoint = zoneSetpoint[z];
                anyValid = true;
            }
        }
        if (!anyValid) {
            return;
        }

        // 8 zone temp offsets (regs 8-11, bytes data[12..19]) — signed int8
        // tenths of °C, RELATIVE TO THAT ZONE'S OWN SETPOINT.
        for (int z = 0; z < 8; z++) {
            int8_t offset = (int8_t) data[12 + z];
            zoneTemperature[z] = zoneSetpoint[z] + (double(offset) * 0.1);
        }

        // Master setpoint = the wall-controller zone's setpoint (Phase 2 #11
        // confirmed 2026-04-30: bumping master on the LCD only changes that
        // zone's setpoint cleanly; other active zones get pulled too but
        // some end up clamped to the master±2°C window so they're not
        // reliable). The wall-controller zone is configured via the climate
        // component's `control_zone:` yaml setting, which calls
        // setControlZone(N, true). We pick the first such zone with a valid
        // setpoint and fall back to the first non-zero setpoint for
        // backward compatibility on rigs that don't configure it.
        double masterSetpoint = -1.0;
        for (int z = 0; z < 8; z++) {
            if (zoneControlled[z] && zoneSetpoint[z] > 0.0) {
                masterSetpoint = zoneSetpoint[z];
                break;
            }
        }
        stateMessage2.setpoint = (masterSetpoint > 0.0) ? masterSetpoint : firstValidSetpoint;

        // Master indoor temp stop-gap remains the mean of all 8 zone temps
        // until Phase 2 task #2 locates the wall-controller thermistor
        // register. With per-zone setpoints now decoded correctly the mean
        // drifts a bit further from the LCD than under Phase 1's all-equal
        // assumption, but it's still usable for graceful degradation.
        double zoneSum = 0.0;
        for (int z = 0; z < 8; z++) {
            zoneSum += zoneTemperature[z];
        }
        stateMessage2.temperature = zoneSum / 8.0;

        stateMessage2.initialised = true;
        statusLastReceivedTime = platformMillis();
    }

    uint8_t Controller::expectedActronMessageLength(MessageType messageType) {
        switch (messageType) {
            case MessageType::CommandMasterSetpoint:
            case MessageType::CommandFanMode:
            case MessageType::CommandOperatingMode:
            case MessageType::CommandZoneState:
                return 2;
            case MessageType::CustomCommandChangeZoneSetpoint:
                return ZoneSetpointCustomCommand::messageLength;
            case MessageType::ZoneWallController:
                return ZoneToMasterMessage::messageLength;
            case MessageType::ZoneMasterController:
                return MasterToZoneMessage::messageLength;
            case MessageType::IndoorBoard2:
                return StateMessage2::stateMessageLength;
            case MessageType::Stat1:
                return StateMessage::stateMessageLength;
            case MessageType::Stat2:
                return stat2MessageLength;
            case MessageType::UltimaState:
                return UltimaState::stateMessageLength;
            case MessageType::IndoorBoard1:
            case MessageType::Unknown:
                return 0;
        }
        return 0;
    }

    bool Controller::isProbableModbusFunction(uint8_t functionCode) {
        return functionCode == 0x03 || functionCode == 0x04 || functionCode == 0x06 || functionCode == 0x10;
    }

    uint8_t Controller::expectedModbusMessageLength() {
        if (_serialBufferIndex < 2) {
            return 0;
        }

        // Typical Modbus RTU address range 1..247
        if (_serialBuffer[0] == 0x00 || _serialBuffer[0] > 0xF7) {
            return 0;
        }

        uint8_t functionCode = _serialBuffer[1];
        if (!isProbableModbusFunction(functionCode)) {
            return 0;
        }

        if (functionCode == 0x06) {
            return 8;
        }

        if (functionCode == 0x03 || functionCode == 0x04) {
            // Could be request (8 bytes) or response (5 + byte_count)
            uint8_t expected = 8;
            if (_serialBufferIndex >= 3) {
                uint16_t responseLength = (uint16_t) (5 + _serialBuffer[2]);
                if (responseLength >= 8 && responseLength <= _serialBufferSize) {
                    expected = responseLength;
                }
            }
            return expected;
        }

        if (functionCode == 0x10) {
            // Could be response (8 bytes) or request (9 + byte_count)
            uint8_t expected = 8;
            if (_serialBufferIndex >= 7) {
                uint16_t requestLength = (uint16_t) (9 + _serialBuffer[6]);
                if (requestLength >= 8 && requestLength <= _serialBufferSize) {
                    expected = requestLength;
                }
            }
            return expected;
        }

        return 0;
    }

    uint16_t Controller::checksumModbus(const uint8_t *data, uint8_t length) {
        uint16_t checksum = 0xFFFF;

        for (uint8_t i = 0; i < length; i++) {
            checksum ^= data[i];
            for (uint8_t bit = 0; bit < 8; bit++) {
                if (checksum & 0x0001) {
                    checksum = (checksum >> 1) ^ 0xA001;
                } else {
                    checksum = checksum >> 1;
                }
            }
        }
        return checksum;
    }

    bool Controller::isModbusMessage() {
        if (_serialBufferIndex < 4) {
            return false;
        }

        uint8_t expectedLength = expectedModbusMessageLength();
        if (expectedLength == 0 || _serialBufferIndex != expectedLength) {
            return false;
        }

        uint16_t expectedChecksum = checksumModbus(_serialBuffer, _serialBufferIndex - 2);
        uint8_t expectedChecksumLow = expectedChecksum & 0xFF;
        uint8_t expectedChecksumHigh = (expectedChecksum >> 8) & 0xFF;

        return _serialBuffer[_serialBufferIndex - 2] == expectedChecksumLow && _serialBuffer[_serialBufferIndex - 1] == expectedChecksumHigh;
    }

    ///////////////////////////////////
    // Message type

    MessageType Controller::detectActronMessageType(uint8_t firstByte) {
        if (firstByte == (uint8_t) MessageType::CommandMasterSetpoint) {
            return MessageType::CommandMasterSetpoint;
        } else if (firstByte == (uint8_t) MessageType::CommandFanMode) {
            return MessageType::CommandFanMode;
        } else if (firstByte == (uint8_t) MessageType::CommandOperatingMode) {
            return MessageType::CommandOperatingMode;
        } else if (firstByte == (uint8_t) MessageType::CommandZoneState) {
            return MessageType::CommandZoneState;
        } else if (firstByte == (uint8_t) MessageType::IndoorBoard1) {
            return MessageType::IndoorBoard1;
        } else if (firstByte == (uint8_t) MessageType::IndoorBoard2) {
            return MessageType::IndoorBoard2;
        } else if (firstByte == (uint8_t) MessageType::Stat1) {
            return MessageType::Stat1;
        } else if (firstByte == (uint8_t) MessageType::Stat2) {
            return MessageType::Stat2;
        } else if (firstByte == (uint8_t) MessageType::UltimaState) {
            return MessageType::UltimaState;
        } else if ((firstByte & (uint8_t) MessageType::ZoneWallController) == (uint8_t) MessageType::ZoneWallController) {
            return MessageType::ZoneWallController;
        } else if ((firstByte & (uint8_t) MessageType::ZoneMasterController) == (uint8_t) MessageType::ZoneMasterController) {
            return MessageType::ZoneMasterController;
        }

        return MessageType::Unknown;
    }

    void Controller::loop() {
        if (_serial == nullptr) {
            return;
        }

        unsigned long now = platformMillis();
        long serialLastReceivedTime = now - _serialBufferReceivedTime;

        if (_serialBufferIndex > 0) {
            unsigned long serialFrameTimeout = _serialBufferBreak * 4;
            if (serialFrameTimeout < 30) {
                serialFrameTimeout = 30;
            }

            bool messageBreak = serialLastReceivedTime > _serialBufferBreak;
            bool messageTimeout = serialLastReceivedTime > serialFrameTimeout;

            if ((_serialBufferExpectedLength == 0 && messageBreak) || (_serialBufferExpectedLength > 0 && messageTimeout)) {
                flushSerialBuffer();
            }
        }

        // A gap send our message?
        if ((now - dataLastReceivedTime) > 500 && (now - dataLastReceivedTime) < 1000 && (now - _lastQuietPeriodDetectedTime) > 900) {
            _lastQuietPeriodDetectedTime = now;
            attemptToSendQueuedCommand();
        }

        const uint8_t maxBytesPerLoop = 64;
        uint8_t bytesRead = 0;

        while(_serial->available() > 0 && _serialBufferIndex < _serialBufferSize && bytesRead < maxBytesPerLoop) {
            uint8_t byte = _serial->read();
            _serialBuffer[_serialBufferIndex] = byte;
            _serialBufferIndex++;
            bytesRead++;
            _serialBufferReceivedTime = platformMillis();

            MessageType firstByteType = MessageType::Unknown;
            if (_serialBufferIndex >= 1) {
                firstByteType = detectActronMessageType(_serialBuffer[0]);
            }

            if (_serialBufferIndex == 1) {
                _serialBufferExpectedLength = expectedActronMessageLength(firstByteType);
            }

            // Only fall through to Modbus framing if the first byte didn't already match a known
            // Actron message type. Several Actron types (notably IndoorBoard1 = 0x01) collide with
            // valid Modbus slave addresses, and their second byte can match a Modbus function code,
            // which would otherwise cause Modbus framing to chop the Actron frame in half.
            if (firstByteType == MessageType::Unknown) {
                uint8_t modbusLength = expectedModbusMessageLength();
                if (modbusLength > 0 && (_serialBufferExpectedLength == 0 || modbusLength > _serialBufferExpectedLength)) {
                    _serialBufferExpectedLength = modbusLength;
                }
            }

            if (_serialBufferExpectedLength > 0 && _serialBufferIndex >= _serialBufferExpectedLength) {
                flushSerialBuffer();
            }
        }

        if (_serialBufferIndex >= _serialBufferSize) {
            flushSerialBuffer();
        }
    }

    void Controller::attemptToSendQueuedCommand() {
        // Phase 1: read-only. The legacy custom-byte command frames produced
        // by sendQueuedCommand() are not compatible with the Modbus RTU bus on
        // 2020+ Que masters and would cause CRC garbage if injected. Phase 3
        // will replace this with quiet-window Modbus Write Multiple Registers
        // frames once Phase 2 has identified the writable command registers.
        unsigned long now = platformMillis();
        if ((now - dataLastSentTime) > 1999) {
            boardComms1Index = 0;
            if (printOut && (printOutMode == PrintOutMode::AllMessages || printOutMode == PrintOutMode::CorrelationCapture)) {
                printOut->println("Time to Send");
            }
            // Drain command queue without transmitting so callers don't see
            // perpetually-pending commands. Real injection arrives in Phase 3.
            sendOperatingModeCommand = false;
            sendZoneStateCommand = false;
            sendSetpointCommand = false;
            sendFanModeCommand = false;
            sendZoneSetpointCustomCommand = false;
            for (int z = 0; z < 8; z++) {
                sendMasterToZoneMessage[z] = false;
            }
        }
    }

    void Controller::processMessage(uint8_t *data, uint8_t length) {
        unsigned long now = platformMillis();
        dataLastReceivedTime = now;
        bool printChangesOnly = printOutMode == PrintOutMode::ChangedMessages;
        bool printAll = (printOutMode == PrintOutMode::AllMessages);
        bool printCapture = (printOutMode == PrintOutMode::CorrelationCapture);
        bool changed = false;

        uint8_t zone = 0;

        MessageType messageType = MessageType::Unknown;
        
        if ((now - dataLastSentTime) < 50) {
            // This will be a response to our command
            if (printOut) {
                printOut->println("Response Message Received");
            }

        } else {
            messageType = detectActronMessageType(data[0]);
            uint8_t expectedMessageLength;
            switch (messageType) {
                case MessageType::Unknown:
                    if (printOut) {
                        printOut->println("Unknown Message received");
                    }
                    changed = true;
                    break;
                case MessageType::CommandMasterSetpoint:
                    // We don't care about this command
                    break;
                case MessageType::CommandFanMode:
                    // We don't care about this command
                    break;
                case MessageType::CommandOperatingMode:
                    // We don't care about this command
                    break;
                case MessageType::CommandZoneState:
                    // We don't care about this command
                    break;
                case MessageType::CustomCommandChangeZoneSetpoint:
                    {
                        expectedMessageLength = ZoneSetpointCustomCommand::messageLength;
                        if (!messageLengthCheck(length, expectedMessageLength, "Zone Setpoint Command", data)) {
                            break;
                        }
                        ZoneSetpointCustomCommand command;
                        command.parse(data);
                        if (command.zone >= 1 && command.zone <= 8 && zoneControlled[zindex(command.zone)]) {
                            setZoneSetpointTemperature(command.zone, command.temperature, command.adjustMaster);
                        }
                    }
                    break;
                case MessageType::ZoneWallController:
                    zone = data[0] & 0x0F;
                    if (!(0 < zone && zone <= 8)) {
                        break;
                    }
                    expectedMessageLength = zoneMessage[zindex(zone)].messageLength;
                    if (!messageLengthCheck(length, expectedMessageLength, "Zone Message", data)) {
                        break;
                    }
                    if (zoneMessage[zindex(zone)].parse(data)) {
                        changed = copyBytes(data, zoneWallMessageRaw[zindex(zone)], expectedMessageLength);
                        if (printCapture) {
                            printCaptureZoneWallMessage(zone, data, expectedMessageLength);
                        }
                        if (printOut && (printAll || (printChangesOnly && changed))) {
                            zoneMessage[zindex(zone)].print();
                            printOut->println();
                        }
                    } else if (printOut) {
                        printOut->println("Zone Message: Checksum failed");
                    }
                    break;
                case MessageType::ZoneMasterController:
                    zone = data[0] & 0x0F;
                    if (!(0 < zone && zone <= 8)) {
                        break;
                    }
                    expectedMessageLength = masterToZoneMessage[zindex(zone)].messageLength;
                    if (!messageLengthCheck(length, expectedMessageLength, "Master to Zone", data)) {
                        break;
                    }
                    if (masterToZoneMessage[zindex(zone)].parse(data)) {
                        changed = copyBytes(data, zoneMasterMessageRaw[zindex(zone)], expectedMessageLength);
                        if (printCapture) {
                            printCaptureZoneMasterMessage(zone, data, expectedMessageLength);
                        }

                        if (printOut && (printAll || (printChangesOnly && changed))) {
                            masterToZoneMessage[zindex(zone)].print();
                            printOut->println();
                        }
                    } else if (printOut) {
                        printOut->println("Master to Zone: Checksum failed");
                    }
                    break;
                case MessageType::IndoorBoard1:
                    expectedMessageLength = sizeof(boardComms1Message[boardComms1Index]);
                    if (length > expectedMessageLength && printOut) {
                        printOut->print("Indoor Board Message: Truncating length ");
                        printOut->print(length);
                        printOut->print(" to ");
                        printOut->print(expectedMessageLength);
                        printOut->println();
                    }
                    {
                        uint8_t copiedLength = (length > expectedMessageLength ? expectedMessageLength : length);
                        changed = copyBytes(data, boardComms1Message[boardComms1Index], copiedLength);
                        boardComms1MessageLength[boardComms1Index] = copiedLength;
                    }
                    boardComms1Index = (boardComms1Index + 1)%2;
                    break;
                case MessageType::IndoorBoard2:
                    expectedMessageLength = stateMessage2.stateMessageLength;
                    if (!messageLengthCheck(length, expectedMessageLength, "State Message 2", data)) {
                        break;
                    }
                    changed = copyBytes(data, stateMessage2Raw, expectedMessageLength);
                    stateMessage2.parse(data);
                    statusLastReceivedTime = now;

                    if (sendZoneStateCommand == false) {
                        // Here we can assume we now have the correct zone on/off state
                        _sendZoneStateCommandCleared = true;
                    }

                    if (printOut && (printAll || (printChangesOnly && changed))) {
                        stateMessage2.print();
                        printOut->println();
                    }
                    if (printCapture) {
                        printCaptureStateSnapshot(data, expectedMessageLength, "STATE2");
                    }
                    break;
                case MessageType::Stat1:
                    expectedMessageLength = stateMessage.stateMessageLength;
                    if (!messageLengthCheck(length, expectedMessageLength, "Stat Message 1", data)) {
                        break;
                    }
                    changed = copyBytes(data, stateMessageRaw, expectedMessageLength);
                    stateMessage.parse(data);
                    statusLastReceivedTime = now;

                    if (sendZoneStateCommand == false) {
                        // Here we can assume we now have the correct zone on/off state
                        _sendZoneStateCommandCleared = true;
                    }

                    if (printOut && (printAll || (printChangesOnly && changed))) {
                        stateMessage.print();
                        printOut->println();
                    }
                    if (printCapture) {
                        printCaptureStateSnapshot(data, expectedMessageLength, "STATE");
                    }
                    break;
                case MessageType::Stat2:
                    expectedMessageLength = stat2MessageLength;
                    if (!messageLengthCheck(length, expectedMessageLength, "Stat Message 2", data)) {
                        break;
                    }
                    changed = copyBytes(data, stat2Message, expectedMessageLength);
                    break;
                case MessageType::UltimaState:
                    expectedMessageLength = ultimaState.stateMessageLength;
                    if (!messageLengthCheck(length, expectedMessageLength, "Ultima State", data)) {
                        break;
                    }
                    changed = copyBytes(data, ultimaStateMessageRaw, expectedMessageLength);
                    ultimaState.parse(data);

                    if (printOut && (printAll || (printChangesOnly && changed))) {
                        ultimaState.print();
                        printOut->println();
                    }
                    if (printCapture) {
                        printCaptureStateSnapshot(data, expectedMessageLength, "ULTIMA");
                    }
                    break;
            }
        }

        if (printOut && (printAll || (printChangesOnly && changed))) {
            printBytes(data, length);
            printOut->println();
            printOut->println();
        }

        // We need to process after printing, else the logs appear out of order
        if (0 < zone && zone <= 8) {
            switch (messageType) {
                case MessageType::ZoneWallController:
                    processZoneMessage(zoneMessage[zindex(zone)]);
                    break;
                case MessageType::ZoneMasterController:
                    processMasterMessage(masterToZoneMessage[zindex(zone)]);
                    break;
            }
        }
    }

     //////////////////////
    /// Convenient functions, that are the typical use for this module

    bool Controller::receivingData() {
        return (platformMillis() - dataLastReceivedTime) < 3000;
    }

    // Setup

    void Controller::setControlZone(uint8_t zone, bool control) {
        zoneControlled[zindex(zone)] = control;
    }

    bool Controller::getControlZone(uint8_t zone) {
        return zoneControlled[zindex(zone)];
    }

    // System Control

    void Controller::setSystemOn(bool on) {
        if (!receivingData()) {
            return;
        }

        OperatingMode currentMode = getOperatingMode();
        
        if (on) {
            switch (currentMode) {
                case OperatingMode::Off:
                    nextOperatingModeCommand.mode = OperatingMode::FanOnly;
                    sendOperatingModeCommand = true;
                    break;
                case OperatingMode::OffAuto:
                    nextOperatingModeCommand.mode = OperatingMode::Auto;
                    sendOperatingModeCommand = true;
                    break;
                case OperatingMode::OffHeat:
                    nextOperatingModeCommand.mode = OperatingMode::Heat;
                    sendOperatingModeCommand = true;
                    break;
                case OperatingMode::OffCool:
                    nextOperatingModeCommand.mode = OperatingMode::Cool;
                    sendOperatingModeCommand = true;
                    break;
            }
        } else {
            switch (currentMode) {
                case OperatingMode::FanOnly:
                    nextOperatingModeCommand.mode = OperatingMode::Off;
                    sendOperatingModeCommand = true;
                    break;
                case OperatingMode::Auto:
                    nextOperatingModeCommand.mode = OperatingMode::OffAuto;
                    sendOperatingModeCommand = true;
                    break;
                case OperatingMode::Heat:
                    nextOperatingModeCommand.mode = OperatingMode::OffHeat;
                    sendOperatingModeCommand = true;
                    break;
                case OperatingMode::Cool:
                    nextOperatingModeCommand.mode = OperatingMode::OffCool;
                    sendOperatingModeCommand = true;
                    break;
            }
        }
    }

    bool Controller::getSystemOn() {
        OperatingMode currentMode = getOperatingMode();

        switch (currentMode) {
            case OperatingMode::FanOnly:
                return true;
            case OperatingMode::Auto:
                return true;
            case OperatingMode::Heat:
                return true;
            case OperatingMode::Cool:
                return true;
            default:
                return false;
        }
    }

    void Controller::setFanSpeed(FanMode fanSpeed) {
        if (!receivingData()) {
            return;
        }

        bool continuous = getContinuousFanMode();
        switch (fanSpeed) {
            case FanMode::Low:
                nextFanModeCommand.fanMode = continuous ? FanMode::LowContinuous : fanSpeed;
                sendFanModeCommand = true;
                break;
            case FanMode::Medium:
                nextFanModeCommand.fanMode = continuous ? FanMode::MediumContinuous : fanSpeed;
                sendFanModeCommand = true;
                break;
            case FanMode::High:
                nextFanModeCommand.fanMode = continuous ? FanMode::HighContinuous : fanSpeed;
                sendFanModeCommand = true;
                break;
            case FanMode::Esp:
                nextFanModeCommand.fanMode = continuous ? FanMode::EspContinuous : fanSpeed;
                sendFanModeCommand = true;
                break;
        }
    }

    FanMode Controller::getFanSpeed() {
        if (stateMessage.initialised == true) {
            // Read from State Message
            return stateMessage.fanMode;
        } else if (stateMessage2.initialised == true) {
            // Read from State 2 Message
            return stateMessage2.fanMode;
        }
        return FanMode::Off;
    }

    FanMode Controller::getRunningFanSpeed() {
        if (stateMessage.initialised == true) {
            // Read from State Message
            return stateMessage.runningFanMode;
        } else if (stateMessage2.initialised == true) {
            // Read from State 2 Message
            return stateMessage2.runningFanMode;
        }
        return FanMode::Off;
    }

    void Controller::setFanSpeedAbsolute(FanMode fanSpeed) {
        if (!receivingData()) {
            return;
        }

        nextFanModeCommand.fanMode = fanSpeed;
        sendFanModeCommand = true;
    }

    void Controller::setContinuousFanMode(bool on) {
        if (!receivingData()) {
            return;
        }

        FanMode fanSpeed = getFanSpeed();
        switch (fanSpeed) {
            case FanMode::Low:
                nextFanModeCommand.fanMode = on ? FanMode::LowContinuous : fanSpeed;
                sendFanModeCommand = true;
                break;
            case FanMode::Medium:
                nextFanModeCommand.fanMode = on ? FanMode::MediumContinuous : fanSpeed;
                sendFanModeCommand = true;
                break;
            case FanMode::High:
                nextFanModeCommand.fanMode = on ? FanMode::HighContinuous : fanSpeed;
                sendFanModeCommand = true;
                break;
            case FanMode::Esp:
                nextFanModeCommand.fanMode = on ? FanMode::EspContinuous : fanSpeed;
                sendFanModeCommand = true;
                break;
        }
    }

    bool Controller::getContinuousFanMode() {
        if (stateMessage.initialised == true) {
            // Read from State Message
            return stateMessage.continuousFan;
        } else if (stateMessage2.initialised == true) {
            // Read from State 2 Message
            return stateMessage2.continuousFan;
        }
        return false;
    }

    bool Controller::getQuietMode() {
        if (stateMessage2.initialised == true) {
            return stateMessage2.quietMode;
        }
        return false;
    }

    void Controller::setOperatingMode(OperatingMode mode) {
        if (!receivingData()) {
            return;
        }

        if (mode == OperatingMode::Off) {
            setSystemOn(false);
        } else {
            nextOperatingModeCommand.mode = mode;
            sendOperatingModeCommand = true;
        }
    }

    OperatingMode Controller::getOperatingMode() {
        if (stateMessage.initialised == true) {
            // Read from State Message
            return stateMessage.operatingMode;
        } else if (stateMessage2.initialised == true) {
            // Read from State 2 Message
            return stateMessage2.operatingMode;
        }
        return OperatingMode::Off;
    }

    void Controller::setMasterSetpoint(double temperature) {
        if (!receivingData()) {
            return;
        }

        nextSetpointCommand.temperature = temperature;
        sendSetpointCommand = true;
    }
    
    double Controller::getMasterSetpoint() {
        if (stateMessage.initialised == true) {
            // Read from State Message
            return stateMessage.setpoint;
        } else if (stateMessage2.initialised == true) {
            // Read from State 2 Message
            return stateMessage2.setpoint;
        }
        return 0;
    }

    double Controller::getMasterCurrentTemperature() {
        if (stateMessage.initialised == true) {
            // Read from State Message
            return stateMessage.temperature;
        } else if (stateMessage2.initialised == true) {
            // Read from State 2 Message
            return stateMessage2.temperature;
        }
        return 0;
    }

    CompressorMode Controller::getCompressorMode() {
        if (stateMessage.initialised == true) {
            // Read from State Message
            return stateMessage.compressorMode;
        } else if (stateMessage2.initialised == true) {
            // Read from State 2 Message
            return stateMessage2.compressorMode;
        }
        return CompressorMode::Unknown;
    }

    bool Controller::isFanIdle() {
        if (stateMessage.initialised == true) {
            // Read from State Message
            return stateMessage.fanActive == false;;
        } else if (stateMessage2.initialised == true) {
            // Read from State 2 Message
            return stateMessage2.fanActive == false;;
        }
        return true;
    }

    /// Zone Control

    void Controller::setZoneOn(uint8_t zone, bool on) {
        if (!receivingData()) {
            return;
        }

        if (sendZoneStateCommand || _sendZoneStateCommandCleared == false) {
            // Preparing to send previous zone state command, adjust pending message
            nextZoneStateCommand.zoneOn[zindex(zone)] = on;
        } else {
            // No pending change, take a snap shot of the current state, set our zone to the desired state
            for (int i=0; i<8; i++) {
                nextZoneStateCommand.zoneOn[i] = (i == zindex(zone) ? on : getZoneOn(i+1));
            }
        }

        _sendZoneStateCommandCleared = false;
        sendZoneStateCommand = true;
    }

    bool Controller::getZoneOn(uint8_t zone) {
        if (stateMessage.initialised == true) {
            // Read from State Message
            return stateMessage.zoneOn[zindex(zone)];
        } else if (stateMessage2.initialised == true) {
            // Read from State 2 Message
            return stateMessage2.zoneOn[zindex(zone)];
        }
        return false;
    }

    void Controller::setZoneSetpointTemperatureCustom(uint8_t zone, double temperature, bool adjustMaster) {
        if (!receivingData()) {
            return;
        }

        if (zoneControlled[zindex(zone)] == true) {
            // Check if we need to adjust the master first
            if (adjustMaster) {
                double minAllowed = masterToZoneMessage[zindex(zone)].minSetpoint;
                double maxAllowed = masterToZoneMessage[zindex(zone)].maxSetpoint;
                double diff = 0;
                if (temperature<minAllowed) {
                    diff = minAllowed-temperature;
                } else if (temperature>maxAllowed) {
                    diff = maxAllowed-temperature;
                }
                // If the difference is not 0 adjust
                if (diff != 0) {
                    double newTemperature = getMasterSetpoint() - diff;
                    setMasterSetpoint(newTemperature);
                }
            }
            zoneSetpoint[zindex(zone)] = temperature;

        } else {
            // Send the custom zone setpoint message, the official 
            nextZoneSetpointCustomCommand.temperature = temperature;
            nextZoneSetpointCustomCommand.adjustMaster = adjustMaster;
            nextZoneSetpointCustomCommand.zone = zone;
            sendZoneSetpointCustomCommand = true;
        }
    }

    void Controller::setZoneSetpointTemperature(uint8_t zone, double temperature, bool adjustMaster) {
        if (!receivingData()) {
            return;
        }
        
        // An uncontrolled zone? Set the master set point as this zone follows it
        if (zoneMessage[zindex(zone)].type == ZoneMessageType::InitZone) {
            setMasterSetpoint(temperature);
            return;
        }

        // Check if we need to adjust the master first
        if (adjustMaster) {
            double minAllowed = masterToZoneMessage[zindex(zone)].minSetpoint;
            double maxAllowed = masterToZoneMessage[zindex(zone)].maxSetpoint;
            double diff = 0;
            if (temperature<minAllowed) {
                diff = minAllowed-temperature;
            } else if (temperature>maxAllowed) {
                diff = maxAllowed-temperature;
            }
            // If the difference is not 0 adjust
            if (diff != 0) {
                double newTemperature = getMasterSetpoint() - diff;
                setMasterSetpoint(newTemperature);
            }
        }

        if (zoneControlled[zindex(zone)] == true) {
            // We are directly controlling this
            zoneSetpoint[zindex(zone)] = temperature;
        } else {
            // Send the custom zone setpoint message, the official 
            nextMasterToZoneMessage[zindex(zone)] = masterToZoneMessage[zindex(zone)];
            nextMasterToZoneMessage[zindex(zone)].minSetpoint = temperature;
            nextMasterToZoneMessage[zindex(zone)].maxSetpoint = temperature;
            nextMasterToZoneMessage[zindex(zone)].setpoint = temperature;
            sendMasterToZoneMessage[zindex(zone)] = true;
        }
    }

    double Controller::getZoneSetpointTemperature(uint8_t zone) {
        // Phase 2: per-zone setpoints decoded from slave 11 reg 4-7 (one
        // byte per zone, byte = setpoint × 2). zoneSetpoint[] is populated
        // in applySlave11StateBroadcast.
        if (zone >= 1 && zone <= 8) {
            return zoneSetpoint[zindex(zone)];
        }
        return 0;
    }

    void Controller::setZoneCurrentTemperature(uint8_t zone, double temperature) {
        zoneTemperature[zindex(zone)] = temperature;
    }

    double Controller::getZoneCurrentTemperature(uint8_t zone) {
        // Phase 1 reads zone temps from the slave 11 broadcast directly into
        // zoneTemperature[]. Bypass the legacy stateMessage/ultimaState
        // resolution path which never gets populated by the Modbus decoder.
        if (zone >= 1 && zone <= 8) {
            return zoneTemperature[zindex(zone)];
        }
        return 0;
    }

    double Controller::getZoneDamperPosition(uint8_t zone) {
        // Phase 2: locate damper position register. Until then report
        // "open" so the UI doesn't claim every zone is closed.
        return 1.0;
    }

}
