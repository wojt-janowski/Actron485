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
            // Always update the register cache so /api/v1/bus has live
            // state regardless of logging mode. printRegisterDelta()
            // gates its own console output on printOutMode internally.
            captureFrameRegisters();
            if (printOut && (printOutMode == PrintOutMode::AllMessages
                           || printOutMode == PrintOutMode::CorrelationCapture)) {
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

            // Slave-responder mode: capture writes addressed to us into our
            // register buffer so the API/diagnostics can observe them. e.g.
            // when impersonating slave 3, the AMIB writes the compressor PWM
            // block (reg 21-28) to us — we don't act on it, just store it.
            if (_slaveResponderEnabled && slave == _slaveResponderId) {
                for (uint16_t i = 0; i < regCount; i++) {
                    uint16_t addr = startAddr + i;
                    if (addr >= _slaveRegistersCount) break;
                    _slaveRegisters[addr] =
                        (uint16_t(data[i * 2]) << 8) | data[i * 2 + 1];
                }
                // Per Modbus spec, function 0x10 expects an echo response:
                // [slave][0x10][addrHi][addrLo][cntHi][cntLo][crcLo][crcHi].
                // Without it the AMIB may retry / mark us offline.
                uint8_t resp[8];
                resp[0] = _slaveResponderId;
                resp[1] = 0x10;
                resp[2] = uint8_t(startAddr >> 8);
                resp[3] = uint8_t(startAddr & 0xFF);
                resp[4] = uint8_t(regCount >> 8);
                resp[5] = uint8_t(regCount & 0xFF);
                transmitModbusFrame(resp, 6);
                return;
            }

            if (slave == 11) {
                applySlave11StateBroadcast(startAddr, regCount, data, byteCount);
            }
            return;
        }

        // Function 0x06: Write Single Register Request (slave-responder only).
        // Layout: [slave][0x06][addrHi][addrLo][valHi][valLo][crcLo][crcHi].
        // Capture-and-echo, mirroring 0x10 behaviour above. We've never seen
        // the AMIB use 0x06 in normal operation but the BMS spec documents
        // it as supported, so handle it for completeness.
        if (function == 0x06 && length == 8) {
            if (_slaveResponderEnabled && slave == _slaveResponderId) {
                uint16_t addr = (uint16_t(frame[2]) << 8) | frame[3];
                uint16_t value = (uint16_t(frame[4]) << 8) | frame[5];
                if (addr < _slaveRegistersCount) {
                    _slaveRegisters[addr] = value;
                }
                // 0x06 echo is byte-identical to the request frame (sans CRC,
                // which we recompute).
                uint8_t resp[8];
                memcpy(resp, frame, 6);
                transmitModbusFrame(resp, 6);
                return;
            }
        }

        // Function 0x03: Read Holding Registers
        if (function == 0x03) {
            // Request: 8 bytes total
            if (length == 8) {
                // Slave-responder mode: AMIB is asking us for state. Build
                // and transmit response from our register buffer. Must happen
                // before the read-context bookkeeping below — that path is
                // for *us* tracking other slaves' responses, not for our own.
                if (_slaveResponderEnabled && slave == _slaveResponderId) {
                    uint16_t startAddr = (uint16_t(frame[2]) << 8) | frame[3];
                    uint16_t regCount = (uint16_t(frame[4]) << 8) | frame[5];
                    transmitSlaveReadResponse(startAddr, regCount);
                    return;
                }
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
        //
        // In slave-3 responder mode this would be a self-referential feedback
        // loop: we encode offset = (zoneTemperature - zoneSetpoint) × 10 into
        // reg 5-12 → AMIB rebroadcasts → we parse the same offset back into
        // zoneTemperature. Net no-op in steady state, but if zoneTemperature
        // is ever bogus (e.g. NVS load loaded a corrupted setpoint that the
        // encoder clamped to 127.5 °C — observed 2026-05-30), the loop
        // preserves the bogosity forever because the offset round-trips
        // unchanged. Skip the update in responder mode: the *only*
        // authoritative source for zoneTemperature is external sensor
        // pushes via /api/v1/zones/{n}/temperature (setZoneCurrentTemperature).
        if (!_slaveResponderEnabled) {
            for (int z = 0; z < 8; z++) {
                int8_t offset = (int8_t) data[12 + z];
                zoneTemperature[z] = zoneSetpoint[z] + (double(offset) * 0.1);
            }
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

    // In slave-responder mode, the setter `receivingData()` guards become
    // architecturally wrong — we ARE the bus's authoritative slave-3
    // responder. A transient ≥3s lull in master polls (e.g. cable glitch,
    // AMIB restart) shouldn't silently swallow a user's power-off or
    // setpoint command. Use this helper for setter preconditions instead
    // of `receivingData()` directly.
    bool Controller::allowControlCommand() {
        return _slaveResponderEnabled || receivingData();
    }

    // Setup

    void Controller::setControlZone(uint8_t zone, bool control) {
        zoneControlled[zindex(zone)] = control;
    }

    bool Controller::getControlZone(uint8_t zone) {
        return zoneControlled[zindex(zone)];
    }

    // Modbus slave-responder mode (Phase 3, 2026-05-01).
    //
    // When enabled, inbound Modbus requests addressed to `_slaveResponderId`
    // are answered from `_slaveRegisters`. Function 0x03 (read) builds and
    // transmits a response; 0x06/0x10 (write) update our buffer so we can
    // observe what the AMIB master writes to us (e.g. compressor PWM block
    // at slave 3 reg 21-28).
    //
    // INTENDED USE: physically disconnect the wall controller from the J6
    // DATA bus, then enable this with `slaveId = 3`. The AMIB will poll us
    // as if we were the wall controller; whatever state we expose drives
    // the AC. Leaving this disabled is safe — the responder makes no bus
    // transmissions and `_slaveRegisters` is just inert RAM.
    void Controller::setSlaveResponderMode(uint8_t slaveId, bool enabled) {
        _slaveResponderId = slaveId;
        _slaveResponderEnabled = enabled;
        // When stepping into slave-3 responder mode, prime the buffer so the
        // first AMIB poll (within ~6s) sees coherent state rather than zeros.
        // initSlave3Defaults() seeds the static / configuration regs; the
        // live-state regs get filled by renderSlave3State() reading from
        // stateMessage2 + zoneSetpoint[] + zoneTemperature[].
        if (enabled && slaveId == 3) {
            initSlave3Defaults();
            renderSlave3State();
        }
    }

    void Controller::setSlaveRegister(uint16_t address, uint16_t value) {
        if (address >= _slaveRegistersCount) {
            return;
        }
        _slaveRegisters[address] = value;
    }

    uint16_t Controller::getSlaveRegister(uint16_t address) {
        if (address >= _slaveRegistersCount) {
            return 0;
        }
        return _slaveRegisters[address];
    }

    // Build a Modbus 0x03 Read Holding Registers response for the requested
    // address range, sourcing values from our register buffer, and transmit
    // it on the bus. Modbus inter-frame timeout at 4800 baud is ~7 ms — the
    // ESP32 reaches `serialWrite(true)` and the first byte well within that.
    void Controller::transmitSlaveReadResponse(uint16_t startAddr, uint16_t regCount) {
        // Cap at 125 registers per Modbus spec, plus our buffer bounds.
        if (regCount == 0 || regCount > 125) {
            return;
        }
        if (uint32_t(startAddr) + regCount > _slaveRegistersCount) {
            return;
        }
        // Frame: [slave][0x03][byteCount][hi0 lo0 ... hiN loN][crcLo][crcHi]
        // Max payload: 125 * 2 = 250 bytes. Total max: 5 + 250 = 255 bytes.
        uint8_t byteCount = uint8_t(regCount * 2);
        uint8_t frameLen = 3 + byteCount;  // before CRC
        uint8_t frame[256];
        frame[0] = _slaveResponderId;
        frame[1] = 0x03;
        frame[2] = byteCount;
        for (uint16_t i = 0; i < regCount; i++) {
            uint16_t value = _slaveRegisters[startAddr + i];
            frame[3 + i * 2] = uint8_t(value >> 8);
            frame[3 + i * 2 + 1] = uint8_t(value & 0xFF);
            // Mirror the transmitted values into the (slave, address) cache
            // that /api/v1/bus reads. RS485 half-duplex means our own TX
            // doesn't loop back through processModbusFrame/captureFrameRegisters,
            // so without this the bus snapshot would be missing every slave-3
            // register the AMIB has read from us — exactly the data we'd
            // want when debugging the responder.
            updateRegisterCache(_slaveResponderId, startAddr + i, value);
        }
        transmitModbusFrame(frame, frameLen);
    }

    // Static slave-3 register values the AMIB expects to find. Decoded from
    // captures (see PROTOCOL_NOTES.md — "Slave 3" sections and "Phase 3A"
    // address-space inventory). regs 0-1 are constants the wall LCD always
    // returns; regs 30-33 are static cool/heat min/max bounds in tenths °C;
    // reg 100 is a system-alive sentinel polled at low cadence.
    void Controller::initSlave3Defaults() {
        _slaveRegisters[0]   = 0x00B4;
        _slaveRegisters[1]   = 0x0C72;
        _slaveRegisters[30]  = 0x05DC;  // 1500 → 15.0 °C
        _slaveRegisters[31]  = 0x050A;  // 1290 → 12.9 °C
        _slaveRegisters[32]  = 0x047E;  // 1150 → 11.5 °C
        _slaveRegisters[33]  = 0x0A00;  // 2560 → 25.6 °C
        _slaveRegisters[100] = 0x00FF;

        // Seed zoneSetpoint to a safe room-temp default so the first AMIB
        // poll doesn't see "every zone at 0°C with 0 offset" before any
        // HTTP write lands. NaN for zoneTemperature signals "no reading"
        // — encodeSlave3ZoneRegister maps NaN to a zero offset byte, which
        // the AMIB treats as at-setpoint (no demand). zoneTemperature
        // defaults to 0.0 from in-class init; set NaN here explicitly.
        for (int z = 0; z < 8; z++) {
            if (zoneSetpoint[z] == 0.0) {
                zoneSetpoint[z] = 22.0;
            }
            if (zoneTemperature[z] == 0.0) {
                zoneTemperature[z] = std::nan("");
            }
        }

        // Seed stateMessage2 to match what renderSlave3State() will write to
        // the wire. Without this, getFanSpeed() returns FanMode::Off (the
        // in-class default) until the user explicitly POSTs to /api/v1/fan,
        // even though we're already serving reg 3 lo = 0x00 = Esp/Auto on
        // the bus — so the API would lie about the AC's actual behaviour.
        //
        // Only seed when stateMessage2 hasn't already been populated — e.g.
        // by a persistence-layer restore from NVS that ran before
        // setSlaveResponderMode (see Actron485Climate::load_slave3_state_).
        if (!stateMessage2.initialised) {
            stateMessage2.fanMode = FanMode::Esp;
            stateMessage2.runningFanMode = FanMode::Off;
            stateMessage2.initialised = true;
        }
    }

    // Pack one zone's setpoint and observed temperature into the wire format
    // used by slave-3 regs 5-12: high byte = setpoint × 2 (0.5 °C step), low
    // byte = signed int8 tenths-of-°C offset (current − setpoint). When the
    // current temp is missing (NaN), we report a zero offset so the AMIB
    // treats the zone as at-setpoint — better than feeding it -12.8 °C of
    // spurious demand that would force the system to call for heat/cool.
    uint16_t Controller::encodeSlave3ZoneRegister(double setpoint, double currentTemperature) {
        // Clamp setpoint to the bounds expected by the AMIB (regs 30-33).
        // 0.0-127.5 fits in an unsigned byte once × 2; the legal user range
        // is much narrower (~12-30 °C) but the encoding is the constraint.
        if (setpoint < 0.0) setpoint = 0.0;
        if (setpoint > 127.5) setpoint = 127.5;
        uint8_t setpointByte = uint8_t(setpoint * 2.0 + 0.5);

        int8_t offsetByte = 0;
        if (std::isfinite(currentTemperature) && std::isfinite(setpoint)) {
            double tenths = (currentTemperature - setpoint) * 10.0;
            if (tenths > 127.0) tenths = 127.0;
            if (tenths < -128.0) tenths = -128.0;
            offsetByte = int8_t(tenths >= 0 ? tenths + 0.5 : tenths - 0.5);
        }
        return (uint16_t(setpointByte) << 8) | uint8_t(offsetByte);
    }

    // Re-encode the entire live-state region of the slave-3 buffer (regs 2-12)
    // from the controller's typed state. Cheap enough to call from every
    // setter — eight zone regs plus three header regs = ~40 lines of byte
    // arithmetic. No-op when slave-responder mode is disabled, so callers
    // can invoke it unconditionally.
    void Controller::renderSlave3State() {
        if (!_slaveResponderEnabled || _slaveResponderId != 3) {
            return;
        }

        // Mirror the read-side decode in applySlave3ReadResponse(): mode lives
        // in reg 2 high byte. We only express user-facing state here (the
        // compressor sub-state in the low byte is set by the AMIB/AC head
        // downstream — we report "idle" values so we don't lie about the
        // hardware). Quiet bit (0x80) is OR'd in when stateMessage2 says so.
        uint8_t reg2Hi = 0x00;
        uint8_t reg2Lo = 0x00;
        OperatingMode mode = stateMessage2.initialised ? stateMessage2.operatingMode : OperatingMode::Off;
        switch (mode) {
            case OperatingMode::Heat:
                reg2Hi = 0x01;
                break;
            case OperatingMode::Cool:
                reg2Hi = 0x02;
                reg2Lo = 0x00;
                break;
            case OperatingMode::Auto:
                reg2Hi = 0x02;
                reg2Lo = 0x23;
                break;
            case OperatingMode::FanOnly:
                reg2Hi = 0x08;
                break;
            case OperatingMode::Off:
            default:
                reg2Hi = 0x00;
                reg2Lo = 0x00;
                break;
        }
        if (stateMessage2.quietMode) {
            reg2Hi |= 0x80;
        }
        _slaveRegisters[2] = (uint16_t(reg2Hi) << 8) | reg2Lo;

        // Reg 3: high byte = status flags, low byte = fan speed.
        //
        // Decoded from LCD probes on 2026-05-30 (PROTOCOL_NOTES.md "Reg 3 hi
        // status flags"):
        //   bit 0 = system armed (set whenever any zone is enabled)
        //   bit 1 = fan running (set transiently when fan is spinning)
        //   bit 2 = continuous fan flag
        //   bit 3 = unknown — set in Heat-armed states regardless of fan
        //           speed or continuous-fan setting; safe to leave 0 from us.
        // The earlier hypothesis that the high byte is a zone bitmap was
        // wrong: zones live in reg 4 hi (see below).
        uint8_t reg3Hi = 0x00;
        bool anyZoneOn = false;
        for (int z = 0; z < 8; z++) {
            if (stateMessage2.zoneOn[z]) { anyZoneOn = true; break; }
        }
        if (anyZoneOn)                  reg3Hi |= 0x01;  // system armed
        if (stateMessage2.continuousFan) reg3Hi |= 0x04;  // continuous fan
        uint8_t reg3Lo;
        switch (stateMessage2.fanMode) {
            case FanMode::Low:
            case FanMode::LowContinuous:       reg3Lo = 0x28; break;
            case FanMode::Medium:
            case FanMode::MediumContinuous:    reg3Lo = 0x3D; break;
            case FanMode::High:
            case FanMode::HighContinuous:      reg3Lo = 0x59; break;
            case FanMode::Esp:
            case FanMode::EspContinuous:
            default:                           reg3Lo = 0x00; break;
        }
        _slaveRegisters[3] = (uint16_t(reg3Hi) << 8) | reg3Lo;

        // Reg 4: high byte = zone-enable bitmap (zone N → bit N-1); low byte
        // = constant 0x23. The zone bitmap doubles as the system-on signal —
        // when zoneBitmap == 0x00 the AMIB treats the AC as off, no matter
        // what reg 2 hi says. The earlier "bit 1 = system on" reading was a
        // misinterpretation: bit 1 just happened to be zone 2, which the
        // AMIB defaulted to whenever we asked for "system on" without
        // specifying a zone. Decoded from LCD probes on 2026-05-30.
        uint8_t zoneBitmap = 0x00;
        for (int z = 0; z < 8; z++) {
            if (stateMessage2.zoneOn[z]) zoneBitmap |= uint8_t(1 << z);
        }
        _slaveRegisters[4] = (uint16_t(zoneBitmap) << 8) | 0x23;

        // Regs 5-12: one per zone. Setpoint comes from zoneSetpoint[]
        // (driven by setZoneSetpointTemperatureCustom + setMasterSetpoint),
        // current temperature from zoneTemperature[] (driven by remote-sensor
        // POSTs via setZoneCurrentTemperature).
        for (int z = 0; z < 8; z++) {
            _slaveRegisters[5 + z] = encodeSlave3ZoneRegister(
                zoneSetpoint[z], zoneTemperature[z]);
        }
    }

    // Append CRC-16 (Modbus, poly 0xA001) to `frame` and transmit. Caller
    // sized `frame` to length+2.
    //
    // dataLastSentTime is intentionally NOT bumped here. That timestamp
    // tracks "user command was just transmitted" — used by climate-entity
    // update_status() to debounce state republishes after a command. In
    // slave-responder mode every AMIB poll triggers a response from us, so
    // bumping it would keep the climate entity perpetually inside its
    // debounce window and the ESPHome web dashboard would never refresh.
    // Bus-driven passive responses aren't user commands.
    void Controller::transmitModbusFrame(uint8_t *frame, uint8_t length) {
        if (_serial == nullptr) {
            return;
        }
        uint16_t crc = checksumModbus(frame, length);
        frame[length] = uint8_t(crc & 0xFF);
        frame[length + 1] = uint8_t(crc >> 8);
        uint8_t total = uint8_t(length + 2);

        serialWrite(true);
        _serial->write(frame, total);
        _serial->flush();
        serialWrite(false);
    }

    // System Control

    void Controller::setSystemOn(bool on) {
        if (!allowControlCommand()) {
            return;
        }

        OperatingMode currentMode = getOperatingMode();
        OperatingMode nextMode = currentMode;

        if (on) {
            switch (currentMode) {
                case OperatingMode::Off:     nextMode = OperatingMode::FanOnly; break;
                case OperatingMode::OffAuto: nextMode = OperatingMode::Auto;    break;
                case OperatingMode::OffHeat: nextMode = OperatingMode::Heat;    break;
                case OperatingMode::OffCool: nextMode = OperatingMode::Cool;    break;
                default: break;  // already on
            }
        } else {
            switch (currentMode) {
                case OperatingMode::FanOnly: nextMode = OperatingMode::Off;     break;
                case OperatingMode::Auto:    nextMode = OperatingMode::OffAuto; break;
                case OperatingMode::Heat:    nextMode = OperatingMode::OffHeat; break;
                case OperatingMode::Cool:    nextMode = OperatingMode::OffCool; break;
                default: break;  // already off
            }
        }

        if (nextMode == currentMode) {
            return;
        }
        nextOperatingModeCommand.mode = nextMode;
        sendOperatingModeCommand = true;

        // Slave-3 responder: we are the source of truth. The legacy queue
        // above is drained as a no-op (attemptToSendQueuedCommand), so we
        // must apply the state ourselves and re-render the registers the
        // AMIB will poll. Setting stateMessage2 directly also makes the
        // HTTP API return the new state immediately rather than after the
        // next ~6s poll cycle bounces our own response back to us.
        if (_slaveResponderEnabled && _slaveResponderId == 3) {
            stateMessage2.initialised = true;
            // Collapse the Off* variants to plain Off for the renderer.
            bool goingActive = (nextMode != OperatingMode::Off
                && nextMode != OperatingMode::OffAuto
                && nextMode != OperatingMode::OffCool
                && nextMode != OperatingMode::OffHeat);
            stateMessage2.operatingMode = goingActive ? nextMode : OperatingMode::Off;

            // When transitioning to an active mode, the AMIB needs a
            // non-zero zone bitmap in reg 4 hi or it treats the AC as off.
            // If the user hasn't selected any zone, enable the first one we
            // own (via setControlZone), or fall back to zone 1 — mirrors the
            // wall LCD's behaviour of keeping at least one zone armed
            // whenever the system is running.
            if (goingActive) {
                bool anyZone = false;
                for (int z = 0; z < 8; z++) {
                    if (stateMessage2.zoneOn[z]) { anyZone = true; break; }
                }
                if (!anyZone) {
                    int fallback = -1;
                    for (int z = 0; z < 8; z++) {
                        if (zoneControlled[z]) { fallback = z; break; }
                    }
                    if (fallback < 0) fallback = 0;  // zone 1
                    stateMessage2.zoneOn[fallback] = true;
                }
            } else {
                // Power-off: clear the zone bitmap so reg 4 hi goes to 0.
                // Leaving zoneOn[] set would publish "Off mode with zones
                // armed" — an undocumented state the AMIB has never been
                // observed in, and which gets persisted to NVS surviving
                // reboots. Restore the zones on the next active-mode call
                // via the fallback above.
                for (int z = 0; z < 8; z++) {
                    stateMessage2.zoneOn[z] = false;
                }
            }
            renderSlave3State();
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
        if (!allowControlCommand()) {
            return;
        }

        bool continuous = getContinuousFanMode();
        FanMode resolved = fanSpeed;
        switch (fanSpeed) {
            case FanMode::Low:    resolved = continuous ? FanMode::LowContinuous    : fanSpeed; break;
            case FanMode::Medium: resolved = continuous ? FanMode::MediumContinuous : fanSpeed; break;
            case FanMode::High:   resolved = continuous ? FanMode::HighContinuous   : fanSpeed; break;
            case FanMode::Esp:    resolved = continuous ? FanMode::EspContinuous    : fanSpeed; break;
            default: return;  // Off / *Continuous from caller: ignore (use setFanSpeedAbsolute)
        }

        nextFanModeCommand.fanMode = resolved;
        sendFanModeCommand = true;

        if (_slaveResponderEnabled && _slaveResponderId == 3) {
            stateMessage2.initialised = true;
            stateMessage2.fanMode = resolved;
            renderSlave3State();
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
        if (!allowControlCommand()) {
            return;
        }

        nextFanModeCommand.fanMode = fanSpeed;
        sendFanModeCommand = true;

        if (_slaveResponderEnabled && _slaveResponderId == 3) {
            stateMessage2.initialised = true;
            stateMessage2.fanMode = fanSpeed;
            stateMessage2.continuousFan = (fanSpeed == FanMode::LowContinuous
                || fanSpeed == FanMode::MediumContinuous
                || fanSpeed == FanMode::HighContinuous
                || fanSpeed == FanMode::EspContinuous);
            renderSlave3State();
        }
    }

    void Controller::setContinuousFanMode(bool on) {
        if (!allowControlCommand()) {
            return;
        }

        FanMode fanSpeed = getFanSpeed();
        FanMode resolved = fanSpeed;
        switch (fanSpeed) {
            case FanMode::Low:    resolved = on ? FanMode::LowContinuous    : fanSpeed; break;
            case FanMode::Medium: resolved = on ? FanMode::MediumContinuous : fanSpeed; break;
            case FanMode::High:   resolved = on ? FanMode::HighContinuous   : fanSpeed; break;
            case FanMode::Esp:    resolved = on ? FanMode::EspContinuous    : fanSpeed; break;
            default: return;
        }
        nextFanModeCommand.fanMode = resolved;
        sendFanModeCommand = true;

        if (_slaveResponderEnabled && _slaveResponderId == 3) {
            stateMessage2.initialised = true;
            stateMessage2.fanMode = resolved;
            stateMessage2.continuousFan = on;
            renderSlave3State();
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

    void Controller::setQuietMode(bool on) {
        if (!allowControlCommand()) {
            return;
        }
        // Legacy bus has no documented command path for quiet mode, so the
        // setter is only meaningful in slave-3 responder mode. Outside it
        // we still update stateMessage2 so the API/HA round-trip works for
        // testing in demo mode, but no register is touched.
        if (_slaveResponderEnabled && _slaveResponderId == 3) {
            stateMessage2.initialised = true;
            stateMessage2.quietMode = on;
            renderSlave3State();
        } else {
            stateMessage2.quietMode = on;
        }
    }

    void Controller::setOperatingMode(OperatingMode mode) {
        if (!allowControlCommand()) {
            return;
        }

        if (mode == OperatingMode::Off) {
            setSystemOn(false);
            return;
        }

        nextOperatingModeCommand.mode = mode;
        sendOperatingModeCommand = true;

        if (_slaveResponderEnabled && _slaveResponderId == 3) {
            stateMessage2.initialised = true;
            stateMessage2.operatingMode = mode;
            // Going active with no zones enabled would write a zero zone
            // bitmap to reg 4 hi, which the AMIB treats as "system off".
            // Auto-enable a controlled zone (or zone 1) to keep the request
            // meaningful. See setSystemOn() for the same logic.
            bool anyZone = false;
            for (int z = 0; z < 8; z++) {
                if (stateMessage2.zoneOn[z]) { anyZone = true; break; }
            }
            if (!anyZone) {
                int fallback = -1;
                for (int z = 0; z < 8; z++) {
                    if (zoneControlled[z]) { fallback = z; break; }
                }
                if (fallback < 0) fallback = 0;
                stateMessage2.zoneOn[fallback] = true;
            }
            renderSlave3State();
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
        if (!allowControlCommand()) {
            return;
        }

        nextSetpointCommand.temperature = temperature;
        sendSetpointCommand = true;

        // Slave-3 responder: master setpoint = control-zone's setpoint
        // (PROTOCOL_NOTES.md line ~222). Any zone marked controlled by this
        // module receives the new setpoint; in the typical single-zone rig
        // only one is set, but multi-zone configs cascade as expected.
        if (_slaveResponderEnabled && _slaveResponderId == 3) {
            for (int z = 0; z < 8; z++) {
                if (zoneControlled[z]) {
                    zoneSetpoint[z] = temperature;
                }
            }
            stateMessage2.initialised = true;
            stateMessage2.setpoint = temperature;
            renderSlave3State();
        }
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
        if (!allowControlCommand()) {
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

        if (_slaveResponderEnabled && _slaveResponderId == 3) {
            stateMessage2.initialised = true;
            stateMessage2.zoneOn[zindex(zone)] = on;
            renderSlave3State();
        }
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
        if (!allowControlCommand()) {
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

            // Slave-3 responder: zone is "ours" by virtue of us being the
            // wall controller, so just write the setpoint directly. ICAMIB
            // documents per-zone setpoints as read-only over its BMS port
            // — that's a gateway constraint, not a bus one; the underlying
            // slave-3 register is plain readable state we author.
            if (_slaveResponderEnabled && _slaveResponderId == 3) {
                zoneSetpoint[zindex(zone)] = temperature;
            }
        }

        if (_slaveResponderEnabled && _slaveResponderId == 3) {
            // Keep stateMessage2.setpoint in sync with the control zone's
            // setpoint so the ESPHome climate entity and /api/v1/state.setpoint
            // don't lag the wire value. Master setpoint IS the control
            // zone's setpoint by design — see PROTOCOL_NOTES.md.
            if (zoneControlled[zindex(zone)]) {
                stateMessage2.initialised = true;
                stateMessage2.setpoint = temperature;
            }
            renderSlave3State();
        }
    }

    void Controller::setZoneSetpointTemperature(uint8_t zone, double temperature, bool adjustMaster) {
        if (!allowControlCommand()) {
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

        // Slave-3 responder: this is THE input that drives the AMIB's
        // per-zone-temperature awareness. Re-encode the affected reg
        // (reg 5 + zindex(zone)) so the next AMIB poll sees the new offset
        // byte. encodeSlave3ZoneRegister handles NaN by reporting zero
        // offset — the AMIB treats that as at-setpoint.
        if (_slaveResponderEnabled && _slaveResponderId == 3
            && zone >= 1 && zone <= 8) {
            _slaveRegisters[5 + zindex(zone)] = encodeSlave3ZoneRegister(
                zoneSetpoint[zindex(zone)], temperature);
        }
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
