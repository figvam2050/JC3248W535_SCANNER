#include "rs485_sniffer.h"
#include "sd_logger.h"

RS485Sniffer Sniffer;

const uint32_t RS485Sniffer::_baudRates[] = {9600, 19200, 115200, 2400};
const size_t RS485Sniffer::_baudRatesCount = sizeof(RS485Sniffer::_baudRates) / sizeof(RS485Sniffer::_baudRates[0]);

uint16_t RS485Sniffer::calculateCRC(const uint8_t* buf, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void RS485Sniffer::setBaudRate(uint32_t baud) {
    if (!_port) return;
    _port->end();
    delay(10);
    _port->begin(baud, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
    _rxLen = 0;
    _lastBaudSwitchMs = millis();
    Serial.printf("[RS485] Baud rate set to %u bps\n", baud);
}

void RS485Sniffer::nextBaudRate() {
    _baudIdx = (_baudIdx + 1) % _baudRatesCount;
    setBaudRate(_baudRates[_baudIdx]);
}

void RS485Sniffer::sendBatteryResponse(uint8_t slaveAddr, uint8_t fc, uint16_t startReg, uint16_t regCount) {
    if (!_port || regCount == 0 || regCount > 64) return;

    // Standard Modbus RTU Response:
    // [Addr] [FC] [ByteCount] [Reg0_Hi] [Reg0_Lo] ... [CRCLo] [CRCHi]
    uint8_t byteCount = regCount * 2;
    uint8_t txBuf[140];
    txBuf[0] = slaveAddr;
    txBuf[1] = fc;
    txBuf[2] = byteCount;

    // Fill simulated 15S battery data:
    // Reg 0: Voltage 52.50V (5250)
    // Reg 1: Current 0.0A (0)
    // Reg 2..16: 15 Cells voltages (3500 mV each)
    // Reg 18: Temp PCB 25C
    // Reg 19: Temp Avg 24C
    // Reg 21: Capacity Remaining 100 Ah
    // Reg 24: SOC 85%
    // Reg 25: Status 0x0001 (Charging / Normal)
    for (uint16_t r = 0; r < regCount; r++) {
        uint16_t regIdx = startReg + r;
        uint16_t val = 0;

        if (regIdx == 0) val = 5250;              // 52.50 V (0.01V)
        else if (regIdx == 1) val = 0;            // 0.0 A
        else if (regIdx >= 2 && regIdx <= 16) val = 3500; // 3.500 V per cell
        else if (regIdx == 18) val = 25;          // 25 °C PCB
        else if (regIdx == 19) val = 24;          // 24 °C Avg
        else if (regIdx == 21) val = 100;         // 100 Ah
        else if (regIdx == 23) val = 100;         // 100% SOH
        else if (regIdx == 24) val = 85;          // 85% SOC
        else if (regIdx == 25) val = 0x0001;      // Status Normal
        // Sofar Solar 0x0200 block support
        else if (regIdx == 0x020C) val = 5250;    // Bat Voltage 52.50V
        else if (regIdx == 0x020D) val = 0;       // Bat Current 0.0A
        else if (regIdx == 0x020F) val = 85;      // Bat SOC 85%
        else if (regIdx == 0x0210) val = 25;      // Bat Temp 25C
        // Felicity / SRNE / Growatt support
        else if (regIdx >= 0x1000 && regIdx <= 0x100F) val = 3500;
        else if (regIdx >= 0x3000 && regIdx <= 0x3010) val = 525; // 0.1V scale for Solis
        else if (regIdx >= 840 && regIdx <= 843) val = (regIdx == 843) ? 850 : 525; // Victron 0.1 scale
        else val = 0;

        txBuf[3 + r * 2] = (val >> 8) & 0xFF;
        txBuf[3 + r * 2 + 1] = val & 0xFF;
    }

    size_t payloadLen = 3 + byteCount;
    uint16_t crc = calculateCRC(txBuf, payloadLen);
    txBuf[payloadLen] = crc & 0xFF;
    txBuf[payloadLen + 1] = (crc >> 8) & 0xFF;
    size_t totalTxLen = payloadLen + 2;

    // Small delay to simulate BMS processing time (2-5 ms)
    delay(3);
    _port->write(txBuf, totalTxLen);
    _port->flush();

    inverterInfo.totalResponsesSent++;

    // Record our TX response in SD raw log
    Logger.logRawPacket(_baudRates[_baudIdx], txBuf, totalTxLen, true);
    Serial.printf("[EMULATOR TX] Sent %u bytes battery response to inverter Addr %02X\n", (unsigned int)totalTxLen, slaveAddr);
}

void RS485Sniffer::begin(HardwareSerial* serialPort) {
    _port = serialPort;
    _baudIdx = 0;
    _lastBaudSwitchMs = millis();
    // RX on GPIO 44, TX on GPIO 43 for battery response emulation
    _port->begin(_baudRates[_baudIdx], SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
    _rxLen = 0;
    _rxPackets = 0;
    _crcErrors = 0;
    _rawBytesCount = 0;
    inverterInfo.state = SCANNER_STATE_SEARCHING;
}

void RS485Sniffer::parseFrame(const uint8_t* frame, size_t len, BatteryPack* packs, SemaphoreHandle_t mutex) {
    if (len < 5) return;
    uint8_t addr = frame[0];
    uint8_t fc = frame[1];

    // Check CRC
    uint16_t expectedCrc = (frame[len - 1] << 8) | frame[len - 2];
    bool crcOk = (calculateCRC(frame, len - 2) == expectedCrc);

    // Record raw frame to SD card logger
    Logger.logRawPacket(_baudRates[_baudIdx], frame, len, crcOk);

    if (!crcOk) {
        _crcErrors++;
        return;
    }

    _rxPackets++;

    // --- CASE A: Master Query from INVERTER (Standard 8-byte Modbus RTU) ---
    // [Addr] [FC: 0x03 or 0x04] [StartRegHi] [StartRegLo] [RegCountHi] [RegCountLo] [CRCLo] [CRCHi]
    if (len == 8 && (fc == 0x03 || fc == 0x04)) {
        uint16_t startReg = (frame[2] << 8) | frame[3];
        uint16_t regCount = (frame[4] << 8) | frame[5];

        uint32_t now = millis();
        if (inverterInfo.lastSeenMs > 0) {
            inverterInfo.pollIntervalMs = now - inverterInfo.lastSeenMs;
        }
        inverterInfo.isDetected = true;
        inverterInfo.lastSeenMs = now;
        inverterInfo.totalQueries++;
        inverterInfo.baudRate = _baudRates[_baudIdx];
        inverterInfo.targetSlaveAddr = addr;
        inverterInfo.functionCode = fc;
        inverterInfo.startRegister = startReg;
        inverterInfo.registerCount = regCount;

        // Auto signature identification across Top 15 Solar Inverter Brands
        if (addr == 0x01 && (startReg == 0x0000 || startReg == 0x0100)) {
            inverterInfo.detectedSignature = "Deye / Luxpower / Pylontech";
        } else if (addr == 0x01 && (startReg >= 0x1000 && startReg <= 0x1100)) {
            inverterInfo.detectedSignature = "Felicity Solar Modbus";
        } else if (addr == 0x01 && (startReg >= 0x0100 && startReg <= 0x0140)) {
            inverterInfo.detectedSignature = "SRNE / PowMr / Anenji";
        } else if (addr == 0x01 && (startReg >= 0x0020 && startReg <= 0x0060)) {
            inverterInfo.detectedSignature = "Growatt Lithium (L01/L51)";
        } else if (startReg >= 0x3000 && startReg <= 0x3500) {
            inverterInfo.detectedSignature = "Solis (Ginlong RHI/S5/S6)";
        } else if (startReg >= 0x0200 && startReg <= 0x0250) {
            inverterInfo.detectedSignature = "Sofar Solar (HYD / ME3000)";
        } else if (startReg >= 0x6200 && startReg <= 0x6300) {
            inverterInfo.detectedSignature = "Must Solar (PH1800/PV1800)";
        } else if (addr == 0xF7 || addr == 247) {
            inverterInfo.detectedSignature = "GoodWe ES/EM / SRNE Broadcast";
        } else if (addr == 0xD6 || addr == 0xD7) {
            inverterInfo.detectedSignature = "Huawei ESM Industrial";
        } else if (addr == 100 || (startReg >= 840 && startReg <= 860)) {
            inverterInfo.detectedSignature = "Victron Energy Venus/GX";
        } else if (addr >= 0x01 && addr <= 0x04) {
            inverterInfo.detectedSignature = "Voltronic / Easun / Axioma";
        } else {
            inverterInfo.detectedSignature = "Standard Modbus RTU Inverter";
        }

        // Format HEX for live UI display
        snprintf(inverterInfo.lastHex, sizeof(inverterInfo.lastHex),
            "%02X %02X %02X %02X %02X %02X %02X %02X",
            frame[0], frame[1], frame[2], frame[3], frame[4], frame[5], frame[6], frame[7]
        );

        if (demoMode) {
            demoMode = false;
            Serial.printf("[INVERTER DETECTED] %s at %u bps! Addr: %02X, Regs: 0x%04X..0x%04X\n",
                inverterInfo.detectedSignature.c_str(), _baudRates[_baudIdx], addr, startReg, startReg + regCount - 1);
        }

        // State Machine Transition: SEARCHING -> TESTING
        if (inverterInfo.state == SCANNER_STATE_SEARCHING) {
            inverterInfo.state = SCANNER_STATE_TESTING;
            inverterInfo.testStartMs = millis();
        }

        // Check if inverter continues polling after our response => MATCH SUCCESS!
        if (inverterInfo.totalResponsesSent > 0) {
            inverterInfo.followUpQueriesCount++;
            if (inverterInfo.followUpQueriesCount >= 2) {
                inverterInfo.handShakeSuccess = true;
            }
        }

        // Send battery response emulation if enabled
        if (enableEmulation && inverterInfo.state == SCANNER_STATE_TESTING) {
            sendBatteryResponse(addr, fc, startReg, regCount);
        }
        return;
    }

    // --- CASE B: Slave Response (e.g. from real BMS or battery emulator) ---
    uint8_t byteCount = frame[2];
    if (fc != 0x03 || addr < 1 || addr > MAX_BATTERIES) return;
    if (len != (size_t)(byteCount + 5)) return;

    // We have real verified RS485 traffic: disable demo mode and lock baud rate!
    if (demoMode) {
        demoMode = false;
        Serial.printf("[RS485] Valid Modbus traffic detected at %u bps! Demo mode disabled.\n", _baudRates[_baudIdx]);
    }

    // Parse registers
    size_t regCount = byteCount / 2;
    const uint8_t* data = &frame[3];
    uint16_t regs[130];
    if (regCount > 130) regCount = 130;
    for (size_t i = 0; i < regCount; i++) {
        regs[i] = (data[i * 2] << 8) | data[i * 2 + 1];
    }

    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        BatteryPack& pack = packs[addr - 1];
        pack.isOnline = true;
        pack.lastSeenMs = millis();

        if (regCount > 0) pack.voltage = regs[0] / 100.0f;
        if (regCount > 1) pack.current = (int16_t)regs[1] / 100.0f;

        uint16_t minMv = 5000, maxMv = 0;
        for (int i = 0; i < CELLS_PER_BATTERY; i++) {
            if (i + 2 < (int)regCount) {
                pack.cells[i] = regs[2 + i];
                if (pack.cells[i] > 1000 && pack.cells[i] < 4500) {
                    if (pack.cells[i] < minMv) minMv = pack.cells[i];
                    if (pack.cells[i] > maxMv) maxMv = pack.cells[i];
                }
            }
        }
        pack.cellMin = (minMv == 5000) ? 0 : minMv;
        pack.cellMax = maxMv;
        pack.cellDiff = (pack.cellMax > pack.cellMin) ? (pack.cellMax - pack.cellMin) : 0;

        if (regCount > 18) pack.tempPcb = (int16_t)regs[18];
        if (regCount > 19) pack.tempAvg = (int16_t)regs[19];
        if (regCount > 20) pack.tempMax = (int16_t)regs[20];
        if (regCount > 21) pack.capacityRemaining = regs[21];
        if (regCount > 23) pack.soh = regs[23];
        if (regCount > 24) pack.soc = regs[24];

        if (regCount > 25) {
            pack.statusRaw = regs[25];
            uint8_t low = pack.statusRaw & 0xFF;
            if (low == 0x01) pack.statusName = "Charging";
            else if (low == 0x02) pack.statusName = "Discharging";
            else if (low == 0x04) pack.statusName = "Protect";
            else if (low == 0x08) pack.statusName = "Chg Limit";
            else pack.statusName = "Standby";
            if (pack.statusRaw & 0x8000) pack.statusName += " + Heat";
        }

        if (regCount > 26) pack.warningRaw = regs[26];
        if (regCount > 27) pack.protectionRaw = regs[27];
        if (regCount > 28) pack.errorRaw = regs[28];
        if (regCount > 30) pack.cycles = ((uint32_t)regs[29] << 16) | regs[30];
        if (regCount > 38) pack.balanceMask = regs[38];

        xSemaphoreGive(mutex);
    }
}

void RS485Sniffer::update(BatteryPack* packs, SemaphoreHandle_t mutex) {
    if (!_port) return;

    // 1. Read all incoming bytes from UART
    size_t newBytes = 0;
    while (_port->available()) {
        uint8_t b = _port->read();
        _rawBytesCount++;
        newBytes++;
        _lastByteTimeMs = millis();

        if (_rxLen < sizeof(_rxBuf)) {
            _rxBuf[_rxLen++] = b;
        } else {
            _rxLen = 0; // Buffer overflow protection
        }
    }

    // 2. Modbus inter-frame silence gap (t3.5 timeout)
    // If line has been silent > 20ms and frame is incomplete, reset buffer
    if (_rxLen > 0 && (millis() - _lastByteTimeMs > 25)) {
        // If data looks like noise or partial broken frame, log if enabled and clear
        if (_rxLen < 5) {
            _rxLen = 0;
        }
    }

    // 3. Process Modbus frame if complete
    if (_rxLen >= 5) {
        size_t i = 0;
        while (i + 4 < _rxLen) {
            uint8_t addr = _rxBuf[i];
            uint8_t fc = _rxBuf[i + 1];

            // Check A: Standard 8-byte Inverter Modbus Request (FC03 or FC04)
            if ((fc == 0x03 || fc == 0x04) && (i + 8 <= _rxLen)) {
                uint16_t expectedCrc = (_rxBuf[i + 7] << 8) | _rxBuf[i + 6];
                if (calculateCRC(&_rxBuf[i], 6) == expectedCrc) {
                    parseFrame(&_rxBuf[i], 8, packs, mutex);
                    i += 8;
                    continue;
                }
            }

            // Check B: Slave Response with byteCount (FC03)
            uint8_t byteCount = _rxBuf[i + 2];
            if (fc == 0x03 && addr >= 1 && addr <= MAX_BATTERIES && byteCount >= 10 && byteCount <= 120 && (byteCount % 2 == 0)) {
                size_t frameLen = byteCount + 5;
                if (i + frameLen <= _rxLen) {
                    parseFrame(&_rxBuf[i], frameLen, packs, mutex);
                    i += frameLen;
                    continue;
                } else {
                    break; // Frame is still receiving
                }
            }
            i++;
        }
        if (i > 0) {
            memmove(_rxBuf, &_rxBuf[i], _rxLen - i);
            _rxLen -= i;
        }
    }

    // 4. Auto-scan baud rate if no valid packets received within 3.5 seconds
    if (_autoScan && demoMode) {
        if (millis() - _lastBaudSwitchMs > 3500) {
            nextBaudRate();
        }
    }

    // 5. Test Window Timer (State Machine TESTING -> FINISHED)
    if (inverterInfo.state == SCANNER_STATE_TESTING) {
        if (millis() - inverterInfo.testStartMs >= (inverterInfo.testDurationSec * 1000UL)) {
            inverterInfo.state = SCANNER_STATE_FINISHED;
            Serial.println("[SCANNER] Test window complete! Status: FINISHED.");
        }
    }

    // 6. If demoMode is still active, run simulation for screen preview
    if (demoMode) {
        updateDemo(packs, mutex);
        return;
    }

    // Check timeouts for offline state (> 6 sec silence)
    uint32_t now = millis();
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        for (int i = 0; i < MAX_BATTERIES; i++) {
            if (packs[i].isOnline && (now - packs[i].lastSeenMs > 6000)) {
                packs[i].isOnline = false;
                packs[i].statusName = "Offline";
            }
        }
        xSemaphoreGive(mutex);
    }
}

void RS485Sniffer::updateDemo(BatteryPack* packs, SemaphoreHandle_t mutex) {
    static uint32_t lastDemoTick = 0;
    if (millis() - lastDemoTick < 1000) return;
    lastDemoTick = millis();

    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        // Battery 1: Healthy pack charging
        BatteryPack& b1 = packs[0];
        b1.isOnline = true;
        b1.lastSeenMs = millis();
        b1.voltage = 51.85f + (float)(random(-10, 10)) / 100.0f;
        b1.current = 15.4f + (float)(random(-5, 5)) / 10.0f;
        b1.soc = 76;
        b1.soh = 99;
        b1.tempPcb = 24;
        b1.tempAvg = 23;
        b1.tempMax = 25;
        b1.statusName = "Charging";
        b1.cycles = 42;
        b1.balanceMask = 0;

        uint16_t base1 = 3450 + random(-5, 5);
        uint16_t b1Min = 5000, b1Max = 0;
        for (int i = 0; i < CELLS_PER_BATTERY; i++) {
            b1.cells[i] = base1 + (i % 3) * 3 - (i % 2) * 2;
            if (b1.cells[i] < b1Min) b1Min = b1.cells[i];
            if (b1.cells[i] > b1Max) b1Max = b1.cells[i];
        }
        b1.cellMin = b1Min;
        b1.cellMax = b1Max;
        b1.cellDiff = b1Max - b1Min;

        // Battery 2: Pack with delta & active balancer
        BatteryPack& b2 = packs[1];
        b2.isOnline = true;
        b2.lastSeenMs = millis();
        b2.voltage = 51.72f + (float)(random(-10, 10)) / 100.0f;
        b2.current = 14.8f + (float)(random(-5, 5)) / 10.0f;
        b2.soc = 74;
        b2.soh = 97;
        b2.tempPcb = 26;
        b2.tempAvg = 25;
        b2.tempMax = 28;
        b2.statusName = "Charging";
        b2.cycles = 118;
        b2.balanceMask = (1 << 11) | (1 << 13); // Cell 12 and 14 balancing

        uint16_t base2 = 3445 + random(-4, 4);
        uint16_t b2Min = 5000, b2Max = 0;
        for (int i = 0; i < CELLS_PER_BATTERY; i++) {
            if (i == 4) b2.cells[i] = base2 - 32 + random(-2, 2);       // Lagging cell #5
            else if (i == 11) b2.cells[i] = base2 + 18 + random(-2, 2); // High cell #12 [BAL]
            else b2.cells[i] = base2 + (i % 4) * 2 + random(-1, 2);
            if (b2.cells[i] < b2Min) b2Min = b2.cells[i];
            if (b2.cells[i] > b2Max) b2Max = b2.cells[i];
        }
        b2.cellMin = b2Min;
        b2.cellMax = b2Max;
        b2.cellDiff = b2Max - b2Min; // ~50 mV delta

        // Demo periodic simulation: every 15 seconds simulate a mild warning on Pack #2
        static uint8_t warnCounter = 0;
        warnCounter++;
        if ((warnCounter % 15) >= 11) {
            b2.warningRaw = 0x0002; // Bit 1: Cell OV Warning
        } else {
            b2.warningRaw = 0;
        }

        // Battery 3: Healthy pack in float/charge
        BatteryPack& b3 = packs[2];
        b3.isOnline = true;
        b3.lastSeenMs = millis();
        b3.voltage = 51.80f + (float)(random(-8, 8)) / 100.0f;
        b3.current = 15.1f + (float)(random(-4, 4)) / 10.0f;
        b3.soc = 75;
        b3.soh = 98;
        b3.tempPcb = 25;
        b3.tempAvg = 24;
        b3.tempMax = 26;
        b3.statusName = "Charging";
        b3.cycles = 55;
        b3.balanceMask = 0;

        uint16_t base3 = 3448 + random(-4, 4);
        uint16_t b3Min = 5000, b3Max = 0;
        for (int i = 0; i < CELLS_PER_BATTERY; i++) {
            b3.cells[i] = base3 + (i % 3) * 2;
            if (b3.cells[i] < b3Min) b3Min = b3.cells[i];
            if (b3.cells[i] > b3Max) b3Max = b3.cells[i];
        }
        b3.cellMin = b3Min;
        b3.cellMax = b3Max;
        b3.cellDiff = b3Max - b3Min;
        b3.warningRaw = 0;

        // Battery 4: Healthy pack in charge
        BatteryPack& b4 = packs[3];
        b4.isOnline = true;
        b4.lastSeenMs = millis();
        b4.voltage = 51.88f + (float)(random(-8, 8)) / 100.0f;
        b4.current = 15.6f + (float)(random(-4, 4)) / 10.0f;
        b4.soc = 77;
        b4.soh = 100;
        b4.tempPcb = 24;
        b4.tempAvg = 23;
        b4.tempMax = 25;
        b4.statusName = "Charging";
        b4.cycles = 28;
        b4.balanceMask = 0;

        uint16_t base4 = 3452 + random(-4, 4);
        uint16_t b4Min = 5000, b4Max = 0;
        for (int i = 0; i < CELLS_PER_BATTERY; i++) {
            b4.cells[i] = base4 + (i % 2) * 3;
            if (b4.cells[i] < b4Min) b4Min = b4.cells[i];
            if (b4.cells[i] > b4Max) b4Max = b4.cells[i];
        }
        b4.cellMin = b4Min;
        b4.cellMax = b4Max;
        b4.cellDiff = b4Max - b4Min;
        b4.warningRaw = 0;

        xSemaphoreGive(mutex);
    }
}

