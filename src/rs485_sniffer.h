#ifndef RS485_SNIFFER_H
#define RS485_SNIFFER_H

#include "bms_data.h"

// JST 1.25 Power/UART connector (P1)
#define RS485_RX_PIN 44
#define RS485_TX_PIN 43 // Kept disabled for pure passive sniffer mode

class RS485Sniffer {
public:
    void begin(HardwareSerial* serialPort = &Serial2);
    void update(BatteryPack* packs, SemaphoreHandle_t mutex);
    
    bool demoMode = true; // Enabled by default until hardware RS485 signal arrives
    void updateDemo(BatteryPack* packs, SemaphoreHandle_t mutex);

    // Scanner / Diagnostic Info
    uint32_t getCurrentBaud() const { return _baudRates[_baudIdx]; }
    uint32_t getRxPackets() const { return _rxPackets; }
    uint32_t getCrcErrors() const { return _crcErrors; }
    uint32_t getRawBytes() const { return _rawBytesCount; }
    bool isAutoScanEnabled() const { return _autoScan; }
    void setAutoScan(bool enable) { _autoScan = enable; }
    void setBaudRate(uint32_t baud);

private:
    HardwareSerial* _port = nullptr;
    uint8_t _rxBuf[512];
    size_t _rxLen = 0;
    uint32_t _lastByteTimeMs = 0;

    // Auto-scan baud configuration
    static const uint32_t _baudRates[];
    static const size_t _baudRatesCount;
    size_t _baudIdx = 0;
    uint32_t _lastBaudSwitchMs = 0;
    bool _autoScan = true;

    // Traffic statistics
    uint32_t _rxPackets = 0;
    uint32_t _crcErrors = 0;
    uint32_t _rawBytesCount = 0;
    
    void nextBaudRate();
    uint16_t calculateCRC(const uint8_t* buf, size_t len);
    void parseFrame(const uint8_t* frame, size_t len, BatteryPack* packs, SemaphoreHandle_t mutex);
};

extern RS485Sniffer Sniffer;

#endif // RS485_SNIFFER_H

