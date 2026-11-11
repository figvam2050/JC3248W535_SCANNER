#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include "bms_data.h"
#include <FS.h>
#include <SD_MMC.h>

// MicroSD 1-bit SDMMC on JC3248W535
#define SD_CLK_PIN  12
#define SD_CMD_PIN  11
#define SD_D0_PIN   13

class SdLogger {
public:
    void begin();
    bool isAvailable() const { return _cardAvailable; }

    // Toggle All (Bank overview)
    bool toggleBankLog();
    bool isBankLogging() const { return _bankLogging; }
    void setBankLogging(bool enable);

    // Toggle specific Pack (packIdx: 0..15)
    bool togglePackLog(uint8_t packIdx);
    bool isPackLogging(uint8_t packIdx) const;
    void setPackLogging(uint8_t packIdx, bool enable);

    // Auto Alarm Logging
    bool isAutoAlarmLogging() const { return _alarmLogging; }
    uint32_t getAlarmRows() const { return _alarmRows; }

    // File Management
    struct FileInfo {
        char name[32];
        size_t size;
    };
    int getFilesList(FileInfo* outList, int maxFiles);
    bool deleteFile(const char* filename);
    bool deleteAllLogs();

    // Periodic worker
    void update(const BatteryPack* packs, size_t count);

    // Raw RS485 Packet Logger
    bool toggleRawLog();
    bool isRawLogging() const { return _rawLogging; }
    void setRawLogging(bool enable);
    void logRawPacket(uint32_t baud, const uint8_t* data, size_t len, bool isCrcValid);
    uint32_t getRawPacketsLogged() const { return _rawPacketsLogged; }
    const char* getCurrentRawLogName() const { return _currentRawFileName; }

    uint32_t getBankRows() const { return _bankRows; }
    uint32_t getPackRows(uint8_t packIdx) const;

private:
    bool _cardAvailable = false;
    bool _bankLogging = false;
    uint32_t _bankRows = 0;

    bool _rawLogging = false;
    uint32_t _rawPacketsLogged = 0;
    File _rawLogFile;
    char _currentRawFileName[32] = {0};
    uint32_t _lastRawFlushMs = 0;

    bool _alarmLogging = false;
    uint32_t _alarmRows = 0;

    bool _packLogging[MAX_BATTERIES] = {false};
    uint32_t _packRows[MAX_BATTERIES] = {0};

    uint32_t _lastLogMs = 0;
};

extern SdLogger Logger;

#endif
