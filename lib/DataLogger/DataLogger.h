#pragma once
#include <SdFat.h>

class SDLogger {
private:
    SdSpiConfig _cfg;
    SdFs _sd;
    bool _mounted;

public:
    explicit SDLogger(const SdSpiConfig& cfg);

    bool begin();
    void end();
    bool isMounted() const;
    void listFiles();

    template<typename Data>
    bool log(const char* fileName, unsigned long timestamp, const Data& data);

    template<typename Data>
    bool logOnce(const char* fileName, unsigned long timestamp, const Data& data);
};

#ifndef LOG_BUFFER_SIZE
#define LOG_BUFFER_SIZE 128
#endif

template<typename Data>
bool SDLogger::log(const char* fileName, unsigned long timestamp, const Data& data) {
    if (!fileName || fileName[0] == '\0') return false;
    if (!_mounted && !begin()) return false;

    FsFile file;
    bool exists = _sd.exists(fileName);

    if (!file.open(fileName, O_WRONLY | O_CREAT | O_APPEND)) {
        return false;
    }

    if (!exists || file.size() == 0) {
        file.print("timestamp,");
        file.println(Data::header());
    }

    file.print(timestamp);
    file.print(',');

    char buffer[LOG_BUFFER_SIZE];
    int len = data.toCSV(buffer, sizeof(buffer));

    if (len <= 0 || len >= (int)sizeof(buffer)) {
        file.close();
        return false;
    }

    file.write((const uint8_t*)buffer, len);
    file.println();

    bool ok = !file.getWriteError();
    file.flush();
    file.close();

    return ok;
}

template<typename Data>
bool SDLogger::logOnce(const char* fileName, unsigned long timestamp, const Data& data) {
    if (!begin()) return false;
    bool ok = log(fileName, timestamp, data);
    end();
    return ok;
}