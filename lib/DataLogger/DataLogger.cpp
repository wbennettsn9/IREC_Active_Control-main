#include "DataLogger.h"

SDLogger::SDLogger(const SdSpiConfig& cfg)
    : _cfg(cfg), _mounted(false) {}

bool SDLogger::begin() {
    if (_mounted) return true;
    if (!_sd.begin(_cfg)) return false;
    _mounted = true;
    return true;
}

void SDLogger::end() {
    if (_mounted) {
        _sd.end();
        _mounted = false;
    }
}

bool SDLogger::isMounted() const {
    return _mounted;
}

void SDLogger::listFiles() {
    if (!_mounted && !begin()) return;
    _sd.ls(LS_R | LS_DATE | LS_SIZE);
}