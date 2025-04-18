#pragma once

#include "Common.h"
#include "DisplayOverlay/Menu.h"
#include "FPGA.h"

struct GamePadData {
    int8_t   lx, ly;
    int8_t   rx, ry;
    uint8_t  lt, rt;
    uint16_t buttons;
};

enum class FpgaCoreType {
    AquariusPlus,
};

class FpgaCore {
public:
    virtual void resetCore() {}
    virtual bool keyScancode(uint8_t modifiers, unsigned scanCode, bool keyDown) { return false; }
    virtual void keyChar(uint8_t ch, bool isRepeat) {}
    virtual void mouseReport(int dx, int dy, uint8_t buttonMask, int dWheel, bool absPos = false) {}
    virtual void gamepadReport(unsigned idx, const GamePadData &data) {}

    virtual int uartCommand(uint8_t cmd, const uint8_t *buf, size_t len) { return -1; }

    virtual bool loadBitstream(const void *data, size_t length) = 0;

    virtual void addMainMenuItems(Menu &menu) = 0;

    virtual void getCoreInfo(CoreInfo *coreInfo)                 = 0;
    virtual bool getGamePadData(unsigned idx, GamePadData &data) = 0;
};

std::shared_ptr<FpgaCore> getFpgaCore();
void                      unloadFpgaCore();
std::shared_ptr<FpgaCore> loadFpgaCore(FpgaCoreType type, const void *data = nullptr, size_t length = 0);
