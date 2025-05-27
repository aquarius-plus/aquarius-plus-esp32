#include "Common.h"
#include "FpgaCore.h"
#include "FPGA.h"
#include "Keyboard.h"
#include "UartProtocol.h"
#include "VFS.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <nvs_flash.h>
#include "XzDecompress.h"
#include "DisplayOverlay/DisplayOverlay.h"
#include "DisplayOverlay/FileListMenu.h"

enum {
    IO_VCTRL    = 0xE0,
    IO_VPALSEL  = 0xEA,
    IO_VPALDATA = 0xEB,
    IO_BANK0    = 0xF0,
    IO_BANK1    = 0xF1,
    IO_BANK2    = 0xF2,
    IO_BANK3    = 0xF3,
};

enum {
    FLAG_HAS_Z80       = (1 << 0),
    FLAG_MOUSE_SUPPORT = (1 << 1),
    FLAG_VIDEO_TIMING  = (1 << 2),
    FLAG_AQPLUS        = (1 << 3),
    FLAG_FORCE_TURBO   = (1 << 4),
};

class CoreAquarius32 : public FpgaCore {
public:
    SemaphoreHandle_t mutex;
    GamePadData       gamePads[2];
    bool              gamepadNavigation = false;

    // Mouse state
    bool    mousePresent = false;
    float   mouseX       = 0;
    float   mouseY       = 0;
    uint8_t mouseButtons = 0;
    int     mouseWheel   = 0;

    uint8_t mouseSensitivityDiv = 4;

    CoreAquarius32() {
        memset(gamePads, 0, sizeof(gamePads));
        mutex = xSemaphoreCreateRecursiveMutex();

        UartProtocol::instance()->setBaudrate(25175000 / 6);
        applySettings();
        Keyboard::instance()->reset(false);
    }

    virtual ~CoreAquarius32() {
        vSemaphoreDelete(mutex);
    }

    void applySettings() {
        nvs_handle_t h;
        if (nvs_open("settings", NVS_READONLY, &h) == ESP_OK) {
            uint8_t mouseDiv = 0;
            if (nvs_get_u8(h, "mouseDiv", &mouseDiv) == ESP_OK) {
                mouseSensitivityDiv = mouseDiv;
            }

            uint8_t val8 = 0;
            if (nvs_get_u8(h, "gamepadNav", &val8) == ESP_OK) {
                gamepadNavigation = val8 != 0;
            }

            nvs_close(h);
        }
        resetCore();
    }

    void aqpWriteKeybBuffer16(uint16_t data) {
        // | Bit | Description                  |
        // | --: | ---------------------------- |
        // |  14 | Scancode(1) / Character(0)   |
        // |  13 | Scancode key up(0) / down(1) |
        // |  12 | Repeated                     |
        // |  11 | Modifier: Gui                |
        // |  10 | Modifier: Alt                |
        // |   9 | Modifier: Shift              |
        // |   8 | Modifier: Ctrl               |
        // | 7:0 | Character / Scancode         |

        auto               fpga = FPGA::instance();
        RecursiveMutexLock lock(fpga->getMutex());
        fpga->spiSel(true);
        uint8_t cmd[] = {CMD_WRITE_KBBUF16, (uint8_t)(data & 0xFF), (uint8_t)(data >> 8)};
        fpga->spiTx(cmd, sizeof(cmd));
        fpga->spiSel(false);
    }

    void resetCore() override {
        auto               fpga = FPGA::instance();
        RecursiveMutexLock lock(fpga->getMutex());

        fpga->spiSel(true);
        uint8_t cmd[] = {CMD_RESET, 0};
        fpga->spiTx(cmd, sizeof(cmd));
        fpga->spiSel(false);
    }

    bool keyScancode(uint8_t modifiers, unsigned scanCode, bool keyDown) override {
        RecursiveMutexLock lock(mutex);

        if (scanCode <= 255) {
            aqpWriteKeybBuffer16(
                (1 << 14) | // Scancode
                (keyDown ? (1 << 13) : 0) |
                (((modifiers >> 4) | (modifiers & 0xF)) << 8) |
                scanCode);
        }

        // Special keys
        {
            uint8_t combinedModifiers = (modifiers & 0xF) | (modifiers >> 4);
            if (scanCode == SCANCODE_ESCAPE && keyDown) {
                if (combinedModifiers == ModLCtrl) {
                    resetCore();
                    return true;
                } else if (combinedModifiers == (ModLShift | ModLCtrl)) {
                    // CTRL-SHIFT-ESCAPE -> reset ESP32 (somewhat equivalent to power cycle)
                    esp_restart();
                    return true;
                }
            }
        }
        return false;
    }

    void keyChar(uint8_t ch, bool isRepeat, uint8_t modifiers) override {
        RecursiveMutexLock lock(mutex);
        aqpWriteKeybBuffer16(
            (0 << 14) | // Character
            (isRepeat ? (1 << 12) : 0) |
            (((modifiers >> 4) | (modifiers & 0xF)) << 8) |
            ch);
    }

    void mouseReport(int dx, int dy, uint8_t buttonMask, int dWheel, bool absPos) override {
        RecursiveMutexLock lock(mutex);

        if (absPos) {
            if (dx < 0 || dy < 0)
                return;

            dy -= 32;
            dy /= 2;
            dx /= 2;
        }

        float sensitivity = 1.0f / (float)mouseSensitivityDiv;
        mouseX            = std::max(0.0f, std::min(319.0f, absPos ? dx : (mouseX + (float)(dx * sensitivity))));
        mouseY            = std::max(0.0f, std::min(199.0f, absPos ? dy : (mouseY + (float)(dy * sensitivity))));
        mouseButtons      = buttonMask;
        mousePresent      = true;
        mouseWheel        = mouseWheel + dWheel;
    }

    void gamepadReport(unsigned idx, const GamePadData &data) override {
        if (idx > 1)
            return;

        RecursiveMutexLock lock(mutex);

        uint16_t pressed = (~gamePads[idx].buttons & data.buttons);
        uint16_t changed = (gamePads[idx].buttons ^ data.buttons);

        // printf("idx=%u pressed=%04X\n", idx, pressed);

        if (idx == 0) {
            bool overlayVisible = getDisplayOverlay()->isVisible();
            auto kb             = Keyboard::instance();

            if (gamepadNavigation) {
                if (pressed & GCB_GUIDE) {
                    kb->handleScancode(SCANCODE_LCTRL, true);
                    kb->handleScancode(SCANCODE_TAB, true);
                    kb->handleScancode(SCANCODE_TAB, false);
                    kb->handleScancode(SCANCODE_LCTRL, false);
                }

                if (overlayVisible) {
                    if (changed & GCB_DPAD_UP)
                        kb->handleScancode(SCANCODE_UP, (data.buttons & GCB_DPAD_UP) != 0);
                    if (changed & GCB_DPAD_DOWN)
                        kb->handleScancode(SCANCODE_DOWN, (data.buttons & GCB_DPAD_DOWN) != 0);
                    if (changed & GCB_DPAD_LEFT)
                        kb->handleScancode(SCANCODE_LEFT, (data.buttons & GCB_DPAD_LEFT) != 0);
                    if (changed & GCB_DPAD_RIGHT)
                        kb->handleScancode(SCANCODE_RIGHT, (data.buttons & GCB_DPAD_RIGHT) != 0);
                    if (changed & GCB_A)
                        kb->handleScancode(SCANCODE_RETURN, (data.buttons & GCB_A) != 0);
                    if (changed & GCB_B)
                        kb->handleScancode(SCANCODE_ESCAPE, (data.buttons & GCB_B) != 0);
                }
            }
        }

        gamePads[idx] = data;
    }

    bool getGamePadData(unsigned idx, GamePadData &data) override {
        if (idx > 1)
            return false;

        RecursiveMutexLock lock(mutex);
        data = gamePads[idx];
        return true;
    }

    void cmdGetMouse() {
        // DBGF("GETMOUSE()");

        auto up = UartProtocol::instance();
        up->txStart();
        if (!mousePresent) {
            up->txWrite(ERR_NOT_FOUND);
            return;
        }

        up->txWrite(0);

        uint16_t x = (uint16_t)mouseX;
        uint8_t  y = (uint8_t)mouseY;
        up->txWrite(x & 0xFF);
        up->txWrite(x >> 8);
        up->txWrite(y);
        up->txWrite(mouseButtons);
        up->txWrite((int8_t)std::max(-128, std::min(mouseWheel, 127)));
        mouseWheel = 0;
    }

    void cmdGetGameCtrl(uint8_t idx) {
        // DBGF("GETGAMECTRL");

        auto up = UartProtocol::instance();
        up->txStart();
        if (idx > 1) {
            up->txWrite(ERR_NOT_FOUND);
            return;
        }
        up->txWrite(0);
        up->txWrite(gamePads[idx].lx);
        up->txWrite(gamePads[idx].ly);
        up->txWrite(gamePads[idx].rx);
        up->txWrite(gamePads[idx].ry);
        up->txWrite(gamePads[idx].lt);
        up->txWrite(gamePads[idx].rt);
        up->txWrite(gamePads[idx].buttons & 0xFF);
        up->txWrite(gamePads[idx].buttons >> 8);
    }

    int uartCommand(uint8_t cmd, const uint8_t *buf, size_t len) override {
        RecursiveMutexLock lock(mutex);
        switch (cmd) {
            case ESPCMD_GETMOUSE: {
                cmdGetMouse();
                return 1;
            }
            case ESPCMD_GETGAMECTRL: {
                if (len == 1) {
                    cmdGetGameCtrl(buf[0]);
                    return 1;
                }
                return 0;
            }
            default: break;
        }
        return -1;
    }

    std::string getPresetPath(std::string presetType) {
        auto coreInfo = getCoreInfo();
        return std::string("/config/esp32/") + coreInfo->name + "/" + presetType;
    }

    void savePreset(Menu &menu, std::string presetType, const void *buf, size_t size) {
        std::string presetName;
        if (menu.editString("Enter preset name", presetName, 32)) {
            presetName = trim(presetName, " \t\n\r\f\v/\\");

            if (!presetName.empty()) {
                auto vfs = getSDCardVFS();

                std::string path = getPresetPath(presetType);
                if (createPath(path)) {
                    path += "/" + presetName;

                    int fd = vfs->open(FO_WRONLY | FO_CREATE, path);
                    if (fd >= 0) {
                        vfs->write(fd, size, buf);
                        vfs->close(fd);
                    }
                }
            }
        }
    }
    void loadPreset(std::string presetType, void *buf, size_t size) {
        FileListMenu menu;
        menu.title    = "Select preset";
        menu.path     = getPresetPath(presetType);
        menu.onSelect = [buf, size](const std::string &path) {
            auto vfs = getSDCardVFS();
            int  fd  = vfs->open(FO_RDONLY, path);
            if (fd >= 0) {
                vfs->read(fd, size, buf);
                vfs->close(fd);
            }
        };
        menu.show();
    }

    void addMainMenuItems(Menu &menu) override {
        auto coreInfo = getCoreInfo();
        {
            auto &item   = menu.items.emplace_back(MenuItemType::subMenu, "Reset CPU (CTRL-ESC)");
            item.onEnter = [this]() {
                resetCore();
            };
        }
        menu.items.emplace_back(MenuItemType::separator);
        {
            auto &item  = menu.items.emplace_back(MenuItemType::onOff, "Navigate menu using gamepad");
            item.setter = [this](int newVal) {
                gamepadNavigation = (newVal != 0);

                nvs_handle_t h;
                if (nvs_open("settings", NVS_READWRITE, &h) == ESP_OK) {
                    if (nvs_set_u8(h, "gamepadNav", gamepadNavigation ? 1 : 0) == ESP_OK) {
                        nvs_commit(h);
                    }
                    nvs_close(h);
                }
            };
            item.getter = [this]() { return gamepadNavigation ? 1 : 0; };
        }
        menu.items.emplace_back(MenuItemType::separator);
        if (coreInfo->flags & FLAG_MOUSE_SUPPORT) {
            auto &item  = menu.items.emplace_back(MenuItemType::percentage, "Mouse sensitivity");
            item.setter = [this](int newVal) {
                newVal = std::max(1, std::min(newVal, 8));
                if (newVal != mouseSensitivityDiv) {
                    mouseSensitivityDiv = newVal;

                    nvs_handle_t h;
                    if (nvs_open("settings", NVS_READWRITE, &h) == ESP_OK) {
                        if (nvs_set_u8(h, "mouseDiv", mouseSensitivityDiv) == ESP_OK) {
                            nvs_commit(h);
                        }
                        nvs_close(h);
                    }
                }
            };
            item.getter = [this]() { return mouseSensitivityDiv; };
        }
    }
};

std::shared_ptr<FpgaCore> newCoreAquarius32() {
    return std::make_shared<CoreAquarius32>();
}
