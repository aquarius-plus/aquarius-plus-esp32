#include "Menus.h"
#include "FpgaCore.h"

#include "DisplayOverlay.h"
#include "EspSettingsMenu.h"
#include "VersionMenu.h"

#ifdef CONFIG_MACHINE_TYPE_MORPHBOOK
#include "Settings.h"
#endif

static EspSettingsMenu espSettingsMenu;

//////////////////////////////////////////////////////////////////////////////
// Main menu
//////////////////////////////////////////////////////////////////////////////
class MainMenu : public Menu {
public:
    MainMenu() : Menu("", 38) {
        isRootMenu = true;
    }

    void onUpdate() override {
        setNeedsRedraw();

        // Update title
        {
            time_t now;
            time(&now);
            struct tm timeinfo = *localtime(&now);

            char strftime_buf[20];
            strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%d %H:%M:%S", &timeinfo);

            char tmp[40];

            const CoreInfo *coreInfo = FpgaCore::getCoreInfo();
            snprintf(tmp, sizeof(tmp), "%-16s %s", coreInfo->name, strftime_buf);
            title = tmp;
        }

        items.clear();

#ifdef CONFIG_MACHINE_TYPE_MORPHBOOK
        {
            auto &item  = items.emplace_back(MenuItemType::percentage, "Volume");
            item.setter = [](int newVal) { getSettings()->setVolume(newVal); };
            item.getter = []() { return getSettings()->getVolume(); };
        }
        {
            auto &item  = items.emplace_back(MenuItemType::onOff, "Speakers");
            item.setter = [](int newVal) { getSettings()->setSpeakersOn(newVal != 0); };
            item.getter = []() { return getSettings()->getSpeakersOn(); };
        }
        {
            auto &item  = items.emplace_back(MenuItemType::percentage, "Brightness");
            item.setter = [](int newVal) { getSettings()->setBrightness(newVal); };
            item.getter = []() { return getSettings()->getBrightness(); };
        }
        items.emplace_back(MenuItemType::separator);
#endif

        auto fpgaCore = FpgaCore::get();
        if (fpgaCore) {
            fpgaCore->addMainMenuItems(*this);
            while (!items.empty() && items.back().type == MenuItemType::separator)
                items.pop_back();

            items.emplace_back(MenuItemType::separator);
        }

        {
            auto &item   = items.emplace_back(MenuItemType::subMenu, "Restart ESP (CTRL-SHIFT-ESC)");
            item.onEnter = [&]() { SystemRestart(); };
        }
        {
            auto &item   = items.emplace_back(MenuItemType::subMenu, "ESP settings");
            item.onEnter = []() { espSettingsMenu.show(); };
        }
        {
            auto &item   = items.emplace_back(MenuItemType::subMenu, "Version");
            item.onEnter = []() {
                VersionMenu subMenu;
                subMenu.show();
            };
        }
    }

    bool onTick() override {
        setNeedsUpdate();
        return false;
    }
};

Menu *getMainMenu() {
    static MainMenu obj;
    return &obj;
}
