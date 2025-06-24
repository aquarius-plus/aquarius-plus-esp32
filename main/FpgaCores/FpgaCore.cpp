#include "FpgaCore.h"
#include "FPGA.h"
#include "xz.h"
#include "DisplayOverlay/DisplayOverlay.h"
#include "Keyboard.h"

static const char *TAG = "FpgaCore";

static std::shared_ptr<FpgaCore> currentCore;
CoreInfo                         FpgaCore::coreInfo;

const CoreInfo *FpgaCore::getCoreInfo() {
    return &coreInfo;
}

std::shared_ptr<FpgaCore> FpgaCore::get() {
    return currentCore;
}

void FpgaCore::unload() {
    currentCore = nullptr;
    memset(&coreInfo, 0, sizeof(coreInfo));

    Keyboard::instance()->reset();
}

std::shared_ptr<FpgaCore> FpgaCore::load(const void *data, size_t length) {
    unload();

    if (!FPGA::instance()->loadBitstream(data, length))
        return nullptr;

    FPGA::instance()->getCoreInfo(&coreInfo);
    switch ((FpgaCoreType)coreInfo.coreType) {
        case FpgaCoreType::AquariusPlus: currentCore = newCoreAquariusPlus(); break;
        case FpgaCoreType::Aquarius32: currentCore = newCoreAquarius32(); break;
        default: break;
    }

    if (!currentCore) {
        ESP_LOGE(TAG, "Error creating core handler");
        return nullptr;
    }

    getDisplayOverlay()->reinit();
    return currentCore;
}

std::shared_ptr<FpgaCore> FpgaCore::loadAqPlus() {
    const void *data   = "aqplus.core";
    size_t      length = 0;

#ifndef EMULATOR
#ifdef CONFIG_MACHINE_TYPE_AQPLUS
    std::vector<uint8_t> fpgaImage;
    {
        extern const uint8_t fpgaImageXzhStart[] asm("_binary_aqp_top_bit_xzh_start");
        extern const uint8_t fpgaImageXzhEnd[] asm("_binary_aqp_top_bit_xzh_end");

        length = *(uint32_t *)fpgaImageXzhStart;
        fpgaImage.resize(length);

        if (xz_decompress(
                fpgaImageXzhStart + 4,
                (fpgaImageXzhEnd - fpgaImageXzhStart) - 4,
                fpgaImage.data()) == XZ_SUCCESS) {

            data = fpgaImage.data();
        } else {
            fpgaImage.clear();
            length = 0;
        }
    }
#else
    extern const uint8_t fpgaImageStart[] asm("_binary_morphbook_aqplus_impl1_bit_start");
    extern const uint8_t fpgaImageEnd[] asm("_binary_morphbook_aqplus_impl1_bit_end");
    data   = fpgaImageStart;
    length = fpgaImageEnd - fpgaImageStart;
#endif
#endif

    return load(data, length);
}
