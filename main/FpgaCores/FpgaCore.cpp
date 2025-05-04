#include "FpgaCore.h"
#include "FPGA.h"
#include "XzDecompress.h"
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
    extern const uint8_t fpgaImageXzhStart[] asm("_binary_aqp_top_bit_xzh_start");
    extern const uint8_t fpgaImageXzhEnd[] asm("_binary_aqp_top_bit_xzh_end");
    auto                 fpgaImage = xzhDecompress(fpgaImageXzhStart, fpgaImageXzhEnd - fpgaImageXzhStart);
    data                           = fpgaImage.data();
    length                         = fpgaImage.size();
#else
    extern const uint8_t fpgaImageStart[] asm("_binary_morphbook_aqplus_impl1_bit_start");
    extern const uint8_t fpgaImageEnd[] asm("_binary_morphbook_aqplus_impl1_bit_end");
    data   = fpgaImageStart;
    length = fpgaImageEnd - fpgaImageStart;
#endif
#endif

    return load(data, length);
}
