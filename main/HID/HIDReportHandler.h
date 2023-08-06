#pragma once

#include "Common.h"
#include "HIDReportDescriptor.h"

class HIDReportHandler {
public:
    enum Type {
        TUndefined,
        TKeyboard,
        TMouse,
        TGamepad,
    };

    static HIDReportHandler *getReportHandlersForDescriptor(const void *reportDescBuf, size_t reportDescLen);

    HIDReportHandler(Type type);
    virtual ~HIDReportHandler();

    virtual bool init(const HIDReportDescriptor::HIDCollection *collection);

    virtual void addInputField(const HIDReportDescriptor::HIDField &field);
    virtual void inputReport(const uint8_t *buf, size_t length) = 0;

    HIDReportHandler *next = nullptr;
    Type              type = TUndefined;

protected:
    void enumerateCollection(const HIDReportDescriptor::HIDCollection *collection);

    int32_t readBits(const void *buf, size_t bufLen, uint32_t bitOffset, uint32_t bitLength, bool signExtend);
};
