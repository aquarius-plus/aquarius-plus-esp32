/*
 * NimBLERemoteService.h
 *
 *  Created: on Jan 27 2020
 *      Author H2zero
 *
 * Originally:
 *
 * BLERemoteService.h
 *
 *  Created on: Jul 8, 2017
 *      Author: kolban
 */

#pragma once

#include "nimconfig.h"

#include "NimBLEClient.h"
#include "NimBLEUUID.h"
#include "NimBLERemoteCharacteristic.h"

#include <vector>

class NimBLEClient;
class NimBLERemoteCharacteristic;

/**
 * @brief A model of a remote %BLE service.
 */
class NimBLERemoteService {
public:
    virtual ~NimBLERemoteService();

    // Public methods
    std::vector<NimBLERemoteCharacteristic *>::iterator begin() {
        return m_characteristicVector.begin();
    }
    std::vector<NimBLERemoteCharacteristic *>::iterator end() {
        return m_characteristicVector.end();
    }
    NimBLERemoteCharacteristic *getCharacteristic(const char *uuid) {
        return getCharacteristic(NimBLEUUID(uuid));
    }
    NimBLERemoteCharacteristic *getCharacteristic(const NimBLEUUID &uuid);
    void                        deleteCharacteristics();
    size_t                      deleteCharacteristic(const NimBLEUUID &uuid);
    NimBLEClient               *getClient() {
        return m_pClient;
    }
    NimBLEUUID getUUID() {
        return m_uuid;
    }

    std::string                                getValue(const NimBLEUUID &characteristicUuid);
    bool                                       setValue(const NimBLEUUID &characteristicUuid, const std::string &value);
    std::string                                toString();
    std::vector<NimBLERemoteCharacteristic *> *getCharacteristics(bool refresh = false);

private:
    // Private constructor ... never meant to be created by a user application.
    NimBLERemoteService(NimBLEClient *pClient, const struct ble_gatt_svc *service);

    // Friends
    friend class NimBLEClient;
    friend class NimBLERemoteCharacteristic;

    // Private methods
    bool       retrieveCharacteristics(const NimBLEUUID *uuid_filter = nullptr);
    static int characteristicDiscCB(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr, void *arg);

    uint16_t getStartHandle() {
        return m_startHandle;
    }
    uint16_t getEndHandle() {
        return m_endHandle;
    }
    void releaseSemaphores();

    // Properties

    // We maintain a vector of characteristics owned by this service.
    std::vector<NimBLERemoteCharacteristic *> m_characteristicVector;

    NimBLEClient *m_pClient;
    NimBLEUUID    m_uuid;
    uint16_t      m_startHandle;
    uint16_t      m_endHandle;
};
