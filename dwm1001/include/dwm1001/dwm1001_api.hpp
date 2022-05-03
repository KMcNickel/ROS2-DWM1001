//The API header for the DWM1001

enum DWM1001TLVTypes
{
    DWM1001_TLV_TYPE_COMMAND_INVALID = 0x00,
    DWM1001_TLV_TYPE_COMMAND_POS_SET = 0x01,
    DWM1001_TLV_TYPE_COMMAND_POS_GET = 0x02,
    DWM1001_TLV_TYPE_COMMAND_UPDATE_RATE_SET = 0x03,
    DWM1001_TLV_TYPE_COMMAND_UPDATE_RATE_GET = 0x04,
    DWM1001_TLV_TYPE_COMMAND_CONFIG_TAG_SET = 0x05,
    DWM1001_TLV_TYPE_COMMAND_CONFIG_ANCHOR_SET = 0x07,
    DWM1001_TLV_TYPE_COMMAND_CONFIG_GET = 0x08,
    DWM1001_TLV_TYPE_COMMAND_SLEEP = 0x0A,
    DWM1001_TLV_TYPE_COMMAND_ANCHOR_LIST_GET = 0x0B,
    DWM1001_TLV_TYPE_COMMAND_LOCATION_GET = 0x0C,
    DWM1001_TLV_TYPE_COMMAND_BLUETOOTH_ADDRESS_SET = 0x0F,
    DWM1001_TLV_TYPE_COMMAND_BLUETOOTH_ADDRESS_GET = 0x10,
    DWM1001_TLV_TYPE_COMMAND_STATIONARY_CONFIG_SET = 0x11,
    DWM1001_TLV_TYPE_COMMAND_STATIONARY_CONFIG_GET = 0x12,
    DWM1001_TLV_TYPE_COMMAND_FACTORY_RESET = 0x13,
    DWM1001_TLV_TYPE_COMMAND_RESET = 0x14,
    DWM1001_TLV_TYPE_COMMAND_VERSION_GET = 0x15,
    DWM1001_TLV_TYPE_COMMAND_UWB_CONFIG_SET = 0x17,
    DWM1001_TLV_TYPE_COMMAND_UWB_CONFIG_GET = 0x18,
    DWM1001_TLV_TYPE_COMMAND_USER_DATA_READ = 0x19,
    DWM1001_TLV_TYPE_COMMAND_USER_DATA_WRITE = 0x1A,
    DWM1001_TLV_TYPE_COMMAND_LABEL_READ = 0x1C,
    DWM1001_TLV_TYPE_COMMAND_LABEL_WRITE = 0x1D,
    DWM1001_TLV_TYPE_COMMAND_GPIO_CONFIG_OUTPUT = 0x28,
    DWM1001_TLV_TYPE_COMMAND_GPIO_CONFIG_INPUT = 0x29,
    DWM1001_TLV_TYPE_COMMAND_GPIO_VALUE_SET = 0x2A,
    DWM1001_TLV_TYPE_COMMAND_GPIO_VALUE_GET = 0x2B,
    DWM1001_TLV_TYPE_COMMAND_GPIO_VALUE_TOGGLE = 0x2C,
    DWM1001_TLV_TYPE_COMMAND_PANID_SET = 0x2E,
    DWM1001_TLV_TYPE_COMMAND_PANID_GET = 0x2F,
    DWM1001_TLV_TYPE_COMMAND_NODEID_GET = 0x30,
    DWM1001_TLV_TYPE_COMMAND_STATUS_GET = 0x32,
    DWM1001_TLV_TYPE_COMMAND_INTERRUPT_CONFIG_SET = 0x34,
    DWM1001_TLV_TYPE_COMMAND_INTERRUPT_CONFIG_GET = 0x35,
    DWM1001_TLV_TYPE_COMMAND_BACKHAUL_TRANSFER = 0x37,
    DWM1001_TLV_TYPE_COMMAND_BACKHAUL_STATUS_GET = 0x3A,
    DWM1001_TLV_TYPE_COMMAND_ENCRYPTION_KEY_SET = 0x3C,
    DWM1001_TLV_TYPE_COMMAND_ENCRYPTION_KEY_CLEAR = 0x3D,
    //The hex listing (table 3) ends here, but there is another table (Table 5)
            //with decimal values and I'm too lazy to convert / change anything, so...
    DWM1001_TLV_TYPE_RESPONSE_RETURN_VALUE = 64,
    DWM1001_TLV_TYPE_RESPONSE_POSITION_XYZ = 65,
    DWM1001_TLV_TYPE_RESPONSE_POSITION_X = 66,
    DWM1001_TLV_TYPE_RESPONSE_POSITION_Y = 67,
    DWM1001_TLV_TYPE_RESPONSE_POSITION_Z = 68,
    DWM1001_TLV_TYPE_RESPONSE_UPDATE_RATE = 69,
    DWM1001_TLV_TYPE_RESPONSE_GENERAL_CONFIG = 70,
    DWM1001_TLV_TYPE_RESPONSE_INTERRUPT_CONFIG = 71,
    DWM1001_TLV_TYPE_RESPONSE_RANGING_ANCHOR_DISTANCES = 72,
    DWM1001_TLV_TYPE_RESPONSE_RANGING_ANCHOR_DISTANCES_AND_POSITIONS = 73,
    DWM1001_TLV_TYPE_RESPONSE_STATIONARY_SENSITIVITY = 74,
    DWM1001_TLV_TYPE_RESPONSE_USER_DATA = 75,
    DWM1001_TLV_TYPE_RESPONSE_LABEL = 76,
    DWM1001_TLV_TYPE_RESPONSE_PAN_ID = 77,
    DWM1001_TLV_TYPE_RESPONSE_NODE_ID = 78,
    DWM1001_TLV_TYPE_RESPONSE_UWB_CONFIGURATION = 79,
    DWM1001_TLV_TYPE_RESPONSE_FIRMWARE_VERSION = 80,
    DWM1001_TLV_TYPE_RESPONSE_CONFIG_VERSION = 81,
    DWM1001_TLV_TYPE_RESPONSE_HARDWARE_VERSION = 82,
    DWM1001_TLV_TYPE_RESPONSE_PIN_VALUE = 85,
    DWM1001_TLV_TYPE_RESPONSE_ANCHOR_LIST = 86,
    DWM1001_TLV_TYPE_RESPONSE_STATUS = 90,
    DWM1001_TLV_TYPE_RESPONSE_UWB_PREAMBLE_CODE = 91,
    DWM1001_TLV_TYPE_RESPONSE_UWB_SCAN_RESULT = 92,
    DWM1001_TLV_TYPE_RESPONSE_UWBMAC_STATUS = 93,
    DWM1001_TLV_TYPE_RESPONSE_BLE_ADDRESS = 95,
    DWM1001_TLV_TYPE_RESPONSE_DOWNLINK_DATA_CHUNK_0 = 100,
    DWM1001_TLV_TYPE_RESPONSE_BUFFER_INDEX = 105,
    DWM1001_TLV_TYPE_RESPONSE_BUFFER_SIZE = 106,
    DWM1001_TLV_TYPE_RESPONSE_UPLINK_DATA_CHUNK_0 = 110,
    DWM1001_TLV_TYPE_RESPONSE_UPLINK_LOCATION_DATA = 120,
    DWM1001_TLV_TYPE_RESPONSE_UPLINK_IOT_DATA = 121,
    DWM1001_TLV_TYPE_RESPONSE_VARIABLE_LENGTH_PARAMETER = 127,
    DWM1001_TLV_TYPE_COMMAND_VA_ARG_SET_POSITION = 128
};

enum DWM1001ErrorCodes
{
    DWM1001_ERROR_NONE = 0,
    DWM1001_ERROR_UNKNOWN_OR_BROKEN_COMMAND = 1,
    DWM1001_ERROR_INTERNAL = 2,
    DWM1001_ERROR_INVALID_PARAMETER = 3,
    DWM1001_ERROR_BUSY = 4,
    DWM1001_ERROR_OPERATION_NOT_PERMITTED = 5
};

struct DWM1001PositionData
{
    int32_t x;
    int32_t y;
    int32_t z;
    int8_t quality;

    void fillData(unsigned char * data)
    {
        x = *data;
        y = *(data + 1);
        z = *(data + 2);
        quality = *(data + 3);
    }

    std::string to_string()
    {
        return "X: " + std::to_string(x) + "   Y: " + std::to_string(y) + 
                "   Z: " + std::to_string(z) + "   Quality: " + std::to_string(quality) + "%";
    }
};

struct DWM1001PositionAndDistance
{
    uint8_t uwbAddress[8];
    uint32_t distance;
    int8_t quality;
    DWM1001PositionData position;

    void fillData(unsigned char * data, bool isTag)
    {
        unsigned char curOffset = 2;
        uwbAddress[0] = *data;
        uwbAddress[1] = *(data + 1);
        if(!isTag)
        {
            uwbAddress[2] = *(data + 2);
            uwbAddress[3] = *(data + 3);
            uwbAddress[4] = *(data + 4);
            uwbAddress[5] = *(data + 5);
            uwbAddress[6] = *(data + 6);
            uwbAddress[7] = *(data + 7);
            curOffset = 8;
        }

        distance = *(data + curOffset++);
        distance = ((*(data + curOffset++)) << 8);
        distance = ((*(data + curOffset++)) << 16);
        distance = ((*(data + curOffset++)) << 24);

        quality = *(data + curOffset++);

        position.fillData(data + curOffset);
    }

    std::string to_string()
    {
        return "ID: " + std::to_string(uwbAddress[0]) + std::to_string(uwbAddress[1]) +
                "   Distance: " + std::to_string(distance) + "   Quality: " + 
                std::to_string(quality) + "   Position: " + position.to_string();
    }
};

struct DWM1001AnchorList
{
    int8_t anchorCount;
    DWM1001PositionAndDistance anchors[15];

    void fillData(unsigned char * data, bool isTag)
    {
        unsigned char * curData = data + 1;

        anchorCount = *data;
        for(int i = 0; i < anchorCount; i++)
        {
            anchors[i].fillData(curData, isTag);
            curData += 20;
            if(!isTag)
                curData += 6;
        }
    }

    std::string to_string()
    {
        std::string output;

        output = "Anchor count: " + anchorCount;
        for(int i = 0; i < anchorCount; i++)
        {
            output = output + "/n/tAnchor " + std::to_string(i + 1) + ": " + anchors[i].to_string();
        }

        return output;
    }
};

enum DWM1001GPIOPins
{
    DWM1001_GPIO_PIN_2 = 2,
    DWM1001_GPIO_PIN_3 = 3,
    DWM1001_GPIO_PIN_8 = 8,
    DWM1001_GPIO_PIN_9 = 9,
    DWM1001_GPIO_PIN_10 = 10,
    DWM1001_GPIO_PIN_12 = 12,
    DWM1001_GPIO_PIN_13 = 13,
    DWM1001_GPIO_PIN_14 = 14,
    DWM1001_GPIO_PIN_15 = 15,
    DWM1001_GPIO_PIN_22 = 22,
    DWM1001_GPIO_PIN_23 = 23,
    DWM1001_GPIO_PIN_30 = 30,
    DWM1001_GPIO_PIN_31 = 31
};

enum DWM1001GPIOValue
{
    DWM1001_GPIO_VALUE_LOW = 0,
    DWM1001_GPIO_VALUE_HIGH = 1
};

enum DWM1001GPIOPull
{
    DWM1001_GPIO_PULL_NONE = 0,
    DWM1001_GPIO_PULL_DOWN = 1,
    DWM1001_GPIO_PULL_UP = 3
};

struct DWM1001FirmwareVersion
{
    int8_t major;
    int8_t minor;
    int8_t patch;
    int8_t variant;

    void fillData(unsigned char * data)
    {
        major = *data;
        minor = *(data + 1);
        patch = *(data + 2);
        variant = (*(data + 3)) & 0b00001111;
    }

    std::string to_string()
    {
        return std::to_string(major) + "." + std::to_string(minor) + "." + std::to_string(patch) + "." + std::to_string(variant);
    }
};

struct DWM1001TagConfiguration
{
    int8_t uwbMode;
    bool firmwareUpdateEnabled;
    bool bleEnabled;
    bool ledEnabled;
    bool encryptionEnabled;
    bool locationEngineEnabled;
    bool lowPowerMode;
    int8_t measurementMode;
    bool stationaryModeEnabled;

    void packData(unsigned char * buffer)
    {
        *buffer = uwbMode;
        *buffer = *buffer | firmwareUpdateEnabled << 2;
        *buffer = *buffer | bleEnabled << 3;
        *buffer = *buffer | ledEnabled << 4;
        *buffer = *buffer | encryptionEnabled << 5;
        *buffer = *buffer | locationEngineEnabled << 6;
        *buffer = *buffer | lowPowerMode << 7;
        *(buffer + 1) = measurementMode;
        *(buffer + 1) = *buffer | stationaryModeEnabled << 2;
    }
};

struct DWM1001AnchorConfiguration
{
    bool firmwareUpdateEnabled;
    bool bleEnabled;
    bool ledEnabled;
    bool encryptionEnabled;
    bool bridgeMode;
    bool initiatorMode;

    void packData(unsigned char * buffer)
    {
        *buffer = firmwareUpdateEnabled << 2;
        *buffer = *buffer | bleEnabled << 2;
        *buffer = *buffer | ledEnabled << 2;
        *buffer = *buffer | encryptionEnabled << 2;
        *buffer = *buffer | bridgeMode << 2;
        *buffer = *buffer | initiatorMode << 2;
        *(buffer + 1) = 0;
    }
};

struct DWM1001Configuration
{
    int8_t uwbMode;
    bool firmwareUpdateEnabled;
    bool bleEnabled;
    bool ledEnabled;
    bool encryptionEnabled;
    bool locationEngineEnabled;
    bool lowPowerMode;
    int8_t measurementMode;
    bool stationaryModeEnabled;
    bool bridgeMode;
    bool initiatorMode;
    bool isAnchor;              //false for tag

    void fillData(unsigned char * data)
    {
        uwbMode = *data & 0b00000011;
        firmwareUpdateEnabled = *data & 0b00000100;
        bleEnabled = *data & 0b00001000;
        ledEnabled = *data & 0b00010000;
        encryptionEnabled = *data & 0b00100000;
        locationEngineEnabled = *data & 0b01000000;
        lowPowerMode = *data & 0b10000000;
        measurementMode = *(data + 1) & 0b00000011;
        stationaryModeEnabled = *(data + 1) & 0b00000100;
        bridgeMode = *(data + 1) & 0b00001000;
        initiatorMode = *(data + 1) & 0b00010000;
        isAnchor = *(data + 1) & 0b00100000;
    }

    std::string to_string()
    {
        return "\n\tUWB Mode: " + std::to_string(uwbMode) +
                "\n\tFirmware Update Enabled: " + std::to_string(firmwareUpdateEnabled) +
                "\n\tBLE Enabled: " + std::to_string(bleEnabled) +
                "\n\tLED Enabled: " + std::to_string(ledEnabled) +
                "\n\tEncryption Enabled: " + std::to_string(encryptionEnabled) +
                "\n\tLocation Engine Enabled: " + std::to_string(locationEngineEnabled) +
                "\n\tLow Power Mode: " + std::to_string(lowPowerMode) +
                "\n\tMeasurement Mode: " + std::to_string(measurementMode) +
                "\n\tStationary Mode Enabled: " + std::to_string(stationaryModeEnabled) +
                "\n\tBridge Mode: " + std::to_string(bridgeMode) +
                "\n\tInitiator Mode: " + std::to_string(initiatorMode) +
                "\n\tAnchor Mode: " + std::to_string(isAnchor);
    }
};

struct DWM1001InterruptConfiguration
{
    bool locationReady;
    bool spiDataReady;
    bool backhaulStatusChanged;
    bool backhaulDataReady;
    bool backhaulInitializedChanged;
    bool uwbScanReady;
    bool userDataReady;
    bool uwbMacJoinedChanged;
    bool userDataSent;

    void fillData(unsigned char * data)
    {
        locationReady = *data & 0b00000001;
        spiDataReady = *data & 0b00000010;
        backhaulStatusChanged = *data & 0b00000100;
        backhaulDataReady = *data & 0b00001000;
        backhaulInitializedChanged = *data & 0b00010000;
        uwbScanReady = *data & 0b00100000;
        userDataReady = *data & 0b01000000;
        uwbMacJoinedChanged = *data & 0b10000000;
        userDataSent = (*(data) + 1) & 0b00000001;
    }

        void packData(unsigned char * buffer)
    {
        *buffer = locationReady;
        *buffer = *buffer | spiDataReady << 1;
        *(buffer + 1) = 0;
    }

    std::string to_string()
    {
        return "\n\tLocation Ready: " + std::to_string(locationReady) +
                "\n\tSPI Data Ready: " + std::to_string(spiDataReady) +
                "\n\tBackhaul Status Changed: " + std::to_string(backhaulStatusChanged) +
                "\n\tBackhaul Data Ready: " + std::to_string(backhaulDataReady) +
                "\n\tBackhaul Initialized Changed: " + std::to_string(backhaulInitializedChanged) +
                "\n\tUWB Scan Ready: " + std::to_string(uwbScanReady) +
                "\n\tUser Data Ready: " + std::to_string(userDataReady) +
                "\n\tUWB MAC Joined Changed: " + std::to_string(uwbMacJoinedChanged) +
                "\n\tUser Data Sent: " + std::to_string(userDataSent);
    }
};

enum DWM1001StationarySensitivity
{
    DWM1001_STATIONARY_SENSITIVITY_LOW = 0,
    DWM1001_STATIONARY_SENSITIVITY_MEDIUM = 1,
    DWM1001_STATIONARY_SENSITIVITY_HIGH = 2,
};
