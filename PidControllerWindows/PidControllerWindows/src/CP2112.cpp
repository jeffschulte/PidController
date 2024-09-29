#include "CP2112.h"

/// <summary>
/// Initializes the CP2112 by running through several steps
/// 
/// <list type="number">
/// <item>
/// <description>Verify the library Version function works</description>
/// </item>
/// <item>
/// <description>Get the number of devices (filtered)</description>
/// </item>
/// <item>
/// <description>List devices (filtered)</description>
/// </item>
/// <item>
/// <description>Open target device</description>
/// </item>
/// <item>
/// <description>Verify open status (sanity check)</description>
/// </item>
/// <item>
/// <description>Set device configuration</description>
/// </item>
/// </list>
/// </summary>
/// <returns>0 on success</returns>
int CP2112::initialize()
{
    // 1. we talk to the library to make sure we're linked correctly
    // if this fails, you need to find your DLLs
    BYTE major;
    BYTE minor;
    BOOL release;
    HID_SMBUS_STATUS status = HidSmbus_GetHidLibraryVersion(&major, &minor, &release);
    if (status != HID_SMBUS_SUCCESS) {
        std::cerr << "HidSmbus_GetHidLibraryVersion failed with error code: " << status << std::endl;
        return status;
    }
    //std::cout << "Library version; major: " << int(major) << " minor: " << int(minor) << " release: " << release << std::endl;

    // 2. get the number of devices
    //    if we set vendor_id and product_id, we filter the response
    //    Note that if we don't set the vender_id and product_id, we will et errors on the next step for already open devices (your computer probably has some open HID devices)
    DWORD number_of_devices;
    WORD vendor_id = 0x10C4;  // vendor id of my CP2112 board
    WORD product_id = 0xEA90; // product id of my CP2112 board
    status = HidSmbus_GetNumDevices(&number_of_devices, vendor_id, product_id);
    if (status != HID_SMBUS_SUCCESS) {
        std::cerr << "HidSmbus_GetNumDevices failed with error code: " << status << std::endl;
        return status;
    }
    //std::cout << "number of devices found: " << number_of_devices << std::endl;

    // 3. List devices
    DWORD device_number = 0;
    for (; device_number < number_of_devices; device_number++) {
        char device_string[260];
        const HID_SMBUS_GETSTRING options = HID_SMBUS_GET_PRODUCT_STR;
        status = HidSmbus_GetString(device_number, vendor_id, product_id, device_string, options);
        if (status != HID_SMBUS_SUCCESS) {
            // I have had this fail with HID_SMBUS_DEVICE_ALREADY_OPENED. Make sure to filter by vendor_id and product_id.
            std::cerr << "HidSmbus_GetString failed with error code: " << int(status) << std::endl;
            return status;
        }
        std::string target = "CP2112 HID USB-to-SMBus Bridge";
        //std::cout << "Found device: " << device_string << std::endl;
        if (target == std::string(device_string)) {
            //std::cout << "Target device found." << std::endl;
            break;
        }
    }

    // 4. Open device
    status = HidSmbus_Open(&device, device_number, vendor_id, product_id);
    if (status != HID_SMBUS_SUCCESS) {
        std::cerr << "HidSmbus_Open failed with error code: " << int(status) << std::endl;
        return status;
    }

    // 5. Check opened status
    //    Note: This is completely unnecessary, but allows us to play with an additional library function
    char opened_device_string[260];
    const HID_SMBUS_GETSTRING opened_options = HID_SMBUS_GET_PRODUCT_STR;
    status = HidSmbus_GetOpenedString(device, opened_device_string, opened_options);
    if (status != HID_SMBUS_SUCCESS) {
        std::cerr << "HidSmbus_GetOpenedString failed with error code: " << int(status) << std::endl;
        return status;
    }
    //std::cout << "Opened device string (sanity check): " << opened_device_string << std::endl;

    // 6. Set device configuration. These are sane values that have been tested
    DWORD bitrate = 100000;
    BYTE address = 0xA0;
    BOOL auto_read_respond = 0;
    WORD write_timeout = 1000; // 0 indicates infinite
    WORD read_timeout = 1000;  // 0 indicates infinite
    BOOL scl_low_timeout = 0; // default, disabled
    WORD transfer_retries = 0; // 0 indicates infinite
    status = HidSmbus_SetSmbusConfig(device, bitrate, address, auto_read_respond, write_timeout, read_timeout, scl_low_timeout, transfer_retries);
    if (status != HID_SMBUS_SUCCESS) {
        std::cerr << "HidSmbus_SetSmbusConfig failed with error code: " << int(status) << std::endl;
        return status;
    }
    //std::cout << "Device configuration set." << std::endl;
	return status;
}

/// <summary>
/// Reads from the target slave device
/// </summary>
/// <param name="address">address of the slave device</param>
/// <param name="read_register">register on the slave device to read from</param>
/// <param name="data">buffer (of at least 61 bytes) to write the data received from the device</param>
/// <param name="buffer_length">size of the buffer passed in (must be at least 61)</param>
/// <param name="num_bytes">The number of bytes actually requested to be read. Can be smaller than 61.</param>
/// <returns>0 on success</returns>
int CP2112::read(uint8_t address, uint8_t read_register, uint8_t* data, std::size_t buffer_length, std::size_t num_bytes)
{
    uint8_t address_buffer[1] = { read_register };
    auto status = HidSmbus_AddressReadRequest(device, address, num_bytes, 1, address_buffer);
    if (status != HID_SMBUS_SUCCESS) {
        std::cerr << "HidSmbus_AddressReadRequest failed with error code: " << int(status) << std::endl;
        return status;
    }
    status = HidSmbus_ForceReadResponse(device, num_bytes);
    if (status != HID_SMBUS_SUCCESS) {
        std::cerr << "HidSmbus_ForceReadResponse failed with error code: " << int(status) << std::endl;
        return status;
    }

    HID_SMBUS_S0 read_status = 0;
    BYTE num_bytes_read = 0;
    status = HidSmbus_GetReadResponse(device, &read_status, data, buffer_length, &num_bytes_read);
    //std::cout << "Status read:" << std::endl;
    //std::cout << "Read status: " << int(read_status) << std::endl;
    //std::cout << "Num bytes read (should be 1): " << int(num_bytes_read) << std::endl;
    if (status != HID_SMBUS_SUCCESS) {
        std::cerr << "HidSmbus_GetReadResponse failed with error code: " << int(status) << std::endl;
        return status;
    }
    if (num_bytes_read != num_bytes) {
        std::cerr << "Bytes read do not match bytes requested: " << int(num_bytes_read) << ", " << num_bytes << std::endl;
        return -1; // we consider this an error, even if the CP2112 didn't
    }
    return status;
}

/// <summary>
/// Write to the target slave device
/// </summary>
/// <param name="address">address of the slave device</param>
/// <param name="data">buffer of the bytes to write to the slave device (commands typically start with a register byte, followed by a command byte)</param>
/// <param name="num_bytes">number of bytes to write</param>
/// <returns>0 on success</returns>
int CP2112::write(uint8_t address, uint8_t* data, std::size_t num_bytes)
{
    HID_SMBUS_STATUS status = HidSmbus_WriteRequest(device, address, data, num_bytes);
    if (status != HID_SMBUS_SUCCESS) {
        std::cerr << "HidSmbus_WriteRequest failed with error code: " << int(status) << std::endl;
    }
    return status;
}

/// <summary>
/// Closes the CP2112 device
/// </summary>
/// <returns>0 on success</returns>
int CP2112::cleanup()
{
    auto status = HidSmbus_Close(device);
    if (status != HID_SMBUS_SUCCESS) {
        std::cerr << "HidSmbus_Close failed with error code: " << int(status) << std::endl;
    }
    return status;
}
