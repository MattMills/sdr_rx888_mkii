// libusb transport for Cypress FX3 based SDDC receivers: enumeration,
// firmware upload into the FX3 ROM bootloader, vendor requests and
// asynchronous bulk-IN streaming.
//
// Written for this project; the firmware upload and streaming flow follow
// ExtIO_sddc's libusb backend (Core/arch/linux, by Franco Venturi, GPL-3.0)
// and fxload's fx3_load_ram (GPL-2.0+). See THIRD_PARTY_NOTICES.md.
#pragma once
#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct libusb_context;
struct libusb_device_handle;
struct libusb_transfer;

namespace rx888 {

struct UsbDeviceInfo {
    std::string path;    // stable physical location, e.g. "1-4.2"
    uint16_t vid = 0;
    uint16_t pid = 0;
    bool bootloader = false;
    bool accessible = false; // could be opened (a WinUSB/libusb driver is bound)
    int speed = 0;           // libusb_speed
    std::string serial;
    std::string product;
    std::string error;       // reason it could not be opened
};

const char* usbSpeedName(int speed);

class Fx3Device {
public:
    // Called from the libusb event thread for every completed bulk transfer.
    using DataCallback = std::function<void(const uint8_t* data, size_t len)>;

    Fx3Device();
    ~Fx3Device();
    Fx3Device(const Fx3Device&) = delete;
    Fx3Device& operator=(const Fx3Device&) = delete;

    static std::vector<UsbDeviceInfo> enumerate();

    // Upload a Cypress .img firmware image into the device at `path`, which
    // must be in bootloader mode, then wait (up to timeoutMs) for it to
    // re-enumerate as the streamer. The streamer may come back on a different
    // port path (e.g. the SuperSpeed side of a USB 3 port); it is returned in
    // newPath. Returns false and sets err on failure.
    static bool loadFirmware(const std::string& path, const uint8_t* img, size_t len,
                             std::string& newPath, std::string& err, int timeoutMs = 8000);

    // Open a device running the SDDC firmware.
    bool open(const std::string& path, std::string& err);
    void close();
    bool isOpen() const { return handle != nullptr; }
    int speed() const { return usbSpeed; }
    const std::string& path() const { return devPath; }

    // Vendor requests. Return false on USB error.
    bool vendorOut(uint8_t request, uint16_t value, uint16_t index, const void* data, uint16_t len);
    bool vendorIn(uint8_t request, uint16_t value, uint16_t index, void* data, uint16_t len);

    // Bulk streaming. transferSize must be a multiple of the endpoint's
    // max packet size * burst (16 KiB on SuperSpeed).
    bool startStreaming(size_t transferSize, int numTransfers, DataCallback cb, std::string& err);
    void stopStreaming();
    bool isStreaming() const { return streaming; }

    // Count of transfers that completed with an error since startStreaming().
    uint64_t transferErrors() const { return errorCount; }
    std::string lastError() const;

private:
    static void transferCallback(libusb_transfer* xfer);
    void eventLoop();

    libusb_context* ctx = nullptr;
    libusb_device_handle* handle = nullptr;
    std::string devPath;
    int usbSpeed = 0;
    uint8_t bulkEp = 0;
    uint16_t bulkMaxPacket = 0;
    uint8_t bulkMaxBurst = 0;

    std::vector<libusb_transfer*> transfers;
    std::vector<uint8_t*> transferBufs;
    DataCallback callback;
    std::atomic<bool> streaming{ false };
    std::atomic<bool> stopRequested{ false };
    std::atomic<int> pending{ 0 };
    std::atomic<uint64_t> errorCount{ 0 };
    std::thread eventThread;
    mutable std::mutex errMtx;
    std::string lastErr;
};

} // namespace rx888
