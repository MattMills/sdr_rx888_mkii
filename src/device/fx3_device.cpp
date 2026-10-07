#include "fx3_device.h"
#include "fx3_protocol.h"
#include <libusb.h>
#include <chrono>
#include <algorithm>
#include <cstring>
#include <cstdio>

namespace rx888 {

namespace {

constexpr unsigned CONTROL_TIMEOUT_MS = 3000;
constexpr unsigned BULK_TIMEOUT_MS = 1000;

std::string makePath(libusb_device* dev) {
    uint8_t ports[8];
    int n = libusb_get_port_numbers(dev, ports, sizeof(ports));
    std::string p = std::to_string(libusb_get_bus_number(dev));
    for (int i = 0; i < n; i++) {
        p += (i == 0) ? '-' : '.';
        p += std::to_string(ports[i]);
    }
    return p;
}

bool isFx3(const libusb_device_descriptor& d) {
    return d.idVendor == FX3_VID && (d.idProduct == FX3_PID_BOOTLOADER || d.idProduct == FX3_PID_STREAMER);
}

std::string usbErr(int rc) {
    return std::string(libusb_error_name(rc)) + " (" + libusb_strerror((libusb_error)rc) + ")";
}

// Find and open the FX3 device at `path` with the given PID.
libusb_device_handle* openAt(libusb_context* ctx, const std::string& path, uint16_t pid,
                             int* speed, std::string& err) {
    libusb_device** list = nullptr;
    ssize_t n = libusb_get_device_list(ctx, &list);
    if (n < 0) {
        err = "libusb_get_device_list: " + usbErr((int)n);
        return nullptr;
    }
    libusb_device_handle* h = nullptr;
    bool found = false;
    for (ssize_t i = 0; i < n; i++) {
        libusb_device_descriptor desc;
        if (libusb_get_device_descriptor(list[i], &desc) != 0) { continue; }
        if (desc.idVendor != FX3_VID || desc.idProduct != pid) { continue; }
        if (!path.empty() && makePath(list[i]) != path) { continue; }
        found = true;
        int rc = libusb_open(list[i], &h);
        if (rc != 0) {
            h = nullptr;
            err = "libusb_open: " + usbErr(rc);
            if (rc == LIBUSB_ERROR_NOT_SUPPORTED || rc == LIBUSB_ERROR_NOT_FOUND) {
                err += " - no WinUSB driver bound to 04B4:" + std::string(pid == FX3_PID_BOOTLOADER ? "00F3" : "00F1") +
                       " (install it with Zadig)";
            }
        }
        else if (speed) {
            *speed = libusb_get_device_speed(list[i]);
        }
        break;
    }
    libusb_free_device_list(list, 1);
    if (!found) { err = "device not found at " + path; }
    return h;
}

} // namespace

const char* usbSpeedName(int speed) {
    switch (speed) {
    case LIBUSB_SPEED_LOW: return "USB 1.x Low Speed";
    case LIBUSB_SPEED_FULL: return "USB 1.x Full Speed";
    case LIBUSB_SPEED_HIGH: return "USB 2.0 High Speed";
    case LIBUSB_SPEED_SUPER: return "USB 3.x SuperSpeed (5 Gb/s)";
    case LIBUSB_SPEED_SUPER_PLUS: return "USB 3.x SuperSpeed+ (10 Gb/s)";
    default: return "unknown speed";
    }
}

Fx3Device::Fx3Device() {}

Fx3Device::~Fx3Device() {
    close();
}

std::vector<UsbDeviceInfo> Fx3Device::enumerate() {
    std::vector<UsbDeviceInfo> out;
    libusb_context* ctx = nullptr;
    if (libusb_init(&ctx) != 0) { return out; }

    libusb_device** list = nullptr;
    ssize_t n = libusb_get_device_list(ctx, &list);
    for (ssize_t i = 0; i < n; i++) {
        libusb_device_descriptor desc;
        if (libusb_get_device_descriptor(list[i], &desc) != 0 || !isFx3(desc)) { continue; }

        UsbDeviceInfo info;
        info.path = makePath(list[i]);
        info.vid = desc.idVendor;
        info.pid = desc.idProduct;
        info.bootloader = (desc.idProduct == FX3_PID_BOOTLOADER);
        info.speed = libusb_get_device_speed(list[i]);

        libusb_device_handle* h = nullptr;
        int rc = libusb_open(list[i], &h);
        if (rc == 0) {
            info.accessible = true;
            unsigned char buf[256];
            if (desc.iSerialNumber && libusb_get_string_descriptor_ascii(h, desc.iSerialNumber, buf, sizeof(buf)) > 0) {
                info.serial = (const char*)buf;
            }
            if (desc.iProduct && libusb_get_string_descriptor_ascii(h, desc.iProduct, buf, sizeof(buf)) > 0) {
                info.product = (const char*)buf;
            }
            libusb_close(h);
        }
        else {
            info.error = usbErr(rc);
            if (rc == LIBUSB_ERROR_NOT_SUPPORTED || rc == LIBUSB_ERROR_NOT_FOUND) {
                info.error = "no WinUSB driver bound (install with Zadig)";
            }
        }
        out.push_back(info);
    }
    if (n >= 0) { libusb_free_device_list(list, 1); }
    libusb_exit(ctx);
    return out;
}

bool Fx3Device::loadFirmware(const std::string& path, const uint8_t* img, size_t len, std::string& newPath, std::string& err, int timeoutMs) {
    // Validate the Cypress boot image (AN76405): "CY", bImageCTL, bImageType,
    // then {uint32 lengthWords, uint32 address, data...} sections, a section
    // with length 0 whose address is the entry point, then a 32-bit checksum.
    if (len < 12 || img[0] != 'C' || img[1] != 'Y') {
        err = "firmware image has no Cypress 'CY' signature";
        return false;
    }
    if (img[3] != 0xB0) {
        err = "unsupported firmware image type";
        return false;
    }
    struct Section {
        uint32_t addr;
        const uint8_t* data;
        uint32_t bytes;
    };
    std::vector<Section> sections;
    size_t off = 4;
    uint32_t checksum = 0;
    uint32_t entry = 0;
    while (true) {
        if (off + 8 > len) {
            err = "firmware image truncated";
            return false;
        }
        uint32_t words, addr;
        memcpy(&words, img + off, 4);
        memcpy(&addr, img + off + 4, 4);
        off += 8;
        if (words == 0) {
            entry = addr;
            break;
        }
        if (off + (size_t)words * 4 > len) {
            err = "firmware image section exceeds file size";
            return false;
        }
        for (uint32_t w = 0; w < words; w++) {
            uint32_t v;
            memcpy(&v, img + off + w * 4, 4);
            checksum += v;
        }
        sections.push_back({ addr, img + off, words * 4 });
        off += (size_t)words * 4;
    }
    if (off + 4 > len) {
        err = "firmware image has no checksum";
        return false;
    }
    uint32_t expected;
    memcpy(&expected, img + off, 4);
    if (expected != checksum) {
        err = "firmware image checksum mismatch";
        return false;
    }

    // Streamers that already exist, to recognise the new one afterwards.
    std::vector<std::string> before;
    for (const auto& d : enumerate()) {
        if (d.pid == FX3_PID_STREAMER) { before.push_back(d.path); }
    }

    libusb_context* ctx = nullptr;
    int rc = libusb_init(&ctx);
    if (rc != 0) {
        err = "libusb_init: " + usbErr(rc);
        return false;
    }
    libusb_device_handle* h = openAt(ctx, path, FX3_PID_BOOTLOADER, nullptr, err);
    if (!h) {
        libusb_exit(ctx);
        return false;
    }
    libusb_claim_interface(h, 0); // not strictly needed for EP0, harmless

    const uint8_t outType = LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE;
    const uint8_t inType = LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE;
    std::vector<uint8_t> verify(4096);
    bool ok = true;
    for (const auto& s : sections) {
        uint32_t done = 0;
        while (done < s.bytes) {
            uint32_t chunk = std::min<uint32_t>(4096, s.bytes - done);
            uint32_t addr = s.addr + done;
            rc = libusb_control_transfer(h, outType, FX3_BOOT_RW_INTERNAL, addr & 0xFFFF, addr >> 16,
                                         (unsigned char*)s.data + done, (uint16_t)chunk, CONTROL_TIMEOUT_MS);
            if (rc != (int)chunk) {
                err = "firmware write failed at 0x" + std::to_string(addr) + ": " + (rc < 0 ? usbErr(rc) : "short write");
                ok = false;
                break;
            }
            rc = libusb_control_transfer(h, inType, FX3_BOOT_RW_INTERNAL, addr & 0xFFFF, addr >> 16,
                                         verify.data(), (uint16_t)chunk, CONTROL_TIMEOUT_MS);
            if (rc != (int)chunk || memcmp(verify.data(), s.data + done, chunk) != 0) {
                err = "firmware verify failed at address " + std::to_string(addr);
                ok = false;
                break;
            }
            done += chunk;
        }
        if (!ok) { break; }
    }
    if (ok) {
        // Jump to the entry point. The device detaches immediately, so an I/O
        // error here is expected.
        libusb_control_transfer(h, outType, FX3_BOOT_RW_INTERNAL, entry & 0xFFFF, entry >> 16, nullptr, 0, 1000);
    }
    libusb_release_interface(h, 0);
    libusb_close(h);
    libusb_exit(ctx);
    if (!ok) { return false; }

    // Wait for the streamer to show up, at the same port path or a new one.
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    bool bootloaderGone = false;
    while (std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        std::string candidate;
        bootloaderGone = true;
        for (const auto& d : enumerate()) {
            if (d.path == path && d.pid == FX3_PID_BOOTLOADER) { bootloaderGone = false; }
            if (d.pid != FX3_PID_STREAMER) { continue; }
            if (d.path == path) {
                candidate = d.path;
                break;
            }
            if (std::find(before.begin(), before.end(), d.path) == before.end()) { candidate = d.path; }
        }
        if (!candidate.empty()) {
            newPath = candidate;
            return true;
        }
    }
    if (bootloaderGone) {
        // On Windows libusb does not list devices without a driver.
        err = "the firmware is running but the device (04B4:00F1, \"RX888mk2\") is not accessible: "
              "install the WinUSB driver for it with Zadig";
    }
    else {
        err = "firmware uploaded but the device stayed in bootloader mode";
    }
    return false;
}

bool Fx3Device::open(const std::string& path, std::string& err) {
    close();
    int rc = libusb_init(&ctx);
    if (rc != 0) {
        err = "libusb_init: " + usbErr(rc);
        ctx = nullptr;
        return false;
    }
    handle = openAt(ctx, path, FX3_PID_STREAMER, &usbSpeed, err);
    if (!handle) {
        libusb_exit(ctx);
        ctx = nullptr;
        return false;
    }
    rc = libusb_claim_interface(handle, 0);
    if (rc != 0) {
        err = "libusb_claim_interface: " + usbErr(rc);
        close();
        return false;
    }

    // Find the bulk IN endpoint and its SuperSpeed burst size.
    libusb_device* dev = libusb_get_device(handle);
    libusb_config_descriptor* cfg = nullptr;
    rc = libusb_get_active_config_descriptor(dev, &cfg);
    if (rc != 0) {
        err = "libusb_get_active_config_descriptor: " + usbErr(rc);
        close();
        return false;
    }
    bulkEp = 0;
    for (int i = 0; i < cfg->bNumInterfaces && !bulkEp; i++) {
        const libusb_interface& itf = cfg->interface[i];
        for (int a = 0; a < itf.num_altsetting && !bulkEp; a++) {
            const libusb_interface_descriptor& alt = itf.altsetting[a];
            for (int e = 0; e < alt.bNumEndpoints; e++) {
                const libusb_endpoint_descriptor& ep = alt.endpoint[e];
                if ((ep.bmAttributes & 0x03) == LIBUSB_TRANSFER_TYPE_BULK && (ep.bEndpointAddress & 0x80)) {
                    bulkEp = ep.bEndpointAddress;
                    bulkMaxPacket = ep.wMaxPacketSize;
                    libusb_ss_endpoint_companion_descriptor* ss = nullptr;
                    if (libusb_get_ss_endpoint_companion_descriptor(ctx, &ep, &ss) == 0) {
                        bulkMaxBurst = ss->bMaxBurst;
                        libusb_free_ss_endpoint_companion_descriptor(ss);
                    }
                    break;
                }
            }
        }
    }
    libusb_free_config_descriptor(cfg);
    if (!bulkEp) {
        err = "bulk IN endpoint not found";
        close();
        return false;
    }
    devPath = path;
    return true;
}

void Fx3Device::close() {
    stopStreaming();
    if (handle) {
        libusb_release_interface(handle, 0);
        libusb_close(handle);
        handle = nullptr;
    }
    if (ctx) {
        libusb_exit(ctx);
        ctx = nullptr;
    }
    devPath.clear();
}

bool Fx3Device::vendorOut(uint8_t request, uint16_t value, uint16_t index, const void* data, uint16_t len) {
    if (!handle) { return false; }
    int rc = libusb_control_transfer(handle, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
                                     request, value, index, (unsigned char*)data, len, CONTROL_TIMEOUT_MS);
    if (rc < 0) {
        std::lock_guard<std::mutex> lck(errMtx);
        char buf[64];
        snprintf(buf, sizeof(buf), "vendor request 0x%02X: ", request);
        lastErr = buf + usbErr(rc);
        return false;
    }
    return true;
}

bool Fx3Device::vendorIn(uint8_t request, uint16_t value, uint16_t index, void* data, uint16_t len) {
    if (!handle) { return false; }
    int rc = libusb_control_transfer(handle, LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
                                     request, value, index, (unsigned char*)data, len, CONTROL_TIMEOUT_MS);
    if (rc < 0) {
        std::lock_guard<std::mutex> lck(errMtx);
        char buf[64];
        snprintf(buf, sizeof(buf), "vendor request 0x%02X: ", request);
        lastErr = buf + usbErr(rc);
        return false;
    }
    return true;
}

std::string Fx3Device::lastError() const {
    std::lock_guard<std::mutex> lck(errMtx);
    return lastErr;
}

bool Fx3Device::startStreaming(size_t transferSize, int numTransfers, DataCallback cb, std::string& err) {
    if (!handle) {
        err = "device not open";
        return false;
    }
    if (streaming) { stopStreaming(); }
    size_t unit = (size_t)bulkMaxPacket * (bulkMaxBurst + 1);
    if (unit == 0 || transferSize % unit != 0) {
        err = "transfer size must be a multiple of " + std::to_string(unit);
        return false;
    }

    callback = std::move(cb);
    stopRequested = false;
    errorCount = 0;
    pending = 0;
    transfers.resize(numTransfers, nullptr);
    transferBufs.resize(numTransfers, nullptr);
    for (int i = 0; i < numTransfers; i++) {
        transferBufs[i] = new uint8_t[transferSize];
        transfers[i] = libusb_alloc_transfer(0);
        libusb_fill_bulk_transfer(transfers[i], handle, bulkEp, transferBufs[i], (int)transferSize,
                                  &Fx3Device::transferCallback, this, BULK_TIMEOUT_MS);
    }
    for (int i = 0; i < numTransfers; i++) {
        int rc = libusb_submit_transfer(transfers[i]);
        if (rc != 0) {
            err = "libusb_submit_transfer: " + usbErr(rc);
            break;
        }
        pending++;
    }
    streaming = true;
    eventThread = std::thread(&Fx3Device::eventLoop, this);
    if (!err.empty()) {
        stopStreaming();
        return false;
    }
    return true;
}

void Fx3Device::stopStreaming() {
    if (!streaming) { return; }
    stopRequested = true;
    for (auto* t : transfers) {
        if (t) { libusb_cancel_transfer(t); }
    }
    if (eventThread.joinable()) { eventThread.join(); }
    for (auto* t : transfers) {
        if (t) { libusb_free_transfer(t); }
    }
    for (auto* b : transferBufs) { delete[] b; }
    transfers.clear();
    transferBufs.clear();
    streaming = false;
}

void Fx3Device::eventLoop() {
    auto stopDeadline = std::chrono::steady_clock::time_point::max();
    while (pending > 0) {
        timeval tv = { 0, 100000 };
        libusb_handle_events_timeout_completed(ctx, &tv, nullptr);
        if (stopRequested) {
            auto now = std::chrono::steady_clock::now();
            if (stopDeadline == std::chrono::steady_clock::time_point::max()) {
                stopDeadline = now + std::chrono::seconds(3);
            }
            else if (now > stopDeadline) {
                break; // should not happen; avoid hanging forever
            }
        }
    }
}

void Fx3Device::transferCallback(libusb_transfer* xfer) {
    Fx3Device* self = (Fx3Device*)xfer->user_data;
    if (xfer->status == LIBUSB_TRANSFER_COMPLETED) {
        if (!self->stopRequested && xfer->actual_length > 0 && self->callback) {
            self->callback(xfer->buffer, (size_t)xfer->actual_length);
        }
    }
    else if (xfer->status != LIBUSB_TRANSFER_CANCELLED) {
        self->errorCount++;
        std::lock_guard<std::mutex> lck(self->errMtx);
        self->lastErr = std::string("bulk transfer: ") + libusb_error_name(xfer->status == LIBUSB_TRANSFER_TIMED_OUT ? LIBUSB_ERROR_TIMEOUT
                                                                          : xfer->status == LIBUSB_TRANSFER_NO_DEVICE ? LIBUSB_ERROR_NO_DEVICE
                                                                          : xfer->status == LIBUSB_TRANSFER_OVERFLOW ? LIBUSB_ERROR_OVERFLOW
                                                                          : xfer->status == LIBUSB_TRANSFER_STALL     ? LIBUSB_ERROR_PIPE
                                                                                                                      : LIBUSB_ERROR_IO);
    }

    bool resubmit = !self->stopRequested && xfer->status != LIBUSB_TRANSFER_CANCELLED && xfer->status != LIBUSB_TRANSFER_NO_DEVICE;
    if (resubmit && libusb_submit_transfer(xfer) == 0) { return; }
    self->pending--;
}

} // namespace rx888
