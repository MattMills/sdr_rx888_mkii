// SDDC_FX3.img firmware embedded at build time (see cmake/embed_file.cmake).
#pragma once
#include <cstddef>

namespace rx888 {
extern const unsigned char sddc_fx3_firmware[];
extern const size_t sddc_fx3_firmware_size;
} // namespace rx888
