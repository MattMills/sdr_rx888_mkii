# Converts a binary file into a C++ source with a byte array.
# Usage: cmake -DINPUT=<file> -DOUTPUT=<file.cpp> -DSYMBOL=<name> -P embed_file.cmake
file(READ "${INPUT}" HEX_CONTENT HEX)
file(SIZE "${INPUT}" INPUT_SIZE)
string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," BYTES "${HEX_CONTENT}")
string(REGEX REPLACE "(0x..,0x..,0x..,0x..,0x..,0x..,0x..,0x..,0x..,0x..,0x..,0x..,0x..,0x..,0x..,0x..,)" "\\1\n" BYTES "${BYTES}")
file(WRITE "${OUTPUT}" "// Generated from ${INPUT}. Do not edit.\n#include <cstddef>\nnamespace rx888 {\nextern const unsigned char ${SYMBOL}[] = {\n${BYTES}\n};\nextern const size_t ${SYMBOL}_size = ${INPUT_SIZE};\n}\n")
