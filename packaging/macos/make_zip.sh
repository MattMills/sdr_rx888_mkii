#!/bin/bash
# Package the plugin for SDR++.app with SDR++'s own bundling helpers, so that
# its library references are rewritten the same way as the in-tree modules.
#   make_zip.sh <sdrpp_source_dir> <plugin_build_dir> <output.zip>
set -e
SDRPP=$1
BUILD=$2
OUT=$3
SRC=$(cd "$(dirname "$0")/../.." && pwd)
STAGE=$(mktemp -d)/RX888

source "$SDRPP/macos/bundle_utils.sh"
mkdir -p "$STAGE/Contents/Plugins" "$STAGE/Contents/Frameworks"
bundle_install_binary "$STAGE" "$STAGE/Contents/Plugins" "$BUILD/rx888_mkii_source.dylib"

# install_name_tool invalidates signatures; re-sign ad hoc (required on arm64).
for f in "$STAGE"/Contents/Plugins/*.dylib "$STAGE"/Contents/Frameworks/*.dylib; do
    [ -f "$f" ] && codesign --force -s - "$f"
done

cp "$BUILD/rx888_tool" "$STAGE/" 2>/dev/null || true
cp "$SRC/packaging/macos/INSTALL.txt" "$SRC/README.md" "$SRC/LICENSE" "$SRC/THIRD_PARTY_NOTICES.md" "$STAGE/"
[ -f "$BUILD/BUILD_INFO.txt" ] && cp "$BUILD/BUILD_INFO.txt" "$STAGE/"

echo "Plugin dependencies:"
otool -L "$STAGE/Contents/Plugins/rx888_mkii_source.dylib"
(cd "$(dirname "$STAGE")" && zip -r -y "$OUT" RX888)
