#!/bin/sh
# Build a .deb of the plugin from a configured+built CMake tree.
#   make_deb.sh <build_dir> <version> <output.deb>
set -e
BUILD_DIR=$1
VERSION=$2
OUT=$3
ARCH=$(dpkg --print-architecture)
STAGE=$(mktemp -d)

DESTDIR=$STAGE cmake --install "$BUILD_DIR" --prefix /usr

DOC=$STAGE/usr/share/doc/rx888-mkii-source
mkdir -p "$DOC"
SRC=$(dirname "$0")/../..
cp "$SRC/README.md" "$SRC/THIRD_PARTY_NOTICES.md" "$DOC/"
cp "$SRC/LICENSE" "$DOC/copyright"
[ -f "$BUILD_DIR/BUILD_INFO.txt" ] && cp "$BUILD_DIR/BUILD_INFO.txt" "$DOC/"

mkdir -p "$STAGE/DEBIAN"
cat > "$STAGE/DEBIAN/control" <<EOF
Package: rx888-mkii-source
Version: $VERSION
Architecture: $ARCH
Maintainer: Matt Mills <mmills@2bn.net>
Depends: sdrpp, libusb-1.0-0, libfftw3-single3
Section: hamradio
Priority: optional
Homepage: https://github.com/MattMills/sdr_rx888_mkii
Description: RX888 mkII source module for SDR++
 SDR++ plugin for the RX888 mkII software defined radio: HF direct sampling
 and the R828D VHF/UHF tuner, with on-the-fly FX3 firmware upload.
 Includes rx888_tool and a udev rule granting access to the receiver.
EOF
cat > "$STAGE/DEBIAN/postinst" <<'EOF'
#!/bin/sh
set -e
if command -v udevadm >/dev/null 2>&1; then
    udevadm control --reload-rules >/dev/null 2>&1 || true
    udevadm trigger --subsystem-match=usb >/dev/null 2>&1 || true
fi
EOF
chmod 755 "$STAGE/DEBIAN/postinst"

dpkg-deb --root-owner-group --build "$STAGE" "$OUT" 2>/dev/null || dpkg-deb --build "$STAGE" "$OUT"
rm -rf "$STAGE"
dpkg-deb --info "$OUT"
dpkg-deb --contents "$OUT"
