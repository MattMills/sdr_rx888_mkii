# OpenWebRX+ with the RX888 mkII connector built from this repository.
#
# Build from the repository root:
#   docker build -f openwebrx/Dockerfile -t openwebrx-rx888 .
# or use openwebrx/docker-compose.yml.
#
# The connector is built in a stage derived from the same image, so it links
# against exactly the libowrx-connector, csdr, FFTW and libusb that run it.
ARG OWRX_IMAGE=slechev/openwebrxplus:latest

FROM ${OWRX_IMAGE} AS build
RUN apt-get update \
 && apt-get install -y --no-install-recommends build-essential cmake pkg-config \
        libusb-1.0-0-dev libfftw3-dev libowrx-connector-dev libcsdr-dev \
 && rm -rf /var/lib/apt/lists/*
COPY CMakeLists.txt /src/
COPY cmake /src/cmake
COPY firmware /src/firmware
COPY packaging/linux /src/packaging/linux
COPY src /src/src
COPY tools /src/tools
COPY openwebrx/rx888_connector.cpp /src/openwebrx/
RUN cmake -S /src -B /build -DCMAKE_BUILD_TYPE=Release -DRX888_BUILD_OWRX_CONNECTOR=ON \
 && cmake --build /build -j"$(nproc)" --target rx888_connector rx888_tool \
 && (/build/rx888_tool selftest > /build/selftest.txt; s=$?; grep -E "FAIL|SELFTEST" /build/selftest.txt; exit $s)

FROM ${OWRX_IMAGE}
COPY --from=build /build/rx888_connector /build/rx888_tool /usr/local/bin/
# OpenWebRX's "SDDC" device type runs sddc_connector: answer to that name too.
RUN ln -sf rx888_connector /usr/local/bin/sddc_connector
# On a shared host, run OpenWebRX and the connector it starts ahead of batch
# work: the start script launches OpenWebRX under nice (OWRX_NICE, default
# -10). Without CAP_SYS_NICE, nice warns and runs it at normal priority.
RUN sed -i -e 's|^\( *\)command openwebrx \$@$|\1nice -n "${OWRX_NICE:--10}" openwebrx $@|' \
           -e 's|^\( *\)\./openwebrx\.py \$@$|\1nice -n "${OWRX_NICE:--10}" ./openwebrx.py $@|' /run.sh \
 && [ "$(grep -c OWRX_NICE /run.sh)" = 2 ]
# Seed settings with the RX888 and its profiles. They are copied into a new,
# empty /var/lib/openwebrx volume only; an existing settings.json is kept.
COPY --chown=openwebrx:openwebrx openwebrx/settings.json /var/lib/openwebrx/settings.json
COPY --chown=openwebrx:openwebrx openwebrx/settings.json /owrx-init/var/settings.json
