#!/bin/bash
set -e

VERSION="v1.8.1"
BIN_DIR="../../bin"

# Funktion der pakker et specifikt format
package_format() {
    FORMAT_NAME=$1
    FILE_NAME=$2

    FOLDER_NAME="DrumCloud-${FORMAT_NAME}"
    ZIP_NAME="DrumCloud-Fuimadane-${FORMAT_NAME}-Linux-${VERSION}.zip"

    echo "📦 Pakker ${ZIP_NAME}..."

    # Lav mappen
    mkdir -p "${FOLDER_NAME}"

    # Kopier fælles tekst- og script-filer
    cp BUILD.md INSTALL.txt LICENSE.txt README.txt build.sh install.sh "${FOLDER_NAME}/"

    # Kopier selve plugin-filen (vi bruger -r hvis det er en mappe som LV2)
    cp -r "${BIN_DIR}/${FILE_NAME}" "${FOLDER_NAME}/"

    # Zip mappen (gem output for at holde terminalen pæn)
    zip -r -q "${ZIP_NAME}" "${FOLDER_NAME}"

    # Slet den midlertidige mappe
    rm -rf "${FOLDER_NAME}"
}

echo "Starter pakning af DrumCloud version ${VERSION}..."

# Pak alle 4 formater
package_format "CLAP" "d_drumcloud.clap"
package_format "VST3" "d_drumcloud.vst3"
package_format "LV2"  "d_drumcloud.lv2"
package_format "VST2" "d_drumcloud-vst.so"

echo "✅ Færdig! Dine 4 zip-filer ligger nu klar i mappen."
