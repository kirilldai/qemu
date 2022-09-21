#!/bin/bash -ex

QEMU_DIR=$(dirname ${BASH_SOURCE})
X=$(readlink -f "$0")
Y=$(readlink -f "${BASH_SOURCE}")
echo "#${QEMU_DIR}# #${BASH_SOURCE}# #${X}# #${Y}#"

# create
rm -rf build/
mkdir build/
cd build

# make qemu
${QEMU_DIR}/configure --target-list="x86_64-softmmu"
make -j8

# copy relevant build outputs
mkdir -p qemu/pc-bios
cp qemu-system-x86_64 qemu
cp pc-bios/*.bin pc-bios/*.rom qemu/pc-bios

# create BUILD.go file
cat <<EOT > qemu/BUILD.go
package qemu

var Qemu = in("qemu-system-x86_64")

var Bios = in("pc-bios")

var QemuCommand = []string { Qemu.Absolute(), "-L", Bios.Absolute() }
EOT

# package everything up
tar -czf qemu.tar.gz qemu/

echo "Done. Output is in 'build/qemu.tar.gz'."
