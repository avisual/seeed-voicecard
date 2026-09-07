#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-only
#
# install-pi5.sh - ReSpeaker 6-Mic HAT on a Raspberry Pi 5.
#
# Builds and installs everything the HAT needs on a Pi 5:
#   1. the seeed-voicecard / ac10x codec modules from the repository root
#      (control plane: AC101 + 2x AC108 on i2c1, card "seeed8micvoicec"),
#   2. snd-pio-tdm from this directory (the audio data path on RP1 PIO,
#      card "seeed8micpio"),
#   3. the device-tree overlay, the modules-load.d entries, the config.txt
#      line and the codec-clock heartbeat unit.
#
# Run it from this directory on the Pi.  It does NOT reboot; the overlay only
# takes effect at boot.  NEVER `dtoverlay`/`dtoverlay -r` at runtime on
# 6.18.34 - it Oopses and wedges configfs until a power-cycle.
#
# Both drivers are installed through DKMS, so /etc/kernel/postinst.d/dkms
# rebuilds them for every kernel apt installs and a kernel upgrade cannot
# silently leave the HAT without a driver.  (If dkms is absent the script falls
# back to a plain build into extra/, which does NOT survive a kernel upgrade.)
#
# Prerequisites (Raspberry Pi OS Trixie 64-bit):
#   sudo apt install build-essential device-tree-compiler i2c-tools alsa-utils dkms
#   sudo apt install linux-headers-rpi-2712       # headers for the running kernel
#
# The PIO driver is built against the in-kernel rp1-pio API, which is not a
# stable ABI.  Pin the kernel you build against - with DKMS in place this is a
# safety belt (you choose when to move), not the only thing holding the card up:
#   sudo apt-mark hold linux-image-rpi-2712 linux-headers-rpi-2712 \
#        "linux-image-$(uname -r)" "linux-headers-$(uname -r)" \
#        "linux-headers-$(uname -r | sed 's/-rpi-.*/-common-rpi/')"
#
# To take a new kernel later: unhold the image AND headers together, apt
# upgrade, watch the DKMS build in the apt output, check `dkms status` lists
# both packages "installed" for the new kernel, reboot, verify the cards are
# back (i2cdetect / arecord -l / bus_alive), then re-hold.

set -eu
export PATH=$PATH:/usr/sbin:/sbin
cd "$(dirname "$0")"

PI5DIR=$(pwd)
TOPDIR=$(cd .. && pwd)
KR=$(uname -r)

# dt-bindings live in the *common* headers package of the RUNNING kernel, which
# is not the alphabetically first one: 6.18.34+rpt-rpi-2712 needs
# linux-headers-6.18.34+rpt-common-rpi.
HDR="/usr/src/linux-headers-$KR"
HDR_COMMON="/usr/src/linux-headers-$(echo "$KR" | sed 's/-rpi-.*/-common-rpi/')"
[ -d "$HDR/include" ]        || { echo "no headers for $KR at $HDR" >&2; exit 1; }
[ -d "$HDR_COMMON/include" ] || { echo "no common headers for $KR at $HDR_COMMON" >&2; exit 1; }

BOOTDIR=/boot/firmware
[ -d "$BOOTDIR" ] || BOOTDIR=/boot

# dkms_deploy <source dir> <package name> <package version>
# Copies the tree to /usr/src/<name>-<version> and runs add/build/install for
# the running kernel.  Any earlier registration of the same name+version is
# removed first, because DKMS keeps a symlink into /usr/src.
dkms_deploy() {
        src=$1; name=$2; ver=$3
        dest="/usr/src/$name-$ver"
        if dkms status -m "$name" -v "$ver" 2>/dev/null | grep -q .; then
                sudo dkms remove -m "$name" -v "$ver" --all
        fi
        sudo rm -rf "$dest"
        sudo mkdir -p "$dest"
        # source only - no build products, no .git
        (cd "$src" && tar cf - --exclude=.git --exclude='*.ko' --exclude='*.o' \
                --exclude='.*.cmd' --exclude='*.mod*' --exclude=Module.symvers \
                --exclude=modules.order .) | sudo tar xf - -C "$dest"
        sudo dkms add     -m "$name" -v "$ver"
        sudo dkms build   -m "$name" -v "$ver" -k "$KR"
        sudo dkms install -m "$name" -v "$ver" -k "$KR" --force
}

# Codec package version comes from the repository-root dkms.conf so the two
# never drift apart.
CODEC_VER=$(sed -n 's/^PACKAGE_VERSION="\(.*\)"/\1/p' "$TOPDIR/dkms.conf")
: "${CODEC_VER:=0.3}"

if command -v dkms >/dev/null 2>&1; then
        echo "== 1/6  codec modules via DKMS (seeed-voicecard $CODEC_VER) =="
        dkms_deploy "$TOPDIR" seeed-voicecard "$CODEC_VER"

        echo "== 2/6  snd-pio-tdm via DKMS (1.0) =="
        dkms_deploy "$PI5DIR" snd-pio-tdm 1.0
else
        echo "WARNING: dkms is not installed.  Falling back to a manual build:" >&2
        echo "         the modules will NOT be rebuilt on a kernel upgrade." >&2
        echo "         sudo apt install dkms and re-run to fix that." >&2

        echo "== 1/6  codec modules (snd-soc-ac108, snd-soc-seeed-voicecard) =="
        make -C "$TOPDIR" clean >/dev/null 2>&1 || true
        make -C /lib/modules/"$KR"/build M="$TOPDIR" modules
        sudo install -D -m 644 "$TOPDIR/snd-soc-ac108.ko" \
                "/lib/modules/$KR/extra/snd-soc-ac108.ko"
        sudo install -D -m 644 "$TOPDIR/snd-soc-seeed-voicecard.ko" \
                "/lib/modules/$KR/extra/snd-soc-seeed-voicecard.ko"

        echo "== 2/6  snd-pio-tdm =="
        make -C "$PI5DIR" KDIR=/lib/modules/"$KR"/build
        sudo install -D -m 644 "$PI5DIR/snd-pio-tdm.ko" \
                "/lib/modules/$KR/extra/snd-pio-tdm.ko"
fi

sudo depmod -a
if command -v dkms >/dev/null 2>&1; then
        dkms status
fi
for m in snd-soc-ac108 snd-soc-seeed-voicecard snd-pio-tdm; do
        echo "  $m -> $(modinfo -n $m)"
done

echo "== 3/6  module load order (/etc/modules-load.d) =="
printf 'i2c-dev\n' | sudo tee /etc/modules-load.d/i2c-dev.conf >/dev/null
printf 'snd-soc-seeed-voicecard\nsnd-soc-ac108\n' \
        | sudo tee /etc/modules-load.d/seeed-voicecard.conf >/dev/null
printf 'snd-pio-tdm\n' | sudo tee /etc/modules-load.d/snd-pio-tdm.conf >/dev/null

echo "== 4/6  device-tree overlay =="
cpp -nostdinc -I "$HDR/include" -I "$HDR_COMMON/include" \
    -undef -x assembler-with-cpp seeed-8mic-pi5-overlay.dts \
    | dtc -@ -H epapr -O dtb -o seeed-8mic-pi5.dtbo -
# refuse to install an overlay that will not apply to the board's base dtb
fdtoverlay -i "$BOOTDIR/bcm2712-rpi-5-b.dtb" -o /dev/null seeed-8mic-pi5.dtbo
sudo install -m 644 seeed-8mic-pi5.dtbo "$BOOTDIR/overlays/seeed-8mic-pi5.dtbo"

echo "== 5/6  $BOOTDIR/config.txt =="
# i2c1 for the codec control plane; the HAT overlay.  Deliberately NOT
# dtparam=i2s=on and NOT i2s-mmap: the overlay wires I2S1 itself and gives
# GPIO21 (SDO) to PIO.
for line in 'dtparam=i2c_arm=on' 'dtparam=i2c_arm_baudrate=400000' \
            'dtoverlay=seeed-8mic-pi5'; do
        grep -qxF "$line" "$BOOTDIR/config.txt" \
                || echo "$line" | sudo tee -a "$BOOTDIR/config.txt" >/dev/null
done

echo "== 6/6  seeed-codec-heartbeat.service =="
# The AC101 is the bus master and only clocks BCLK/WS while a capture stream is
# triggered on the seeed card, and the FIRST stream after boot must be 8-ch S32
# (it sets the 256-BCLK TDM frame).  This unit is that stream: an 8-ch S32
# arecord to /dev/null, held open forever.  Every consumer of seeed8micpio must
# be ordered After= / Requires= it.  Never `pkill arecord` by pattern.
sudo tee /etc/systemd/system/seeed-codec-heartbeat.service >/dev/null <<'UNIT'
[Unit]
Description=ReSpeaker codec clock heartbeat (8-ch capture keeps the AC101/AC108 clocks running)
After=sound.target systemd-modules-load.service
Wants=systemd-modules-load.service

[Service]
Type=simple
# Runs as root so it never depends on a particular login user being in the
# audio group.  If you prefer to drop privileges, replace these two lines with
#   User=<your user>
# (that user must be in the audio group).
DynamicUser=no
# The card is behind PCIe (rp1 clocks) and probes late - wait up to 20 s for it.
ExecStartPre=/bin/sh -c 'for i in $(seq 1 40); do [ -e /proc/asound/seeed8micvoicec ] && exit 0; sleep 0.5; done; echo "seeed8micvoicec card not present after 20s" >&2; exit 1'
ExecStart=/usr/bin/arecord -D hw:CARD=seeed8micvoicec,DEV=0 -f S32_LE -c 8 -r 16000 -t raw /dev/null
Restart=always
RestartSec=2
Nice=-5

[Install]
WantedBy=multi-user.target
UNIT
sudo systemctl daemon-reload
sudo systemctl enable seeed-codec-heartbeat.service

sync
cat <<'DONE'

Installed.  Reboot to apply the overlay:
    ps -eo stat,comm | awk '$1~/D/'      # must be empty
    sudo systemctl reboot

After the reboot:
    i2cdetect -y 1                       # must show 0x1a, 0x35, 0x3b
                                         # (0x1a missing = reseat the FPC ribbon)
    arecord -l | grep -E 'seeed8micvoicec|seeed8micpio'
    systemctl status seeed-codec-heartbeat
    dkms status                          # both packages "installed" for this kernel

Then set the mixer once and save it (values for the 6-Mic HAT):
    amixer -c seeed8micvoicec cset name='ADC1 PGA gain' 12   # ... ADC2..ADC6 too
    amixer -c seeed8micvoicec cset name='ADC7 PGA gain' 0    # AC101 loopback slots:
    amixer -c seeed8micvoicec cset name='ADC8 PGA gain' 0    # line level, +12 dB clips
    amixer -c seeed8micvoicec cset name='DAC volume' 160
    amixer -c seeed8micvoicec sset 'Speaker' 18
    amixer -c seeed8micvoicec cset name='Speaker Amp Switch' on   # GPIO17, default off
    sudo alsactl store

Capture and playback then run on the PIO card:
    arecord -D plughw:CARD=seeed8micpio -c8 -r16000 -f S16_LE -d5 test.wav
    aplay   -D plughw:CARD=seeed8micpio test.wav
DONE
