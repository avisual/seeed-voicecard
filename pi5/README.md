# `pi5/` — ReSpeaker 6-Mic HAT on the Raspberry Pi 5

The 6-Mic HAT does not work on a Raspberry Pi 5 with the stock driver, and no amount of
device-tree work makes it: the Pi 5 has no TDM-capable I2S controller. This directory carries
`snd-pio-tdm`, an out-of-tree ALSA card that moves both audio directions onto RP1's PIO block
instead, so the HAT works on a Pi 5 with its six microphones, its AC101 loopback and its
speaker output.

The codec control plane (AC101 + 2× AC108 on i2c1, card `seeed8micvoicec`) is still the
`seeed-voicecard` / `ac10x` driver in the repository root, with the Pi 5 changes from the
previous commit. This card only carries clocks and mixer controls on a Pi 5; the audio data
path is the separate PIO card `seeed8micpio`.

---

## Why: RP1 I2S cannot do TDM

The HAT's AC101 is the **bus master**. It drives a DSP_A/TDM frame: BCLK 4.096 MHz on GPIO18, a
**1-BCLK-wide** frame sync on GPIO19 every 256 BCLK, and eight 32-bit slots per frame on GPIO20
(six microphones plus two AC101 loopback channels). Every Pi before the Pi 5 handles this with
the BCM SoC's `bcm2835-i2s`, which supports the frame layout.

The Pi 5's audio pins belong to RP1, whose I2S blocks are Synopsys DesignWare I2S. Raspberry Pi's
own documentation is explicit — white paper **RP-009699-WP-1** (20 Feb 2026), p.5:

> *"TDM is not supported on any Raspberry Pi SBCs"*

and the kernel maintainer says the same in **raspberrypi/linux#6568**. In practice on
6.18.34+rpt-rpi-2712:

* the DesignWare **receiver** can be coaxed into reading this bus (it is what the codec-clock
  heartbeat uses), but
* the DesignWare **transmitter never shifts a single word** in consumer mode against a
  pulse-width frame sync — there is no playback at all, and
* `dwc-i2s` "PIO" (interrupt-driven FIFO) mode is not a way out either: `dw_pcm_hardware` is
  `channels_min = channels_max = 2`, so both `-c 8` capture and `-c 8` playback fail at
  `hw_params` with `invalid channels number`. That path is recorded, with the exact
  `interrupts = <15 4>;` line, in a comment in `seeed-8mic-pi5-overlay.dts`.

RP1 does, however, have the **PIO block** — the same programmable I/O as the RP2040, with a
200 MHz clock against a 4.096 MHz BCLK (≈ 48 state-machine cycles per bit) and access to every
header GPIO. Framing a TDM bus is exactly what it is for.

## How: two PIO state machines with their own DMA rings

`snd-pio-tdm` is a platform driver on `compatible = "seeed,snd-pio-tdm"` that claims two RP1 PIO
state machines:

* **SM0 = RX.** Waits for the frame sync, then clocks in exactly 256 bits per frame, autopushing
  every 32. Framing is done *in the state machine*, so the DMA stream **is** 8-channel S32_LE —
  no CPU unpacking, and channel rotation is structurally impossible. The loop re-synchronises to
  the frame sync every frame.
* **SM1 = TX.** Waits for the frame sync and drives slots 0/1 (the AC101's timeslot 0, L/R) on
  GPIO21, 32 bits each, autopulling every 32; the line is held low elsewhere in the frame.

Each direction owns a **dw-axi-dmac channel** (requested from the driver's own DT node with the
"heavy"/burst-8 flag) running a hardware cyclic ring straight into the ALSA managed buffer. The
PIO DMACTRL DREQ threshold is set to 8 words on a joined 16-deep FIFO in both directions
(`0x80000108`); the kernel defaults were measured to duplicate or drop words against the 8-word
DMAC burst.

Every `pio_*` call is a sleeping RP1 firmware mailbox RPC, so the PCM is `nonatomic` and
`snd_pcm_period_elapsed()` is called from a work item.

**Card `seeed8micpio`:** capture 8 ch × S32_LE @ 16 kHz, playback 2 ch × S32_LE @ 16 kHz.
`plughw:CARD=seeed8micpio` bridges the usual S16_LE / mono consumers.

**Slot map (measured with a tone in the loopback):**

| Slot | Source |
|------|--------|
| 0, 1 | AC101 loopback — the L+R speaker mix, an anti-phase pair (correlation −0.995) |
| 2–7  | the six microphones |

Note this is *not* the Pi 3 slot map: an echo-canceller's reference channel is slot 0 (or 1) and
the first mic is slot 2.

The full spec — PIO programs word by word, the DMA design, the atomic-context rules, the ALSA
constraints and the device tree — is in `DESIGN.md`.

## Results

* **Bit-exact self-capture.** With the TX state machine driving a known pattern into slots 0/1
  and the RX state machine reading the same bus back, the capture is what was written, frame
  aligned: a 4 s `hw:` capture is 64000 frames exactly, and the low byte of every 32-bit sample
  ORs to 0 (the codec is 24-bit in a 32-bit slot).
* **BCLK measured at 4.0955 MHz** on GPIO18 (nominal 4.096 MHz; AC101 PLL from the 24 MHz
  MCLK on GPIO4, BCLK_DIV ÷6, LRCK_DIV 256 BCLK = 8 × 32 per frame).
* **Playback works** — 1 kHz tones and then speech, audible from the HAT's J2 speaker.
  Electrically: the tone appears in loopback slots 0/1 at 1000.0 Hz, −33 dBFS, 119 dB above the
  slot noise floor.
* **60 s full duplex:** 0 faults / 0 forced XRUNs / 0 stalls. Drift between the two cards'
  `hw_ptr` was ±2 frames, which is the ±125 µs skew of reading the two `/proc` files in
  sequence — no accumulation. CPU ≈ 2 % softirq system-wide; `arecord`/`aplay` ≈ 0 %.
* **8 h soak: 0 faults, 0 XRUNs, 0 stalls, 0 restarts** — 462 M frames each direction, drift
  −6 frames over 8 h, max SoC temperature 55.4 °C.
* Recovery is deterministic: stopping the codec-clock heartbeat forces an XRUN at 500 ms, the
  consumer's re-`prepare` returns `-EIO` while the bus is down, and a fresh stream is clean once
  the clocks return. 15/15 simultaneous capture+playback starts clean. `rmmod`/`insmod` with
  streams closed is silent and releases both DMA channels and GPIO21.

## Install

```sh
sudo apt install dkms # once
./install-pi5.sh      # from this directory, on the Pi
sudo systemctl reboot # the overlay only applies at boot
```

It installs the codec modules from the repository root and `snd-pio-tdm` from here **as DKMS
packages** (`seeed-voicecard` and `snd-pio-tdm` 1.0 — sources copied to `/usr/src/<name>-<ver>`,
then `dkms add`/`build`/`install`), writes the `modules-load.d` entries, builds and validates the
overlay with `cpp | dtc` (plus `fdtoverlay` against the board's base dtb), adds the `config.txt`
lines, and installs `seeed-codec-heartbeat.service`. It prints the mixer settings to apply once
and `alsactl store`. Prerequisites and the `apt-mark hold` line are in the script's header
comment. Without `dkms` present it falls back to a plain build into `extra/` and says so.

### Why DKMS

`snd-pio-tdm` is built against the in-kernel rp1-pio API, which is not a stable ABI, so the
kernel is normally pinned with `apt-mark hold`. Both `dkms.conf` files set `AUTOINSTALL="yes"`,
which means `/etc/kernel/postinst.d/dkms` rebuilds and installs both drivers for any kernel apt
puts on the box — the hold becomes a safety belt (you choose when to move) instead of the only
thing keeping the card alive. DKMS installs to `/lib/modules/<kr>/updates/dkms/*.ko.xz` on
Debian (it overrides `DEST_MODULE_LOCATION`), which `depmod` ranks above `extra/`; it also
archives any hand-built `extra/` copy it displaces and restores it on `dkms uninstall`.

Checks that do not require reloading anything:

```sh
dkms status                      # both packages "installed" for the running kernel
modinfo -n snd-pio-tdm           # resolves to updates/dkms, not a stale extra/ copy
modinfo -F srcversion snd-pio-tdm
cat /sys/module/snd_pio_tdm/srcversion   # must equal the line above = same source
```

Taking a new kernel: unhold the image **and** headers together, `apt upgrade`, watch the DKMS
build in the apt output, confirm `dkms status` lists both packages `installed` for the new
kernel, reboot, verify the cards (`i2cdetect -y 1`, `arecord -l`, `bus_alive`), then re-hold. If
a DKMS build fails, re-hold and do not reboot — the new kernel would come up without the HAT.

## Runtime contract

* **The codec clocks only run while an 8-ch S32 capture is open on `seeed8micvoicec`**, and the
  **first stream after boot must be 8-ch S32** — it is what sets the 256-BCLK TDM frame. That is
  `seeed-codec-heartbeat.service`. Every consumer of `seeed8micpio` must be `After=` /
  `Requires=` that unit. Never `pkill arecord` by pattern.
* `.prepare` runs a bus-alive probe (RX SM for 1.5 ms; the FIFO must fill to 1..16 words) and
  returns `-EIO` when BCLK/WS are absent. A 250 ms watchdog forces an XRUN after 500 ms without
  DMA progress; the consumer's re-prepare then either resumes (clocks back) or gets `-EIO` (still
  down) → exit → systemd restart. `substream->wait_time` is raised to 3 s so ALSA's own
  `wait_for_avail` timeout cannot pre-empt this path. The watchdog also polls the DMA cookie: a
  cyclic ring that has left `DMA_IN_PROGRESS` (the dw-axi-dmac error path) is XRUNed at once
  (`*_dma_errors`), and `.pointer` reports `SNDRV_PCM_POS_XRUN` in that state.
* **Period constraints:** `periods_min = 3` and `period_size >= 80 frames` (5 ms) in both
  directions. Smaller requests are negotiated up by alsa-lib. Reason: the rpi dw-axi-dmac
  re-validates each period's LLIs from the period IRQ, so the IRQ-latency budget is
  (periods − 1) periods; 2 × 1 ms was one late IRQ away from a dead ring.
* **Playback:** the AC101 DAC path is powered by the **codec driver** (`snd-soc-ac108`
  `dac_always_on`, applied at heartbeat start). The J2 speaker is audible only with
  `amixer -c seeed8micvoicec cset name='Speaker Amp Switch' on` (GPIO17, default off). The
  loopback is the AC101 HP path, upstream of the amp, so playback can be verified electrically
  with the amp off.
* **Mic gains:** `ADC1–6 PGA gain` 12 for the microphones; `ADC7/ADC8 PGA gain` **0** — those
  two are the AC101 loopback slots and they are line level, where +12 dB saturates the AC108
  input (clips at ~0.11 FS).
* The first RX frame after START is DMA start-up junk (8 identical words, 62.5 µs; inherent to
  arming DMA before the state machine). It is left in the stream — consumers should skip one
  frame.

## Caveats

* **The kernel is pinned.** The driver is built against the in-kernel `rp1-pio` API
  (`<linux/pio_rp1.h>`), which is not a stable ABI, on `6.18.34+rpt-rpi-2712`. Hold that kernel
  (`apt-mark hold`, see the script header) and expect to revisit the driver on an upgrade.
* **Out of tree**, and not a general TDM-over-PIO framework: it is fixed at 16 kHz, 32-bit
  slots, 8 slots per frame, and it assumes the codec is the bus master.
* **One board tested.** ReSpeaker 6-Mic HAT (2× AC108 + AC101) in codec-master mode, 4.096 MHz
  BCLK, 256-bit frames, on a Pi 5. Other HATs, rates and slot counts are unexercised.
* **Never `dtoverlay` or `dtoverlay -r` at runtime** on 6.18.34 — it Oopses and wedges configfs
  until a power-cycle. The overlay applies at boot only.
* **Reloading the codec modules needs the card unbound first** (ASoC holds a module reference):
  unbind, `rmmod`, `modprobe`. Reloading `snd-pio-tdm` alone is fine with its streams closed.
* The AC101 headphone/loopback path needs a forced 0→1 edge on the DC-removal enable bits after
  a rebind — the register's reset default already reads "set", so `regmap_update_bits()` skips
  the write and the output stage comes up dead with every enable bit reading correct. That fix
  lives in `../ac101.c` (previous commit), not here.
* Two card names, easy to confuse: **`seeed8micvoicec`** is the codec control card (clocks, PGA
  and DAC mixer controls, `Speaker Amp Switch`); **`seeed8micpio`** is the audio data path
  (`arecord`/`aplay` go here).

## Files

| File | Purpose |
|------|---------|
| `snd-pio-tdm.c` | the driver (platform driver on `compatible = "seeed,snd-pio-tdm"`) |
| `Makefile` | `make` builds against `/lib/modules/$(uname -r)/build`; `make install` → `extra/` + `depmod` (dev loop; the shipping path is DKMS) |
| `dkms.conf` | DKMS packaging for `snd-pio-tdm` 1.0, `AUTOINSTALL=yes` — rebuilt for every kernel apt installs |
| `seeed-8mic-pi5-overlay.dts` | the full HAT overlay: seeed control-plane fragments + I2S1 trimmed to GPIO18-20 + `pio_sdo_pin` (GPIO21) + the `snd-pio-tdm` node under `&rp1` |
| `install-pi5.sh` | the whole install: both drivers via DKMS, overlay, `modules-load.d`, `config.txt`, heartbeat unit |
| `DESIGN.md` | implementer's spec — PIO programs, DMA, locking, ALSA constraints, device tree |
| `test/analyse_pio.py` | numpy analyser for raw 8-ch S32 captures (RMS/slot, low-byte OR, tone peak + SNR) |

## sysfs

Under `/sys/bus/platform/devices/*:rp1:snd-pio-tdm/`:

`rx_frames rx_periods rx_faults rx_restarts rx_xruns_forced rx_dma_errors rx_sm rx_prog_origin
rx_dma_chan rx_dma_burst rx_dmactrl rx_running rx_fifo_level` (and the same `tx_*`), plus
`bus_stalls`, `bus_alive` (last probe: 1/0/−1) and `gpios`.

`*_faults` counts RX-FIFO-full / TX-FIFO-empty snapshots seen while running (one mailbox call
per watchdog tick, only while `running` under `tdm->lock`, and only a reply of exactly 0/1
counts). `*_restarts` = state machine re-initialised by `.prepare` after a fault.
`*_xruns_forced` = watchdog stops (stall or dead ring). `*_dma_errors` = a cyclic ring that left
`DMA_IN_PROGRESS` while running (the dmac `Bad descriptor` path) — expected 0.

## Module parameters

* `rx_origin` / `tx_origin` — where to load each PIO program. Default −1 lets the firmware place
  it; the RP1 loader relocates program-relative JMPs (measured), so a fixed origin is not needed
  and the 30 words no longer fit two fixed slots anyway.
* `rx_dreq` / `tx_dreq` — the DMACTRL DREQ threshold, default 8 with joined 16-deep FIFOs. The
  kernel defaults were measured to duplicate or drop words.
* `bus_probe` — set to 0 to disable the `-EIO` bus-alive gate in `.prepare`.

## Static checks

Built with `make C=1 CF="-Wsparse-all"` against 6.18.34: **0 sparse warnings**. `checkpatch.pl --file`
(rpi-6.18.y): 0 errors of substance — the two remaining notes are the usual false positives for
sysfs attribute-declaration macros (`PIO_TDM_STREAM_ATTRS` declares attributes, so it cannot be a
`do { } while (0)`; `PIO_TDM_STREAM_ATTR_LIST` is an initialiser list, so it cannot be parenthesised).
Not fuzzed and not formally audited: it is a kernel module, so treat bugs as a local crash risk. It
takes no untrusted input — its only interfaces are standard ALSA calls, root-only module parameters
and read-only sysfs counters.
