# `snd-pio-tdm` — full-duplex ALSA card on RP1 PIO (Pi 5 + ReSpeaker 6-Mic HAT)

Implementer's spec for the driver in this directory; `README.md` has the results and the
runtime contract. Every claim marked **VERIFIED** was read from the installed 6.18.34 headers,
the running device tree, or the referenced driver source; **VERIFY** marks an item that the
userspace piolib proof or the first module load had to confirm. Kernel:
`6.18.34+rpt-rpi-2712`, pinned with `apt-mark hold`.

## 0. Decisions at a glance

| # | Decision | Why (one line) |
|---|----------|----------------|
| D1 | **PIO-side framing**: RX SM waits for WS, then clocks in exactly 256 bits/frame; the DMA stream *is* 8-ch S32_LE. | Zero CPU unpacking; channel rotation is *structurally impossible* (§1.3). |
| D2 | **Own dmaengine channels + hardware cyclic ring** (`dma_request_chan` from our DT node, `dmaengine_prep_dma_cyclic`) — NOT `pio_sm_xfer_data()` descriptor chains. | Gapless LLI ring (no CPU between periods); we control `maxburst` and the DMACTRL threshold; standard ALSA pattern proven on this DMAC by the I2S1 heartbeat. `pio_sm_xfer_data` ring = documented fallback (Appendix B). |
| D3 | RX SM claims **SM0**, TX SM **SM1** (explicit `pio_sm_claim`), programs at **fixed origins 0 and 16**. | DMA handshake ids are per-SM (`RP1_DMA_PIO_CH0_RX`, `RP1_DMA_PIO_CH1_TX`); neither the kernel nor the firmware is known to relocate JMP targets (§1.5). |
| D4 | `pcm->nonatomic = true`; ALL `pio_*` control calls only from `.prepare/.trigger/.hw_free/.close` and workqueue; **`snd_pcm_period_elapsed()` from a work item**, never from the DMA callback. | Every `pio_*` call is a sleeping firmware mailbox RPC; in nonatomic mode the PCM stream lock is a *mutex*, so period_elapsed from the DMA tasklet would sleep in atomic context (§3). |
| D5 | GPIO21 (SDO) taken from I2S1 by **trimming I2S1's pinctrl group to 18–20** and muxing 21 to the `"pio"` pinctrl function on our node (+ `pio_gpio_init(21)` + SM pindir out). 18/19/20 are read by PIO **without** requesting them (pinctrl-rp1 is `strict`). | §1.4, §5. |
| D6 | DMA channels requested with the **heavy flag** `(RP1_DMA_PIO_CHn_xx \| 0x100)` → burst-8 channels dma2chan0/1; DMACTRL RX = `0x80000108`, TX = `0x8000010{8|0}` (§2.4). | Matches how the base DT wires rp1-pio's own channels (VERIFIED: pio `dmas` = 0x138…0x13f). |
| D7 | Card `seeed8micpio`, one PCM: capture 8ch/S32_LE/16k fixed, playback 2ch/S32_LE/16k fixed, period_bytes step 32, periods ≥ 2 integer, 20 ms target. | §4. |
| D8 | Heartbeat (`seeed-codec-heartbeat.service`, see `install-pi5.sh`) stays the codec-clock trigger; PIO card is the audio path; bus-alive probe in `.prepare`, no-progress watchdog → `snd_pcm_stop_xrun()`. | §6. |

---

## 1. PIO programs

### 1.0 Bus facts used (from the plan; do not re-derive)
BCLK 4.096 MHz on GPIO18 (AC101 master). WS on GPIO19 = 1-BCLK-wide pulse every 256 BCLK.
DSP_A: with `e` = the BCLK **rising** edge at which WS first samples high, slot 0 bit 0 is sampled
at rising edge `e+1`; bits run `e+1 … e+256`; the next WS pulse is high at `e+256`, i.e. the
**WS pulse overlaps the last bit (slot 7 LSB) of the previous frame**. SDI = GPIO20 (codec→Pi,
8 × 32-bit slots, MSB first). SDO = GPIO21 (Pi→codec); the AC101 DAC reads *timeslot 0* = the first
64 BCLK = our slots 0 (L) and 1 (R) (`ac101.c:545-551`, `AIF1_DACDAT_CTRL DA0`), 24-bit word in a
32-bit slot, MSB-justified (`aif1_word_size = 24`, `aif1_slot_size = 32`, `ac101.c:1047-1124`).
PIO clock 200 MHz (`clock_get_hz(clk_sys)`), clkdiv 1.0 → 48.8 SM cycles per BCLK, 24.4 per half.
`wait gpio` is absolute-GPIO indexed and level-sensitive; `jmp pin` uses `EXECCTRL.JMP_PIN`
(absolute GPIO; `sm_config_set_jmp_pin()` exists in the installed header — VERIFIED).

### 1.1 RX program (SM0, origin 0, 13 instructions) — encodings hand-derived, see §1.6

```
; in_base = 20 (GPIO20 → in pins bit0), jmp_pin = 19, shift LEFT, autopush @32, FIFO join RX (16 deep)
 0: 0x2013  resync: wait 0 gpio 19      ; guarantee we catch a *fresh* WS rising edge
 1: 0x2093          wait 1 gpio 19      ; WS rising
 2: 0x2092          wait 1 gpio 18      ; consume/pass BCLK edge e (WS is high here) — data starts at e+1
 3: 0xe047  frame:  set y, 7            ; 8 slots
 4: 0xe03f  slot:   set x, 31           ; 32 bits
 5: 0x2012  bit:    wait 0 gpio 18
 6: 0x2092          wait 1 gpio 18      ; rising edge = sample point (proven correct in Gate 1)
 7: 0x4001          in pins, 1          ; autopush every 32 bits → exactly 8 words per frame
 8: 0x0045          jmp x--, bit
 9: 0x0884          jmp y--, slot [8]   ; 8-cycle delay: puts the WS check ~70 ns after the edge (mid high-half)
10: 0x00c3          jmp pin, frame      ; WS high on the 256th edge ⇒ aligned ⇒ next frame, no gap
11: 0xc000          irq set 0           ; misaligned: flag it (harmless on this kernel, countable on newer ones)
12: 0x0000          jmp resync          ; …and re-acquire WS
```
Config: `sm_config_set_in_pins(&c,20); sm_config_set_jmp_pin(&c,19); sm_config_set_in_shift(&c,false,true,32);
sm_config_set_fifo_join(&c,PIO_FIFO_JOIN_RX); sm_config_set_clkdiv_int_frac(&c,1,0); sm_config_set_wrap(&c,0,12);`
Initial PC = 0. No `pio_gpio_init()` on 18/19/20 (they stay I2S1's; PIO reads the pads — proven).

**Timing (200 MHz, 2-cycle input synchroniser ≈ 10 ns):** between the `in` at edge `e_k` and the
next rising edge there are 244 ns. Worst path per slot boundary: in(5) + jmp x--(5) + jmp y-- with
delay(45) + set x(5) ≈ 60 ns before `wait 0 gpio 18`; because `wait 0` is level-sensitive, arriving
before *or after* the falling edge is fine — only arriving after the *next rising edge* would lose a
bit, and the budget for that is the full 244 ns. The `jmp pin` at ~70 ns after the 256th edge sits
mid-high-half, so it reads WS high whether the AC101 changes WS on the falling edge (WS high from
−122 ns to +122 ns) or on the rising edge (high from ≈+5 ns to +250 ns). Likewise the prologue
`wait 1 gpio 19 → wait 1 gpio 18` lands the first `in` at `e+1` under both conventions (§1.6 note).

**Why 256 = 8 × 32 nested loops:** `set` is 5-bit (max 31). Outer Y=7 (8 iters) × inner X=31 (32
iters) = 256 `in`s = exactly 8 autopushes. Alternative single-loop X=255 needs `set y,31; in y,5;
in y,3; mov x,isr; mov isr,null` (5 instr) — no shorter, and touches the ISR; rejected.

### 1.2 TX program (SM1, origin 16, 12 instructions)

```
; out_base = 21 count 1, set_base = 21 count 1, shift LEFT (MSB first), autopull @32, FIFO join TX (16 deep)
16: 0xe000  idle:   set pins, 0         ; SDO low while not in slots 0/1 (slots 2-7 undriven by us)
17: 0x2013          wait 0 gpio 19
18: 0x2093          wait 1 gpio 19      ; WS rising
19: 0x2092          wait 1 gpio 18      ; edge e (codec samples WS). DSP_A: bit 0 must be valid at e+1
20: 0xe041          set y, 1            ; 2 slots (L, R) = the AC101's timeslot 0
21: 0xe03f  slot:   set x, 31
22: 0x2012  bit:    wait 0 gpio 18      ; drive on the FALLING edge (opposite to the codec's rising-edge sample)
23: 0x6001          out pins, 1         ; autopull @32 → exactly 2 words per frame
24: 0x2092          wait 1 gpio 18      ; codec samples here
25: 0x0056          jmp x--, bit
26: 0x0095          jmp y--, slot
27: 0x2012          wait 0 gpio 18      ; hold bit 63 through its sampling edge, then .wrap → set pins,0
```
Config: `sm_config_set_out_pins(&c,21,1); sm_config_set_set_pins(&c,21,1); sm_config_set_out_shift(&c,false,true,32);
sm_config_set_fifo_join(&c,PIO_FIFO_JOIN_TX); sm_config_set_clkdiv_int_frac(&c,1,0); sm_config_set_wrap(&c,16,27);`
Initial PC = 16. Before enabling: `pio_gpio_init(client,21)` (funcsel → PIO), `pio_sm_set_pins_with_mask(client,1,0,BIT(21))`,
`pio_sm_set_consecutive_pindirs(client,1,21,1,true)` — **the pad's output enable comes from the SM
PINDIRS; without it the pin floats** (same sequence as `rp1-pio-uart.c:552-553` and `pwm-pio-rp1.c`
`pwm_program_init`). **VERIFY (userspace piolib TX proof):** this sequence alone drives the pad; if not, add
`pio_gpio_set_oeover(client,21,GPIO_OVERRIDE_HIGH)`.

Setup/hold: data changes ≈ 15 ns after the falling edge → ≈ 107 ns setup before the codec's rising
edge, ≥ 15 ns hold after it. Standard I2S behaviour.

**Underrun policy — blocking autopull, deliberately.** `pull noblock` would replay the stale OSR
(a DC/tone artefact) and `pull ifempty` semantics don't compose with autopull. With blocking
autopull an empty FIFO stalls the SM at `out`; it misses BCLK edges, later emits the rest of the word
into slots the AC101 ignores (2–7) or at worst one wrong sample into slot 0/1, finishes its 64 bits,
`set pins,0`, then **waits for the next WS** — so an underrun can never shift the frame permanently:
damage ≤ 1 frame, self-healing. With the hardware cyclic ring (§2) the DMAC never starves the FIFO
(16-word joined FIFO = 8 frames = 500 µs slack against a hardware DREQ latency of ~µs); an ALSA-level
underrun replays the ring and is reported to userspace as an XRUN by the pointer logic — standard.

**DSP_A vs DSP_B for the DAC:** the seeed card configures the AC101 DAI as `dsp_a` (1-BCLK delay). If
the tone test shows a 6 dB / sign anomaly, the DAC wants DSP_B → delete instruction 19 (drive bit 0 at
the falling edge *before* e). One-line change; VERIFY with the tone.

### 1.3 Why PIO framing beats "capture WS+DATA, de-interleave in the kernel"
* **CPU:** WS+DATA = 2 bits/BCLK → 1 MB/s of raw bits that must be bit-sliced into 8 × 32-bit words
  per 256 bits with a WS search and carry-over across period boundaries: ≈ 4.1 M bit-ops/s ≈ 3–5 % of
  an A76 core in an optimised C slicer, plus a second buffer and a copy into the ALSA ring (DMA can't
  land decoded data directly). PIO framing: **0 CPU**, DMA lands S32_LE frames straight in the ALSA buffer.
* **Complexity:** no intermediate ring, no slicer state machine, no partial-frame carry, no
  "which bit is WS" versioning between tools.
* **Alignment safety is *better*, not worse:** the SM pushes in units of whole frames (8 words) —
  a stall (RX FIFO full → `in` blocks; it does not drop words) or a WS glitch can only *lose or
  garble whole frames*; it can never rotate channel k into position k±1 in the DMA stream, because the
  number of words per WS-to-WS cycle is fixed at 8 by the program. **Channel rotation, the failure
  that would silently gut AEC, is impossible by construction.** The software-framing design was
  chosen in Phase 0 to make rotation *detectable*; PIO framing makes it *unrepresentable*.
* **Residual failure mode = frame loss** (BCLK stall, FIFO full). Detection: §6.3.

### 1.4 Pins
| GPIO | Signal | Owner after this design | PIO use |
|------|--------|-------------------------|---------|
| 18 | BCLK | I2S1 (pinctrl `i2s1`, input) | `wait gpio 18` (both SMs) — read-only, no funcsel change |
| 19 | WS   | I2S1 | `wait gpio 19`, `jmp pin` (RX) |
| 20 | SDI  | I2S1 (heartbeat RX still consumes it) | `in pins` (RX, in_base 20) |
| 21 | SDO  | **snd-pio-tdm** (pinctrl function `"pio"` = funcsel 7; VERIFIED function 36 "pio" exists on RP1) | `out pins`/`set pins` (TX), pindir out |

`pinctrl-rp1` is `.strict = true` (VERIFIED, `pinctrl-rp1.c:1366`) → `gpiod_get()` on a pin muxed to
i2s1 fails with -EBUSY, and would re-mux it to GPIO if it succeeded. Therefore the driver **parses**
`*-gpios` phandles (`of_parse_phandle_with_args(np, "bclk-gpios", "#gpio-cells", 0, &a)` →
`a.args[0]`, check `of_device_is_compatible(a.np, "raspberrypi,rp1-gpio")`) and never requests them.
Currently (VERIFIED `pinmux-pins`): pins 18–21 are `1f000a4000.i2s … function i2s1` — hence the
I2S1 group must be trimmed to 18–20 in the overlay or pinctrl refuses our node's `pio_sdo_pin`.

### 1.5 Program placement — fixed origins, no relocation assumed
`rp1_pio_add_program()` (`rp1-pio.c:236-270`, VERIFIED) forwards the instruction words to the
firmware untouched; piolib does the same (`pio_rp1.c:304-314`). Whether the RP1 firmware relocates
JMP targets when `origin = ANY` is unknown → **bake absolute targets and load with
`pio_add_program_at_offset(client, &rx_prog, 0)` / `(…, &tx_prog, 16)`; treat any non-matching
return as -EBUSY and fail probe** ("PIO instruction memory 0-12/16-27 busy — is tdmcap running?").
Note `rp1_pio_find_program()` dedups byte-identical programs (refcounted), so a concurrent identical
userspace loader is harmless. Instruction budget: 13 + 12 = 25 of 32; leaves 7 for diagnostics
(tdmcap needs 3, at 28–30 or 13–15).

### 1.6 Encoding notes (pico-sdk `pio_instructions.h`, VERIFIED identical copy at
`/usr/src/linux-headers-6.18.34+rpt-common-rpi/include/linux/pio_instructions.h`)
`instr | (arg1<<5) | (arg2&0x1f)`; delay = bits 12:8. `wait 1 gpio n` = `0x2080|n`, `wait 0 gpio n`
= `0x2000|n`; `jmp x--` cond 2, `jmp y--` cond 4, `jmp pin` cond 6; `set x`=0xe020, `set y`=0xe040,
`set pins`=0xe000; `in pins,1`=0x4001; `out pins,1`=0x6001; `irq set 0`=0xc000. Cross-checked against
the published words in `rp1-pio-uart.c` (0x4001, 0x6001, 0x00c8 = jmp pin 8, 0x0642 = jmp x--,2 [6],
0xea27 = set x,7 [10]). **All 25 words are hand-derived — userspace piolib TX proof: re-generate with piolib's
`pio_encode_*` and diff before the first kernel load.** Program arrays are in §8.

WS-edge convention: `tdmcap` samples at rising edges ~15 ns after the edge and saw WS=0 at e−1, WS=1
at e, so WS rises somewhere in (e−1+15 ns, e+15 ns]. Both programs are correct anywhere in that
window (the level-sensitive `wait 1 gpio 18` either consumes edge e or passes through during its
high half). A master that changed WS more than half a BCLK before e is excluded by the measurement.

---

## 2. DMA — own channels, hardware cyclic ring

### 2.1 Facts (VERIFIED against the running kernel / source)
* RP1 DMAC = `snps,axi-dma-1.01a` (`dw-axi-dmac`, built-in `CONFIG_DW_AXI_DMAC=y`), 8 channels,
  `#dma-cells = <1>`, **`snps,axi-max-burst-len = <8 8 4 4 4 4 4 4>`** (chan0/1 burst 8, others 4),
  `snps,chan-flags = <0x100 0x100 0 0 0 0 0 0>`, `snps,block-size = <0x40000 ×8>`.
  `residue_granularity = BURST`, `device_prep_dma_cyclic` implemented, callbacks per period via
  `vchan_cyclic_callback` (tasklet), LLI ring re-armed by the driver inside the IRQ handler with no
  gap in the hardware chain (`dw-axi-dmac-platform.c:1259-1305`).
* `dw_axi_dma_of_xlate()` (`:1549-1608`): the single DT cell is scored against `snps,chan-flags`
  (bit 0x100 → prefers chan 0/1) and `(u8)cell` becomes the hardware handshake number. **The base DT's
  pio node uses `dmas = <&rp1_dma 0x138>…<0x13f>` = `RP1_DMA_PIO_CHn_{TX,RX} | 0x100`** — that is why
  tdmcap measured an 8-word burst. I2S1 uses plain `0x22/0x21` → light channels (dma2chan4/5 in use).
  Free now: dma2chan0, 1, 6, 7.
* Handshake ids (`include/dt-bindings/mfd/rp1.h:226-233`): `RP1_DMA_PIO_CH0_TX 0x38`, `CH0_RX 0x39`,
  `CH1_TX 0x3a`, `CH1_RX 0x3b`, …
* Slave (FIFO) address: rp1-pio uses the **CPU-view** resource start (`pio->phys_addr = ioresource->start`
  = 0x1f00178000; `rp1-pio.c:1339`) + `0x10 + 4·sm` (RX) / `0x00 + 4·sm` (TX); dw-axi-dmac converts it
  with `phys_to_dma(chan->chip->dev, …)` through the DMAC's dma-ranges (pcie: CPU 0x1f00000000 ↔ PCI
  0x0; rp1: PCI 0x0 ↔ RP1-bus 0xc0_40000000). Do exactly the same: `of_address_to_resource(pio_np,0,&r)`
  → `r.start + 0x10 + 4*sm_rx`.
* Memory side: RP1-bus ↔ RAM is identity (rp1 dma-ranges entry 3, pcie entry 2), **no IOMMU**
  (`iommu-map` empty). Still allocate on the right device: `chan->device->dev` (the DMAC) — that is
  what rp1-pio does and what `snd_dmaengine_pcm` does.
* `dmaengine_prep_*` in dw-axi-dmac uses `GFP_NOWAIT` (atomic-safe); `dmaengine_terminate_async()`
  atomic-safe; `dmaengine_synchronize()` sleeps.

### 2.2 Why not `pio_sm_xfer_data()` (the plan's original ring)
`rp1_pio_sm_xfer_data()` (`rp1-pio.c:925-997`, signature VERIFIED identical in the installed
`pio_rp1.h:191-193`) with `dma_addr != 0` does skip the bounce path (`if (!dma_addr) { kmalloc(GFP_KERNEL)… }`)
and preps `DMA_DEV_TO_MEM/MEM_TO_DEV` slave_sg straight to our address with our callback — but:
1. Each transfer is a separate descriptor; dw-axi-dmac starts the next queued descriptor from its
   **IRQ handler** (`axi_chan_start_first_queued`, `:1313`). Between two periods the FIFO must absorb
   the host IRQ latency: 16 words joined = **125 µs**. A late start stalls the RX SM mid-frame → one
   garbage frame + resync, 50×/s exposure. Cyclic LLI has no such gap.
2. `pio_sm_config_xfer()` (mandatory — it requests the channel and does `dmaengine_slave_config`) sets
   `maxburst = dma_caps.max_burst` (8 on the heavy channel) but programs the RX DREQ threshold to **1**
   on this kernel (`(DEFAULT & ~0x1f) | 1`, `:703`; fixed in rpi-6.18.y HEAD to `DEFAULT | maxburst`,
   VERIFIED HEAD `:1123`) → must be overridden with `pio_sm_set_dmactrl(client,sm,false,0x80000108)` —
   exported (VERIFIED `Module.symvers`) and inline in the header, so it *is* fixable, but you never
   learn which burst you got (the channel is chosen inside rp1-pio).
3. It allocates 1–4 bounce buffers you never use and takes the DMA claim bookkeeping out of your hands.
Appendix B keeps the exact recipe as the fallback if 2.3 hits an unexpected wall.

### 2.3 Design
```c
/* probe */
rx.chan = dma_request_chan(dev, "rx");   /* DT: dmas = <&rp1_dma (RP1_DMA_PIO_CH0_RX|0x100)>, … */
tx.chan = dma_request_chan(dev, "tx");   /*            <&rp1_dma (RP1_DMA_PIO_CH1_TX|0x100)>   */
dma_get_slave_caps(rx.chan, &caps);  rx.burst = min(caps.max_burst, 8u);   /* expect 8 */
cfg = (struct dma_slave_config){
    .direction = DMA_DEV_TO_MEM, .src_addr = pio_res.start + 0x10 + 4*SM_RX,
    .src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES, .dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
    .src_maxburst = rx.burst };
dmaengine_slave_config(rx.chan, &cfg);
pio_sm_set_dmactrl(pio, SM_RX, false, 0x80000100 | rx.burst);          /* 0x80000108 — MEASURED good */
/* TX mirrored: DMA_MEM_TO_DEV, .dst_addr = pio_res.start + 0x00 + 4*SM_TX, .dst_maxburst = tx.burst,
   pio_sm_set_dmactrl(pio, SM_TX, true, 0x80000100 | (TX_FIFO_DEPTH - tx.burst))  — see 2.4 */
dmadev = rx.chan->device->dev;   /* buffers: snd_pcm_set_managed_buffer(substream, SNDRV_DMA_TYPE_DEV, dmadev, pre, max) */

/* trigger START (nonatomic, may sleep) */
desc = dmaengine_prep_dma_cyclic(chan, runtime->dma_addr, buffer_bytes, period_bytes, dir,
                                 DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
desc->callback = pio_tdm_dma_cb; desc->callback_param = strm;
strm->cookie = dmaengine_submit(desc); dma_async_issue_pending(chan);
/* then the SM (order matters: RX — DMA first so DREQ is honoured from word 8; TX — DMA first so the
   FIFO is full before the SM meets its first WS): */
pio_sm_set_enabled(pio, sm, false); pio_sm_clear_fifos(pio, sm); pio_sm_restart(pio, sm);
pio_sm_exec(pio, sm, pio_encode_jmp(origin)); pio_sm_set_enabled(pio, sm, true);

/* trigger STOP */
pio_sm_set_enabled(pio, sm, false); dmaengine_terminate_async(chan);
if (tx) pio_sm_set_pins_with_mask(pio, SM_TX, 0, BIT(21));
/* .hw_free / .close: dmaengine_synchronize(chan); cancel_work_sync(&strm->period_work); cancel_delayed_work_sync(&wdog) */

/* DMA callback — tasklet context: touch nothing that sleeps */
static void pio_tdm_dma_cb(void *p) { struct strm *s = p; atomic_inc(&s->periods); queue_work(system_highpri_wq, &s->period_work); }
static void period_work(struct work_struct *w) { …; snd_pcm_period_elapsed(s->substream); }   /* takes the nonatomic mutex — OK here */

/* .pointer (process ctx, under the stream mutex) */
dmaengine_tx_status(chan, s->cookie, &st);
pos = st.residue ? buffer_bytes - st.residue : (atomic_read(&s->periods) * period_bytes) % buffer_bytes;
return bytes_to_frames(runtime, pos);
```
Constraints the DMAC imposes (VERIFIED `dw_axi_dma_chan_prep_cyclic`, `dw_axi_dma_set_hw_desc`):
`buffer_bytes % period_bytes == 0` (→ `snd_pcm_hw_constraint_integer(PERIODS)`), `len % 4 == 0`,
period ≤ block_size×4 = 1 MB (irrelevant), `maxburst ≤ axi_rw_burst_len[chan]` else a "data may be
lost" warning — hence `min(caps.max_burst, 8)`. Period bytes a multiple of `burst × 4 = 32` keeps every
period an integral number of bursts (§4).

### 2.4 DMACTRL threshold — resolved from the header + both driver generations
`pio_sm_set_dmactrl(client, sm, bool is_tx, u32 ctrl)` — inline in the installed `pio_rp1.h`,
`rp1_pio_sm_set_dmactrl` exported (VERIFIED). It is a bare mailbox message with no claim check, so
it can be called without `pio_sm_config_xfer()`. Register semantics (from the two driver generations):
bit 31 enable, bit 8 always set (`RP1_PIO_DMACTRL_DEFAULT` = 0x80000104 in 6.12, 0x80000100 in HEAD),
bits 4:0 = DREQ threshold. HEAD (`rp1-pio.c:1121-1126`): **RX ctrl = DEFAULT | maxburst** (DREQ when
level ≥ burst — MEASURED correct here as 0x80000108), **TX ctrl = DEFAULT | (FIFO_DEPTH − maxburst)**
(DREQ when level ≤ depth − burst, i.e. a full burst fits). With `FIFO_DEPTH = 8` (unjoined) and burst
8 that is `0x80000100`. We join the TX FIFO (16 deep) → `0x80000108` if the threshold really is a
level (VERIFY with piolib: `pio_sm_set_dmactrl(pio, sm, true, 0x80000108)` + the tone test; a wrong
value shows as clicks/dropouts or a tone at the wrong pitch). **Fallback: unjoined TX FIFO + `0x80000100`.**
`pio_sm_get_dmactrl` does not exist on this kernel (HEAD only).

---

## 3. Atomic context — who may call what

Every `pio_*`/`rp1_pio_*` call = `rp1_firmware_message()` → **sleeps** (mutex + completion, ~10 µs).
`snd_pcm_period_elapsed()` in nonatomic mode takes `substream->self_group.mutex`
(`_snd_pcm_stream_lock_irqsave`: `if (pcm->nonatomic) mutex_lock(…)`) → **must not be called from
tasklet/IRQ**. `CONFIG_DEBUG_ATOMIC_SLEEP` is off on this kernel, so a violation would be silent until
it deadlocks — follow the table.

| ALSA op | Context | PIO mailbox allowed | DMA engine allowed | Notes |
|---------|---------|---------------------|--------------------|-------|
| `.open/.close` | process | yes | request/release (we hold chans from probe) | `.close`: `cancel_work_sync`, `dmaengine_synchronize` |
| `.hw_params/.hw_free` | process | yes | `dmaengine_slave_config` (already done in probe), `terminate_sync` | managed buffer alloc/free automatic |
| `.prepare` | process | yes — **bus-alive probe** (§6.2) | — | |
| `.trigger` | process (nonatomic) | **yes** (enable/disable/restart/clear_fifos/set_pins) | prep_cyclic/submit/issue_pending/terminate_async | START ≈ 5 mailbox calls ≈ 60 µs |
| `.pointer` | process (mutex held) | no (unneeded) | `dmaengine_tx_status` | |
| DMA callback | tasklet | **NO** | prep/submit only (unused with cyclic) | `atomic_inc` + `queue_work` only |
| `period_work` | workqueue | allowed but unneeded | — | `snd_pcm_period_elapsed()` |
| watchdog `delayed_work` | workqueue | yes (fifo_state snapshot) | `tx_status` | `snd_pcm_stop_xrun()` |
| sysfs show | process | yes (fifo level) — keep ≤ 2 calls | — | |
| probe/remove | process | yes | yes | `pio_open()` returns -EPROBE_DEFER until rp1-pio is up (`rp1-pio.c:1048-1053`) — propagate it |

`pcm->nonatomic = true` set right after `snd_pcm_new()`, before `snd_card_register()`.

---

## 4. ALSA card

```c
snd_card_new(dev, -1, "seeed8micpio", THIS_MODULE, 0, &card);        /* id must contain "seeed": music.py:34, research.py:35 gate on it */
strscpy(card->driver, "snd-pio-tdm"); strscpy(card->shortname, "seeed8micpio");
snprintf(card->longname, sizeof card->longname, "ReSpeaker 6-Mic via RP1 PIO (SM%u/SM%u)", SM_RX, SM_TX);
snd_pcm_new(card, "PIO TDM", 0, 1 /*playback*/, 1 /*capture*/, &pcm);
pcm->nonatomic = true;  pcm->private_data = priv;  strscpy(pcm->name, "PIO TDM 8x32 in / 2x32 out");
snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,  &pio_tdm_rx_ops);
snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &pio_tdm_tx_ops);
snd_pcm_set_managed_buffer(pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream,  SNDRV_DMA_TYPE_DEV, dmadev, 64*1024, 256*1024);
snd_pcm_set_managed_buffer(pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream, SNDRV_DMA_TYPE_DEV, dmadev, 16*1024,  64*1024);
snd_card_register(card);
```
```c
static const struct snd_pcm_hardware rx_hw = {
  .info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER | SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID,
  .formats = SNDRV_PCM_FMTBIT_S32_LE, .rates = SNDRV_PCM_RATE_16000, .rate_min = 16000, .rate_max = 16000,
  .channels_min = 8, .channels_max = 8,
  .period_bytes_min = 512 /*16 frames*/, .period_bytes_max = 65536 /*2048 frames*/,
  .periods_min = 2, .periods_max = 16, .buffer_bytes_max = 262144, .fifo_size = 16 };
static const struct snd_pcm_hardware tx_hw = { /* same info/format/rate */
  .channels_min = 2, .channels_max = 2,
  .period_bytes_min = 128 /*16 frames*/, .period_bytes_max = 16384 /*2048 frames*/,
  .periods_min = 2, .periods_max = 16, .buffer_bytes_max = 65536, .fifo_size = 16 };
/* .open: */ snd_pcm_hw_constraint_step(rt, 0, SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 32);   /* frame(32B RX / 8B TX) ∧ burst 8 words */
             snd_pcm_hw_constraint_step(rt, 0, SNDRV_PCM_HW_PARAM_BUFFER_BYTES, 32);
             snd_pcm_hw_constraint_integer(rt, SNDRV_PCM_HW_PARAM_PERIODS);
```
Target period = 20 ms = 320 frames = 10 240 B (RX) / 2 560 B (TX) — both multiples of 32. MMAP is
free with coherent managed buffers (`snd_pcm_lib_default_mmap` via the core) and removes any doubt
about alsa-lib access-type negotiation; INFO_BATCH is *not* set because the pointer is residue-based
(32-byte granularity); set it if you fall back to the period-counter pointer.

**plug check (what alsa-lib must do, all supported):**
* `arecord -D plughw:CARD=seeed8micpio -c8 -r16000 -f S16_LE` → hw = 8ch/16k/S32_LE → plug inserts
  **linear S32_LE→S16_LE only**; no route/rate stage. Needs: a linear format (yes), fixed channels/rate
  matching the request (yes), RW_INTERLEAVED (INFO_INTERLEAVED, yes). Default arecord buffer = min(500 ms,
  buffer_bytes_max) = 8192 frames (512 ms), period = 2048 frames (128 ms) — within period_bytes_max
  65536 ✓. (Pi-3 heartbeat ran 2000/8000 — same latency class. `-F 20000 -B 80000` for 20 ms periods.)
* mixer `snd_pcm_set_params(S16_LE, RW_INTERLEAVED, 1 ch, 16000, soft_resample=1, latency)` on
  `plughw:CARD=seeed8micpio` → plug: **route mono→stereo (duplicate) + linear S16→S32**; rate matches
  (no resampler). `set_params` derives period ≈ latency/4 in frames and rounds to our 32-byte step
  (= 4 frames) ✓; periods ≥ 2 ✓. The `amixer sset 'DAC volume'` call stays on `seeed8micvoicec`
  (AC101 control lives on the seeed card).
* `--dump-hw-params` on both is Gate-2 test T1.

Data format on the wire ↔ memory: RX word = 32 bits MSB-first from the slot = the S32 sample
(AC108 24-bit MSB-justified, low byte expected 0 — VERIFY with `analyse2.py` hex dump; used only by
the optional integrity check §6.3). TX word = S32_LE sample, MSB-first out; AC101 uses the top 24 bits.

---

## 5. Device tree, overlay, install

### 5.1 Binding (`seeed,snd-pio-tdm`)
```
snd_pio_tdm: snd-pio-tdm {                  /* child of &rp1 → inherits RP1's dma-ranges (correct DMA offsets for the DMAC) */
    compatible = "seeed,snd-pio-tdm";
    pio = <&rp1_pio>;                         /* reg → FIFO slave addresses; also of_address_to_resource() */
    bclk-gpios = <&rp1_gpio 18 0>;  ws-gpios = <&rp1_gpio 19 0>;   /* parsed only, never requested */
    sdi-gpios  = <&rp1_gpio 20 0>;  sdo-gpios = <&rp1_gpio 21 0>;
    dmas = <&rp1_dma (RP1_DMA_PIO_CH0_RX | 0x100)>, <&rp1_dma (RP1_DMA_PIO_CH1_TX | 0x100)>;
    dma-names = "rx", "tx";                   /* 0x100 = prefer the burst-8 channels (snps,chan-flags); SM numbers are FIXED: rx=SM0, tx=SM1 */
    pinctrl-names = "default";  pinctrl-0 = <&pio_sdo_pin>;
    status = "okay";
};
```
Optional properties: `seeed,rx-sm = <0>`, `seeed,tx-sm = <1>` (must agree with the dma cells — the
driver checks `RP1_DMA_PIO_CHn_RX == (cell & 0xff)` and refuses otherwise).

### 5.2 Overlay edits to `pi5/seeed-8mic-pi5-overlay.dts` (compile: `cpp -nostdinc -I<headers>/include -undef -x assembler-with-cpp` → `dtc -@ -I dts -O dtb`, as today; add `#include <dt-bindings/mfd/rp1.h>`)
```
fragment@0 { target = <&i2s_clk_consumer>; __overlay__ { … pinctrl-0 = <&rp1_i2s1_18_20>; … } };   /* was rp1_i2s1_18_21 — releases GPIO21 */
fragment@2 { target = <&rp1_gpio>; __overlay__ {
        /* existing spk_amp_pins, gpclk0_pin … */
        rp1_i2s1_18_20: rp1_i2s1_18_20 { function = "i2s1"; pins = "gpio18", "gpio19", "gpio20"; bias-disable; };
        pio_sdo_pin: pio_sdo_pin { function = "pio"; pins = "gpio21"; bias-disable; };
} };
fragment@5 { target = <&rp1>; __overlay__ { snd_pio_tdm: snd-pio-tdm { /* §5.1 verbatim */ }; } };
```
`&rp1` and `&rp1_pio`, `&rp1_dma`, `&rp1_gpio` all resolve (VERIFIED `__symbols__`). `rp1_pio` is already
`status = "okay"`. Check after boot: `pinctrl get 18,19,20,21` → 18–20 `a4 … I2S1_*`, **21 `a7 … PIO`**;
`ls /sys/bus/platform/devices/ | grep snd-pio-tdm`; `cat /proc/asound/cards` → `seeed8micpio`.
Install: `sudo cp seeed-8mic-pi5.dtbo /boot/firmware/overlays/` (config.txt already has
`dtoverlay=seeed-8mic-pi5`), `sync`, `ps -eo stat,comm | awk '$1~/D/'` empty, `sudo systemctl reboot`.
**Never** `dtoverlay` at runtime (Rule; configfs wedge → power-cycle).

### 5.3 Module build + autoload (both the seeed modules and this one)
```
pi/snd-pio-tdm/Kbuild:   obj-m := snd-pio-tdm.o
pi/snd-pio-tdm/Makefile: KDIR ?= /lib/modules/$(shell uname -r)/build
                         all: ; $(MAKE) -C $(KDIR) M=$(CURDIR) modules
                         install: all ; sudo install -D -m 644 snd-pio-tdm.ko /lib/modules/$(shell uname -r)/extra/snd-pio-tdm.ko && sudo depmod -a
```
`#include <linux/pio_rp1.h>` (pulls `uapi/misc/rp1_pio_if.h` + `linux/pio_instructions.h`), `<sound/core.h>`,
`<sound/pcm.h>`, `<sound/pcm_params.h>`, `<linux/dmaengine.h>`, `<linux/of_address.h>`, `<linux/pinctrl/consumer.h>`.
All `rp1_pio_*` symbols are `EXPORT_SYMBOL_GPL` (VERIFIED in `Module.symvers`) → `MODULE_LICENSE("GPL")`;
`modules.dep` then pulls `rp1-pio` automatically. `MODULE_DEVICE_TABLE(of, …)` → udev autoloads on the
DT node (modalias `of:Nsnd-pio-tdmT(null)Cseeed,snd-pio-tdm`). Belt-and-braces:
`/etc/modules-load.d/snd-pio-tdm.conf` containing `snd-pio-tdm` (the seeed pair already lives in
`/etc/modules-load.d/seeed-voicecard.conf`: `snd-soc-seeed-voicecard`, `snd-soc-ac108`, both installed
in `/lib/modules/$(uname -r)/extra/` — VERIFIED). Rebuild script re-runs `make install` for all three after
any kernel change (the kernel is `apt-mark hold` — VERIFIED — so that is a deliberate act).

---

## 6. Runtime model

### 6.1 Startup ordering (Phase 2)
1. Firmware/DT: overlay → I2S1 (18–20), GPIO21 = pio, `snd_pio_tdm` node.
2. Kernel: `rp1-fw` → `rp1-pio` (behind PCIe, ~0.5 s) → `snd-pio-tdm` probes (EPROBE_DEFER until then),
   requests dma2chan0/1, claims SM0/SM1, loads programs at 0/16, registers `seeed8micpio`.
   The card exists **before** the bus is clocked; any stream started now fails fast in `.prepare` (6.2).
3. `systemd-modules-load`: `snd-soc-seeed-voicecard`, `snd-soc-ac108` → codecs bind → `seeed8micvoicec`.
4. `seeed-codec-heartbeat.service` (`Restart=always`, waits ≤ 20 s for the card): an 8-ch S32
   `arecord` → the AC101 master starts BCLK/WS (first stream after boot must be 8-ch).
5. Consumers open `plughw:CARD=seeed8micpio` (capture and/or playback). Each gets
   `After=seeed-codec-heartbeat.service` + `Requires=seeed-codec-heartbeat.service`. Do not
   `BindsTo=`: a heartbeat restart is survivable (6.3).

### 6.2 Bus-alive probe (`.prepare`, ~4 mailbox calls)
```
pio_sm_set_enabled(pio,SM_RX,false); pio_sm_clear_fifos(pio,SM_RX); pio_sm_restart(pio,SM_RX);
pio_sm_exec(pio,SM_RX,jmp 0); pio_sm_set_enabled(pio,SM_RX,true); usleep_range(1500,3000);
level = pio_sm_rx_fifo_level(pio,SM_RX); pio_sm_set_enabled(pio,SM_RX,false);
if (level == 0) { dev_err(dev,"codec bus not clocked (no WS/BCLK) — is seeed-codec-heartbeat running?"); return -EIO; }
```
1.5 ms ≥ 24 frames; with BCLK present the FIFO holds 16 words. For TX `.prepare` run the same probe on
the RX SM only if capture is not open (else trust the running capture). `arecord`/the mixer then fail
with EIO instead of hanging.

### 6.3 Failure modes
| Event | Hardware effect | Driver behaviour | Visible as |
|-------|-----------------|------------------|-----------|
| Heartbeat dies → `ac108_set_clock` clears GEN → BCLK/WS stop | RX SM parks at `wait 1 gpio 18`; TX SM likewise; DREQ stops; DMA ring frozen | watchdog (`delayed_work`, 500 ms) sees no period progress for 2 ticks → `bus_stalls++`, `snd_pcm_stop_xrun(substream)` (nonatomic mutex OK) | `arecord: overrun` → its re-`prepare` hits 6.2 → EIO → exits → the consumer unit's `Restart=always` (ordered after the heartbeat) brings it back. A `snd_pcm_recover()` consumer takes the same path. |
| BCLK returns *before* the watchdog fires | RX SM resumes mid-frame → that frame's bits misaligned → `jmp pin` fails → resync at next WS | nothing to do (self-heals) | ≤ 2 garbled/dropped frames (125 µs) — inaudible click at most, **no rotation** |
| RX FIFO full (DMAC late) | `in` stalls (no word loss); same recovery as above | `rx_fifo_full` snapshot counter (watchdog polls `pio_sm_is_rx_fifo_full`) | drift step (below) |
| TX FIFO empty (ALSA underrun) | `out` stalls; ≤ 1 frame damage; SM re-waits WS | ALSA XRUN via pointer wrap (standard) | mixer `EPIPE` → recover |
| 2-ch stream opened first on the *seeed* card (Rule 4 violation) | AC101 emits 64-BCLK frames; WS period 64 | RX SM counts 256 edges (= 4 short frames) then finds WS low at the check → resync every cycle → **every frame garbled**, DMA rate unchanged but content is nonsense | Gate-2 test T6 catches it; fix = restart heartbeat 8-ch |
| rp1-pio unloaded / SM stolen | claims prevent it while we hold them | probe fails if 0–12/16–27 or SM0/1 busy | dmesg |

**Frame-loss detection without FDEBUG:** this kernel exports no register read (no `read_hw`, no
`sm_get_flags`, no PIO IRQ API — the HEAD's `rp1_pio_irq_add_handler`/`rp1_pio_interrupt_get` are absent
from the installed header, VERIFIED). So: (a) **soak comparator** (test-side, exact): both cards are
clocked by the AC101; `Δhw_ptr(seeed8micvoicec capture) − Δhw_ptr(seeed8micpio capture)` over the same
interval must be 0 ± 1 frame (`/proc/asound/cardN/pcm0c/sub0/status`); any resync shows as a
multiple of −1 frame; (b) in-driver **drift estimator**: per period `drift = (ktime − t0) − periods×period_ns`
exposed as `rx_drift_us`; a lost frame is a permanent +62.5 µs step on an otherwise straight
(±0.6 ppm) line; (c) optional `rx_lowbyte_check=1` module param: OR of the low byte of every RX word per
period must be 0 if the AC108 pads 24→32 with zeros (VERIFY first) — a misaligned frame fails it;
(d) `irq set 0` is already in the resync path so a future kernel can count resyncs by interrupt.

### 6.4 sysfs (attribute group on the platform device, `/sys/bus/platform/devices/snd-pio-tdm/`)
`rx_periods tx_periods rx_drift_us bus_stalls rx_xruns_forced tx_xruns_forced rx_fifo_full_hits
rx_lowbyte_bad_periods dma_rx_chan dma_tx_chan dma_rx_burst dma_tx_burst sm_rx sm_tx prog_rx_origin
prog_tx_origin bus_alive` (last `.prepare` probe result). Read-only; each `show` ≤ 1 mailbox call.

### 6.5 Cost
Per 20 ms period: 1 DMAC IRQ + tasklet + 1 work item ≈ 20 µs → ~0.1 % of one A76. Data: 512 KB/s RX
+ 128 KB/s TX DMA (invisible), `arecord`+plug S32→S16 ≈ 1–2 % of a core. PIO framing: 0.

---

## 7. Gate-2 test plan (all automatable except T9)

Preconditions: `systemctl is-active seeed-codec-heartbeat`, `pgrep -a arecord` shows the 8-ch S32
heartbeat, `arecord -l | grep seeed8micpio`, `dmesg | grep snd-pio-tdm` shows `SM0/SM1, prog 0/16,
dma2chan0 burst 8, dma2chan1 burst 8`. Capture analysis: `test/analyse_pio.py`.

| # | Test | Pass |
|---|------|------|
| T1 | `arecord -D plughw:CARD=seeed8micpio -c8 -r16000 -f S16_LE -d1 --dump-hw-params /dev/null`; `aplay -D plughw:CARD=seeed8micpio -c1 -r16000 -f S16_LE -d1 /dev/zero --dump-hw-params`; `hw:` variants with S32 | both open, hw shows S32_LE/8ch and S32_LE/2ch, 16000, period ∈ constraints |
| T2 | 10 s raw capture `hw:CARD=seeed8micpio -f S32_LE -c8` → `analyse_pio.py` (slot RMS dBFS, low-byte OR, per-quarter loudest slot) | quiet room: 6 mic slots within 0.5 dB of each other in −78…−66 dBFS; loudest slot identical across quarters; wall-clock 10.00 ± 0.05 s |
| T3 | Same while `aplay` a −20 dBFS 1 kHz S32 stereo tone (10 s) to `hw:CARD=seeed8micpio` | loopback slots (0/1 or 6/7 — settles the slot-map question) show 1000 ± 2 Hz peak ≥ 40 dB above their noise; other slots unaffected; tone audible on J2 (T9 manual) |
| T4 | Start/stop capture ×10, playback ×10, both concurrently ×5 | every run passes T2/T3 criteria; `bus_stalls` unchanged; no dmesg warnings |
| T5 | 1 h soak: capture→/dev/null + looped tone playback; sample `/proc/asound/*/pcm0c/sub0/status` of both cards every 60 s | Δhw_ptr difference 0 ± 1 frame every interval; `rx_drift_us` straight line, no ≥ 60 µs steps; `rx_lowbyte_bad_periods` 0 (if enabled); CPU: `pidstat 60` arecord+aplay+kworker < 3 % |
| T6 | `systemctl restart seeed-codec-heartbeat` during T5 | consumers get EPIPE/EIO, `bus_stalls` +1, capture resumes with the same loudest-slot index within 5 s; T2 criteria hold afterwards |
| T7 | A userspace piolib bus-capture tool (BCLK/WS/SDI/SDO on GPIO18-21) run concurrently with T5 | it gets a light channel (dma2chan6/7), reports 4.096 MHz; card unaffected |
| T8 | `rmmod snd-pio-tdm && modprobe snd-pio-tdm` (streams closed) | card back; `/sys/class/dma/dma2chan{0,1}/in_use` 0 → 1; pins unchanged |
| T9 | **Speech (manual):** speak at 1 m during T2; then an end-to-end capture→playback path | speech slots +≥ 6 dB, loudest ≥ 15 dB; playback audible from J2 |
| T10 | Consumer contract: capture and playback consumers both on `plughw:CARD=seeed8micpio`, mixer controls still on `seeed8micvoicec` | both start, playback is audible, `amixer -c seeed8micvoicec sset 'DAC volume'` still works |

Slot map: T3 settles which slots carry the AC101 loopback (measured: 0 and 1) and which carry the
six mics (2–7); a downstream echo-canceller's reference-channel index follows from that.

---

## 8. Reference arrays (paste into the module; regenerate with piolib before first load)
```c
#define PIO_TDM_RX_ORIGIN 0
static const u16 pio_tdm_rx_prog[13] = {
	0x2013, 0x2093, 0x2092, 0xe047, 0xe03f, 0x2012, 0x2092, 0x4001,
	0x0045, 0x0884, 0x00c3, 0xc000, 0x0000 };
#define PIO_TDM_TX_ORIGIN 16
static const u16 pio_tdm_tx_prog[12] = {
	0xe000, 0x2013, 0x2093, 0x2092, 0xe041, 0xe03f, 0x2012, 0x6001,
	0x2092, 0x0056, 0x0095, 0x2012 };
static const struct pio_program rx_program = { .instructions = pio_tdm_rx_prog, .length = 13, .origin = PIO_TDM_RX_ORIGIN };
static const struct pio_program tx_program = { .instructions = pio_tdm_tx_prog, .length = 12, .origin = PIO_TDM_TX_ORIGIN };
```
JMP targets are absolute for these origins (RX: 5,4,3,0; TX: 22,21). If a different origin is ever
needed, re-encode: `jmp` word = `cond<<5 | target`.

## 9. VERIFY list for the userspace piolib TX proof (before any kernel load)
1. Re-encode §8 with `pio_encode_*`; diff.
2. RX program at origin 0 (`pio_add_program_at_offset`), SM config as §1.1, `pio_sm_set_dmactrl(…,false,0x80000108)`,
   2 s DMA capture → `analyse_pio.py`: slot RMS pattern matches Gate 1 (−72 dBFS mics), loudest slot
   stable per quarter, low-byte OR (report whether it is 0).
3. TX program at origin 16, `pio_gpio_init(21)` + `pio_sm_set_consecutive_pindirs(out)` + `set_pins 0`;
   DMACTRL TX `0x80000108` with `FJOIN_TX`; 1 kHz tone into slots 0/1; confirm (a) pad actually drives
   (tdmcap SDO bit), (b) tone in loopback slots at 1000 Hz (settles 0/1 vs 6/7), (c) no clicks — else
   try `0x80000100` unjoined, then DSP_B (drop instr 19).
4. Does `pio_add_program` with `origin = -1` relocate JMPs? (Load a 2-instr `jmp` test at ANY, read back
   via `PIO_IOC_READ_HW` if available.) Informational only — the driver uses fixed origins.

## Appendix A — Kernel facts table (installed 6.18.34 vs references)
| Item | Installed 6.18.34 | rpi-6.12.y (`/tmp/rp1-pio-6.12.c`) | rpi-6.18.y HEAD |
|------|-------------------|-------------------------------------|-----------------|
| `rp1_pio_sm_xfer_data(client,sm,dir,bytes,data,dma_addr,cb,param)` | yes (`pio_rp1.h:191`) | `:925` | `:1423` (adds `-EBUSY` instead of blocking) |
| `pio_sm_config_xfer` sets RX DREQ threshold | not readable; **measured 1** (8× dup) | `(DEFAULT&~0x1f)\|1` `:703` | `DEFAULT \| maxburst` `:1123` |
| `rp1_pio_sm_set_dmactrl` exported | **yes** | yes | yes (+`get_dmactrl`) |
| PIO IRQ / `sm_get_flags` / `read_hw` in-kernel | **no** | no | yes |
| cyclic (`FL_DMA_CYCLE`) in rp1-pio | no | no | yes (userspace) |
| `pio_gpio_init`, `pio_sm_set_consecutive_pindirs`, `sm_config_set_jmp_pin`, `sm_config_set_fifo_join` | yes | yes | yes |
| `rp1_pio_open()` before rp1-pio probe | `-EPROBE_DEFER` | same | same |

## Appendix B — Fallback: `pio_sm_xfer_data()` descriptor ring (only if §2.3 fails)
```
pio_sm_config_xfer(pio, SM_RX, PIO_DIR_FROM_SM, 4096, 1);     /* mandatory: requests "rx0" (heavy → dma2chan0, burst 8), slave_config, 1 unused bounce buf */
pio_sm_set_dmactrl(pio, SM_RX, false, 0x80000108);            /* undo the threshold-1 bug */
/* N = periods coherent buffers = the ALSA buffer at period offsets (managed buffer on the DMAC device, found via
   of_find_device_by_node(of_parse_phandle(pio_np,"dmas",0)) or of_find_compatible_node(…"snps,axi-dma-1.01a")) */
for (i = 0; i < 2; i++) pio_sm_xfer_data(pio, SM_RX, PIO_DIR_FROM_SM, period_bytes, NULL /*ok when dma_addr!=0*/,
                                         dma_addr + i*period_bytes, rx_cb, &slot[i]);
rx_cb (tasklet): atomic_inc(periods); queue_work(period_work); pio_sm_xfer_data(... next free period ...);  /* prep is GFP_NOWAIT-safe; NEVER pass dma_addr==0 here (GFP_KERNEL kmalloc + memcpy path) */
```
Keep ≥ 2 in flight; sizes multiples of 32 B; TX mirrored with `PIO_DIR_TO_SM`. Known weakness: a
per-period IRQ-latency gap against 125 µs of FIFO (§2.2). Do not mix with §2.3 on the same SM.
