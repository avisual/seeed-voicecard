// SPDX-License-Identifier: GPL-2.0-only
/*
 * snd-pio-tdm - full-duplex ALSA card on the RP1 PIO block for the
 * ReSpeaker 6-Mic HAT (AC101 + 2x AC108) on a Raspberry Pi 5.
 *
 * The AC101 is the bus master: BCLK 4.096 MHz on GPIO18, a 1-BCLK-wide
 * DSP_A frame sync on GPIO19 every 256 BCLK, eight 32-bit slots on GPIO20
 * (codec -> Pi, SDI) and Pi -> codec data on GPIO21 (SDO).  RP1's
 * DesignWare I2S1 can receive on this bus (it stays as the "codec clock
 * heartbeat", see seeed-codec-heartbeat.service in pi5/install-pi5.sh) but
 * its transmitter never
 * shifts a word against the pulse WS, so both audio directions go through
 * two PIO state machines:
 *
 *   RX SM: waits for WS and clocks in exactly 256 bits per frame, autopush
 *          every 32 -> the DMA stream *is* 8ch S32_LE, no CPU unpacking and
 *          channel rotation is structurally impossible.
 *   TX SM: waits for WS and drives slots 0/1 (the AC101's timeslot 0, L/R)
 *          on GPIO21, 32 bits each, autopull every 32; line low elsewhere.
 *
 * Each direction owns a dw-axi-dmac channel (requested from our DT node with
 * the heavy/burst-8 flag) running a hardware cyclic ring straight into the
 * ALSA buffer.  The PIO DMACTRL DREQ threshold is set to 8 words on a joined
 * 16-deep FIFO for both directions (0x80000108) - the kernel defaults
 * (RX threshold 1, TX 0x80000104 unjoined) were measured to duplicate / drop
 * words against the 8-word DMAC burst.
 *
 * Every pio_* call is a sleeping RP1 firmware mailbox RPC, hence
 * pcm->nonatomic = true and snd_pcm_period_elapsed() from a work item.
 *
 * Design and measurements: pi5/DESIGN.md, pi5/README.md.
 */

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pio_rp1.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

#define DRV_NAME		"snd-pio-tdm"
#define CARD_ID			"seeed8micpio"

#define PIO_TDM_RATE		16000
#define PIO_TDM_RX_CHANNELS	8
#define PIO_TDM_TX_CHANNELS	2
#define PIO_TDM_SAMPLE_BYTES	4

/* RP1 PIO FIFO register offsets (data ports the DMAC reads/writes) */
#define RP1_PIO_FIFO_TX0	0x00
#define RP1_PIO_FIFO_RX0	0x10

/* RP1 DMA handshake ids (dt-bindings/mfd/rp1.h): PIO_CHn_TX = 0x38 + 2n, RX = 0x39 + 2n */
#define RP1_DMA_PIO_CH0_TX	0x38
#define RP1_DMA_PIO_CH0_RX	0x39

/* DMACTRL: bit 31 enable, bit 8 always set, bits 4:0 DREQ threshold (words). */
#define PIO_TDM_DMACTRL(thresh)	(0x80000100u | ((thresh) & 0x1f))
#define PIO_TDM_FIFO_JOINED	16
#define PIO_TDM_MAX_BURST	8

#define PIO_TDM_WDOG_MS		250
#define PIO_TDM_WDOG_STALL_TICKS 2	/* 500..750 ms without a DMA period = bus stalled */
/*
 * ALSA's own read/write wait (wait_for_avail) gives up with -EIO after
 * max(100 ms, 1.1 x buffer time); that would beat the watchdog and turn a
 * codec-clock stall into an opaque EIO instead of the XRUN -> re-prepare ->
 * bus probe path the design wants.  Give the core a longer leash; the watchdog
 * wakes the waiter through snd_pcm_stop_xrun() long before this expires.
 */
#define PIO_TDM_RW_WAIT_MS	3000

static int rx_origin = -1;
module_param(rx_origin, int, 0444);
MODULE_PARM_DESC(rx_origin, "PIO instruction memory origin for the RX program (-1 = any)");

static int tx_origin = -1;
module_param(tx_origin, int, 0444);
MODULE_PARM_DESC(tx_origin, "PIO instruction memory origin for the TX program (-1 = any)");

static uint rx_dreq = 8;
module_param(rx_dreq, uint, 0444);
MODULE_PARM_DESC(rx_dreq, "RX DMACTRL DREQ threshold in words (default 8 = DMAC burst)");

static uint tx_dreq = 8;
module_param(tx_dreq, uint, 0444);
MODULE_PARM_DESC(tx_dreq, "TX DMACTRL DREQ threshold in words (default 8 = 16-deep FIFO minus burst)");

static bool bus_probe = true;
module_param(bus_probe, bool, 0644);
MODULE_PARM_DESC(bus_probe, "Refuse .prepare with -EIO when no BCLK/WS is seen (default on)");

/*
 * Diagnostic: point the RX SM at a different pin than sdi-gpios.  Setting this
 * to the SDO pin makes the (unmodified) RX program clock in our own TX line,
 * bit-exact and frame-aligned, so a capture shows literally what we put on the
 * wire.  -1 = normal operation.
 */
static int rx_gpio = -1;
module_param(rx_gpio, int, 0444);
MODULE_PARM_DESC(rx_gpio, "Override the RX SM input GPIO for loopback diagnostics (-1 = use sdi-gpios)");

/*
 * PIO programs.  Instruction words are program-relative (the RP1 firmware
 * loader adds the load offset to every JMP target, SDK-style - measured).
 * The tables below are the words verified on the bus by the userspace proof
 * (tdmrx/tdmtx); the encoder-built copies must match them for GPIO 18/19/20/21.
 */
#define PIO_TDM_RX_LEN		19
#define PIO_TDM_RX_WRAP_TARGET	3
#define PIO_TDM_RX_WRAP		18
static const u16 pio_tdm_rx_ref[PIO_TDM_RX_LEN] = {
	0x2093,	/*  0: wait 1 gpio 19   ; WS pulse (entry only)                  */
	0x2092,	/*  1: wait 1 gpio 18   ; R0 = previous frame's bit 255            */
	0x2012,	/*  2: wait 0 gpio 18   ; F0                                       */
	0xe046,	/*  3: set y, 6         ; .wrap_target - 7 full slots              */
	0xe03f,	/*  4: set x, 31                                                   */
	0x2092,	/*  5: wait 1 gpio 18   ; R(k+1): sample                           */
	0x4001,	/*  6: in pins, 1       ; bit k (GPIO20)                           */
	0x2012,	/*  7: wait 0 gpio 18                                              */
	0x0045,	/*  8: jmp x--, 5                                                  */
	0x0084,	/*  9: jmp y--, 4                                                  */
	0xe03e,	/* 10: set x, 30        ; slot 7 bits 0..30                        */
	0x2092,	/* 11: wait 1 gpio 18                                              */
	0x4001,	/* 12: in pins, 1                                                  */
	0x2012,	/* 13: wait 0 gpio 18   ; F255 - WS rises here                     */
	0x004b,	/* 14: jmp x--, 11                                                 */
	0x2093,	/* 15: wait 1 gpio 19   ; per-frame WS resync, zero bit cost       */
	0x2092,	/* 16: wait 1 gpio 18   ; R256                                     */
	0x4001,	/* 17: in pins, 1       ; bit 255 -> 8th autopush                  */
	0x2012,	/* 18: wait 0 gpio 18   ; F256, .wrap -> 3                         */
};

#define PIO_TDM_TX_LEN		11
#define PIO_TDM_TX_WRAP_TARGET	0
#define PIO_TDM_TX_WRAP		10
static const u16 pio_tdm_tx_ref[PIO_TDM_TX_LEN] = {
	0x2093,	/*  0: wait 1 gpio 19   ; WS pulse                                 */
	0x2092,	/*  1: wait 1 gpio 18   ; R0 (DSP_A: bit 0 sampled at R1)          */
	0xe041,	/*  2: set y, 1         ; 2 slots                                  */
	0xe03f,	/*  3: set x, 31        ; 32 bits                                  */
	0x2012,	/*  4: wait 0 gpio 18   ; F(k): drive on the falling edge          */
	0x6001,	/*  5: out pins, 1                                                 */
	0x2092,	/*  6: wait 1 gpio 18   ; R(k+1): codec samples                    */
	0x0044,	/*  7: jmp x--, 4                                                  */
	0x0083,	/*  8: jmp y--, 3                                                  */
	0x2012,	/*  9: wait 0 gpio 18   ; F64                                      */
	0xe000,	/* 10: set pins, 0      ; line low for slots 2..7, .wrap           */
};

struct pio_tdm;

struct pio_tdm_stream {
	struct pio_tdm *tdm;
	const char *name;
	bool is_tx;
	unsigned int sm;
	unsigned int origin;
	unsigned int dreq;
	u16 insns[PIO_TDM_RX_LEN];
	unsigned int insn_count;
	pio_sm_config cfg;

	struct dma_chan *chan;
	unsigned int burst;
	dma_addr_t fifo_addr;
	dma_cookie_t cookie;

	struct snd_pcm_substream *substream;
	bool running;			/* written under the PCM stream lock AND tdm->lock */
	atomic_t periods;		/* DMA period callbacks since START */
	unsigned int wdog_last;
	unsigned int wdog_stall;
	struct work_struct period_work;
	struct delayed_work wdog;

	/* counters */
	atomic_long_t total_periods;
	atomic_long_t period_bytes_last;
	atomic_long_t faults;		/* RX FIFO full seen / TX FIFO empty seen while running */
	atomic_long_t restarts;		/* SM re-initialised by .prepare after a fault */
	atomic_long_t xruns_forced;	/* watchdog forced snd_pcm_stop_xrun() */
	atomic_long_t dma_errors;	/* cyclic ring died (dmac handle_err completed the cookie) */
	bool faulted;
};

struct pio_tdm {
	struct device *dev;
	struct rp1_pio_client *pio;
	struct mutex lock;		/* serialises multi-RPC PIO sequences between the two streams */
	phys_addr_t pio_base;
	unsigned int gpio_bclk, gpio_ws, gpio_sdi, gpio_sdo;
	struct pio_tdm_stream rx, tx;
	struct snd_card *card;
	struct snd_pcm *pcm;
	atomic_long_t bus_stalls;
	int bus_alive;			/* result of the last .prepare probe: 1 alive, 0 dead, -1 unknown */
};

/* ------------------------------------------------------------------------ */
/* PIO program construction                                                 */

static void pio_tdm_build_rx(struct pio_tdm *tdm, struct pio_tdm_stream *s)
{
	const unsigned int ws = tdm->gpio_ws, bclk = tdm->gpio_bclk;
	u16 *p = s->insns;

	p[0] = pio_encode_wait_gpio(true, ws);
	p[1] = pio_encode_wait_gpio(true, bclk);
	p[2] = pio_encode_wait_gpio(false, bclk);
	p[3] = pio_encode_set(pio_y, 6);
	p[4] = pio_encode_set(pio_x, 31);
	p[5] = pio_encode_wait_gpio(true, bclk);
	p[6] = pio_encode_in(pio_pins, 1);
	p[7] = pio_encode_wait_gpio(false, bclk);
	p[8] = pio_encode_jmp_x_dec(5);
	p[9] = pio_encode_jmp_y_dec(4);
	p[10] = pio_encode_set(pio_x, 30);
	p[11] = pio_encode_wait_gpio(true, bclk);
	p[12] = pio_encode_in(pio_pins, 1);
	p[13] = pio_encode_wait_gpio(false, bclk);
	p[14] = pio_encode_jmp_x_dec(11);
	p[15] = pio_encode_wait_gpio(true, ws);
	p[16] = pio_encode_wait_gpio(true, bclk);
	p[17] = pio_encode_in(pio_pins, 1);
	p[18] = pio_encode_wait_gpio(false, bclk);
	s->insn_count = PIO_TDM_RX_LEN;
}

static void pio_tdm_build_tx(struct pio_tdm *tdm, struct pio_tdm_stream *s)
{
	const unsigned int ws = tdm->gpio_ws, bclk = tdm->gpio_bclk;
	u16 *p = s->insns;

	p[0] = pio_encode_wait_gpio(true, ws);
	p[1] = pio_encode_wait_gpio(true, bclk);
	p[2] = pio_encode_set(pio_y, 1);
	p[3] = pio_encode_set(pio_x, 31);
	p[4] = pio_encode_wait_gpio(false, bclk);
	p[5] = pio_encode_out(pio_pins, 1);
	p[6] = pio_encode_wait_gpio(true, bclk);
	p[7] = pio_encode_jmp_x_dec(4);
	p[8] = pio_encode_jmp_y_dec(3);
	p[9] = pio_encode_wait_gpio(false, bclk);
	p[10] = pio_encode_set(pio_pins, 0);
	s->insn_count = PIO_TDM_TX_LEN;
}

static void pio_tdm_build_config(struct pio_tdm *tdm, struct pio_tdm_stream *s)
{
	pio_sm_config c = pio_get_default_sm_config();

	sm_config_set_clkdiv_int_frac(&c, 1, 0);
	if (s->is_tx) {
		sm_config_set_out_pins(&c, tdm->gpio_sdo, 1);
		sm_config_set_set_pins(&c, tdm->gpio_sdo, 1);
		sm_config_set_out_shift(&c, false, true, 32);	/* MSB first, autopull @32 */
		sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
		sm_config_set_wrap(&c, s->origin + PIO_TDM_TX_WRAP_TARGET,
				   s->origin + PIO_TDM_TX_WRAP);
	} else {
		sm_config_set_in_pins(&c, rx_gpio >= 0 ? rx_gpio : tdm->gpio_sdi);
		sm_config_set_in_shift(&c, false, true, 32);	/* MSB first, autopush @32 */
		sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
		sm_config_set_wrap(&c, s->origin + PIO_TDM_RX_WRAP_TARGET,
				   s->origin + PIO_TDM_RX_WRAP);
	}
	s->cfg = c;
}

/*
 * Load a program.  A fixed origin is honoured; -1 lets the firmware pick.
 * rp1_pio_add_program() returns the offset (>= 0) or a negative errno.
 */
static int pio_tdm_load_program(struct pio_tdm *tdm, struct pio_tdm_stream *s, int origin)
{
	struct pio_program prog = {
		.instructions = s->insns,
		.length = s->insn_count,
		.origin = -1,
	};
	int ret;

	if (origin >= 0) {
		if (origin + s->insn_count > PIO_INSTRUCTION_COUNT)
			return -EINVAL;
		ret = pio_add_program_at_offset(tdm->pio, &prog, origin);
	} else {
		uint off = pio_add_program(tdm->pio, &prog);

		ret = (off == PIO_ORIGIN_INVALID) ? -ENOSPC : (int)off;
	}
	if (ret < 0)
		return ret;
	if (ret + s->insn_count > PIO_INSTRUCTION_COUNT)
		return -EIO;
	s->origin = ret;
	return 0;
}

/* Put an SM into the "configured, stopped, FIFOs clear, PC at origin" state. */
static int pio_tdm_sm_reset(struct pio_tdm *tdm, struct pio_tdm_stream *s)
{
	int ret;

	ret = pio_sm_set_enabled(tdm->pio, s->sm, false);
	if (ret < 0)
		return ret;
	ret = pio_sm_init(tdm->pio, s->sm, s->origin, &s->cfg);
	if (ret < 0)
		return ret;
	ret = pio_sm_clear_fifos(tdm->pio, s->sm);
	if (ret < 0)
		return ret;
	/* SM_INIT may (re)write shift/exec state; make sure the DREQ threshold is ours. */
	ret = pio_sm_set_dmactrl(tdm->pio, s->sm, s->is_tx, PIO_TDM_DMACTRL(s->dreq));
	if (ret < 0)
		return ret;
	if (s->is_tx) {
		/* SDO low and driven; the pad's OE comes from the SM PINDIRS. */
		ret = pio_sm_set_pins_with_mask(tdm->pio, s->sm, 0, BIT(tdm->gpio_sdo));
		if (ret < 0)
			return ret;
		ret = pio_sm_set_consecutive_pindirs(tdm->pio, s->sm, tdm->gpio_sdo, 1, true);
		if (ret < 0)
			return ret;
	}
	return 0;
}

/*
 * Bus-alive probe: run the RX SM for ~1.5 ms with no DMA attached; with
 * BCLK/WS present the joined FIFO fills (16 words).  Leaves the RX SM reset.
 * Caller holds tdm->lock and guarantees the RX stream is not running.
 */
static int pio_tdm_probe_bus(struct pio_tdm *tdm)
{
	struct pio_tdm_stream *s = &tdm->rx;
	int level, ret;

	ret = pio_tdm_sm_reset(tdm, s);
	if (ret < 0)
		return ret;
	ret = pio_sm_set_enabled(tdm->pio, s->sm, true);
	if (ret < 0)
		return ret;
	usleep_range(1500, 3000);
	level = pio_sm_rx_fifo_level(tdm->pio, s->sm);
	ret = pio_tdm_sm_reset(tdm, s);
	if (level < 0)
		return level;
	if (ret < 0)
		return ret;
	/*
	 * pio_sm_rx_fifo_level() returns the level only when the firmware reply
	 * is exactly sizeof(fifo_state_args); any other positive value is a raw
	 * byte count from a protocol change - never mistake that for a live bus.
	 */
	if (level > PIO_TDM_FIFO_JOINED) {
		dev_warn_once(tdm->dev, "fifo_state protocol mismatch (level %d > %u)\n",
			      level, PIO_TDM_FIFO_JOINED);
		tdm->bus_alive = -1;
		return -EIO;
	}
	tdm->bus_alive = level > 0;
	return level > 0 ? 0 : -EIO;
}

/* ------------------------------------------------------------------------ */
/* Work items                                                                */

static void pio_tdm_dma_callback(void *param)
{
	struct pio_tdm_stream *s = param;

	/* tasklet context: no mailbox calls, no PCM (nonatomic mutex) here */
	atomic_inc(&s->periods);
	atomic_long_inc(&s->total_periods);
	queue_work(system_highpri_wq, &s->period_work);
}

static void pio_tdm_period_work(struct work_struct *work)
{
	struct pio_tdm_stream *s = container_of(work, struct pio_tdm_stream, period_work);
	struct snd_pcm_substream *substream = READ_ONCE(s->substream);

	if (substream)
		snd_pcm_period_elapsed(substream);
}

static void pio_tdm_wdog_work(struct work_struct *work)
{
	struct pio_tdm_stream *s = container_of(to_delayed_work(work),
						struct pio_tdm_stream, wdog);
	struct pio_tdm *tdm = s->tdm;
	struct snd_pcm_substream *substream = READ_ONCE(s->substream);
	enum dma_status dma;
	unsigned int now;
	bool dead;
	int st;

	if (!substream || !READ_ONCE(s->running))
		return;

	/*
	 * The hardware checks run under tdm->lock, where trigger(STOP) also flips
	 * `running`: a stream stopped a moment ago must not have its parked SM
	 * (trivially full RX / empty TX FIFO) or its terminated ring counted as a
	 * fault.  tdm->lock is NEVER held across snd_pcm_stop_xrun() (stream lock)
	 * - trigger holds the stream lock and then takes tdm->lock.
	 */
	mutex_lock(&tdm->lock);
	if (!READ_ONCE(s->running)) {
		mutex_unlock(&tdm->lock);
		return;
	}
	/*
	 * A cyclic descriptor is DMA_IN_PROGRESS for its whole life; the only
	 * way it completes is the dmac's error path (LLI fetched before the IRQ
	 * re-validated it -> channel disabled, cookie completed) = ring dead.
	 */
	dma = dmaengine_tx_status(s->chan, s->cookie, NULL);
	dead = dma != DMA_IN_PROGRESS;
	if (!dead) {
		/* One mailbox call per tick: a full RX FIFO / empty TX FIFO while running = fault. */
		st = s->is_tx ? pio_sm_is_tx_fifo_empty(tdm->pio, s->sm)
			      : pio_sm_is_rx_fifo_full(tdm->pio, s->sm);
		if (st == 1) {
			atomic_long_inc(&s->faults);
			s->faulted = true;
		} else if (st != 0) {
			/* raw byte count or errno: fifo_state reply protocol changed */
			dev_warn_once(tdm->dev, "%s: fifo_state returned %d - protocol mismatch?\n",
				      s->name, st);
		}
	}
	mutex_unlock(&tdm->lock);

	if (dead) {
		dev_warn(tdm->dev, "%s: DMA ring stopped (status %d) - forcing XRUN\n", s->name, dma);
		atomic_long_inc(&s->dma_errors);
		atomic_long_inc(&s->xruns_forced);
		s->faulted = true;
		snd_pcm_stop_xrun(substream);
		return;
	}

	now = atomic_read(&s->periods);
	if (now == s->wdog_last) {
		if (++s->wdog_stall >= PIO_TDM_WDOG_STALL_TICKS) {
			dev_warn(tdm->dev, "%s: no DMA progress for %u ms - bus stalled? forcing XRUN\n",
				 s->name, PIO_TDM_WDOG_STALL_TICKS * PIO_TDM_WDOG_MS);
			atomic_long_inc(&tdm->bus_stalls);
			atomic_long_inc(&s->xruns_forced);
			s->faulted = true;
			tdm->bus_alive = 0;
			snd_pcm_stop_xrun(substream);
			return;
		}
	} else {
		s->wdog_stall = 0;
		s->wdog_last = now;
	}

	if (READ_ONCE(s->running))
		schedule_delayed_work(&s->wdog, msecs_to_jiffies(PIO_TDM_WDOG_MS));
}

/* ------------------------------------------------------------------------ */
/* ALSA PCM                                                                  */

/*
 * The rpi dw-axi-dmac cyclic ring re-validates each period's LLIs from the
 * period IRQ; if the DMAC reaches a period whose LLI_VALID the IRQ has not yet
 * restored the channel errors out and the ring dies.  The IRQ latency budget
 * is therefore (periods - 1) periods: require >= 3 periods of >= 5 ms so one
 * late IRQ (or two periods coalesced into one tasklet run) costs nothing.
 */
#define PIO_TDM_PERIOD_MIN_FRAMES	80	/* 5 ms @ 16 kHz */
#define PIO_TDM_PERIODS_MIN		3

static const struct snd_pcm_hardware pio_tdm_rx_hw = {
	.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID,
	.formats = SNDRV_PCM_FMTBIT_S32_LE,
	.rates = SNDRV_PCM_RATE_16000,
	.rate_min = PIO_TDM_RATE,
	.rate_max = PIO_TDM_RATE,
	.channels_min = PIO_TDM_RX_CHANNELS,
	.channels_max = PIO_TDM_RX_CHANNELS,
	.period_bytes_min = PIO_TDM_PERIOD_MIN_FRAMES * PIO_TDM_RX_CHANNELS * PIO_TDM_SAMPLE_BYTES,
	.period_bytes_max = 65536,
	.periods_min = PIO_TDM_PERIODS_MIN,
	.periods_max = 16,
	.buffer_bytes_max = 262144,
	.fifo_size = PIO_TDM_FIFO_JOINED,
};

static const struct snd_pcm_hardware pio_tdm_tx_hw = {
	.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID,
	.formats = SNDRV_PCM_FMTBIT_S32_LE,
	.rates = SNDRV_PCM_RATE_16000,
	.rate_min = PIO_TDM_RATE,
	.rate_max = PIO_TDM_RATE,
	.channels_min = PIO_TDM_TX_CHANNELS,
	.channels_max = PIO_TDM_TX_CHANNELS,
	.period_bytes_min = PIO_TDM_PERIOD_MIN_FRAMES * PIO_TDM_TX_CHANNELS * PIO_TDM_SAMPLE_BYTES,
	.period_bytes_max = 16384,
	.periods_min = PIO_TDM_PERIODS_MIN,
	.periods_max = 16,
	.buffer_bytes_max = 65536,
	.fifo_size = PIO_TDM_FIFO_JOINED,
};

static struct pio_tdm_stream *pio_tdm_substream(struct snd_pcm_substream *substream)
{
	struct pio_tdm *tdm = snd_pcm_substream_chip(substream);

	return substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? &tdm->tx : &tdm->rx;
}

static int pio_tdm_pcm_open(struct snd_pcm_substream *substream)
{
	struct pio_tdm_stream *s = pio_tdm_substream(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret;

	runtime->hw = s->is_tx ? pio_tdm_tx_hw : pio_tdm_rx_hw;

	/* Periods and buffers must be whole 8-word DMAC bursts (32 B) - also whole RX frames. */
	ret = snd_pcm_hw_constraint_step(runtime, 0, SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 32);
	if (ret < 0)
		return ret;
	ret = snd_pcm_hw_constraint_step(runtime, 0, SNDRV_PCM_HW_PARAM_BUFFER_BYTES, 32);
	if (ret < 0)
		return ret;
	ret = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		return ret;

	substream->wait_time = PIO_TDM_RW_WAIT_MS;
	s->faulted = false;
	WRITE_ONCE(s->substream, substream);
	return 0;
}

static int pio_tdm_pcm_close(struct snd_pcm_substream *substream)
{
	struct pio_tdm_stream *s = pio_tdm_substream(substream);

	cancel_delayed_work_sync(&s->wdog);
	cancel_work_sync(&s->period_work);
	WRITE_ONCE(s->substream, NULL);
	return 0;
}

static int pio_tdm_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct pio_tdm_stream *s = pio_tdm_substream(substream);

	/* trigger(STOP) used terminate_async; make the channel quiescent before the buffer goes */
	dmaengine_terminate_sync(s->chan);
	cancel_delayed_work_sync(&s->wdog);
	cancel_work_sync(&s->period_work);
	return 0;
}

static int pio_tdm_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct pio_tdm_stream *s = pio_tdm_substream(substream);
	struct pio_tdm *tdm = s->tdm;
	int ret;

	mutex_lock(&tdm->lock);

	if (s->faulted) {
		atomic_long_inc(&s->restarts);
		s->faulted = false;
	}

	/*
	 * Bus-alive probe on the RX SM.  Only when the RX stream is not running:
	 * a running capture is itself proof the bus is clocked (its own watchdog
	 * would otherwise have stopped it).
	 */
	if (bus_probe && !READ_ONCE(tdm->rx.running)) {
		ret = pio_tdm_probe_bus(tdm);
		if (ret == -EIO) {
			dev_err_ratelimited(tdm->dev,
				"%s: codec bus not clocked (no BCLK/WS on GPIO%u/%u) - is seeed-codec-heartbeat running?\n",
				s->name, tdm->gpio_bclk, tdm->gpio_ws);
			goto out;
		}
		if (ret < 0)
			goto out;
	}

	ret = pio_tdm_sm_reset(tdm, s);
out:
	mutex_unlock(&tdm->lock);
	return ret;
}

static int pio_tdm_start(struct pio_tdm_stream *s)
{
	struct pio_tdm *tdm = s->tdm;
	struct snd_pcm_runtime *runtime = s->substream->runtime;
	struct dma_async_tx_descriptor *desc;
	size_t buffer_bytes = snd_pcm_lib_buffer_bytes(s->substream);
	size_t period_bytes = snd_pcm_lib_period_bytes(s->substream);
	int ret;

	/*
	 * Whole sequence under tdm->lock: the other stream's .prepare bus probe
	 * runs the RX SM without DMA and reads its FIFO level - an RX DMA armed
	 * meanwhile would drain that fill and fake a dead bus.
	 */
	mutex_lock(&tdm->lock);

	/* Arm the DMA first: RX so DREQ is honoured from word 8, TX so the FIFO is full at the first WS. */
	desc = dmaengine_prep_dma_cyclic(s->chan, runtime->dma_addr, buffer_bytes, period_bytes,
					 s->is_tx ? DMA_MEM_TO_DEV : DMA_DEV_TO_MEM,
					 DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc) {
		dev_err(tdm->dev, "%s: dmaengine_prep_dma_cyclic(%zu/%zu) failed\n",
			s->name, buffer_bytes, period_bytes);
		ret = -EIO;
		goto out;
	}
	desc->callback = pio_tdm_dma_callback;
	desc->callback_param = s;
	s->cookie = dmaengine_submit(desc);
	ret = dma_submit_error(s->cookie);
	if (ret) {
		dev_err(tdm->dev, "%s: dmaengine_submit failed (%d)\n", s->name, ret);
		goto out;
	}
	atomic_set(&s->periods, 0);
	atomic_long_set(&s->period_bytes_last, period_bytes);
	s->wdog_last = 0;
	s->wdog_stall = 0;
	dma_async_issue_pending(s->chan);

	/* Give the DMAC time to fill the TX FIFO before the SM meets its first WS. */
	if (s->is_tx)
		usleep_range(50, 100);

	ret = pio_sm_set_enabled(tdm->pio, s->sm, true);
	if (ret < 0) {
		dmaengine_terminate_async(s->chan);
		goto out;
	}
	WRITE_ONCE(s->running, true);
	schedule_delayed_work(&s->wdog, msecs_to_jiffies(PIO_TDM_WDOG_MS));
out:
	mutex_unlock(&tdm->lock);
	return ret;
}

static int pio_tdm_stop(struct pio_tdm_stream *s)
{
	struct pio_tdm *tdm = s->tdm;
	int ret;

	/*
	 * Whole sequence under tdm->lock, mirroring START: the other stream's
	 * .prepare bus probe (which trusts rx.running) and the watchdog's FIFO
	 * poll must see either a running stream or a fully parked one - never
	 * a draining DMA with `running` already false.
	 */
	mutex_lock(&tdm->lock);
	WRITE_ONCE(s->running, false);
	cancel_delayed_work(&s->wdog);	/* not _sync: the watchdog may be waiting on the stream lock we hold */

	/*
	 * Terminate the DMA while the SM is still moving data so the DMAC is not
	 * parked on a DREQ that will never come (dw-axi-dmac "failed to stop").
	 * _sync (allowed: nonatomic trigger) drains the vchan tasklet here rather
	 * than leaving it to run against the just-freed cyclic descriptor.
	 */
	dmaengine_terminate_sync(s->chan);
	ret = pio_sm_set_enabled(tdm->pio, s->sm, false);
	if (s->is_tx)
		pio_sm_set_pins_with_mask(tdm->pio, s->sm, 0, BIT(tdm->gpio_sdo));
	mutex_unlock(&tdm->lock);
	return ret;
}

static int pio_tdm_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct pio_tdm_stream *s = pio_tdm_substream(substream);

	/* nonatomic PCM: this runs in process context under a mutex and may sleep */
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		return pio_tdm_start(s);
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		return pio_tdm_stop(s);
	default:
		return -EINVAL;
	}
}

static snd_pcm_uframes_t pio_tdm_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct pio_tdm_stream *s = pio_tdm_substream(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	size_t buffer_bytes = snd_pcm_lib_buffer_bytes(substream);
	size_t period_bytes = snd_pcm_lib_period_bytes(substream);
	struct dma_tx_state state;
	enum dma_status status;
	size_t pos;

	status = dmaengine_tx_status(s->chan, s->cookie, &state);
	if (status != DMA_IN_PROGRESS && READ_ONCE(s->running)) {
		/* cyclic ring died (dmac error path) - no valid position: XRUN now, not in 500 ms */
		dev_warn_ratelimited(s->tdm->dev, "%s: DMA ring stopped (status %d) - XRUN\n",
				     s->name, status);
		atomic_long_inc(&s->dma_errors);
		s->faulted = true;
		return SNDRV_PCM_POS_XRUN;
	}
	if (state.residue > 0 && state.residue <= buffer_bytes)
		pos = buffer_bytes - state.residue;
	else
		pos = ((size_t)atomic_read(&s->periods) * period_bytes) % buffer_bytes;
	if (pos >= buffer_bytes)
		pos = 0;
	return bytes_to_frames(runtime, pos);
}

static const struct snd_pcm_ops pio_tdm_pcm_ops = {
	.open = pio_tdm_pcm_open,
	.close = pio_tdm_pcm_close,
	.hw_free = pio_tdm_pcm_hw_free,
	.prepare = pio_tdm_pcm_prepare,
	.trigger = pio_tdm_pcm_trigger,
	.pointer = pio_tdm_pcm_pointer,
};

/* ------------------------------------------------------------------------ */
/* sysfs                                                                     */

static ssize_t pio_tdm_stream_frames(struct pio_tdm_stream *s, char *buf)
{
	unsigned long periods = atomic_long_read(&s->total_periods);
	unsigned long pb = atomic_long_read(&s->period_bytes_last);
	unsigned int frame_bytes = (s->is_tx ? PIO_TDM_TX_CHANNELS : PIO_TDM_RX_CHANNELS) *
				   PIO_TDM_SAMPLE_BYTES;

	return sysfs_emit(buf, "%lu\n", pb ? periods * (pb / frame_bytes) : 0);
}

#define PIO_TDM_STREAM_ATTR(_dir, _name, _expr)						\
static ssize_t _dir##_##_name##_show(struct device *dev, struct device_attribute *attr,	\
				     char *buf)						\
{											\
	struct pio_tdm *tdm = dev_get_drvdata(dev);					\
	struct pio_tdm_stream *s = &tdm->_dir;						\
											\
	(void)s;									\
	return _expr;									\
}											\
static DEVICE_ATTR_RO(_dir##_##_name)

#define PIO_TDM_STREAM_ATTRS(_dir)							\
PIO_TDM_STREAM_ATTR(_dir, frames, pio_tdm_stream_frames(s, buf));			\
PIO_TDM_STREAM_ATTR(_dir, periods, sysfs_emit(buf, "%lu\n", atomic_long_read(&s->total_periods))); \
PIO_TDM_STREAM_ATTR(_dir, faults, sysfs_emit(buf, "%lu\n", atomic_long_read(&s->faults)));	\
PIO_TDM_STREAM_ATTR(_dir, restarts, sysfs_emit(buf, "%lu\n", atomic_long_read(&s->restarts))); \
PIO_TDM_STREAM_ATTR(_dir, xruns_forced, sysfs_emit(buf, "%lu\n", atomic_long_read(&s->xruns_forced))); \
PIO_TDM_STREAM_ATTR(_dir, dma_errors, sysfs_emit(buf, "%lu\n", atomic_long_read(&s->dma_errors))); \
PIO_TDM_STREAM_ATTR(_dir, sm, sysfs_emit(buf, "%u\n", s->sm));				\
PIO_TDM_STREAM_ATTR(_dir, prog_origin, sysfs_emit(buf, "%u\n", s->origin));		\
PIO_TDM_STREAM_ATTR(_dir, dma_chan, sysfs_emit(buf, "%s\n", dma_chan_name(s->chan)));	\
PIO_TDM_STREAM_ATTR(_dir, dma_burst, sysfs_emit(buf, "%u\n", s->burst));		\
PIO_TDM_STREAM_ATTR(_dir, dmactrl, sysfs_emit(buf, "0x%08x\n", PIO_TDM_DMACTRL(s->dreq))); \
PIO_TDM_STREAM_ATTR(_dir, running, sysfs_emit(buf, "%d\n", READ_ONCE(s->running)));	\
PIO_TDM_STREAM_ATTR(_dir, fifo_level, sysfs_emit(buf, "%d\n", s->is_tx ?		\
			pio_sm_tx_fifo_level(tdm->pio, s->sm) : pio_sm_rx_fifo_level(tdm->pio, s->sm)))

PIO_TDM_STREAM_ATTRS(rx);
PIO_TDM_STREAM_ATTRS(tx);

static ssize_t bus_stalls_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pio_tdm *tdm = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%lu\n", atomic_long_read(&tdm->bus_stalls));
}
static DEVICE_ATTR_RO(bus_stalls);

static ssize_t bus_alive_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pio_tdm *tdm = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", tdm->bus_alive);
}
static DEVICE_ATTR_RO(bus_alive);

static ssize_t gpios_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pio_tdm *tdm = dev_get_drvdata(dev);

	return sysfs_emit(buf, "bclk=%u ws=%u sdi=%u sdo=%u\n", tdm->gpio_bclk, tdm->gpio_ws,
			  tdm->gpio_sdi, tdm->gpio_sdo);
}
static DEVICE_ATTR_RO(gpios);

#define PIO_TDM_STREAM_ATTR_LIST(_dir)							\
	&dev_attr_##_dir##_frames.attr, &dev_attr_##_dir##_periods.attr,		\
	&dev_attr_##_dir##_faults.attr, &dev_attr_##_dir##_restarts.attr,		\
	&dev_attr_##_dir##_xruns_forced.attr, &dev_attr_##_dir##_dma_errors.attr,	\
	&dev_attr_##_dir##_sm.attr,							\
	&dev_attr_##_dir##_prog_origin.attr, &dev_attr_##_dir##_dma_chan.attr,		\
	&dev_attr_##_dir##_dma_burst.attr, &dev_attr_##_dir##_dmactrl.attr,		\
	&dev_attr_##_dir##_running.attr, &dev_attr_##_dir##_fifo_level.attr

static struct attribute *pio_tdm_attrs[] = {
	PIO_TDM_STREAM_ATTR_LIST(rx),
	PIO_TDM_STREAM_ATTR_LIST(tx),
	&dev_attr_bus_stalls.attr,
	&dev_attr_bus_alive.attr,
	&dev_attr_gpios.attr,
	NULL,
};
ATTRIBUTE_GROUPS(pio_tdm);

/* ------------------------------------------------------------------------ */
/* Probe / remove                                                            */

static int pio_tdm_parse_gpio(struct device_node *np, const char *prop, unsigned int def,
			      unsigned int *out)
{
	struct of_phandle_args a;
	int ret;

	ret = of_parse_phandle_with_args(np, prop, "#gpio-cells", 0, &a);
	if (ret == -ENOENT || ret == -EINVAL) {
		*out = def;
		return 0;
	}
	if (ret)
		return ret;
	ret = 0;
	if (!of_device_is_compatible(a.np, "raspberrypi,rp1-gpio") || a.args_count < 1 ||
	    a.args[0] >= RP1_PIO_GPIO_COUNT)
		ret = -EINVAL;
	else
		*out = a.args[0];
	of_node_put(a.np);
	return ret;
}

/*
 * Derive the SM number from the DMA handshake id in our own dmas cell.  The
 * cell is looked up by the same dma-names entry dma_request_chan() uses, so
 * the SM and the channel can never be paired from different cells.
 */
static int pio_tdm_dma_cell_sm(struct device_node *np, const char *name, bool is_tx,
			       unsigned int *sm)
{
	struct of_phandle_args a;
	unsigned int hs;
	int idx, ret;

	idx = of_property_match_string(np, "dma-names", name);
	if (idx < 0)
		return idx;
	ret = of_parse_phandle_with_args(np, "dmas", "#dma-cells", idx, &a);
	if (ret)
		return ret;
	of_node_put(a.np);
	if (a.args_count < 1)
		return -EINVAL;
	hs = a.args[0] & 0xff;
	if (hs < RP1_DMA_PIO_CH0_TX || hs > RP1_DMA_PIO_CH0_TX + 2 * NUM_PIO_STATE_MACHINES - 1)
		return -EINVAL;
	if (((hs - RP1_DMA_PIO_CH0_TX) & 1) != (is_tx ? 0 : 1))
		return -EINVAL;
	*sm = (hs - RP1_DMA_PIO_CH0_TX) >> 1;
	return 0;
}

static int pio_tdm_setup_dma(struct pio_tdm *tdm, struct pio_tdm_stream *s)
{
	struct dma_slave_config cfg = {};
	struct dma_slave_caps caps = {};
	int ret;

	s->chan = dma_request_chan(tdm->dev, s->is_tx ? "tx" : "rx");
	if (IS_ERR(s->chan))
		return dev_err_probe(tdm->dev, PTR_ERR(s->chan), "%s: dma_request_chan failed\n",
				     s->name);

	ret = dma_get_slave_caps(s->chan, &caps);
	if (ret)
		caps.max_burst = 4;
	s->burst = min_t(unsigned int, caps.max_burst ?: 1, PIO_TDM_MAX_BURST);

	s->fifo_addr = tdm->pio_base + (s->is_tx ? RP1_PIO_FIFO_TX0 : RP1_PIO_FIFO_RX0) + 4 * s->sm;
	cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	if (s->is_tx) {
		cfg.direction = DMA_MEM_TO_DEV;
		cfg.dst_addr = s->fifo_addr;
		cfg.dst_maxburst = s->burst;
	} else {
		cfg.direction = DMA_DEV_TO_MEM;
		cfg.src_addr = s->fifo_addr;
		cfg.src_maxburst = s->burst;
	}
	ret = dmaengine_slave_config(s->chan, &cfg);
	if (ret)
		return dev_err_probe(tdm->dev, ret, "%s: dmaengine_slave_config failed\n", s->name);

	if (s->burst != PIO_TDM_MAX_BURST)
		dev_warn(tdm->dev, "%s: DMA channel %s bursts %u words, DREQ threshold %u tuned for 8\n",
			 s->name, dma_chan_name(s->chan), s->burst, s->dreq);
	return 0;
}

/*
 * Software-only init, done for BOTH streams before any hardware setup so the
 * probe error path can release either stream unconditionally.
 */
static void pio_tdm_init_stream(struct pio_tdm *tdm, struct pio_tdm_stream *s, bool is_tx)
{
	s->tdm = tdm;
	s->is_tx = is_tx;
	s->name = is_tx ? "tx" : "rx";
	s->dreq = is_tx ? tx_dreq : rx_dreq;
	INIT_WORK(&s->period_work, pio_tdm_period_work);
	INIT_DELAYED_WORK(&s->wdog, pio_tdm_wdog_work);
}

static int pio_tdm_setup_stream(struct pio_tdm *tdm, struct pio_tdm_stream *s)
{
	const bool is_tx = s->is_tx;
	int origin = is_tx ? tx_origin : rx_origin;
	int ret;

	ret = pio_tdm_dma_cell_sm(tdm->dev->of_node, s->name, is_tx, &s->sm);
	if (ret)
		return dev_err_probe(tdm->dev, ret, "%s: bad dmas cell (need RP1_DMA_PIO_CHn_%s)\n",
				     s->name, is_tx ? "TX" : "RX");

	ret = pio_sm_claim(tdm->pio, s->sm);
	if (ret < 0)
		return dev_err_probe(tdm->dev, ret, "%s: cannot claim PIO SM%u\n", s->name, s->sm);

	if (is_tx)
		pio_tdm_build_tx(tdm, s);
	else
		pio_tdm_build_rx(tdm, s);

	if (tdm->gpio_bclk == 18 && tdm->gpio_ws == 19 && tdm->gpio_sdi == 20 &&
	    tdm->gpio_sdo == 21) {
		const u16 *ref = is_tx ? pio_tdm_tx_ref : pio_tdm_rx_ref;

		if (WARN_ON(memcmp(s->insns, ref, s->insn_count * sizeof(u16))))
			return -EINVAL;
	}

	ret = pio_tdm_load_program(tdm, s, origin);
	if (ret < 0)
		return dev_err_probe(tdm->dev, ret,
				     "%s: cannot load %u-word PIO program at %d (instruction memory busy - tdmcap/tdmtx running?)\n",
				     s->name, s->insn_count, origin);

	pio_tdm_build_config(tdm, s);

	ret = pio_tdm_setup_dma(tdm, s);
	if (ret)
		return ret;

	if (is_tx) {
		/* Take GPIO21 for PIO (funcsel a7); pad OE is driven by the SM PINDIRS in sm_reset(). */
		ret = pio_gpio_init(tdm->pio, tdm->gpio_sdo);
		if (ret < 0)
			return dev_err_probe(tdm->dev, ret, "pio_gpio_init(%u) failed\n", tdm->gpio_sdo);
	} else if (rx_gpio >= 0) {
		/*
		 * Diagnostic loopback: the override pin is an output (PIO drives
		 * it), so its pad input buffer must be switched on for the RX SM
		 * to read back what we drive.
		 */
		ret = pio_gpio_set_input_enabled(tdm->pio, rx_gpio, true);
		if (ret < 0)
			return dev_err_probe(tdm->dev, ret,
					     "pio_gpio_set_input_enabled(%d) failed\n", rx_gpio);
		dev_warn(tdm->dev, "DIAGNOSTIC: RX SM reading GPIO%d, not sdi-gpios\n", rx_gpio);
	}

	ret = pio_tdm_sm_reset(tdm, s);
	if (ret < 0)
		return dev_err_probe(tdm->dev, ret, "%s: SM%u init failed\n", s->name, s->sm);

	dev_info(tdm->dev, "%s: SM%u prog@%u..%u %s burst %u fifo %pa dmactrl 0x%08x\n",
		 s->name, s->sm, s->origin, s->origin + s->insn_count - 1,
		 dma_chan_name(s->chan), s->burst, &s->fifo_addr, PIO_TDM_DMACTRL(s->dreq));
	return 0;
}

static void pio_tdm_release_stream(struct pio_tdm *tdm, struct pio_tdm_stream *s)
{
	if (!IS_ERR_OR_NULL(s->chan)) {
		dmaengine_terminate_sync(s->chan);
		dma_release_channel(s->chan);
		s->chan = NULL;
	}
	cancel_delayed_work_sync(&s->wdog);
	cancel_work_sync(&s->period_work);
}

static int pio_tdm_register_card(struct pio_tdm *tdm)
{
	struct snd_card *card;
	struct snd_pcm *pcm;
	int ret;

	ret = snd_card_new(tdm->dev, SNDRV_DEFAULT_IDX1, CARD_ID, THIS_MODULE, 0, &card);
	if (ret < 0)
		return ret;
	tdm->card = card;
	strscpy(card->driver, DRV_NAME, sizeof(card->driver));
	strscpy(card->shortname, CARD_ID, sizeof(card->shortname));
	snprintf(card->longname, sizeof(card->longname),
		 "ReSpeaker 6-Mic via RP1 PIO (RX SM%u, TX SM%u)", tdm->rx.sm, tdm->tx.sm);

	ret = snd_pcm_new(card, "PIO TDM", 0, 1, 1, &pcm);
	if (ret < 0)
		return ret;
	tdm->pcm = pcm;
	pcm->nonatomic = true;
	pcm->private_data = tdm;
	strscpy(pcm->name, "PIO TDM 8x32 in / 2x32 out", sizeof(pcm->name));
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &pio_tdm_pcm_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &pio_tdm_pcm_ops);

	/* Coherent buffers on the DMAC device (its dma-ranges are the ones that matter). */
	snd_pcm_set_managed_buffer(pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream,
				   SNDRV_DMA_TYPE_DEV, tdm->rx.chan->device->dev,
				   64 * 1024, pio_tdm_rx_hw.buffer_bytes_max);
	snd_pcm_set_managed_buffer(pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream,
				   SNDRV_DMA_TYPE_DEV, tdm->tx.chan->device->dev,
				   16 * 1024, pio_tdm_tx_hw.buffer_bytes_max);

	return snd_card_register(card);
}

static int pio_tdm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node, *pio_np;
	struct pio_tdm *tdm;
	struct resource res;
	int ret;

	tdm = devm_kzalloc(dev, sizeof(*tdm), GFP_KERNEL);
	if (!tdm)
		return -ENOMEM;
	tdm->dev = dev;
	tdm->bus_alive = -1;
	mutex_init(&tdm->lock);
	platform_set_drvdata(pdev, tdm);

	ret = pio_tdm_parse_gpio(np, "bclk-gpios", 18, &tdm->gpio_bclk) ?:
	      pio_tdm_parse_gpio(np, "ws-gpios", 19, &tdm->gpio_ws) ?:
	      pio_tdm_parse_gpio(np, "sdi-gpios", 20, &tdm->gpio_sdi) ?:
	      pio_tdm_parse_gpio(np, "sdo-gpios", 21, &tdm->gpio_sdo);
	if (ret)
		return dev_err_probe(dev, ret, "bad *-gpios (must be &rp1_gpio N 0)\n");

	/* FIFO slave address: the PIO block's CPU-view base, exactly as rp1-pio uses it. */
	pio_np = of_parse_phandle(np, "pio", 0);
	if (!pio_np)
		return dev_err_probe(dev, -EINVAL, "missing pio phandle\n");
	ret = of_address_to_resource(pio_np, 0, &res);
	of_node_put(pio_np);
	if (ret)
		return dev_err_probe(dev, ret, "cannot resolve the PIO register base\n");
	tdm->pio_base = res.start;

	tdm->pio = pio_open();
	if (IS_ERR(tdm->pio))
		return dev_err_probe(dev, PTR_ERR(tdm->pio), "rp1-pio not ready\n");

	pio_tdm_init_stream(tdm, &tdm->rx, false);
	pio_tdm_init_stream(tdm, &tdm->tx, true);
	ret = pio_tdm_setup_stream(tdm, &tdm->rx);
	if (ret)
		goto err_streams;
	ret = pio_tdm_setup_stream(tdm, &tdm->tx);
	if (ret)
		goto err_streams;

	ret = pio_tdm_register_card(tdm);
	if (ret) {
		dev_err_probe(dev, ret, "ALSA card registration failed\n");
		goto err_card;
	}

	dev_info(dev, "card %s: PIO base %pa, gpios bclk=%u ws=%u sdi=%u sdo=%u\n", CARD_ID,
		 &tdm->pio_base, tdm->gpio_bclk, tdm->gpio_ws, tdm->gpio_sdi, tdm->gpio_sdo);
	return 0;

err_card:
	if (tdm->card)
		snd_card_free(tdm->card);
err_streams:
	pio_tdm_release_stream(tdm, &tdm->tx);
	pio_tdm_release_stream(tdm, &tdm->rx);
	pio_close(tdm->pio);	/* disables + unclaims our SMs, removes our programs */
	return ret;
}

static void pio_tdm_remove(struct platform_device *pdev)
{
	struct pio_tdm *tdm = platform_get_drvdata(pdev);

	/* Waits for all PCM files to close; trigger(STOP) has already parked the SMs. */
	snd_card_free(tdm->card);

	pio_sm_set_enabled(tdm->pio, tdm->tx.sm, false);
	pio_sm_set_enabled(tdm->pio, tdm->rx.sm, false);
	pio_sm_set_pins_with_mask(tdm->pio, tdm->tx.sm, 0, BIT(tdm->gpio_sdo));
	pio_tdm_release_stream(tdm, &tdm->tx);
	pio_tdm_release_stream(tdm, &tdm->rx);
	pio_close(tdm->pio);
}

static const struct of_device_id pio_tdm_of_match[] = {
	{ .compatible = "seeed,snd-pio-tdm" },
	{ }
};
MODULE_DEVICE_TABLE(of, pio_tdm_of_match);

static struct platform_driver pio_tdm_driver = {
	.probe = pio_tdm_probe,
	.remove = pio_tdm_remove,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = pio_tdm_of_match,
		.dev_groups = pio_tdm_groups,
	},
};
module_platform_driver(pio_tdm_driver);

MODULE_DESCRIPTION("ReSpeaker 6-Mic HAT full-duplex TDM audio via RP1 PIO");
MODULE_AUTHOR("avisual");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_SOFTDEP("pre: rp1-pio");
