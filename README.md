# seeed-voicecard

The drivers for [ReSpeaker Mic Hat](https://www.seeedstudio.com/ReSpeaker-2-Mics-Pi-HAT-p-2874.html), [ReSpeaker 4 Mic Array](https://www.seeedstudio.com/ReSpeaker-4-Mic-Array-for-Raspberry-Pi-p-2941.html), [6-Mics Circular Array Kit](), and [4-Mics Linear Array Kit]() for Raspberry Pi.

### Raspberry Pi 5

The 6-Mic Circular Array HAT needs an **extra driver** on a Pi 5, and the branches here are not
it: the Pi 5's audio pins belong to RP1, whose DesignWare I2S blocks have no TDM (Raspberry Pi
white paper RP-009699-WP-1, p.5: *"TDM is not supported on any Raspberry Pi SBCs"*; see also
raspberrypi/linux#6568), so the HAT's 8-slot frame cannot be captured and its transmitter never
shifts at all. The data path has to move off I2S entirely.

* **[`avisual/respeaker-6mic-pi5`](https://github.com/avisual/respeaker-6mic-pi5)** —
  `snd-pio-tdm`, an out-of-tree ALSA card that carries both audio directions on RP1's **PIO**
  block. Six mics, the AC101 hardware loopback and speaker playback, with DKMS and an overlay.
* **[branch `pi5`](../../tree/pi5)** of this repository — the `ac10x` codec driver building and
  running on kernel 6.18 / Pi 5 (the control plane: clocks and mixer controls), which the above
  needs.

Running since 2026-09-06 on a Pi 5 (2 GB) with kernel `6.18.34+rpt-rpi-2712`.

### Install seeed-voicecard
Get the seeed voice card source code and install all linux kernel drivers
```bash
git clone https://github.com/HinTak/seeed-voicecard
cd seeed-voicecard
sudo ./install.sh
sudo reboot
```
## ReSpeaker Documentation

Up to date documentation for reSpeaker products can be found in [Seeed Studio Wiki](https://wiki.seeedstudio.com/ReSpeaker/)!
![](https://files.seeedstudio.com/wiki/ReSpeakerProductGuide/img/Raspberry_Pi_Mic_Array_Solutions.png)


### Coherence

Estimate the magnitude squared coherence using Welch’s method.
![4-mics-linear-array-kit coherence](https://user-images.githubusercontent.com/3901856/37277486-beb1dd96-261f-11e8-898b-84405bfc7cea.png)  
Note: 'CO 1-2' means the coherence between channel 1 and channel 2.

```bash
# How to get the coherence of the captured audio(a.wav for example).
sudo apt install python-numpy python-scipy python-matplotlib
python tools/coherence.py a.wav

# Requirement of the input audio file:
- format: WAV(Microsoft) signed 16-bit PCM
- channels: >=2
```

### uninstall seeed-voicecard
If you want to upgrade the driver , you need uninstall the driver first.

```
pi@raspberrypi:~/seeed-voicecard $ sudo ./uninstall.sh 
...
------------------------------------------------------
Please reboot your raspberry pi to apply all settings
Thank you!
------------------------------------------------------
```

Enjoy !

### Technical support

For hardware testing purposes we made a Rasperry Pi OS 5.10.17-v7l+ 32-bit image with reSpeaker drivers pre-installed, which you can download by clicking on [this link](https://files.seeedstudio.com/linux/Raspberry%20Pi%204%20reSpeaker/2021-05-07-raspios-buster-armhf-lite-respeaker.img.xz).

We provide official support for using reSpeaker with the following OS:
- 32-bit Raspberry Pi OS
- 64-bit Raspberry Pi OS

And following hardware platforms:
- Raspberry Pi 3 (all models), Raspberry Pi 4 (all models)

Anything beyond the scope of official support is considered to be community supported. Support for other OS/hardware platforms can be added, provided MOQ requirements can be met. 

If you have a technical problem when using reSpeaker with one of the officially supported platforms/OS, feel free to create an issue on Github. For general questions or suggestions, please use [Seeed forum](https://forum.seeedstudio.com/c/products/respeaker/15). 


