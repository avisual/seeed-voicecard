# SPDX-License-Identifier: GPL-2.0-only
# snd-pio-tdm: out-of-tree build against the running kernel's headers.
#   make            -> snd-pio-tdm.ko
#   make install    -> /lib/modules/$(uname -r)/extra/ + depmod (see install.sh for the full deploy)
KDIR ?= /lib/modules/$(shell uname -r)/build

obj-m := snd-pio-tdm.o

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean

install: all
	sudo install -D -m 644 snd-pio-tdm.ko /lib/modules/$(shell uname -r)/extra/snd-pio-tdm.ko
	sudo depmod -a

.PHONY: all clean install
