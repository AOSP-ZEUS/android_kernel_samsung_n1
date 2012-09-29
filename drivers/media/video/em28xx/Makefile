<<<<<<< HEAD
em28xx-objs     := em28xx-video.o em28xx-i2c.o em28xx-cards.o em28xx-core.o \
		   em28xx-input.o em28xx-vbi.o
=======
em28xx-y :=	em28xx-video.o em28xx-i2c.o em28xx-cards.o
em28xx-y +=	em28xx-core.o  em28xx-vbi.o

em28xx-$(CONFIG_VIDEO_EM28XX_RC) += em28xx-input.o
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

em28xx-alsa-objs := em28xx-audio.o

obj-$(CONFIG_VIDEO_EM28XX) += em28xx.o
obj-$(CONFIG_VIDEO_EM28XX_ALSA) += em28xx-alsa.o
obj-$(CONFIG_VIDEO_EM28XX_DVB) += em28xx-dvb.o

EXTRA_CFLAGS += -Idrivers/media/video
EXTRA_CFLAGS += -Idrivers/media/common/tuners
EXTRA_CFLAGS += -Idrivers/media/dvb/dvb-core
EXTRA_CFLAGS += -Idrivers/media/dvb/frontends

