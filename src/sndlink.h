/*
 * sndlink.h: headers, prototypes and declarations for sndlink
 * Creation date: Tue Nov 28 22:19:51 2017
 */

#ifndef _MAIN_INCLUDE_H
#define _MAIN_INCLUDE_H

#include <stdio.h>
#include <config.h> /* General compile-time configuration parameters */
#include <util.h> /* From util: Common utility library */
#include <pthread.h>
#include <sigutils/sigutils.h> /* From sigutils: Signal utils */
#include <sigutils/ncqo.h>
#include <sigutils/log.h>
#include <sigutils/sampling.h>
#include <sigutils/taps.h>
#include <sigutils/iir.h>
#include <alsa/asoundlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <linux/if.h>
#include <linux/if_tun.h>

#define snd_err(fmt, arg...) fprintf(stderr, "[e] " fmt, ##arg)
#define snd_info(fmt, arg...) fprintf(stderr, "[i] " fmt, ##arg)
#define snd_warn(fmt, arg...) fprintf(stderr, "[!] " fmt, ##arg)

#define SNDLINK_DEFAULT_BUFFER_SIZE (4096 / sizeof(int16_t))
#define SNDLINK_MF_SPAN 11
#define SNDLINK_DEV "sndlan"
#define SNDLINK_CLONE_DEV "/dev/net/tun"
#define SNDLINK_ALSA_DEV "default"
#define SNDLINK_FRAME_MTU 1500
#define SNDLINK_PHYS_FRAME_OVERHEAD_LEN sizeof(struct sndlink_snd_frame)
#define SNDLINK_PHYS_FRAME_LEN (SNDLINK_PHYS_FRAME_OVERHEAD_LEN + SNDLINK_FRAME_MTU + sizeof(uint32_t))
#define SNDLINK_SYNC0_SEQ 0x00000000 /* Keep phase */

#define SNDLINK_SYNC_SYMBOL_COUNT (8 * sizeof(uint32_t) / 2)

#define SNDLINK_SYNC_SYMBOL  2u      /* sync symbol 2: 180 deg transitions */
#define SNDLINK_SYNC_PATTERN 0x55555555u /* 01 01 01 01... */
#define SNDLINK_SYNC1_SEQ (SNDLINK_SYNC_PATTERN * SNDLINK_SYNC_SYMBOL)

#define TRYCATCH(expr, action)                  \
  if (!(expr)) {                                \
    fprintf(                                    \
      stderr,                                   \
      "(e) %s: operation `%s' failed\n",        \
      __FUNCTION__,                             \
      STRINGIFY(expr));                         \
    action;                                     \
  }

struct sigutils_alsa_params {
  const char *device;
  SUSCOUNT samp_rate;
  SUSCOUNT fc;
  SUBOOL dc_remove;
};

#define ALSA_INTEGER_BUFFER_SIZE 2048

#define sigutils_alsa_params_INITIALIZER {"default", 44100, 0}

struct sigutils_alsa_state {
  snd_pcm_t *handle;
  uint64_t samp_rate;
  uint64_t fc;
  int16_t buffer[ALSA_INTEGER_BUFFER_SIZE];
  SUCOMPLEX last;
  SUBOOL dc_remove;
};

struct sndlink_snd_frame {
  uint32_t sync0;
  uint32_t sync1;
  uint16_t seq;
  uint16_t len;
  uint8_t  data[0];
};

struct sndlink_params {
  unsigned int samp_rate;
  unsigned int baud_rate;
  unsigned int uplink_freq;
  unsigned int downlink_freq;
};

enum sndlink_demod_state {
  SNDLINK_DEMOD_STATE_SEARCHING,
  SNDLINK_DEMOD_STATE_SYNC,
  SNDLINK_DEMOD_STATE_READING,
};

struct sndlink {
  struct sndlink_params params;

  /* Precalculated parameters */
  unsigned int samp_per_sym;

  /* TAP interface */
  int tapfd;
  char tapname[IFNAMSIZ];

  /* Downlink packet buffers */
  uint8_t *dl_eth_frame; /* MTU bytes */
  size_t   dl_eth_frame_len;

  uint8_t *dl_snd_frame; /* SNDLINK_PHYS_FRAME_LEN bytes */
  size_t   dl_snd_frame_len;

  enum sndlink_demod_state demod_state;
  unsigned demod_ptr;

  /* Uplink packet buffers */
  uint8_t *ul_eth_frame; /* MTU bytes */
  size_t   ul_eth_frame_len;

  uint8_t *ul_snd_frame; /* SNDLINK_PHYS_FRAME_LEN bytes */
  size_t   ul_snd_frame_len;

  uint16_t ul_seq;

  /* Soundcard state */
  snd_pcm_t *h_play;
  int16_t *buffer;
  unsigned int buffer_size;

  /* Modulator objects */
  su_ncqo_t     lo;
  su_iir_filt_t mf;
  su_codec_t   *diffenc;

  /* Demodulator objects */
  su_block_t *alsa;
  su_modem_t *modem;
  su_codec_t *diffdec;

  pthread_t demod_thread;
  SUBOOL demod_thread_created;
};

typedef struct sndlink sndlink_t;

void sndlink_destroy(sndlink_t *link);
const char *sndlink_get_interface(const sndlink_t *link);
sndlink_t *sndlink_new(const struct sndlink_params *params);
SUBOOL sndlink_run(sndlink_t *link);
SUBOOL sndlink_init(void);

#endif /* _MAIN_INCLUDE_H */
