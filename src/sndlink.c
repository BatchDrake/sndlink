/*
  sndlink.c: Build sndlink object

  Copyright (C) 2016 Gonzalo José Carracedo Carballal

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation, either version 3 of the
  License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this program.  If not, see
  <http://www.gnu.org/licenses/>

*/

#include <sndlink.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <ctype.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>

extern struct sigutils_block_class su_block_class_ALSA;

static int
sndlink_tun_alloc(char *new_name, const char *dev, int flags)
{
  struct ifreq ifr;
  int fd = -1;

  if ((fd = open(SNDLINK_CLONE_DEV, O_RDWR)) == -1)
    goto fail;

   /* preparation of the struct ifr, of type "struct ifreq" */
   memset(&ifr, 0, sizeof(ifr));

   ifr.ifr_flags = flags;   /* IFF_TUN or IFF_TAP, plus maybe IFF_NO_PI */

   if (*dev)
     strncpy(ifr.ifr_name, dev, IFNAMSIZ);

   if (ioctl(fd, TUNSETIFF, &ifr) == -1)
     goto fail;

  /* if the operation was successful, write back the name of the
   * interface to the variable "dev", so the caller can know
   * it. Note that the caller MUST reserve space in *dev (see calling
   * code below) */
  strncpy(new_name, ifr.ifr_name, IFNAMSIZ);

  /* this is the special file descriptor that the caller will use to talk
   * with the virtual interface */
  return fd;

fail:
  if (fd != -1)
    close(fd);

  return -1;
}

static snd_pcm_t *
sndlink_open_playback(const char *device, unsigned int rate)
{
  snd_pcm_t *pcm = NULL;
  snd_pcm_hw_params_t *params;
  snd_pcm_uframes_t frames;
  int err = 0;

  SU_TRYCATCH(
      (err = snd_pcm_open(&pcm, device, SND_PCM_STREAM_PLAYBACK, 0)) >= 0,
      goto fail);

  snd_pcm_hw_params_alloca(&params);

  snd_pcm_hw_params_any(pcm, params);

  SU_TRYCATCH(
      (err = snd_pcm_hw_params_set_access(
          pcm,
          params,
          SND_PCM_ACCESS_RW_INTERLEAVED)) >= 0,
      goto fail);

  SU_TRYCATCH(
      (err = snd_pcm_hw_params_set_format(
          pcm,
          params,
          SND_PCM_FORMAT_S16_LE)) >= 0,
      goto fail);

  SU_TRYCATCH(
      (err = snd_pcm_hw_params_set_buffer_size(
          pcm,
          params,
          SNDLINK_DEFAULT_BUFFER_SIZE)) >= 0,
      goto fail);

  SU_TRYCATCH(
      (err = snd_pcm_hw_params_set_channels(pcm, params, 1)) >= 0,
      goto fail);

  SU_TRYCATCH(
      (err = snd_pcm_hw_params_set_rate_near(pcm, params, &rate, 0)) >= 0,
      goto fail);

  SU_TRYCATCH((err = snd_pcm_hw_params(pcm, params)) >= 0, goto fail);

  return pcm;

fail:
  if (err < 0)
    fprintf(stderr, "[e] ALSA playback failed: %s\n", snd_strerror(err));

  if (pcm != NULL) {
    snd_pcm_drain(pcm);
    snd_pcm_close(pcm);
  }

  return NULL;
}

void
sndlink_destroy(sndlink_t *link)
{
  if (link->demod_thread_created)
    pthread_join(link->demod_thread, NULL);

  if (link->tapfd != -1)
    close(link->tapfd);

  if (link->h_play != NULL) {
    snd_pcm_drain(link->h_play);
    snd_pcm_close(link->h_play);
  }

  if (link->buffer != NULL)
    free(link->buffer);

  if (link->modem != NULL)
    su_modem_destroy(link->modem);

  if (link->alsa != NULL)
    su_block_destroy(link->alsa);

  su_iir_filt_finalize(&link->mf);

  if (link->diffenc != NULL)
    su_codec_destroy(link->diffenc);

  if (link->diffdec != NULL)
    su_codec_destroy(link->diffdec);

  free(link);
}

void
hexdump(const void *data, uint32_t size)
{
  const uint8_t *bytes = (const uint8_t *) data;
  int i, j;

  for (i = 0; i < size; ++i) {
    if ((i & 0xf) == 0)
      fprintf(stderr, "[i] RX: %08x  ", i);

    fprintf(stderr, "%s%02x ", (i & 0xf) == 8 ? " " : "", bytes[i]);

    if ((i & 0xf) == 0xf) {
      fprintf(stderr, " | ");

      for (j = i - 15; j <= i; ++j)
        fprintf(stderr, "%c", isprint(bytes[j]) ? bytes[j] : '.');

      fprintf(stderr, "\n");
    }
  }

  if ((i & 0xf) != 0) {
    for (j = i; j < ((size + 0xf) >> 4) << 4; ++j)
      fprintf(stderr, "   %s", (j & 0xf) == 8 ? " " : "");
    fprintf(stderr, " | ");

    for (j = i & ~0xf; j < size; ++j)
      fprintf(stderr, "%c", isprint (bytes[j]) ? bytes[j] : '.');

    fprintf(stderr, "\n");
  }

  fprintf(stderr, "\n[i] RX: %08x  \n", i);
}

/* Shamelessly borrowed from Wikipedia */
static void
sndlink_toggle_scramble(uint8_t *buffer, size_t size)
{
  uint16_t lfsr = 0xace1u;
  unsigned i = 0;
  unsigned int bits = size << 3;
  unsigned int lsb;

  for (i = 0; i < bits; ++i) {
    lsb = lfsr & 1;    /* Get LSB (i.e., the output bit). */
    lfsr >>= 1;        /* Shift register */
    if (lsb) {          /* If the output bit is 1, apply toggle mask. */
      lfsr ^= 0xb400u;
      buffer[i >> 3] ^= 1 << (i & 7);
    }
  }
}

static void *
sndlink_demod_thread(void *data)
{
  sndlink_t *link = (sndlink_t *) data;
  struct sndlink_snd_frame *as_frame =
      (struct sndlink_snd_frame *) link->dl_snd_frame;
  SUBITS bits;
  SUSYMBOL sym;

  fprintf(stderr, "[i] Demodulator: starting modem...\n");

  if (!su_modem_start(link->modem)) {
    fprintf(stderr, "[e] su_modem_start: failed to start modem\n");
    goto done;
  }

  fprintf(stderr, "[i] Demodulator: reading symbols from modem...\n");
  while ((sym = su_modem_read(link->modem)) != SU_EOS) {
    if (sym == SU_NOSYMBOL)
      continue;

    /* This will change with the next modem API. Terrible */
    sym = SU_TOSYM(sym - 1);

    /* Symbol is differentially encoded. Pass to diffdec */
    sym = su_codec_feed(link->diffdec, sym);
    if (SU_ISSYM(sym)) {
      bits = SU_FROMSYM(sym);
      switch (link->demod_state) {
        case SNDLINK_DEMOD_STATE_SEARCHING:
          if (bits == SNDLINK_SYNC_SYMBOL) {
            if (++link->demod_ptr == SNDLINK_SYNC_SYMBOL_COUNT) {
              /*
               * Reached SNDLINK_SYNC_SYMBOL_COUNT consecutive symbols,
               * this counts as a full sync sequence
               */
              link->demod_ptr = 0;
              link->demod_state = SNDLINK_DEMOD_STATE_SYNC;
              as_frame->seq = 0;
              as_frame->len = 0;
            }
          } else {
            link->demod_ptr = 0;
          }
          break;


        case SNDLINK_DEMOD_STATE_SYNC:
          if (link->demod_ptr < 16) {
            as_frame->seq |= bits << link->demod_ptr;
          } else {
            as_frame->len |= bits << (link->demod_ptr - 16);
          }

          link->demod_ptr += 2;

          /* Fields are fully populated, check and switch states */
          if (link->demod_ptr == 32) {
            link->demod_ptr = 0;
            as_frame->seq = ntohs(as_frame->seq);
            as_frame->len = ntohs(as_frame->len);

            if (as_frame->len > SNDLINK_FRAME_MTU) {
              /* Too big, probably an error. Get back to searching */
              link->demod_state = SNDLINK_DEMOD_STATE_SEARCHING;
            } else if (as_frame->len == 0) {
              fprintf(
                  stderr,
                  "[i] RX: Keep-alive frame (seq=%d)\n",
                  as_frame->seq);
              link->demod_state = SNDLINK_DEMOD_STATE_SEARCHING;
            } else {
              /* Everything alright, start reading */
              fprintf(
                  stderr,
                  "[i] RX: Synced to %d byte frame (seq=%d)\n",
                  as_frame->len,
                  as_frame->seq);
              memset(as_frame->data, 0, as_frame->len);
              link->demod_state = SNDLINK_DEMOD_STATE_READING;
            }
          }
          break;

        case SNDLINK_DEMOD_STATE_READING:
          as_frame->data[link->demod_ptr >> 3] |= bits << (link->demod_ptr & 7);
          link->demod_ptr += 2;
          if (link->demod_ptr == (as_frame->len << 3)) {
            link->demod_ptr = 0;

            sndlink_toggle_scramble(as_frame->data, as_frame->len);
            hexdump(as_frame->data, as_frame->len);
            if (write(
                link->tapfd,
                as_frame->data,
                as_frame->len) != as_frame->len) {
              fprintf(stderr, "[e] Failed to send packet back to kernel\n");
            }

            link->demod_state = SNDLINK_DEMOD_STATE_SEARCHING;
          }
          break;
      }
    }
  }

done:
  fprintf(stderr, "[e] Demodulator thread finished\n");

  return NULL;
}

sndlink_t *
sndlink_new(const struct sndlink_params *params)
{
  sndlink_t *new = NULL;
  struct sigutils_alsa_params alsap = sigutils_alsa_params_INITIALIZER;

  TRYCATCH(new = calloc(1, sizeof(sndlink_t)), goto fail);

  new->params = *params;
  new->tapfd = -1;
  new->samp_per_sym = floor(
      1. / SU_ABS2NORM_BAUD(params->samp_rate, params->baud_rate));
  new->buffer_size = SNDLINK_DEFAULT_BUFFER_SIZE;

  /* Audio buffer initialization */
  TRYCATCH(
      new->buffer = malloc(sizeof(int16_t) * new->buffer_size),
      goto fail);

  /* Uplink / downlink buffer initialization */
  TRYCATCH(new->dl_eth_frame = malloc(SNDLINK_FRAME_MTU), goto fail);
  TRYCATCH(new->dl_snd_frame = malloc(SNDLINK_PHYS_FRAME_LEN), goto fail);

  TRYCATCH(new->ul_eth_frame = malloc(SNDLINK_FRAME_MTU), goto fail);
  TRYCATCH(new->ul_snd_frame = malloc(SNDLINK_PHYS_FRAME_LEN), goto fail);

  /* Initialize TAP device */
  TRYCATCH(
      (new->tapfd = sndlink_tun_alloc(
          new->tapname,
          SNDLINK_DEV,
          IFF_TAP)) != -1,
      goto fail);

  /* Open soundcard for playback */
  TRYCATCH(
      new->h_play = sndlink_open_playback(SNDLINK_ALSA_DEV, params->samp_rate),
      goto fail);

  /* Initialize modulator */
  su_ncqo_init(
      &new->lo,
      SU_ABS2NORM_FREQ(params->samp_rate, params->uplink_freq));

  TRYCATCH(
      su_iir_rrc_init(
          &new->mf,
          SNDLINK_MF_SPAN * new->samp_per_sym,
          new->samp_per_sym,
          1.),
      goto fail);

  fprintf(stderr, "[i] RRC size: %d taps\n", new->mf.x_size);

  SU_TRYCATCH(
      new->diffenc = su_codec_new("diff", 2, SU_FALSE),
      goto fail);
  su_codec_set_direction(new->diffenc, SU_CODEC_DIRECTION_FORWARDS);

  /* Initialize demodulator: plug a QPSK modem to ALSA source */
  alsap.samp_rate = params->samp_rate;
  alsap.device = SNDLINK_ALSA_DEV;
  TRYCATCH(new->alsa = su_block_new("alsa", &alsap), goto fail);
  TRYCATCH(new->modem = su_modem_new("qpsk"), goto fail);
  TRYCATCH(su_modem_set_source(new->modem, new->alsa), goto fail);

  /* Configure modem */
  su_modem_set_bool(new->modem, "abc", SU_FALSE); /* Automatic baud rate control */
  su_modem_set_bool(new->modem, "afc", SU_TRUE); /* Automatic frequency control */
  su_modem_set_int(new->modem, "mf_span", 5); /* Matched filter span (in symbols) */
  su_modem_set_float(new->modem, "baud", params->baud_rate); /* Baud rate: 468 baud */
  su_modem_set_float(new->modem, "fc", params->downlink_freq); /* Carrier frequency: 910 Hz */
  su_modem_set_float(new->modem, "rolloff", 1.); /* Roll-off factor of the matched filter */
  su_modem_set_int(new->modem, "samp_rate", params->samp_rate);

  /* Configure decoder */
  SU_TRYCATCH(
      new->diffdec = su_codec_new("diff", 2, SU_FALSE),
      goto fail);
  su_codec_set_direction(new->diffdec, SU_CODEC_DIRECTION_BACKWARDS);

  return new;

fail:
  if (new != NULL)
    sndlink_destroy(new);

  return NULL;
}

static void
sndlink_encode_uplink_frame(sndlink_t *link)
{
  struct sndlink_snd_frame *as_frame =
      (struct sndlink_snd_frame *) link->ul_snd_frame;
  uint16_t payload_len = link->ul_eth_frame_len;

  link->ul_snd_frame_len =
      SNDLINK_PHYS_FRAME_OVERHEAD_LEN
      + payload_len
      + sizeof(uint32_t);

  /* TODO: Do it at the beginning */
  as_frame->sync0 = htonl(SNDLINK_SYNC0_SEQ);
  as_frame->sync1 = htonl(SNDLINK_SYNC1_SEQ);
  as_frame->len   = htons(payload_len);
  as_frame->seq   = htons(link->ul_seq++);

  memcpy(as_frame->data, link->ul_eth_frame, payload_len);
  sndlink_toggle_scramble(as_frame->data, payload_len);

  /* Set last 32 bits to zero */
  memset(as_frame->data + payload_len, 0, sizeof(uint32_t));
}

static int16_t
tos16le(SUFLOAT f)
{
  if (f < -1)
    f = -1;
  else if (f > 1)
    f = 1;

  return floor(f * 0x7fff);
}

static SUBOOL
sndlink_frame_available(const sndlink_t *link)
{
  fd_set fds;
  struct timeval tv = {0, 0};

  FD_ZERO(&fds);
  FD_SET(link->tapfd, &fds);

  if (select(link->tapfd + 1, &fds, NULL, NULL, &tv) == 1)
    return SU_TRUE;

  return SU_FALSE;
}

static SUBOOL
sndlink_mod_loop(sndlink_t *link)
{
  ssize_t got;
  int err;
  unsigned int i, j, n = 0;
  unsigned int frame_bits;
  SUSYMBOL sym = 0;
  SUSYMBOL tx_sym = 0;
  SUCOMPLEX delta;
  SUCOMPLEX baseband;
  SUCOMPLEX carrier;

  for (;;) {
    got = 0;
    if (sndlink_frame_available(link)) {
      if ((got = read(
          link->tapfd,
          link->ul_eth_frame,
          SNDLINK_FRAME_MTU)) == -1) {
        fprintf(stderr, "[e] Read frame failed: %s\n", strerror(errno));
        return SU_FALSE;
      } else {
        fprintf(stderr, "[i] TX: %d byte Ethernet frame\n", got);
      }
    }
    link->ul_eth_frame_len = got;
    /* Encode frame */
    sndlink_encode_uplink_frame(link);

    /* Combute number of bits to send */
    frame_bits = link->ul_snd_frame_len << 3; /* 8 bits per byte */

    /* Each symbol encodes 2 bits */
    for (i = 0; i < frame_bits; i += 2) {
      sym = SU_TOSYM((link->ul_snd_frame[i >> 3] >> (i & 7)) & 3);
      tx_sym = su_codec_feed(link->diffenc, sym);

      /* Got symbol from encoder, perform modulation  */
      if (SU_ISSYM(tx_sym)) {
        delta = -SU_C_EXP(.5 * I * M_PI * SU_FROMSYM(tx_sym));

        for (j = 0; j < link->samp_per_sym; ++j) {
          baseband = su_iir_filt_feed(
              &link->mf,
              .5 * delta * link->samp_per_sym);
          carrier  = su_ncqo_read(&link->lo);

          delta = 0;

          /* Mix and save sample */
          link->buffer[n++] = tos16le(SU_C_REAL(carrier * baseband));

          if (n == link->buffer_size) {
            /* Buffer full, send to soundcard */
            n = 0;
            if ((err = snd_pcm_writei(
                link->h_play,
                link->buffer,
                link->buffer_size)) == -EPIPE) {
              fprintf(stderr, "[!] Samples lost while writing\n");
              snd_pcm_prepare(link->h_play);
            } else if (err < 0) {
              fprintf(
                stderr,
                "[e] Fatal: can't write to PCM device: %s\n",
                snd_strerror(err));
              return SU_FALSE;
            }
          }
        }
      }
    }
  }

  return SU_TRUE;
}

SUBOOL
sndlink_run(sndlink_t *link)
{
  if (link->demod_thread_created)
    return SU_FALSE;

  link->demod_thread_created = SU_TRUE;

  if (pthread_create(
      &link->demod_thread,
      NULL,
      sndlink_demod_thread,
      link) == -1) {
    link->demod_thread_created = SU_FALSE;
    return SU_FALSE;
  }

  /* Demodulator thread created. Start loop */
  return sndlink_mod_loop(link);
}

SUBOOL
sndlink_init(void)
{
  TRYCATCH(su_lib_init(), return SU_FALSE);

  if (!su_block_class_register(&su_block_class_ALSA)) {
    fprintf(
        stderr,
        "%s: failed to register ALSA block\n",
        __FUNCTION__);
    return SU_FALSE;
  }

  return SU_TRUE;
}
