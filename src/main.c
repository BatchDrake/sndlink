/*
 * main.c: entry point for sndlink
 * Creation date: Tue Nov 28 22:19:51 2017
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <getopt.h>

#include <sndlink.h>

#define SNDLINK_DEFAULT_BAUDRATE  1200
#define SNDLINK_DEFAULT_SAMP_RATE 48000
int
main (int argc, char *argv[], char *envp[])
{
  sndlink_t *link = NULL;
  struct sndlink_params params;

  if (argc != 3) {
    fprintf(stderr, "Usage:\n %s uplink_freq downlink_freq\n", argv[0]);
    exit(EXIT_FAILURE);
  }

  params.baud_rate = SNDLINK_DEFAULT_BAUDRATE;
  params.samp_rate = SNDLINK_DEFAULT_SAMP_RATE;

  if (sscanf(argv[1], "%u", &params.uplink_freq) < 1) {
    fprintf(stderr, "%s: invalid uplink frequency\n", argv[0]);
    exit(EXIT_FAILURE);
  }

  if (params.uplink_freq > params.samp_rate / 2) {
    fprintf(
        stderr,
        "%s: uplink frequency too high (max %d)\n",
        params.samp_rate / 2);
    exit(EXIT_FAILURE);
  }

  if (sscanf(argv[2], "%u", &params.downlink_freq) < 1) {
    fprintf(stderr, "%s: invalid downlink frequency\n", argv[0]);
    exit(EXIT_FAILURE);
  }

  if (params.downlink_freq > params.samp_rate / 2) {
    fprintf(
        stderr,
        "%s: downlink frequency too high (max %d)\n",
        params.samp_rate / 2);
    exit(EXIT_FAILURE);
  }

  if (!sndlink_init()) {
    fprintf(
        stderr,
        "%s: failed to initialize sndlink global objects\n",
        argv[0]);
    exit(EXIT_FAILURE);
  }

  if ((link = sndlink_new(&params)) == NULL) {
    fprintf(stderr, "%s: failed to initialize sndlink object\n", argv[0]);
    exit(EXIT_FAILURE);
  }

  snd_info("sndlink v0.1 started\n");
  snd_info("(c) Gonzalo J. Carracedo <BatchDrake@gmail.com>\n");
  snd_info("  Network interface: %s\n", sndlink_get_interface(link));
  snd_info("  Uplink carrier: %d Hz\n", params.uplink_freq);
  snd_info("  Downlink carrier: %d Hz\n", params.downlink_freq);

  if (!sndlink_run(link)) {
    fprintf(stderr, "%s: failed to run sndlink loop\n", argv[0]);
    exit(EXIT_FAILURE);
  }

  sndlink_destroy(link);

  return 0;
}

