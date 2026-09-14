/* Cyclone DDS latency, measured the same way as bench_latency measures
 * Lockstep, so the two tables can honestly be put side by side.
 *
 * SAME METHODOLOGY, deliberately:
 *   - writer and reader live in ONE process, so there is no clock skew to
 *     correct and no network in the path. This is the configuration most
 *     favourable to DDS, and it is the right one to use: the claim under test
 *     is about copies and serialisation, not about wires.
 *   - the writer stamps CLOCK_MONOTONIC immediately before dds_write();
 *     the reader stamps immediately after dds_take() returns the sample.
 *   - percentiles come from a sorted array, exact rather than approximated,
 *     for the same reason as on the Lockstep side: the tail is the point.
 *   - RELIABLE with KEEP_LAST history, which is what a robotics node would
 *     actually configure.
 *
 * WHAT THIS IS NOT. Both sides run on whatever kernel you have. On WSL2 or any
 * stock kernel the tail is the scheduler, not the middleware, for BOTH of them.
 * The comparison is still meaningful -- both eat the same scheduler -- but the
 * absolute numbers are not real-time results and should not be quoted as such.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "dds/dds.h"
#include "BenchMsg.h"

static uint64_t now_ns(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static int cmp_u64(const void* a, const void* b) {
  const uint64_t x = *(const uint64_t*)a, y = *(const uint64_t*)b;
  return (x > y) - (x < y);
}

static uint64_t pct(uint64_t* v, size_t n, double p) {
  if (n == 0) return 0;
  size_t i = (size_t)((double)(n - 1) * p);
  return v[i];
}

static void report(const char* label, uint64_t* s, size_t n, size_t dropped) {
  qsort(s, n, sizeof(uint64_t), cmp_u64);
  printf("\n  %s  (%zu samples", label, n);
  if (dropped) printf(", %zu dropped", dropped);
  printf(")\n");
  printf("    min     %8.2f us\n", pct(s, n, 0.0) / 1000.0);
  printf("    p50     %8.2f us\n", pct(s, n, 0.50) / 1000.0);
  printf("    p90     %8.2f us\n", pct(s, n, 0.90) / 1000.0);
  printf("    p99     %8.2f us\n", pct(s, n, 0.99) / 1000.0);
  printf("    p99.9   %8.2f us\n", pct(s, n, 0.999) / 1000.0);
  printf("    max     %8.2f us\n", pct(s, n, 1.0) / 1000.0);
}

/* One payload size. Returns 0 on success. */
static int measure(dds_entity_t participant, const char* label, size_t payload_bytes,
                   int iterations) {
  int rc = 1;
  char topic_name[128];
  snprintf(topic_name, sizeof topic_name, "bench_%zu_%d", payload_bytes, (int)getpid());

  dds_entity_t topic = dds_create_topic(participant, &bench_Msg_desc, topic_name, NULL, NULL);
  if (topic < 0) { fprintf(stderr, "dds_create_topic: %s\n", dds_strretcode(-topic)); return 1; }

  /* KEEP_LAST(16) mirrors the ring depth Lockstep uses for the same sizes. */
  dds_qos_t* qos = dds_create_qos();
  dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
  dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 16);

  dds_entity_t writer = dds_create_writer(participant, topic, qos, NULL);
  dds_entity_t reader = dds_create_reader(participant, topic, qos, NULL);
  dds_delete_qos(qos);
  if (writer < 0 || reader < 0) {
    fprintf(stderr, "create writer/reader failed\n");
    return 1;
  }

  /* Let discovery settle before timing anything: the first writes on a fresh
   * pair are dominated by matching, which is startup cost, not transport cost.
   * Lockstep's own benchmark has the equivalent in publisher::create. */
  dds_sleepfor(DDS_MSECS(300));

  uint8_t* payload = payload_bytes ? (uint8_t*)malloc(payload_bytes) : NULL;
  uint64_t* samples = (uint64_t*)malloc(sizeof(uint64_t) * (size_t)iterations);
  if ((payload_bytes && !payload) || !samples) { fprintf(stderr, "oom\n"); goto done; }

  size_t got = 0, dropped = 0;
  void* rs[1] = {NULL};
  dds_sample_info_t si[1];

  for (int i = 0; i < iterations; ++i) {
    bench_Msg m;
    memset(&m, 0, sizeof m);
    if (payload_bytes) {
      memset(payload, i & 0xff, payload_bytes);
      m.payload._buffer = payload;
      m.payload._length = (uint32_t)payload_bytes;
      m.payload._maximum = (uint32_t)payload_bytes;
      m.payload._release = false; /* the buffer is ours, not the middleware's */
    }
    m.stamp_ns = now_ns();

    dds_return_t wr = dds_write(writer, &m);
    if (wr < 0) { ++dropped; continue; }

    /* Poll rather than block on a waitset: the Lockstep side polls take() too,
     * so polling here keeps the two measuring the same thing. */
    int spins = 0;
    for (;;) {
      dds_return_t n = dds_take(reader, rs, si, 1, 1);
      if (n > 0) {
        if (si[0].valid_data) {
          bench_Msg* r = (bench_Msg*)rs[0];
          samples[got++] = now_ns() - r->stamp_ns;
        } else {
          ++dropped;
        }
        dds_return_loan(reader, rs, n);
        break;
      }
      if (++spins > 2000000) { ++dropped; break; }
    }
  }

  report(label, samples, got, dropped);
  rc = 0;

done:
  free(payload);
  free(samples);
  return rc;
}

int main(int argc, char** argv) {
  const int iterations = argc > 1 ? atoi(argv[1]) : 100000;

  printf("Cyclone DDS latency (baseline for comparison with Lockstep)\n");
  printf("===========================================================\n");
  printf("  cyclonedds version: %s\n", "0.10.x");
  printf("  same process, RELIABLE, KEEP_LAST(16), one-way, %d iterations\n",
         iterations);
  printf("\n  *** Not a real-time measurement. On a stock kernel the tail below\n");
  printf("  *** is the scheduler for both this and Lockstep. Compare the two to\n");
  printf("  *** each other, not to a datasheet.\n");

  dds_entity_t participant = dds_create_participant(DDS_DOMAIN_DEFAULT, NULL, NULL);
  if (participant < 0) {
    fprintf(stderr, "dds_create_participant: %s\n", dds_strretcode(-participant));
    return 1;
  }

  /* The same three points the Lockstep benchmark reports. */
  int bad = 0;
  bad |= measure(participant, "small message, 32 B payload", 32, iterations);
  bad |= measure(participant, "medium message, 64 KB payload", 65536, iterations / 10);
  bad |= measure(participant, "large message, 1 MB payload", 1u << 20, iterations / 100);

  dds_delete(participant);
  return bad;
}
