/*
 * test_output_dqbuf.c - Test suite for OUTPUT DQBUF fixes
 *
 * Covers all changes in the OUTPUT DQBUF fix series:
 *   Test 1: O_NONBLOCK DQBUF on empty queue returns -EAGAIN (not -EFAULT)
 *   Test 2: Blocking DQBUF waits and is woken by QBUF
 *   Test 3: Blocking DQBUF is unblocked by STREAMOFF (returns -EAGAIN)
 *   Test 4: Buffer queue integrity — repeated QBUF/DQBUF cycles
 *
 * Build:  gcc -o test_output_dqbuf test_output_dqbuf.c -lpthread
 * Usage:  ./test_output_dqbuf /dev/video50
 *
 * Exit codes:
 *   0 = all tests passed
 *   1 = one or more tests failed
 *   2 = setup error
 *
 * Copyright (C) 2026 Shih-Yuan Lee (FourDollars) <sylee@canonical.com>
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <linux/videodev2.h>

#define BUF_COUNT 2
#define WIDTH     640
#define HEIGHT    480
#define PIXFMT    V4L2_PIX_FMT_UYVY
#define TIMEOUT_S 5

static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define PASS(name) do { \
    tests_run++; tests_passed++; \
    fprintf(stderr, "  PASS: %s\n", name); \
} while (0)

#define FAIL(name, reason) do { \
    tests_run++; tests_failed++; \
    fprintf(stderr, "  FAIL: %s — %s\n", name, reason); \
} while (0)

/* ── helpers ─────────────────────────────────────────────────────────── */

struct test_ctx {
    int fd;
    unsigned int buf_count;
    void *mapped[BUF_COUNT];
    size_t lengths[BUF_COUNT];
};

static int setup_device(struct test_ctx *ctx, const char *dev, int flags)
{
    ctx->fd = open(dev, O_RDWR | flags);
    if (ctx->fd < 0) {
        perror("open");
        return -1;
    }

    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    fmt.fmt.pix.width = WIDTH;
    fmt.fmt.pix.height = HEIGHT;
    fmt.fmt.pix.pixelformat = PIXFMT;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (ioctl(ctx->fd, VIDIOC_S_FMT, &fmt) < 0) {
        perror("S_FMT");
        close(ctx->fd);
        return -1;
    }

    struct v4l2_requestbuffers req = {0};
    req.count = BUF_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(ctx->fd, VIDIOC_REQBUFS, &req) < 0) {
        perror("REQBUFS");
        close(ctx->fd);
        return -1;
    }
    ctx->buf_count = req.count;

    for (unsigned i = 0; i < ctx->buf_count; i++) {
        struct v4l2_buffer qb = {0};
        qb.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        qb.memory = V4L2_MEMORY_MMAP;
        qb.index = i;
        if (ioctl(ctx->fd, VIDIOC_QUERYBUF, &qb) < 0) {
            perror("QUERYBUF");
            close(ctx->fd);
            return -1;
        }
        ctx->mapped[i] = mmap(NULL, qb.length, PROT_READ | PROT_WRITE,
                              MAP_SHARED, ctx->fd, qb.m.offset);
        if (ctx->mapped[i] == MAP_FAILED) {
            perror("mmap");
            close(ctx->fd);
            return -1;
        }
        ctx->lengths[i] = qb.length;
        memset(ctx->mapped[i], 0x80, qb.length);
    }
    return 0;
}

static void teardown_device(struct test_ctx *ctx)
{
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    ioctl(ctx->fd, VIDIOC_STREAMOFF, &type);
    for (unsigned i = 0; i < ctx->buf_count; i++) {
        if (ctx->mapped[i] && ctx->mapped[i] != MAP_FAILED)
            munmap(ctx->mapped[i], ctx->lengths[i]);
    }
    close(ctx->fd);
}

static int streamon(struct test_ctx *ctx)
{
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    return ioctl(ctx->fd, VIDIOC_STREAMON, &type);
}

static int streamoff(struct test_ctx *ctx)
{
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    return ioctl(ctx->fd, VIDIOC_STREAMOFF, &type);
}

static int qbuf(struct test_ctx *ctx, unsigned index)
{
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = index;
    return ioctl(ctx->fd, VIDIOC_QBUF, &buf);
}

static int dqbuf(struct test_ctx *ctx, struct v4l2_buffer *buf)
{
    memset(buf, 0, sizeof(*buf));
    buf->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    buf->memory = V4L2_MEMORY_MMAP;
    return ioctl(ctx->fd, VIDIOC_DQBUF, buf);
}

/* ── Test 1: O_NONBLOCK DQBUF returns -EAGAIN ───────────────────────── */

static void test1_nonblock_eagain(const char *dev)
{
    const char *name = "O_NONBLOCK DQBUF on empty queue returns -EAGAIN";
    struct test_ctx ctx = {0};

    if (setup_device(&ctx, dev, O_NONBLOCK) < 0) {
        FAIL(name, "setup failed");
        return;
    }

    /* QBUF all, STREAMON, DQBUF all → empty queue */
    for (unsigned i = 0; i < ctx.buf_count; i++)
        qbuf(&ctx, i);
    if (streamon(&ctx) < 0) {
        FAIL(name, "STREAMON failed");
        teardown_device(&ctx);
        return;
    }
    for (unsigned i = 0; i < ctx.buf_count; i++) {
        struct v4l2_buffer buf;
        dqbuf(&ctx, &buf);
    }

    /* Now queue is empty — DQBUF should return -EAGAIN */
    struct v4l2_buffer buf;
    int ret = dqbuf(&ctx, &buf);

    if (ret < 0 && errno == EAGAIN) {
        PASS(name);
    } else if (ret < 0 && errno == EFAULT) {
        FAIL(name, "got -EFAULT (pre-fix behaviour)");
    } else if (ret < 0) {
        char msg[64];
        snprintf(msg, sizeof(msg), "unexpected errno=%d (%s)", errno, strerror(errno));
        FAIL(name, msg);
    } else {
        FAIL(name, "DQBUF succeeded unexpectedly");
    }
    teardown_device(&ctx);
}

/* ── Test 2: Blocking DQBUF waits, woken by QBUF ────────────────────── */

struct t2_args {
    struct test_ctx *ctx;
    int dqbuf_ret;
    int dqbuf_errno;
    int done;
    unsigned index;
};

static void *t2_dqbuf_thread(void *arg)
{
    struct t2_args *a = arg;
    struct v4l2_buffer buf;
    a->dqbuf_ret = dqbuf(a->ctx, &buf);
    a->dqbuf_errno = errno;
    a->index = buf.index;
    a->done = 1;
    return NULL;
}

static void test2_blocking_qbuf_wakeup(const char *dev)
{
    const char *name = "Blocking DQBUF waits and is woken by QBUF";
    struct test_ctx ctx = {0};

    if (setup_device(&ctx, dev, 0) < 0) {
        FAIL(name, "setup failed");
        return;
    }

    /* QBUF all, STREAMON, DQBUF all → empty queue */
    for (unsigned i = 0; i < ctx.buf_count; i++)
        qbuf(&ctx, i);
    if (streamon(&ctx) < 0) {
        FAIL(name, "STREAMON failed");
        teardown_device(&ctx);
        return;
    }
    for (unsigned i = 0; i < ctx.buf_count; i++) {
        struct v4l2_buffer buf;
        dqbuf(&ctx, &buf);
    }

    /* Spawn thread — should block in DQBUF */
    struct t2_args args = { .ctx = &ctx, .done = 0 };
    pthread_t tid;
    pthread_create(&tid, NULL, t2_dqbuf_thread, &args);

    usleep(500000); /* 0.5s */
    if (args.done) {
        FAIL(name, "DQBUF returned before QBUF (didn't block)");
        pthread_join(tid, NULL);
        teardown_device(&ctx);
        return;
    }

    /* QBUF buffer 0 — should wake thread */
    qbuf(&ctx, 0);

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += TIMEOUT_S;
    int join_ret = pthread_timedjoin_np(tid, NULL, &ts);

    if (join_ret == 0 && args.done && args.dqbuf_ret == 0) {
        PASS(name);
    } else if (join_ret != 0) {
        FAIL(name, "DQBUF thread still blocked after QBUF (timeout)");
        pthread_cancel(tid);
    } else {
        char msg[64];
        snprintf(msg, sizeof(msg), "DQBUF returned error %d (%s)",
                 args.dqbuf_errno, strerror(args.dqbuf_errno));
        FAIL(name, msg);
    }
    teardown_device(&ctx);
}

/* ── Test 3: STREAMOFF wakes blocked DQBUF ───────────────────────────── */

struct t3_args {
    struct test_ctx *ctx;
    int dqbuf_ret;
    int dqbuf_errno;
    int done;
};

static void *t3_dqbuf_thread(void *arg)
{
    struct t3_args *a = arg;
    struct v4l2_buffer buf;
    a->dqbuf_ret = dqbuf(a->ctx, &buf);
    a->dqbuf_errno = errno;
    a->done = 1;
    return NULL;
}

static void test3_streamoff_wakeup(const char *dev)
{
    const char *name = "Blocking DQBUF is unblocked by STREAMOFF (-EAGAIN)";
    struct test_ctx ctx = {0};

    if (setup_device(&ctx, dev, 0) < 0) {
        FAIL(name, "setup failed");
        return;
    }

    /* QBUF all, STREAMON, DQBUF all → empty queue */
    for (unsigned i = 0; i < ctx.buf_count; i++)
        qbuf(&ctx, i);
    if (streamon(&ctx) < 0) {
        FAIL(name, "STREAMON failed");
        teardown_device(&ctx);
        return;
    }
    for (unsigned i = 0; i < ctx.buf_count; i++) {
        struct v4l2_buffer buf;
        dqbuf(&ctx, &buf);
    }

    /* Spawn thread — should block in DQBUF */
    struct t3_args args = { .ctx = &ctx, .done = 0 };
    pthread_t tid;
    pthread_create(&tid, NULL, t3_dqbuf_thread, &args);

    usleep(500000); /* 0.5s - ensure thread is blocked */
    if (args.done) {
        FAIL(name, "DQBUF returned before STREAMOFF (didn't block)");
        pthread_join(tid, NULL);
        teardown_device(&ctx);
        return;
    }

    /* STREAMOFF — should wake blocked thread */
    streamoff(&ctx);

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += TIMEOUT_S;
    int join_ret = pthread_timedjoin_np(tid, NULL, &ts);

    if (join_ret == 0 && args.done && args.dqbuf_ret < 0 &&
        args.dqbuf_errno == EAGAIN) {
        PASS(name);
    } else if (join_ret != 0) {
        FAIL(name, "DQBUF thread still blocked after STREAMOFF (timeout — the bug!)");
        pthread_cancel(tid);
    } else if (args.done && args.dqbuf_ret < 0) {
        char msg[80];
        snprintf(msg, sizeof(msg), "DQBUF returned errno=%d (%s), expected EAGAIN",
                 args.dqbuf_errno, strerror(args.dqbuf_errno));
        FAIL(name, msg);
    } else {
        FAIL(name, "DQBUF succeeded unexpectedly after STREAMOFF");
    }
    /* teardown without extra STREAMOFF since we already called it */
    for (unsigned i = 0; i < ctx.buf_count; i++) {
        if (ctx.mapped[i] && ctx.mapped[i] != MAP_FAILED)
            munmap(ctx.mapped[i], ctx.lengths[i]);
    }
    close(ctx.fd);
}

/* ── Test 4: Buffer queue integrity — repeated QBUF/DQBUF cycles ───── */

static void test4_queue_integrity(const char *dev)
{
    const char *name = "Buffer queue integrity over 100 QBUF/DQBUF cycles";
    struct test_ctx ctx = {0};
    int cycles = 100;

    if (setup_device(&ctx, dev, 0) < 0) {
        FAIL(name, "setup failed");
        return;
    }

    /* QBUF all, STREAMON */
    for (unsigned i = 0; i < ctx.buf_count; i++)
        qbuf(&ctx, i);
    if (streamon(&ctx) < 0) {
        FAIL(name, "STREAMON failed");
        teardown_device(&ctx);
        return;
    }

    /* Run DQBUF → QBUF cycles */
    for (int c = 0; c < cycles; c++) {
        struct v4l2_buffer buf;
        int ret = dqbuf(&ctx, &buf);
        if (ret < 0) {
            char msg[80];
            snprintf(msg, sizeof(msg), "DQBUF failed at cycle %d: errno=%d (%s)",
                     c, errno, strerror(errno));
            FAIL(name, msg);
            teardown_device(&ctx);
            return;
        }
        if (buf.index >= ctx.buf_count) {
            char msg[64];
            snprintf(msg, sizeof(msg), "invalid index %u at cycle %d", buf.index, c);
            FAIL(name, msg);
            teardown_device(&ctx);
            return;
        }
        ret = qbuf(&ctx, buf.index);
        if (ret < 0) {
            char msg[80];
            snprintf(msg, sizeof(msg), "QBUF(%u) failed at cycle %d: errno=%d (%s)",
                     buf.index, c, errno, strerror(errno));
            FAIL(name, msg);
            teardown_device(&ctx);
            return;
        }
    }
    PASS(name);
    teardown_device(&ctx);
}

/* ── main ────────────────────────────────────────────────────────────── */

int main(int argc, char *argv[])
{
    const char *dev = argc > 1 ? argv[1] : "/dev/video50";

    fprintf(stderr, "=== OUTPUT DQBUF test suite ===\n");
    fprintf(stderr, "Device: %s\n\n", dev);

    test1_nonblock_eagain(dev);
    test2_blocking_qbuf_wakeup(dev);
    test3_streamoff_wakeup(dev);
    test4_queue_integrity(dev);

    fprintf(stderr, "\n=== Results: %d run, %d passed, %d failed ===\n",
            tests_run, tests_passed, tests_failed);

    return tests_failed > 0 ? 1 : 0;
}
