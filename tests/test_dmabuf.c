/*
 * test_dmabuf.c -- v4l2loopback DMABUF self-test
 *
 * Exercises VIDIOC_EXPBUF and V4L2_MEMORY_DMABUF on the OUTPUT queue in a
 * single process:
 *   - fill an MMAP OUTPUT buffer, export it with VIDIOC_EXPBUF, and check the
 *     exported dma-buf maps the same pixels
 *   - check EXPBUF rejects unsupported flags
 *   - re-request the queue as V4L2_MEMORY_DMABUF, queue the exported fd back
 *     in, and check QBUF/DQBUF preserve the DMABUF memory type
 *   - check VIDIOC_EXPBUF on an imported slot returns a pass-through fd
 *
 * Run against a loopback node, e.g. ./test_dmabuf /dev/video10
 */

#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#define WIDTH 320
#define HEIGHT 240
#define PATTERN 0xDEADBEEFu

static int fails;

#define OK(cond, msg)                                            \
	do {                                                     \
		if (cond) {                                      \
			printf("ok   - %s\n", (msg));            \
		} else {                                         \
			printf("FAIL - %s: %s\n", (msg),         \
			       strerror(errno));                 \
			fails++;                                 \
		}                                                \
	} while (0)

int main(int argc, char **argv)
{
	enum v4l2_buf_type otype = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	struct v4l2_format fmt = { 0 };
	struct v4l2_requestbuffers req = { 0 };
	struct v4l2_buffer buf = { 0 };
	struct v4l2_exportbuffer eb = { 0 };
	void *vmap = MAP_FAILED, *dmap = MAP_FAILED;
	size_t length;
	unsigned int i;
	int fd, dfd = -1;

	if (argc < 2) {
		printf("usage: %s <videodevice>\n", argv[0]);
		return 2;
	}

	fd = open(argv[1], O_RDWR);
	if (fd < 0) {
		perror("open");
		return 2;
	}

	/* 1. configure the OUTPUT queue as MMAP and fill buffer 0 */
	fmt.type = otype;
	fmt.fmt.pix.width = WIDTH;
	fmt.fmt.pix.height = HEIGHT;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB32;
	OK(ioctl(fd, VIDIOC_S_FMT, &fmt) == 0, "S_FMT");

	req.count = 2;
	req.type = otype;
	req.memory = V4L2_MEMORY_MMAP;
	OK(ioctl(fd, VIDIOC_REQBUFS, &req) == 0, "REQBUFS(MMAP)");

	buf.type = otype;
	buf.memory = V4L2_MEMORY_MMAP;
	buf.index = 0;
	OK(ioctl(fd, VIDIOC_QUERYBUF, &buf) == 0, "QUERYBUF");
	length = buf.length;

	vmap = mmap(NULL, length, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
		    buf.m.offset);
	OK(vmap != MAP_FAILED, "mmap output buffer");
	if (vmap != MAP_FAILED)
		for (i = 0; i < length / 4; i++)
			((uint32_t *)vmap)[i] = PATTERN;

	/* 2. export buffer 0 and check the dma-buf maps the same pixels */
	eb.type = otype;
	eb.index = 0;
	eb.flags = O_CLOEXEC | O_RDWR;
	OK(ioctl(fd, VIDIOC_EXPBUF, &eb) == 0 && eb.fd >= 0,
	   "EXPBUF (MMAP-backed export)");
	dfd = eb.fd;

	if (dfd >= 0) {
		dmap = mmap(NULL, length, PROT_READ, MAP_SHARED, dfd, 0);
		OK(dmap != MAP_FAILED, "mmap exported dma-buf");
	}
	OK(dmap != MAP_FAILED && ((uint32_t *)dmap)[0] == PATTERN &&
		   ((uint32_t *)dmap)[length / 4 - 1] == PATTERN,
	   "exported dma-buf sees the written pixels");

	/* unsupported flags must be rejected */
	{
		struct v4l2_exportbuffer bad = { .type = otype,
						 .index = 0,
						 .flags = O_APPEND };
		OK(ioctl(fd, VIDIOC_EXPBUF, &bad) < 0 && errno == EINVAL,
		   "EXPBUF rejects unsupported flags");
	}

	if (dmap != MAP_FAILED)
		munmap(dmap, length);
	if (vmap != MAP_FAILED)
		munmap(vmap, length); /* unmap so REQBUFS can reconfigure */

	/* 3. reconfigure as DMABUF and queue the exported fd back in */
	memset(&req, 0, sizeof(req));
	req.count = 2;
	req.type = otype;
	req.memory = V4L2_MEMORY_DMABUF;
	OK(ioctl(fd, VIDIOC_REQBUFS, &req) == 0, "REQBUFS(DMABUF)");

	memset(&buf, 0, sizeof(buf));
	buf.type = otype;
	buf.memory = V4L2_MEMORY_DMABUF;
	buf.index = 0;
	buf.m.fd = dfd;
	OK(ioctl(fd, VIDIOC_QBUF, &buf) == 0, "QBUF(DMABUF)");
	OK(buf.memory == V4L2_MEMORY_DMABUF,
	   "QBUF preserves DMABUF memory type");

	/* 4. EXPBUF on the imported slot returns a pass-through fd */
	memset(&eb, 0, sizeof(eb));
	eb.type = otype;
	eb.index = 0;
	eb.flags = O_CLOEXEC;
	OK(ioctl(fd, VIDIOC_EXPBUF, &eb) == 0 && eb.fd >= 0,
	   "EXPBUF pass-through on imported slot");
	if (eb.fd >= 0)
		close(eb.fd);

	OK(ioctl(fd, VIDIOC_STREAMON, &otype) == 0, "STREAMON");

	memset(&buf, 0, sizeof(buf));
	buf.type = otype;
	buf.memory = V4L2_MEMORY_DMABUF;
	OK(ioctl(fd, VIDIOC_DQBUF, &buf) == 0, "DQBUF(DMABUF)");
	OK(buf.memory == V4L2_MEMORY_DMABUF,
	   "DQBUF preserves DMABUF memory type");

	ioctl(fd, VIDIOC_STREAMOFF, &otype);
	if (dfd >= 0)
		close(dfd);
	close(fd);

	printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "PASSED", fails,
	       fails == 1 ? "" : "s");
	return fails ? 1 : 0;
}
