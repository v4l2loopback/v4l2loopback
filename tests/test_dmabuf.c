/*
 * test_dmabuf.c -- v4l2loopback DMABUF self-test
 *
 * Exercises VIDIOC_EXPBUF and V4L2_MEMORY_DMABUF on the OUTPUT queue with a
 * producer and a separate capture consumer:
 *   - fill an MMAP OUTPUT buffer, export it with VIDIOC_EXPBUF, and check the
 *     exported dma-buf maps the same pixels
 *   - check EXPBUF rejects unsupported flags
 *   - re-request the queue as V4L2_MEMORY_DMABUF, queue the exported fd back
 *     in, and check QBUF/DQBUF preserve the DMABUF memory type
 *   - keep a CAPTURE pass-through fd across OUTPUT teardown and producer exit
 *
 * Run against a loopback node, e.g. ./test_dmabuf /dev/video10
 */

#include <errno.h>
#include <fcntl.h>
#include <linux/dma-buf.h>
#include <linux/videodev2.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/wait.h>
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

struct expected_frame {
	dev_t device;
	ino_t inode;
	size_t length;
};

static int producer(const char *device, int peer, int streamoff)
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
	struct stat st;
	struct expected_frame frame = { 0 };
	char ack;

	fd = open(device, O_RDWR);
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
	eb.fd = -1;
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
	buf.bytesused = fmt.fmt.pix.sizeimage;
	OK(ioctl(fd, VIDIOC_QBUF, &buf) == 0, "QBUF(DMABUF)");
	OK(buf.memory == V4L2_MEMORY_DMABUF,
	   "QBUF preserves DMABUF memory type");

	/* 4. EXPBUF on the imported slot returns a pass-through fd */
	memset(&eb, 0, sizeof(eb));
	eb.fd = -1;
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

	if (fails)
		return 1;
	if (fstat(dfd, &st) < 0) {
		perror("fstat source DMA-BUF");
		return 1;
	}
	frame.device = st.st_dev;
	frame.inode = st.st_ino;
	frame.length = length;
	if (send(peer, &frame, sizeof(frame), MSG_NOSIGNAL) != sizeof(frame)) {
		perror("send source identity");
		return 1;
	}
	/* The consumer closes the socket after taking its own export. */
	if (read(peer, &ack, 1) != 0)
		return 1;

	if (streamoff) {
		OK(ioctl(fd, VIDIOC_STREAMOFF, &otype) == 0, "OUTPUT STREAMOFF");
		req.count = 0;
		OK(ioctl(fd, VIDIOC_REQBUFS, &req) == 0, "OUTPUT REQBUFS(0)");
	}
	/* The other case exercises implicit queue teardown on process exit. */
	return fails != 0;
}

static int capture_export(const char *device)
{
	struct v4l2_requestbuffers req = {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.memory = V4L2_MEMORY_MMAP,
		.count = 2,
	};
	struct v4l2_exportbuffer eb = {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.index = 0,
		.flags = O_CLOEXEC | O_RDONLY,
		.fd = -1,
	};
	int fd = open(device, O_RDWR);

	if (fd < 0) {
		perror("open capture");
		return -1;
	}
	if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0 ||
	    ioctl(fd, VIDIOC_EXPBUF, &eb) < 0) {
		perror("capture buffer export");
		close(fd);
		return -1;
	}
	close(fd);
	return eb.fd;
}

static int matches_frame(int fd, size_t length)
{
	const uint32_t *pixels;
	struct dma_buf_sync sync = {
		.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ,
	};
	size_t i;
	int ended;

	if (!length || length % sizeof(*pixels))
		return 0;
	pixels = mmap(NULL, length, PROT_READ, MAP_SHARED, fd, 0);
	if (pixels == MAP_FAILED)
		return 0;
	if (ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync) < 0) {
		perror("begin DMA-BUF read");
		munmap((void *)pixels, length);
		return 0;
	}
	for (i = 0; i < length / sizeof(*pixels); i++)
		if (pixels[i] != PATTERN)
			break;
	sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ;
	ended = ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
	munmap((void *)pixels, length);
	if (i != length / sizeof(*pixels))
		fprintf(stderr, "pixel mismatch at byte %zu of %zu\n",
			i * sizeof(*pixels), length);
	return ended == 0 && i == length / sizeof(*pixels);
}

static int run_case(const char *device, int streamoff)
{
	struct expected_frame frame;
	struct stat st;
	int peer[2], exported = -1, status;
	int same_buffer = 0;
	pid_t child, waited;

	printf("Producer teardown: %s\n",
	       streamoff ? "STREAMOFF and REQBUFS(0)" : "process exit");
	fflush(stdout);
	if (socketpair(AF_UNIX, SOCK_SEQPACKET, 0, peer) < 0) {
		perror("socketpair");
		return 1;
	}
	child = fork();
	if (child < 0) {
		perror("fork");
		close(peer[0]);
		close(peer[1]);
		return 1;
	}
	if (child == 0) {
		int ret;

		close(peer[0]);
		ret = producer(device, peer[1], streamoff);
		fflush(stdout);
		_exit(ret);
	}
	close(peer[1]);
	if (recv(peer[0], &frame, sizeof(frame), 0) == sizeof(frame)) {
		exported = capture_export(device);
		same_buffer = exported >= 0 && fstat(exported, &st) == 0 &&
			      st.st_dev == frame.device && st.st_ino == frame.inode;
	}
	OK(same_buffer, "CAPTURE export is the producer's original DMA-BUF");
	/* The consumer holds only the export while the producer tears down. */
	close(peer[0]);
	do {
		waited = waitpid(child, &status, 0);
	} while (waited < 0 && errno == EINTR);
	if (waited < 0 || !WIFEXITED(status) || WEXITSTATUS(status) != 0) {
		fprintf(stderr, "producer did not finish successfully\n");
		if (exported >= 0)
			close(exported);
		return 1;
	}
	if (same_buffer)
		OK(matches_frame(exported, frame.length),
		   "CAPTURE export maps the complete original frame after producer exit");
	if (exported >= 0)
		close(exported);
	return fails != 0;
}

int main(int argc, char **argv)
{
	int ret;

	if (argc < 2) {
		fprintf(stderr, "usage: %s <videodevice>\n", argv[0]);
		return 2;
	}
	ret = run_case(argv[1], 1);
	if (!ret)
		ret = run_case(argv[1], 0);
	printf("\n%s\n", ret ? "FAILED" : "PASSED");
	return ret;
}
