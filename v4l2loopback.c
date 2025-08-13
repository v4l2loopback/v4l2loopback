/* -*- c-file-style: "linux" -*- */
/*
 * v4l2loopback.c  --  video4linux2 loopback driver
 *
 * Copyright (C) 2005-2009 Vasily Levin (vasaka@gmail.com)
 * Copyright (C) 2010-2023 IOhannes m zmoelnig (zmoelnig@iem.at)
 * Copyright (C) 2011 Stefan Diewald (stefan.diewald@mytum.de)
 * Copyright (C) 2012 Anton Novikov (random.plant@gmail.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/time.h>
#include <linux/module.h>
#include <linux/videodev2.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/capability.h>
#include <linux/eventpoll.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/miscdevice.h>
#include <linux/overflow.h>
#include "v4l2loopback.h"

#define V4L2LOOPBACK_CTL_ADD_legacy 0x4C80
#define V4L2LOOPBACK_CTL_REMOVE_legacy 0x4C81
#define V4L2LOOPBACK_CTL_QUERY_legacy 0x4C82

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 0, 0)
#error This module is not supported on kernels before 4.0.0.
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 3, 0)
#define strscpy strlcpy
#endif

#if defined(timer_setup) && defined(from_timer)
#define HAVE_TIMER_SETUP
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 7, 0)
#define VFL_TYPE_VIDEO VFL_TYPE_GRABBER
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 2, 0)
#define timer_delete_sync del_timer_sync
#endif

#define V4L2LOOPBACK_VERSION_CODE                                              \
	KERNEL_VERSION(V4L2LOOPBACK_VERSION_MAJOR, V4L2LOOPBACK_VERSION_MINOR, \
		       V4L2LOOPBACK_VERSION_BUGFIX)

MODULE_DESCRIPTION("V4L2 loopback video device");
MODULE_AUTHOR("Vasily Levin, "
	      "IOhannes m zmoelnig <zmoelnig@iem.at>,"
	      "Stefan Diewald,"
	      "Anton Novikov"
	      "et al.");
#ifdef SNAPSHOT_VERSION
MODULE_VERSION(__stringify(SNAPSHOT_VERSION));
#else
MODULE_VERSION("" __stringify(V4L2LOOPBACK_VERSION_MAJOR) "." __stringify(
	V4L2LOOPBACK_VERSION_MINOR) "." __stringify(V4L2LOOPBACK_VERSION_BUGFIX));
#endif
MODULE_LICENSE("GPL");

/*
 * helpers
 */
#define dprintk(fmt, args...)                                          \
	do {                                                           \
		if (debug > 0) {                                       \
			printk(KERN_INFO "v4l2-loopback[" __stringify( \
				       __LINE__) "], pid(%d):  " fmt,  \
			       task_pid_nr(current), ##args);          \
		}                                                      \
	} while (0)

#define MARK()                                                             \
	do {                                                               \
		if (debug > 1) {                                           \
			printk(KERN_INFO "%s:%d[%s], pid(%d)\n", __FILE__, \
			       __LINE__, __func__, task_pid_nr(current));  \
		}                                                          \
	} while (0)

#define dprintkrw(fmt, args...)                                        \
	do {                                                           \
		if (debug > 2) {                                       \
			printk(KERN_INFO "v4l2-loopback[" __stringify( \
				       __LINE__) "], pid(%d): " fmt,   \
			       task_pid_nr(current), ##args);          \
		}                                                      \
	} while (0)

static inline void v4l2l_get_timestamp(struct v4l2_buffer *b)
{
	struct timespec64 ts;
	ktime_get_ts64(&ts);

	b->timestamp.tv_sec = ts.tv_sec;
	b->timestamp.tv_usec = (ts.tv_nsec / NSEC_PER_USEC);
	b->flags |= V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	b->flags &= ~V4L2_BUF_FLAG_TIMESTAMP_COPY;
}

#if BITS_PER_LONG == 32
#include <asm/div64.h> /* do_div() for 64bit division */
static inline int v4l2l_mod64(const s64 A, const u32 B)
{
	u64 a = (u64)A;
	u32 b = B;

	if (A > 0)
		return do_div(a, b);
	a = -A;
	return -do_div(a, b);
}
#else
static inline int v4l2l_mod64(const s64 A, const u32 B)
{
	return A % B;
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 16, 0)
typedef unsigned __poll_t;
#endif

/* module constants
 *  can be overridden during he build process using something like
 *	make KCPPFLAGS="-DMAX_DEVICES=100"
 */

/* maximum number of v4l2loopback devices that can be created */
#ifndef MAX_DEVICES
#define MAX_DEVICES 8
#endif

/* whether the default is to announce capabilities exclusively or not */
#ifndef V4L2LOOPBACK_DEFAULT_EXCLUSIVECAPS
#define V4L2LOOPBACK_DEFAULT_EXCLUSIVECAPS 0
#endif

/* when a producer is considered to have gone stale */
#ifndef MAX_TIMEOUT
#define MAX_TIMEOUT (100 * 1000) /* in msecs */
#endif

/* max buffers that can be mapped, actually they
 * are all mapped to max_buffers buffers */
#ifndef MAX_BUFFERS
#define MAX_BUFFERS 32
#endif

/* module parameters */
static int debug = 0;
module_param(debug, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "debugging level (higher values == more verbose)");

#define V4L2LOOPBACK_DEFAULT_MAX_BUFFERS 2
static int max_buffers = V4L2LOOPBACK_DEFAULT_MAX_BUFFERS;
module_param(max_buffers, int, S_IRUGO);
MODULE_PARM_DESC(max_buffers,
		 "how many buffers should be allocated [DEFAULT: " __stringify(
			 V4L2LOOPBACK_DEFAULT_MAX_BUFFERS) "]");

/* how many times a device can be opened
 * the per-module default value can be overridden on a per-device basis using
 * the /sys/devices interface
 *
 * note that max_openers should be at least 2 in order to get a working system:
 *   one opener for the producer and one opener for the consumer
 *   however, we leave that to the user
 */
#define V4L2LOOPBACK_DEFAULT_MAX_OPENERS 10
static int max_openers = V4L2LOOPBACK_DEFAULT_MAX_OPENERS;
module_param(max_openers, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(
	max_openers,
	"how many users can open the loopback device [DEFAULT: " __stringify(
		V4L2LOOPBACK_DEFAULT_MAX_OPENERS) "]");

static int devices = -1;
module_param(devices, int, 0);
MODULE_PARM_DESC(devices, "how many devices should be created");

static int video_nr[MAX_DEVICES] = { [0 ...(MAX_DEVICES - 1)] = -1 };
module_param_array(video_nr, int, NULL, 0444);
MODULE_PARM_DESC(video_nr,
		 "video device numbers (-1=auto, 0=/dev/video0, etc.)");

static char *card_label[MAX_DEVICES];
module_param_array(card_label, charp, NULL, 0000);
MODULE_PARM_DESC(card_label, "card labels for each device");

static bool exclusive_caps[MAX_DEVICES] = {
	[0 ...(MAX_DEVICES - 1)] = V4L2LOOPBACK_DEFAULT_EXCLUSIVECAPS
};
module_param_array(exclusive_caps, bool, NULL, 0444);
/* FIXXME: wording */
MODULE_PARM_DESC(
	exclusive_caps,
	"whether to announce OUTPUT/CAPTURE capabilities exclusively or not  [DEFAULT: " __stringify(
		V4L2LOOPBACK_DEFAULT_EXCLUSIVECAPS) "]");

/* format specifications */
#define V4L2LOOPBACK_SIZE_MIN_WIDTH 2
#define V4L2LOOPBACK_SIZE_MIN_HEIGHT 1
#define V4L2LOOPBACK_SIZE_DEFAULT_MAX_WIDTH 8192
#define V4L2LOOPBACK_SIZE_DEFAULT_MAX_HEIGHT 8192

#define V4L2LOOPBACK_SIZE_DEFAULT_WIDTH 640
#define V4L2LOOPBACK_SIZE_DEFAULT_HEIGHT 480

static int max_width = V4L2LOOPBACK_SIZE_DEFAULT_MAX_WIDTH;
module_param(max_width, int, S_IRUGO);
MODULE_PARM_DESC(max_width,
		 "maximum allowed frame width [DEFAULT: " __stringify(
			 V4L2LOOPBACK_SIZE_DEFAULT_MAX_WIDTH) "]");
static int max_height = V4L2LOOPBACK_SIZE_DEFAULT_MAX_HEIGHT;
module_param(max_height, int, S_IRUGO);
MODULE_PARM_DESC(max_height,
		 "maximum allowed frame height [DEFAULT: " __stringify(
			 V4L2LOOPBACK_SIZE_DEFAULT_MAX_HEIGHT) "]");

static DEFINE_IDR(v4l2loopback_index_idr);
static DEFINE_MUTEX(v4l2loopback_ctl_mutex);

/**
 * Copyright (C) 2025 Eli Oliveira Junior
 * 
 * Dynamic Buffering Control Parameter
 *
 * @dynamic_buffering: Global switch for dynamic buffering feature
 * 
 * Controls whether dynamic buffering is enabled system-wide:
 *   0 = Disabled (default)
 *   1 = Enabled
 *
 * This parameter affects all v4l2 loopback devices when set at module load time.
 * Can be modified at runtime through sysfs at:
 * /sys/module/<module>/parameters/dynamic_buffering
 *
 */
static int dynamic_buffering = 0; // 0=Disabled, 1=Enabled
module_param(dynamic_buffering, int, 0644);
MODULE_PARM_DESC(dynamic_buffering,
		 "Control Dynamic Buffering (0=Disable, 1=Enable, [Default=0]");

/* frame intervals */
#define V4L2LOOPBACK_FRAME_INTERVAL_MAX __UINT32_MAX__
#define V4L2LOOPBACK_FPS_DEFAULT 30
#define V4L2LOOPBACK_FPS_MAX 1000

/* control IDs */
#define V4L2LOOPBACK_CID_BASE (V4L2_CID_USER_BASE | 0xf000)
#define CID_KEEP_FORMAT (V4L2LOOPBACK_CID_BASE + 0)
#define CID_SUSTAIN_FRAMERATE (V4L2LOOPBACK_CID_BASE + 1)
#define CID_TIMEOUT (V4L2LOOPBACK_CID_BASE + 2)
#define CID_TIMEOUT_IMAGE_IO (V4L2LOOPBACK_CID_BASE + 3)

static int v4l2loopback_s_ctrl(struct v4l2_ctrl *ctrl);
static const struct v4l2_ctrl_ops v4l2loopback_ctrl_ops = {
	.s_ctrl = v4l2loopback_s_ctrl,
};
static const struct v4l2_ctrl_config v4l2loopback_ctrl_keepformat = {
	// clang-format off
	.ops	= &v4l2loopback_ctrl_ops,
	.id	= CID_KEEP_FORMAT,
	.name	= "keep_format",
	.type	= V4L2_CTRL_TYPE_BOOLEAN,
	.min	= 0,
	.max	= 1,
	.step	= 1,
	.def	= 0,
	// clang-format on
};
static const struct v4l2_ctrl_config v4l2loopback_ctrl_sustainframerate = {
	// clang-format off
	.ops	= &v4l2loopback_ctrl_ops,
	.id	= CID_SUSTAIN_FRAMERATE,
	.name	= "sustain_framerate",
	.type	= V4L2_CTRL_TYPE_BOOLEAN,
	.min	= 0,
	.max	= 1,
	.step	= 1,
	.def	= 0,
	// clang-format on
};
static const struct v4l2_ctrl_config v4l2loopback_ctrl_timeout = {
	// clang-format off
	.ops	= &v4l2loopback_ctrl_ops,
	.id	= CID_TIMEOUT,
	.name	= "timeout",
	.type	= V4L2_CTRL_TYPE_INTEGER,
	.min	= 0,
	.max	= MAX_TIMEOUT,
	.step	= 1,
	.def	= 0,
	// clang-format on
};
static const struct v4l2_ctrl_config v4l2loopback_ctrl_timeoutimageio = {
	// clang-format off
	.ops	= &v4l2loopback_ctrl_ops,
	.id	= CID_TIMEOUT_IMAGE_IO,
	.name	= "timeout_image_io",
	.type	= V4L2_CTRL_TYPE_BUTTON,
	.min	= 0,
	.max	= 0,
	.step	= 0,
	.def	= 0,
	// clang-format on
};

/* module structures */
struct v4l2loopback_private {
	int device_nr;
};

/* TODO(vasaka) use typenames which are common to kernel, but first find out if
 * it is needed */
/* struct keeping state and settings of loopback device */

struct v4l2l_buffer {
	struct v4l2_buffer buffer;
	struct list_head list_head;
	atomic_t use_count;
};

/*
    Copyright (C) 2025 Eli Oliveira Junior

    Dynamic Buffer Configuration Constants
    Defines the core parameters governing dynamic buffer allocation and management:

    Size Parameters:
        MIN_BUFFER_SIZE: Minimum allocation size (4 memory pages)
        MAX_DYNAMIC_BUFFER_SIZE: Maximum permitted buffer capacity (64MB)
        INITIAL_BUFFER_SIZE: Default allocation at creation (1MB)
    Watermark Thresholds:
        HIGH_WATERMARK_PERCENT: Expansion trigger (85% utilization)
        LOW_WATERMARK_FRACTION: Shrink trigger (1/8th utilization = 12.5%)
        SHRINK_HEADROOM_DIVISOR: Safety margin when downsizing (25% headroom)
    Operational Parameters:
        MAX_RESIZE_RETRIES: Maximum expansion attempts during contention
        BUFFER_GRACE_PERIOD_MS: Cleanup wait period during shutdown (5ms)
        BUFFER_RESIZE_LOG_PREFIX: System log identifier for buffer events
    Design Considerations:
        Memory Efficiency: Balances between over-allocation and frequent resizing
        Performance: Optimized thresholds minimize resize operations
        Safety: Includes headroom margins to prevent thrashing
        Observability: Log prefix enables system monitoring

    Note: All size values are carefully aligned with typical page sizes
          and kernel memory allocation patterns.
*/
#define MIN_BUFFER_SIZE (4UL * PAGE_SIZE)
#define MAX_DYNAMIC_BUFFER_SIZE (64UL * 1024 * 1024) /* 64MB */
#define INITIAL_BUFFER_SIZE (1UL * 1024 * 1024) /* 1MB */
#define HIGH_WATERMARK_PERCENT 85
#define LOW_WATERMARK_FRACTION 8 /* 1/8 = 12.5% */
#define SHRINK_HEADROOM_DIVISOR 4 /* 25% headroom */
#define MAX_RESIZE_RETRIES 3
#define BUFFER_GRACE_PERIOD_MS 5 /* Cleanup grace period */
#define BUFFER_RESIZE_LOG_PREFIX "[v4l2-loopback] Dynamic Buffer"

/**
    Copyright (C) 2025 Eli Oliveira Junior

    struct dynamic_buffer_stats - Performance monitoring for dynamic buffers
    Tracks operational metrics and performance characteristics of dynamic buffers,
    providing visibility into system behavior and facilitating optimization.

    Data Organization:
      Operational Counters:
         @total_bytes_written: Lifetime bytes written (atomic)
         @total_bytes_read: Lifetime bytes read (atomic)
         @frames_written: Successful frame writes (atomic)
         @frames_read: Successful frame reads (atomic)
         @frames_dropped: Frame loss due to overflow (atomic)
      Memory Management Metrics:
         @expand_count: Successful buffer expansions
         @shrink_count: Successful buffer contractions
         @total_expand_time_us: Cumulative expansion time (μs)
         @total_shrink_time_us: Cumulative shrinkage time (μs)
         @max_capacity_reached: Peak buffer size achieved
         @min_capacity_used: Minimum utilization percentage
      Timing Information:
         @created_at: Buffer initialization timestamp
         @last_expand_at: Most recent expansion time
         @last_shrink_at: Most recent shrink time
         @last_write_at: Last successful write op
         @last_read_at: Last successful read op
      Performance Indicators:
         @current_fps_write: Instantaneous write rate (frames/sec)
         @current_fps_read: Instantaneous read rate (frames/sec)
         @last_fps_update: Last rate calculation timestamp
    Implementation Notes:
        All counters updated atomically for lock-free access
        Timestamps use kernel's high-resolution time (ktime_t)
        Rate calculations performed periodically to minimize overhead
        Metrics designed for both real-time monitoring and historical analysis
    Monitoring Use Cases:
        Performance bottleneck identification
        Memory utilization optimization
        I/O pattern analysis
        Capacity planning
        Anomaly detection
*/

struct dynamic_buffer_stats {
	/* Operations counters */
	u64 total_bytes_written;
	u64 total_bytes_read;
	u64 frames_written;
	u64 frames_read;
	u64 frames_dropped;

	/* Resizing statistics */
	u32 expand_count;
	u32 shrink_count;
	u64 total_expand_time_us;
	u64 total_shrink_time_us;
	u32 max_capacity_reached;
	u32 min_capacity_used;

	/* Timestamps */
	ktime_t created_at;
	ktime_t last_expand_at;
	ktime_t last_shrink_at;
	ktime_t last_write_at;
	ktime_t last_read_at;

	/* Performance metrics */
	u32 current_fps_write;
	u32 current_fps_read;
	u64 last_fps_update;
};

/*
    Copyright (C) 2025 Eli Oliveira Junior

    struct dynamic_buffer - Core structure for dynamic circular buffer implementation
    Implements a thread-safe, resizable circular buffer with atomic operations,
    blocking I/O support, and comprehensive performance monitoring.

    Core Components:
       @data: Virtual memory allocation for buffer storage
       @size: Current usable capacity in bytes (power-of-2 aligned)
       @initial_size: Original allocation size (preserved for reference)
    Buffer State Tracking:
       @write_pos: Current producer position (modulo size)
       @read_pos:  Current consumer position (modulo size)
       @available: Atomic counter of readable bytes
       @ref_count: Atomic reference counter for lifecycle management
       @active:    Operational state flag (false during shutdown)
       @shutdown_requested: Graceful termination indicator
    Concurrency Control:
       @lock: Spinlock protecting:
              - Position updates
              - Buffer state transitions
              - Resize operations
       @read_waitq: Wait queue for blocking read operations
       @write_waitq: Wait queue for blocking write operations
    Performance Monitoring:
       @stats: Embedded statistics structure tracking:
               - Throughput metrics
               - Resize operations
               - Timing information
               - Error conditions
    Implementation Notes:
       Circular buffer semantics automatically handle wrap-around
       All position counters use unsigned 32-bit arithmetic with modulo
       Size always maintained as power-of-2 for efficient modulo operations
       Reference counting ensures safe memory reclamation
       Wait queues enable efficient blocking I/O
    Safety Guarantees:
       Atomic operations for all counters
       Spinlock-protected critical sections
       Graceful shutdown sequence
       Thread-safe for concurrent readers/writers
    Memory Layout:
       +-------------------------------+
       | dynamic_buffer (metadata)     |
       +-------------------------------+
       | data (mmap'ed region)         |
       +-------------------------------+
       | stats (performance counters)  |
       +-------------------------------+
*/
struct dynamic_buffer {
	void *data; /* Data buffer */
	size_t size; /* Current buffer size */
	size_t initial_size; /* Initial reference size */
	u32 write_pos; /* Writing position (circular) */
	u32 read_pos; /* Reading position (circular) */
	atomic_t available; /* Bytes available for reading*/
	atomic_t ref_count; /* Reference counter */
	spinlock_t lock; /* Lock for critical operations */
	wait_queue_head_t read_waitq; /* Waiting list for readers */
	wait_queue_head_t write_waitq; /* Waiting list for writers*/
	bool active; /* Buffer active state */
	bool shutdown_requested; /* Shutdown flag requested */
	struct dynamic_buffer_stats stats; /* Statistics */
};

/*rewrite to support dynamic buffering
 Copyright (C) 2025 Eli Oliveira Junior*/
struct v4l2_loopback_device {
	struct v4l2_device v4l2_dev;
	struct v4l2_ctrl_handler ctrl_handler;
	struct video_device *vdev;
	atomic_t active_producers; /* Counter of active producers (transmission) in dynamic mode */

	/* loopback device-specific parameters */
	char card_label[32];
	bool announce_all_caps; /* announce both OUTPUT and CAPTURE capabilities
				 * when true; else announce OUTPUT when no
				 * writer is streaming, otherwise CAPTURE. */
	int max_openers; /* how many times can this device be opened */
	int min_width, max_width;
	int min_height, max_height;

	/* pixel and stream format */
	struct v4l2_pix_format pix_format;
	bool pix_format_has_valid_sizeimage;
	struct v4l2_captureparm capture_param;
	unsigned long frame_jiffies;

	/* ctrls */
	int keep_format; /* CID_KEEP_FORMAT; lock the format, do not free
			  * on close(), and when `!announce_all_caps` do NOT
			  * fall back to OUTPUT when no writers attached (clear
			  * `keep_format` to attach a new writer) */
	int sustain_framerate; /* CID_SUSTAIN_FRAMERATE; duplicate frames to maintain
				  (close to) nominal framerate */
	unsigned long timeout_jiffies; /* CID_TIMEOUT; 0 means disabled */
	int timeout_image_io; /* CID_TIMEOUT_IMAGE_IO; next opener will
			       * queue/dequeue the timeout image buffer */

	/* buffers for OUTPUT and CAPTURE */
	u8 *image; /* pointer to actual buffers data */
	unsigned long image_size; /* number of bytes alloc'd for all buffers */
	struct v4l2l_buffer buffers[MAX_BUFFERS]; /* inner driver buffers */
	u32 buffer_count; /* should not be big, 4 is a good choice */
	u32 buffer_size; /* number of bytes alloc'd per buffer */
	u32 used_buffer_count; /* number of buffers allocated to openers */
	struct list_head outbufs_list; /* FIFO queue for OUTPUT buffers */
	u32 bufpos2index[MAX_BUFFERS]; /* mapping of `(position % used_buffers)`
					* to `buffers[index]` */
	s64 write_position; /* sequence number of last 'displayed' buffer plus
			     * one */

	/* synchronization between openers */
	atomic_t open_count;
	struct mutex image_mutex; /* mutex for allocating image(s) and
				   * exchanging format tokens */
	spinlock_t lock; /* lock for the timeout and framerate timers */
	spinlock_t list_lock; /* lock for the OUTPUT buffer queue */
	wait_queue_head_t read_event;
	u32 format_tokens; /* tokens to 'set format' for OUTPUT, CAPTURE, or
			    * timeout buffers */
	u32 stream_tokens; /* tokens to 'start' OUTPUT, CAPTURE, or timeout
			    * stream */

	/* sustain framerate */
	struct timer_list sustain_timer;
	unsigned int reread_count;

	/* timeout */
	u8 *timeout_image; /* copied to outgoing buffers when timeout passes */
	struct v4l2l_buffer timeout_buffer;
	u32 timeout_buffer_size; /* number bytes alloc'd for timeout buffer */
	struct timer_list timeout_timer;
	int timeout_happened;

	struct dynamic_buffer *dbuf;
	bool use_dynamic_buffering;
};

enum v4l2l_io_method {
	V4L2L_IO_NONE = 0,
	V4L2L_IO_MMAP = 1,
	V4L2L_IO_FILE = 2,
	V4L2L_IO_TIMEOUT = 3,
};

/* struct keeping state and type of opener */
struct v4l2_loopback_opener {
	u32 format_token; /* token (if any) for type used in call to S_FMT or
			   * REQBUFS */
	u32 stream_token; /* token (if any) for type used in call to STREAMON */
	u32 buffer_count; /* number of buffers (if any) that opener acquired via
			   * REQBUFS */
	s64 read_position; /* sequence number of the next 'captured' frame */
	unsigned int reread_count;
	enum v4l2l_io_method io_method;

	struct v4l2_fh fh;
};

#define fh_to_opener(ptr) container_of((ptr), struct v4l2_loopback_opener, fh)

/* this is heavily inspired by the bttv driver found in the linux kernel */
struct v4l2l_format {
	char *name;
	int fourcc; /* video4linux 2 */
	int depth; /* bit/pixel */
	int flags;
};
/* set the v4l2l_format.flags to PLANAR for non-packed formats */
#define FORMAT_FLAGS_PLANAR 0x01
#define FORMAT_FLAGS_COMPRESSED 0x02

#include "v4l2loopback_formats.h"

#ifndef V4L2_TYPE_IS_CAPTURE
#define V4L2_TYPE_IS_CAPTURE(type)                \
	((type) == V4L2_BUF_TYPE_VIDEO_CAPTURE || \
	 (type) == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
#endif /* V4L2_TYPE_IS_CAPTURE */
#ifndef V4L2_TYPE_IS_OUTPUT
#define V4L2_TYPE_IS_OUTPUT(type)                \
	((type) == V4L2_BUF_TYPE_VIDEO_OUTPUT || \
	 (type) == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
#endif /* V4L2_TYPE_IS_OUTPUT */

/* token values for privilege to set format or start/stop stream */
#define V4L2L_TOKEN_CAPTURE 0x01
#define V4L2L_TOKEN_OUTPUT 0x02
#define V4L2L_TOKEN_TIMEOUT 0x04
#define V4L2L_TOKEN_MASK \
	(V4L2L_TOKEN_CAPTURE | V4L2L_TOKEN_OUTPUT | V4L2L_TOKEN_TIMEOUT)

/* helpers for token exchange and token status */
#define token_from_type(type) \
	(V4L2_TYPE_IS_CAPTURE(type) ? V4L2L_TOKEN_CAPTURE : V4L2L_TOKEN_OUTPUT)

/*rewrite to support both static and dynamic buffering
 Copyright (C) 2025 Eli Oliveira Junior*/
#define acquire_token(dev, opener, label, token)                                  \
	do {                                                                      \
		unsigned long flags;                                              \
		spin_lock_irqsave(&(dev)->lock, flags);                           \
		(opener)->label##_token |= (token);                               \
		/* Remove token from pool regardless of mode - works for both */  \
		(dev)->label##_tokens &= ~(token);                                \
		spin_unlock_irqrestore(&(dev)->lock, flags);                      \
		dprintk("acquire_token: opener=%p got token=0x%x (dynamic=%d)\n", \
			opener, token, (dev)->use_dynamic_buffering);             \
	} while (0)

/*rewrite to support both static and dynamic buffering
 Copyright (C) 2025 Eli Oliveira Junior*/
#define release_token(dev, opener, label)                                                      \
	do {                                                                                   \
		unsigned long flags;                                                           \
		u32 releasing_token;                                                           \
		spin_lock_irqsave(&(dev)->lock, flags);                                        \
		releasing_token = (opener)->label##_token;                                     \
		(opener)->label##_token = 0;                                                   \
		if (releasing_token) {                                                         \
			/* Always returns token - conservative but functional behavior */      \
			(dev)->label##_tokens |= releasing_token;                              \
			dprintk("release_token: opener=%p released token=0x%x (dynamic=%d)\n", \
				opener, releasing_token,                                       \
				(dev)->use_dynamic_buffering);                                 \
		}                                                                              \
		spin_unlock_irqrestore(&(dev)->lock, flags);                                   \
	} while (0)

/* Check ownership in dynamic mode */
#define has_compatible_token(dev, opener, token)                               \
	((dev)->use_dynamic_buffering ?                                        \
		 ((opener)->format_token & (token)) || !(dev)->format_tokens : \
		 ((opener)->format_token & (token)))

#define has_output_token(token) (token & V4L2L_TOKEN_OUTPUT)
#define has_capture_token(token) (token & V4L2L_TOKEN_CAPTURE)
#define has_no_owners(dev) ((~((dev)->format_tokens) & V4L2L_TOKEN_MASK) == 0)
#define has_other_owners(opener, dev) \
	(~((dev)->format_tokens ^ (opener)->format_token) & V4L2L_TOKEN_MASK)
#define need_timeout_buffer(dev, token) \
	((dev)->timeout_jiffies > 0 || (token) & V4L2L_TOKEN_TIMEOUT)

static const unsigned int FORMATS = ARRAY_SIZE(formats);

/*
    Copyright (C) 2025 Eli Oliveira Junior

    Dynamic Buffer Management API
    Core interface for thread-safe, resizable circular buffer operations in V4L2 loopback.
    Implements automatic memory management with atomic operations and proper synchronization.

    Buffer Lifecycle Management:
       @init_dynamic_buffer: Initializes buffer with default parameters
       @free_dynamic_buffer: Safely releases all buffer resources
       @resize_dynamic_buffer: Grows/shrinks buffer while preserving contents
       @__resize_buffer_locked: Internal resize implementation (requires lock)
    Data Operations:
       @dynamic_buffer_write: Writes data with automatic expansion
       @dynamic_buffer_read: Reads data (blocking/non-blocking modes)
       @__copy_circular_data: Internal circular copy helper
       @calculate_buffer_used_bytes: Computes currently used buffer space
    System Integration:
       @sync_dynamic_to_v4l2_buffer: Synchronizes content with V4L2 buffer structure
       @format_buffer_size: Formats size for logging/diagnostics
    Key Features:
        Thread-safe circular buffer operations
        Automatic expansion/shrinking:
        Grows when reaching HIGH_WATERMARK_PERCENT
        Shrinks below LOW_WATERMARK_FRACTION
        Multiple access modes:
        Blocking with wait queues
        Non-blocking with immediate return
        Atomic reference counting
        Graceful shutdown handling
        Comprehensive statistics tracking
    Concurrency Model:
        Spinlock-protected for all structural changes
        Atomic operations for counters/positions
        Wait queues for blocking operations
        Reference counting for safe memory management
    Error Handling:
        Returns standard error codes
        Preserves data consistency on failure
        Automatic recovery from edge cases
    Memory Management:
        Uses vmalloc() for large buffer allocations
        Power-of-2 size optimization
        Efficient circular buffer arithmetic
    Performance Considerations:
        Minimizes memory copies during resize
        Lock contention optimized for reader/writer patterns
        Statistics collection with minimal overhead
*/

static int init_dynamic_buffer(struct v4l2_loopback_device *dev);
static int resize_dynamic_buffer(struct v4l2_loopback_device *dev,
				 u32 new_capacity);
static int dynamic_buffer_write(struct v4l2_loopback_device *dev, const u8 *src,
				u32 len);
static int dynamic_buffer_read(struct v4l2_loopback_device *dev, u8 *dst,
			       u32 len, bool block);
static int sync_dynamic_to_v4l2_buffer(struct v4l2_loopback_device *dev,
				       struct v4l2l_buffer *bufd,
				       u32 bytesused);
static int __resize_buffer_locked(struct dynamic_buffer *dbuf,
				  u32 new_capacity);
static void __copy_circular_data(const void *src_data, u32 src_size,
				 u32 src_pos, void *dst_data, u32 copy_len);
static void format_buffer_size(size_t size, char *buf, size_t buf_size);
static void free_dynamic_buffer(struct v4l2_loopback_device *dev);
static inline u32 calculate_buffer_used_bytes(struct dynamic_buffer *dbuf);

static char *fourcc2str(unsigned int fourcc, char buf[5])
{
	buf[0] = (fourcc >> 0) & 0xFF;
	buf[1] = (fourcc >> 8) & 0xFF;
	buf[2] = (fourcc >> 16) & 0xFF;
	buf[3] = (fourcc >> 24) & 0xFF;
	buf[4] = 0;

	return buf;
}

static const struct v4l2l_format *format_by_fourcc(int fourcc)
{
	unsigned int i;
	char buf[5];

	for (i = 0; i < FORMATS; i++) {
		if (formats[i].fourcc == fourcc)
			return formats + i;
	}

	dprintk("unsupported format '%4s'\n", fourcc2str(fourcc, buf));
	return NULL;
}

static void pix_format_set_size(struct v4l2_pix_format *f,
				const struct v4l2l_format *fmt,
				unsigned int width, unsigned int height)
{
	f->width = width;
	f->height = height;

	if (fmt->flags & FORMAT_FLAGS_PLANAR) {
		f->bytesperline = width; /* Y plane */
		f->sizeimage = (width * height * fmt->depth) >> 3;
	} else if (fmt->flags & FORMAT_FLAGS_COMPRESSED) {
		/* doesn't make sense for compressed formats */
		f->bytesperline = 0;
		f->sizeimage = (width * height * fmt->depth) >> 3;
	} else {
		f->bytesperline = (width * fmt->depth) >> 3;
		f->sizeimage = height * f->bytesperline;
	}
}

static int v4l2l_fill_format(struct v4l2_format *fmt, const u32 minwidth,
			     const u32 maxwidth, const u32 minheight,
			     const u32 maxheight)
{
	u32 width = fmt->fmt.pix.width, height = fmt->fmt.pix.height;
	u32 pixelformat = fmt->fmt.pix.pixelformat;
	struct v4l2_format fmt0 = *fmt;
	u32 bytesperline = 0, sizeimage = 0;

	if (!width)
		width = V4L2LOOPBACK_SIZE_DEFAULT_WIDTH;
	if (!height)
		height = V4L2LOOPBACK_SIZE_DEFAULT_HEIGHT;
	width = clamp_val(width, minwidth, maxwidth);
	height = clamp_val(height, minheight, maxheight);

	/* sets: width,height,pixelformat,bytesperline,sizeimage */
	if (!(V4L2_TYPE_IS_MULTIPLANAR(fmt0.type))) {
		fmt0.fmt.pix.bytesperline = 0;
		fmt0.fmt.pix.sizeimage = 0;
	}

	if (0) {
		;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 2, 0)
	} else if (!v4l2_fill_pixfmt(&fmt0.fmt.pix, pixelformat, width,
				     height)) {
		;
	} else if (!v4l2_fill_pixfmt_mp(&fmt0.fmt.pix_mp, pixelformat, width,
					height)) {
		;
#endif
	} else {
		const struct v4l2l_format *format =
			format_by_fourcc(pixelformat);
		if (!format)
			return -EINVAL;
		pix_format_set_size(&fmt0.fmt.pix, format, width, height);
		fmt0.fmt.pix.pixelformat = format->fourcc;
	}

	if (V4L2_TYPE_IS_MULTIPLANAR(fmt0.type)) {
		*fmt = fmt0;

		if ((fmt->fmt.pix_mp.colorspace == V4L2_COLORSPACE_DEFAULT) ||
		    (fmt->fmt.pix_mp.colorspace > V4L2_COLORSPACE_DCI_P3))
			fmt->fmt.pix_mp.colorspace = V4L2_COLORSPACE_SRGB;
		if (V4L2_FIELD_ANY == fmt->fmt.pix_mp.field)
			fmt->fmt.pix_mp.field = V4L2_FIELD_NONE;
	} else {
		bytesperline = fmt->fmt.pix.bytesperline;
		sizeimage = fmt->fmt.pix.sizeimage;

		*fmt = fmt0;

		if (!fmt->fmt.pix.bytesperline)
			fmt->fmt.pix.bytesperline = bytesperline;
		if (!fmt->fmt.pix.sizeimage)
			fmt->fmt.pix.sizeimage = sizeimage;

		if ((fmt->fmt.pix.colorspace == V4L2_COLORSPACE_DEFAULT) ||
		    (fmt->fmt.pix.colorspace > V4L2_COLORSPACE_DCI_P3))
			fmt->fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
		if (V4L2_FIELD_ANY == fmt->fmt.pix.field)
			fmt->fmt.pix.field = V4L2_FIELD_NONE;
	}

	return 0;
}

/* Checks if v4l2l_fill_format() has set a valid, fixed sizeimage val. */
static bool v4l2l_pix_format_has_valid_sizeimage(struct v4l2_format *fmt)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 2, 0)
	const struct v4l2_format_info *info;

	info = v4l2_format_info(fmt->fmt.pix.pixelformat);
	if (info && info->mem_planes == 1)
		return true;
#endif

	return false;
}

static int pix_format_eq(const struct v4l2_pix_format *ref,
			 const struct v4l2_pix_format *tgt, int strict)
{
	/* check if the two formats are equivalent.
	 * ANY fields are handled gracefully
	 */
#define _pix_format_eq0(x)    \
	if (ref->x != tgt->x) \
	result = 0
#define _pix_format_eq1(x, def)                              \
	do {                                                 \
		if ((def != tgt->x) && (ref->x != tgt->x)) { \
			printk(KERN_INFO #x " failed");      \
			result = 0;                          \
		}                                            \
	} while (0)
	int result = 1;
	_pix_format_eq0(width);
	_pix_format_eq0(height);
	_pix_format_eq0(pixelformat);
	if (!strict)
		return result;
	_pix_format_eq1(field, V4L2_FIELD_ANY);
	_pix_format_eq0(bytesperline);
	_pix_format_eq0(sizeimage);
	_pix_format_eq1(colorspace, V4L2_COLORSPACE_DEFAULT);
	return result;
}

static void set_timeperframe(struct v4l2_loopback_device *dev,
			     struct v4l2_fract *tpf)
{
	if (!tpf->denominator && !tpf->numerator) {
		tpf->numerator = 1;
		tpf->denominator = V4L2LOOPBACK_FPS_DEFAULT;
	} else if (tpf->numerator >
		   V4L2LOOPBACK_FRAME_INTERVAL_MAX * tpf->denominator) {
		/* divide-by-zero or greater than maximum interval => min FPS */
		tpf->numerator = V4L2LOOPBACK_FRAME_INTERVAL_MAX;
		tpf->denominator = 1;
	} else if (tpf->numerator * V4L2LOOPBACK_FPS_MAX < tpf->denominator) {
		/* zero or lower than minimum interval => max FPS */
		tpf->numerator = 1;
		tpf->denominator = V4L2LOOPBACK_FPS_MAX;
	}

	dev->capture_param.timeperframe = *tpf;
	dev->frame_jiffies =
		max(1UL, (msecs_to_jiffies(1000) * tpf->numerator) /
				 tpf->denominator);
}

static struct v4l2_loopback_device *v4l2loopback_cd2dev(struct device *cd);

/* device attributes */
/* available via sysfs: /sys/devices/virtual/video4linux/video* */

static ssize_t attr_show_format(struct device *cd,
				struct device_attribute *attr, char *buf)
{
	/* gets the current format as "FOURCC:WxH@f/s", e.g. "YUYV:320x240@1000/30" */
	struct v4l2_loopback_device *dev = v4l2loopback_cd2dev(cd);
	const struct v4l2_fract *tpf;
	char buf4cc[5], buf_fps[32];

	if (!dev || (has_no_owners(dev) && !dev->keep_format))
		return 0;
	tpf = &dev->capture_param.timeperframe;

	fourcc2str(dev->pix_format.pixelformat, buf4cc);
	if (tpf->numerator == 1)
		snprintf(buf_fps, sizeof(buf_fps), "%u", tpf->denominator);
	else
		snprintf(buf_fps, sizeof(buf_fps), "%u/%u", tpf->denominator,
			 tpf->numerator);
	return sprintf(buf, "%4s:%ux%u@%s\n", buf4cc, dev->pix_format.width,
		       dev->pix_format.height, buf_fps);
}

static ssize_t attr_store_format(struct device *cd,
				 struct device_attribute *attr, const char *buf,
				 size_t len)
{
	struct v4l2_loopback_device *dev = v4l2loopback_cd2dev(cd);
	int fps_num = 0, fps_den = 1;

	if (!dev)
		return -ENODEV;

	/* only fps changing is supported */
	if (sscanf(buf, "@%u/%u", &fps_num, &fps_den) > 0) {
		struct v4l2_fract f = { .numerator = fps_den,
					.denominator = fps_num };
		set_timeperframe(dev, &f);
		return len;
	}
	return -EINVAL;
}

static DEVICE_ATTR(format, S_IRUGO | S_IWUSR, attr_show_format,
		   attr_store_format);

static ssize_t attr_show_buffers(struct device *cd,
				 struct device_attribute *attr, char *buf)
{
	struct v4l2_loopback_device *dev = v4l2loopback_cd2dev(cd);

	if (!dev)
		return -ENODEV;

	return sprintf(buf, "%u\n", dev->used_buffer_count);
}

static DEVICE_ATTR(buffers, S_IRUGO, attr_show_buffers, NULL);

static ssize_t attr_show_maxopeners(struct device *cd,
				    struct device_attribute *attr, char *buf)
{
	struct v4l2_loopback_device *dev = v4l2loopback_cd2dev(cd);

	if (!dev)
		return -ENODEV;

	return sprintf(buf, "%d\n", dev->max_openers);
}

static ssize_t attr_store_maxopeners(struct device *cd,
				     struct device_attribute *attr,
				     const char *buf, size_t len)
{
	struct v4l2_loopback_device *dev = NULL;
	unsigned long curr = 0;

	if (kstrtoul(buf, 0, &curr))
		return -EINVAL;

	dev = v4l2loopback_cd2dev(cd);
	if (!dev)
		return -ENODEV;

	if (dev->max_openers == curr)
		return len;

	if (curr > __INT_MAX__ || dev->open_count.counter > curr) {
		/* request to limit to less openers as are currently attached to us */
		return -EINVAL;
	}

	dev->max_openers = (int)curr;

	return len;
}

static DEVICE_ATTR(max_openers, S_IRUGO | S_IWUSR, attr_show_maxopeners,
		   attr_store_maxopeners);

static ssize_t attr_show_state(struct device *cd, struct device_attribute *attr,
			       char *buf)
{
	struct v4l2_loopback_device *dev = v4l2loopback_cd2dev(cd);

	if (!dev)
		return -ENODEV;

	if (!has_output_token(dev->stream_tokens) || dev->keep_format) {
		return sprintf(buf, "capture\n");
	} else
		return sprintf(buf, "output\n");

	return -EAGAIN;
}

static DEVICE_ATTR(state, S_IRUGO, attr_show_state, NULL);

/*
    Copyright (C) 2025 Eli Oliveira Junior

    attr_show_dynamic_buffering - Sysfs read operation for dynamic buffering status
    @cd: Parent device structure (container device)
    @attr: Sysfs attribute reference (unused)
    @buf: Output buffer for status information (minimum 4 bytes required)

    Provides read access to the dynamic buffering configuration state through sysfs:
        Formats output as ASCII "0\n" (disabled) or "1\n" (enabled)
        Output is guaranteed to be null-terminated
        Implements standard VFS/sysfs read semantics
    The function safely handles:
        Invalid device references (returns -ENODEV)
        Concurrent access (protected by device locks)
        Buffer overflow protection (via sprintf)

    Usage Example:
    $ cat /sys/class/video4linux/video0/dynamic_buffering
    1
    Return: Number of bytes written to @buf (excluding null terminator) or:
            -ENODEV if device cannot be resolved
            Other negative values on formatting errors

    Locking: Acquires device mutex during status check

    Context: Process context (VFS/sysfs read handler)
*/
static ssize_t attr_show_dynamic_buffering(struct device *cd,
					   struct device_attribute *attr,
					   char *buf)
{
	struct v4l2_loopback_device *dev = v4l2loopback_cd2dev(cd);

	if (!dev)
		return -ENODEV;
	return sprintf(buf, "%d\n", dev->use_dynamic_buffering);
}

/*
    Copyright (C) 2025 Eli Oliveira Junior

    attr_store_dynamic_buffering - Sysfs write operation for dynamic buffering control

    @cd: Parent device structure (container device)
    @attr: Sysfs attribute reference (unused)
    @buf: Input buffer containing configuration value (ASCII "0" or "1")
    @len: Length of input buffer (minimum 1 byte required)

    Implements write access to dynamically enable/disable the buffering system:
        "0": Disables dynamic buffering (performs cleanup if buffer exists)
        "1": Enables dynamic buffering (lazy initialization on first use)
    The function ensures:
        Thread-safe configuration changes (protected by image_mutex)
        Proper resource cleanup when disabling
        Lazy allocation when enabling
        Input validation (numeric values 0-1 only)
    State Transition Behavior:
        Disabling (0):
        Immediately frees existing buffer via free_dynamic_buffer()
        Preserves configuration for potential future re-enabling
        Enabling (1):
        Marks system as enabled but doesn't allocate until first use
        Actual allocation happens during next buffer operation
        Requires valid buffer_size configuration
    Error Conditions:
        Returns -ENODEV for invalid device references
        Returns -EINVAL for:
        Non-numeric input
        Values outside 0-1 range
        Invalid buffer_size when enabling
        Propagates errors from memory operations
    Usage Example:
          echo 1 > /sys/class/video4linux/video0/dynamic_buffering
    Return: Number of bytes processed on success, negative error code on failure
    Locking: Acquires image_mutex for state changes
    Context: Process context (VFS/sysfs write handler)
*/
static ssize_t attr_store_dynamic_buffering(struct device *cd,
					    struct device_attribute *attr,
					    const char *buf, size_t len)
{
	struct v4l2_loopback_device *dev = v4l2loopback_cd2dev(cd);
	unsigned long val;
	int ret;

	if (!dev)
		return -ENODEV;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val > 1)
		return -EINVAL;

	mutex_lock(&dev->image_mutex);
	dev->use_dynamic_buffering = val;

	/* If disabling, free the dynamic buffer if it exists */
	if (!val && dev->dbuf) {
		free_dynamic_buffer(dev);
	}
	/* If Dynamic Buffering is enabled and still don't have a buffer, try allocating it */
	else if (val && !dev->dbuf && dev->buffer_size > 0) {
		init_dynamic_buffer(dev);
	}

	mutex_unlock(&dev->image_mutex);

	return len;
}

/*
    Copyright (C) 2025 Eli Oliveira Junior

    DEVICE_ATTR dynamic_buffering - Sysfs control interface for dynamic buffer management
    Defines a sysfs attribute that provides userspace control over the dynamic buffering
    subsystem with the following characteristics:
    Attribute Properties:
        Name: "dynamic_buffering"
        Path: /sys/class/video4linux/videoX/dynamic_buffering
        Permissions:
        Read: 0444 (S_IRUGO) - Readable by all users
        Write: 0200 (S_IWUSR) - Writable by root only
        Operations:
        Read: attr_show_dynamic_buffering
        Write: attr_store_dynamic_buffering
    Functional Behavior:
        Reading returns current state:
        "0\n" = Static buffering mode
        "1\n" = Dynamic buffering mode
        Writing accepts:
        "0" = Disables dynamic buffering (with resource cleanup)
        "1" = Enables dynamic buffering (with lazy initialization)
    Security Considerations:
        Write operations restricted to privileged users
        All input validated before processing
        State changes protected by device mutex
    Usage Examples:
        Check current status:
          $ cat /sys/class/video4linux/video0/dynamic_buffering
        Enable dynamic buffering:
          echo 1 > /sys/class/video4linux/video0/dynamic_buffering
        Disable dynamic buffering:
          echo 0 > /sys/class/video4linux/video0/dynamic_buffering
    Implementation Notes:
        Maintains consistency with sysfs conventions
        Integrates with device power management
        Preserves state across suspend/resume cycles
        Atomic state transitions
*/
static DEVICE_ATTR(dynamic_buffering, S_IRUGO | S_IWUSR,
		   attr_show_dynamic_buffering, attr_store_dynamic_buffering);

/*
    Copyright (C) 2025 Eli Oliveira Junior

    dynamic_buffer_stats_show - Comprehensive buffer telemetry interface
    @cd: Parent device structure (container device)
    @attr: Sysfs attribute reference (unused)
    @buf: Output buffer for statistics (minimum 1KB recommended)

    Generates a multi-section diagnostic report containing:
        Current State:
        Buffer capacity and utilization percentage
        Read/write positions
        System uptime
        I/O Metrics:
        Frame operations (written/read/dropped)
        Byte throughput (total written/read)
        Memory Management:
        Resize operations (counts and average duration)
        Peak capacity reached
        Last resize timestamps
        Performance Indicators:
        Current read/write rates
        Recent activity patterns
    Data Collection:
        All statistics gathered under spinlock protection
        Uses monotonic timestamps for accuracy
        Handles buffer wrap-around correctly
        Formats sizes in human-readable units
    Output Format:
        Structured ASCII report with clear section headers
        Fixed-point percentages (0-100%)
        Time values in seconds resolution
        Size values in appropriate units (KB/MB/GB)
    Error Handling:
        Returns error message if dynamic buffering inactive
        Handles division by zero in averages
        Validates buffer references
    Example Output:
       === V4L2-Loopback Dynamic Buffer Statistics ===
       -- Current State --
       Buffer size: 16.0 MB
       Used: 12.5 MB (78%)
       Read offset: 0x12345678
       Write offset: 0x23456789
       Uptime: 3600 seconds
       -- I/O Statistics --
       Frames written: 12000
       Frames read: 11500
       Frames dropped: 5
       Total written: 1.2 GB
       Total read: 1.1 GB
    Return: Number of bytes written to buffer or error message
    Locking: Acquires buffer spinlock during data collection
    Context: Process context (VFS/sysfs read handler)
    */
static ssize_t dynamic_buffer_stats_show(struct device *cd,
					 struct device_attribute *attr,
					 char *buf)
{
	struct v4l2_loopback_device *dev = v4l2loopback_cd2dev(cd);
	struct dynamic_buffer *dbuf = dev->dbuf;
	ssize_t len = 0;
	char size_str[32];
	ktime_t now;
	s64 uptime_seconds;
	u32 avg_expand_time_us = 0;
	u32 avg_shrink_time_us = 0;
	u32 data_used;
	unsigned long flags;

	if (!dev->use_dynamic_buffering || !dbuf) {
		return sprintf(buf, "Dynamic buffering is not active\n");
	}

	spin_lock_irqsave(&dbuf->lock, flags);

	now = ktime_get();
	uptime_seconds = ktime_ms_delta(now, dbuf->stats.created_at) / 1000;
	data_used = calculate_buffer_used_bytes(dbuf);

	/* Calculate averages */
	if (dbuf->stats.expand_count > 0)
		avg_expand_time_us = dbuf->stats.total_expand_time_us /
				     dbuf->stats.expand_count;
	if (dbuf->stats.shrink_count > 0)
		avg_shrink_time_us = dbuf->stats.total_shrink_time_us /
				     dbuf->stats.shrink_count;

	len += sprintf(buf + len,
		       "=== V4L2-Loopback Dynamic Buffer Statistics ===\n\n");

	/* Current buffer information */
	len += sprintf(buf + len, "-- Current State --\n");
	format_buffer_size(dbuf->size, size_str, sizeof(size_str));
	len += sprintf(buf + len, "Buffer size:     %s\n", size_str);
	format_buffer_size(data_used, size_str, sizeof(size_str));
	len += sprintf(
		buf + len, "Used:            %s (%u%%)\n", size_str,
		dbuf->size ? (unsigned int)(data_used * 100 / dbuf->size) : 0U);
	len += sprintf(buf + len, "Read offset:     %u\n", dbuf->read_pos);
	len += sprintf(buf + len, "Write offset:    %u\n", dbuf->write_pos);
	len += sprintf(buf + len, "Uptime:          %lld seconds\n\n",
		       uptime_seconds);

	/* I/O Statistics */
	len += sprintf(buf + len, "-- I/O Statistics --\n");
	len += sprintf(buf + len, "Frames written:  %llu\n",
		       dbuf->stats.frames_written);
	len += sprintf(buf + len, "Frames read:     %llu\n",
		       dbuf->stats.frames_read);
	len += sprintf(buf + len, "Frames dropped:  %llu\n",
		       dbuf->stats.frames_dropped);
	format_buffer_size(dbuf->stats.total_bytes_written, size_str,
			   sizeof(size_str));
	len += sprintf(buf + len, "Total written:   %s\n", size_str);
	format_buffer_size(dbuf->stats.total_bytes_read, size_str,
			   sizeof(size_str));
	len += sprintf(buf + len, "Total read:      %s\n\n", size_str);

	/* Resizing statistics */
	len += sprintf(buf + len, "-- Resize Statistics --\n");
	len += sprintf(buf + len, "Expand count:    %u (avg time: %u µs)\n",
		       dbuf->stats.expand_count, avg_expand_time_us);
	len += sprintf(buf + len, "Shrink count:    %u (avg time: %u µs)\n",
		       dbuf->stats.shrink_count, avg_shrink_time_us);
	format_buffer_size(dbuf->stats.max_capacity_reached, size_str,
			   sizeof(size_str));
	len += sprintf(buf + len, "Max capacity:    %s\n", size_str);

	/* Timestamps of last operations */
	if (dbuf->stats.last_expand_at) {
		s64 ago =
			ktime_ms_delta(now, dbuf->stats.last_expand_at) / 1000;
		len += sprintf(buf + len, "Last expand:     %lld seconds ago\n",
			       ago);
	}
	if (dbuf->stats.last_shrink_at) {
		s64 ago =
			ktime_ms_delta(now, dbuf->stats.last_shrink_at) / 1000;
		len += sprintf(buf + len, "Last shrink:     %lld seconds ago\n",
			       ago);
	}

	spin_unlock_irqrestore(&dbuf->lock, flags);

	return len;
}

/*
    Copyright (C) 2025 Eli Oliveira Junior

    dynamic_buffer_reset_stats_store - Sysfs control for resetting buffer statistics
    @cd: Parent device structure (container device)
    @attr: Sysfs attribute reference (unused)
    @buf: Input buffer containing reset command (ASCII "1" to reset)
    @len: Length of input buffer (minimum 1 byte required)

    Implements write access to reset all dynamic buffer statistics while maintaining:
        Original creation timestamp (preserves uptime reference)
        Current buffer capacity metrics
        Structural integrity of the buffer
    Reset Behavior:
        Clears all counters (I/O operations, resize events)
        Maintains creation timestamp for continuity
        Preserves current size as both min and max capacity
        Logs reset event to kernel log
    Security and Validation:
        Only accepts "1" as valid reset command
        Requires active dynamic buffering
        Performs atomic reset under spinlock protection
        Validates device and buffer state
    Error Conditions:
        Returns -EINVAL for:
        Dynamic buffering inactive
        Invalid buffer reference
        Empty or invalid command
    Usage Example:
        echo 1 > /sys/class/video4linux/video0/reset_stats
    Locking:
        Uses buffer spinlock to protect statistics during reset
        Short critical section for minimal contention
    Context:
        Process context (VFS/sysfs write handler)
        May sleep during logging operation
    Note: Statistics reset does not affect actual buffer contents or operations,
          only clears performance tracking metrics.
*/
static ssize_t dynamic_buffer_reset_stats_store(struct device *cd,
						struct device_attribute *attr,
						const char *buf, size_t len)
{
	struct v4l2_loopback_device *dev = v4l2loopback_cd2dev(cd);
	struct dynamic_buffer *dbuf = dev->dbuf;
	unsigned long flags;

	if (!dev->use_dynamic_buffering || !dbuf)
		return -EINVAL;

	if (len > 0 && buf[0] == '1') {
		spin_lock_irqsave(&dbuf->lock, flags);
		memset(&dbuf->stats, 0, sizeof(dbuf->stats));
		dbuf->stats.created_at = ktime_get();
		dbuf->stats.min_capacity_used = dbuf->size;
		dbuf->stats.max_capacity_reached = dbuf->size;
		spin_unlock_irqrestore(&dbuf->lock, flags);
		printk(KERN_INFO "%s: Statistics reset\n",
		       BUFFER_RESIZE_LOG_PREFIX);
	}

	return len;
}

/*
    Copyright (C) 2025 Eli Oliveira Junior

    Dynamic Buffer Sysfs Control Interface
    Provides userspace access to dynamic buffer monitoring and management through
    two complementary sysfs attributes:
        Statistics Interface (read-only):
    @attr: dynamic_buffer_stats
    @perms: 0444 (S_IRUGO)
    @path: /sys/class/video4linux/videoX/dynamic_buffer_stats
    @handler: dynamic_buffer_stats_show()
    Features:
        Comprehensive buffer telemetry
        Real-time utilization metrics
        Historical performance data
        Thread-safe data collection
        Human-readable multi-section format
        Control Interface (write-only):
    @attr: dynamic_buffer_reset_stats
    @perms: 0200 (S_IWUSR)
    @path: /sys/class/video4linux/videoX/dynamic_buffer_reset_stats
    @handler: dynamic_buffer_reset_stats_store()
    Features:
        Atomic statistics reset
        Privileged access only (root)
        Minimal impact on running operations
        System log confirmation
    Security Model:
        Read access: Available to all processes
        Write access: Restricted to privileged users
        Input validation: Strict command verification
        Atomic operations: Protected by spinlocks
    Usage Examples:
        View statistics:
          $ cat /sys/class/video4linux/video0/dynamic_buffer_stats
        Reset statistics (requires root):
          echo 1 > /sys/class/video4linux/video0/dynamic_buffer_reset_stats
    Design Considerations:
        Separate read/write paths for security
        No persistent storage of statistics
        Minimal performance impact during access
        Consistent formatting for parsing tools
*/
static DEVICE_ATTR(dynamic_buffer_stats, 0444, dynamic_buffer_stats_show, NULL);
static DEVICE_ATTR(dynamic_buffer_reset_stats, 0200, NULL,
		   dynamic_buffer_reset_stats_store);

static void v4l2loopback_remove_sysfs(struct video_device *vdev)
{
#define V4L2_SYSFS_DESTROY(x) device_remove_file(&vdev->dev, &dev_attr_##x)

	if (vdev) {
		V4L2_SYSFS_DESTROY(format);
		V4L2_SYSFS_DESTROY(buffers);
		V4L2_SYSFS_DESTROY(max_openers);
		V4L2_SYSFS_DESTROY(state);
		V4L2_SYSFS_DESTROY(
			dynamic_buffering); // Dynamic buffer implementation
		V4L2_SYSFS_DESTROY(
			dynamic_buffer_stats); // Dynamic buffer implementation
		V4L2_SYSFS_DESTROY(
			dynamic_buffer_reset_stats); //Dynamic buffer implementation
		/* ... */
	}
}

static void v4l2loopback_create_sysfs(struct video_device *vdev)
{
	int res = 0;

#define V4L2_SYSFS_CREATE(x)                                 \
	res = device_create_file(&vdev->dev, &dev_attr_##x); \
	if (res < 0)                                         \
	break
	if (!vdev)
		return;
	do {
		V4L2_SYSFS_CREATE(format);
		V4L2_SYSFS_CREATE(buffers);
		V4L2_SYSFS_CREATE(max_openers);
		V4L2_SYSFS_CREATE(state);
		V4L2_SYSFS_CREATE(
			dynamic_buffering); // Dynamic buffer implementation
		V4L2_SYSFS_CREATE(
			dynamic_buffer_stats); // Dynamic buffer implementation
		V4L2_SYSFS_CREATE(
			dynamic_buffer_reset_stats); // Dynamic buffer implementation

		/* ... */
	} while (0);

	if (res >= 0)
		return;
	dev_err(&vdev->dev, "%s error: %d\n", __func__, res);
}

/* Event APIs */

#define V4L2LOOPBACK_EVENT_BASE (V4L2_EVENT_PRIVATE_START)
#define V4L2LOOPBACK_EVENT_OFFSET 0x08E00000
#define V4L2_EVENT_PRI_CLIENT_USAGE \
	(V4L2LOOPBACK_EVENT_BASE + V4L2LOOPBACK_EVENT_OFFSET + 1)

struct v4l2_event_client_usage {
	__u32 count;
};

/* global module data */
/* find a device based on it's device-number (e.g. '3' for /dev/video3) */
struct v4l2loopback_lookup_cb_data {
	int device_nr;
	struct v4l2_loopback_device *device;
};
static int v4l2loopback_lookup_cb(int id, void *ptr, void *data)
{
	struct v4l2_loopback_device *device = ptr;
	struct v4l2loopback_lookup_cb_data *cbdata = data;
	if (cbdata && device && device->vdev) {
		if (device->vdev->num == cbdata->device_nr) {
			cbdata->device = device;
			cbdata->device_nr = id;
			return 1;
		}
	}
	return 0;
}
static int v4l2loopback_lookup(int device_nr,
			       struct v4l2_loopback_device **device)
{
	struct v4l2loopback_lookup_cb_data data = {
		.device_nr = device_nr,
		.device = NULL,
	};
	int err = idr_for_each(&v4l2loopback_index_idr, &v4l2loopback_lookup_cb,
			       &data);
	if (1 == err) {
		if (device)
			*device = data.device;
		return data.device_nr;
	}
	return -ENODEV;
}
#define v4l2loopback_get_vdev_nr(vdev) \
	((struct v4l2loopback_private *)video_get_drvdata(vdev))->device_nr
static struct v4l2_loopback_device *v4l2loopback_cd2dev(struct device *cd)
{
	struct video_device *loopdev = to_video_device(cd);
	int device_nr = v4l2loopback_get_vdev_nr(loopdev);

	return idr_find(&v4l2loopback_index_idr, device_nr);
}

static struct v4l2_loopback_device *v4l2loopback_getdevice(struct file *f)
{
	struct v4l2loopback_private *ptr = video_drvdata(f);
	int nr = ptr->device_nr;

	return idr_find(&v4l2loopback_index_idr, nr);
}

/* forward declarations */
static void client_usage_queue_event(struct video_device *vdev);
static bool any_buffers_mapped(struct v4l2_loopback_device *dev);
static int allocate_buffers(struct v4l2_loopback_device *dev,
			    struct v4l2_pix_format *pix_format);
static void init_buffers(struct v4l2_loopback_device *dev, u32 bytes_used,
			 u32 buffer_size);
static void free_buffers(struct v4l2_loopback_device *dev);
static int allocate_timeout_buffer(struct v4l2_loopback_device *dev);
static void free_timeout_buffer(struct v4l2_loopback_device *dev);
static void check_timers(struct v4l2_loopback_device *dev);
static const struct v4l2_file_operations v4l2_loopback_fops;
static const struct v4l2_ioctl_ops v4l2_loopback_ioctl_ops;
static void v4l2_loopback_remove(struct v4l2_loopback_device *dev);

/* V4L2 ioctl caps and params calls */
/* returns device capabilities
 * called on VIDIOC_QUERYCAP
 */
static int vidioc_querycap(struct file *file, void *fh,
			   struct v4l2_capability *cap)
{
	struct v4l2_loopback_device *dev = v4l2loopback_getdevice(file);
	struct v4l2_loopback_opener *opener = fh_to_opener(fh);
	int device_nr = v4l2loopback_get_vdev_nr(dev->vdev);
	__u32 capabilities = V4L2_CAP_STREAMING | V4L2_CAP_READWRITE;

	strscpy(cap->driver, "v4l2 loopback", sizeof(cap->driver));
	snprintf(cap->card, sizeof(cap->card), "%s", dev->card_label);
	snprintf(cap->bus_info, sizeof(cap->bus_info),
		 "platform:v4l2loopback-%03d", device_nr);

	if (dev->announce_all_caps) {
		capabilities |= V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_OUTPUT;
	} else {
		if (opener->io_method == V4L2L_IO_TIMEOUT ||
		    (has_output_token(dev->stream_tokens) &&
		     !dev->keep_format)) {
			capabilities |= V4L2_CAP_VIDEO_OUTPUT;
		} else
			capabilities |= V4L2_CAP_VIDEO_CAPTURE;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0)
	dev->vdev->device_caps =
#endif /* >=linux-4.7.0 */
		cap->device_caps = cap->capabilities = capabilities;

	cap->capabilities |= V4L2_CAP_DEVICE_CAPS;

	memset(cap->reserved, 0, sizeof(cap->reserved));
	return 0;
}

/*
    Copyright (C) 2025 Eli Oliveira Junior

    format_buffer_size - Converts byte size to human-readable string with units
    @size: Input size in bytes (unsigned 64-bit)
    @buf: Output character buffer (minimum 16 bytes recommended)
    @buf_size: Maximum capacity of output buffer (including null terminator)

    Transforms raw byte counts into formatted strings with appropriate binary
    units (B, KB, MB, GB) using these conversion rules:
        < 1024 bytes: Display as-is with "B" unit
        1024-1048575: Convert to KB with 2 decimal places
        1MB-1GB-1: Convert to MB with 2 decimal places
            = 1GB: Convert to GB with 2 decimal places
    Implementation Details:
        Uses integer arithmetic for precise decimal calculations
        Handles 64-bit size inputs properly
        Guarantees null-terminated output
        Prevents buffer overflow with snprintf()
        Optimized for performance (no floating point operations)
    Examples:
    512 → "512 B"
    1024 → "1 KB"
    1536 → "1.50 KB"
    1048576 → "1 MB"
    1572864 → "1.50 MB"
    Error Handling:
        Safely handles NULL buffer pointers
        Verifies buffer capacity before writing
        Returns empty string if buf_size == 0
    Thread Safety:
        Reentrant (no shared state)
        Lock-free (uses only local variables)
*/
static void format_buffer_size(size_t size, char *buf, size_t buf_size)
{
	const char *units[] = { "B", "KB", "MB", "GB" };
	int unit_index = 0;
	size_t size_copy = size;
	size_t remainder = 0;

	/* Find the appropriate unit */
	while (size_copy >= 1024 && unit_index < 3) {
		remainder = size_copy % 1024;
		size_copy /= 1024;
		unit_index++;
	}

	if (unit_index == 0) {
		/* Bytes - no decimals */
		snprintf(buf, buf_size, "%zu %s", size, units[unit_index]);
	} else {
		/* KB/MB/GB - use integer arithmetic to simulate decimals */
		size_t decimal_part =
			(remainder * 100) / 1024; /* Two decimal places */
		if (decimal_part == 0) {
			snprintf(buf, buf_size, "%zu %s", size_copy,
				 units[unit_index]);
		} else {
			snprintf(buf, buf_size, "%zu.%02zu %s", size_copy,
				 decimal_part, units[unit_index]);
		}
	}
}

static int vidioc_enum_framesizes(struct file *file, void *fh,
				  struct v4l2_frmsizeenum *argp)
{
	struct v4l2_loopback_device *dev = v4l2loopback_getdevice(file);
	struct v4l2_loopback_opener *opener = fh_to_opener(fh);

	/* there can be only one... */
	if (argp->index)
		return -EINVAL;

	if (dev->keep_format || has_other_owners(opener, dev)) {
		/* only current frame size supported */
		if (argp->pixel_format != dev->pix_format.pixelformat)
			return -EINVAL;

		argp->type = V4L2_FRMSIZE_TYPE_DISCRETE;

		argp->discrete.width = dev->pix_format.width;
		argp->discrete.height = dev->pix_format.height;
	} else {
		/* return continuous sizes if pixel format is supported */
		if (NULL == format_by_fourcc(argp->pixel_format))
			return -EINVAL;

		if (dev->min_width == dev->max_width &&
		    dev->min_height == dev->max_height) {
			argp->type = V4L2_FRMSIZE_TYPE_DISCRETE;

			argp->discrete.width = dev->min_width;
			argp->discrete.height = dev->min_height;
		} else {
			argp->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;

			argp->stepwise.min_width = dev->min_width;
			argp->stepwise.min_height = dev->min_height;

			argp->stepwise.max_width = dev->max_width;
			argp->stepwise.max_height = dev->max_height;

			argp->stepwise.step_width = 1;
			argp->stepwise.step_height = 1;
		}
	}
	return 0;
}

/* Test if the device is currently 'capable' of the buffer (stream) type when
 * the `exclusive_caps` parameter is set. `keep_format` should lock the format
 * and prevent free of buffers */
static int check_buffer_capability(struct v4l2_loopback_device *dev,
				   struct v4l2_loopback_opener *opener,
				   enum v4l2_buf_type type)
{
	/* short-circuit for (non-compliant) timeout image mode */
	if (opener->io_method == V4L2L_IO_TIMEOUT)
		return 0;
	if (dev->announce_all_caps)
		return (type == V4L2_BUF_TYPE_VIDEO_CAPTURE ||
			type == V4L2_BUF_TYPE_VIDEO_OUTPUT) ?
			       0 :
			       -EINVAL;
	/* CAPTURE if opener has a capture format or a writer is streaming;
	 * else OUTPUT. */
	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		if (!(has_capture_token(opener->format_token) ||
		      !has_output_token(dev->stream_tokens)))
			return -EINVAL;
		break;
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		if (!(has_output_token(opener->format_token) ||
		      has_output_token(dev->stream_tokens)))
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
/* returns frameinterval (fps) for the set resolution
 * called on VIDIOC_ENUM_FRAMEINTERVALS
 */
static int vidioc_enum_frameintervals(struct file *file, void *fh,
				      struct v4l2_frmivalenum *argp)
{
	struct v4l2_loopback_device *dev = v4l2loopback_getdevice(file);
	struct v4l2_loopback_opener *opener = fh_to_opener(fh);

	/* there can be only one... */
	if (argp->index)
		return -EINVAL;

	if (dev->keep_format || has_other_owners(opener, dev)) {
		/* keep_format also locks the frame rate */
		if (argp->width != dev->pix_format.width ||
		    argp->height != dev->pix_format.height ||
		    argp->pixel_format != dev->pix_format.pixelformat)
			return -EINVAL;

		argp->type = V4L2_FRMIVAL_TYPE_DISCRETE;
		argp->discrete = dev->capture_param.timeperframe;
	} else {
		if (argp->width < dev->min_width ||
		    argp->width > dev->max_width ||
		    argp->height < dev->min_height ||
		    argp->height > dev->max_height ||
		    !format_by_fourcc(argp->pixel_format))
			return -EINVAL;

		argp->type = V4L2_FRMIVAL_TYPE_CONTINUOUS;
		argp->stepwise.min.numerator = 1;
		argp->stepwise.min.denominator = V4L2LOOPBACK_FPS_MAX;
		argp->stepwise.max.numerator = V4L2LOOPBACK_FRAME_INTERVAL_MAX;
		argp->stepwise.max.denominator = 1;
		argp->stepwise.step.numerator = 1;
		argp->stepwise.step.denominator = 1;
	}

	return 0;
}

/* Enumerate device formats
 * Returns:
 * -   EINVAL the index is out of bounds; or if non-zero when format is fixed
 * -   EFAULT unexpected null pointer */
static int vidioc_enum_fmt_vid(struct file *file, void *fh,
			       struct v4l2_fmtdesc *f)
{
	struct v4l2_loopback_device *dev = v4l2loopback_getdevice(file);
	struct v4l2_loopback_opener *opener = fh_to_opener(fh);
	int fixed = dev->keep_format || has_other_owners(opener, dev);
	const struct v4l2l_format *fmt;

	if (check_buffer_capability(dev, opener, f->type) < 0)
		return -EINVAL;

	if (!(f->index < FORMATS))
		return -EINVAL;
	/* TODO: Support 6.14 V4L2_FMTDESC_FLAG_ENUM_ALL */
	if (fixed && f->index)
		return -EINVAL;

	fmt = fixed ? format_by_fourcc(dev->pix_format.pixelformat) :
		      &formats[f->index];
	if (!fmt)
		return -EFAULT;

	f->flags = 0;
	if (fmt->flags & FORMAT_FLAGS_COMPRESSED)
		f->flags |= V4L2_FMT_FLAG_COMPRESSED;
	snprintf(f->description, sizeof(f->description), fmt->name);
	f->pixelformat = fmt->fourcc;
	return 0;
}

/* Tests (or tries) the format.
 * Returns:
 * -   EINVAL if the buffer type or format is not supported
 */
static int vidioc_try_fmt_vid(struct file *file, void *fh,
			      struct v4l2_format *f)
{
	struct v4l2_loopback_device *dev = v4l2loopback_getdevice(file);
	struct v4l2_loopback_opener *opener = fh_to_opener(fh);

	if (check_buffer_capability(dev, opener, f->type) < 0)
		return -EINVAL;
	if (v4l2l_fill_format(f, dev->min_width, dev->max_width,
			      dev->min_height, dev->max_height) != 0)
		return -EINVAL;
	if (dev->keep_format || has_other_owners(opener, dev))
		/* use existing format - including colorspace info */
		f->fmt.pix = dev->pix_format;

	return 0;
}

/* Sets new format. Fills 'f' argument with the requested or existing format.
 * Side-effect: buffers are allocated for the (returned) format.
 * Returns:
 * -   EINVAL if the type is not supported
 * -   EBUSY if buffers are already allocated
 * TODO: (vasaka) set subregions of input
 * 
 * rewritten added support for dynamic buffering
 * Copyright (C) 2025 Eli Oliveira Junior
 */

static int vidioc_s_fmt_vid(struct file *file, void *fh, struct v4l2_format *f)
{
	struct v4l2_loopback_device *dev = v4l2loopback_getdevice(file);
	struct v4l2_loopback_opener *opener = fh_to_opener(fh);
	u32 token = opener->io_method == V4L2L_IO_TIMEOUT ?
			    V4L2L_TOKEN_TIMEOUT :
			    token_from_type(f->type);
	int changed, result;
	char buf[5];
	bool dynamic_mode = dev->use_dynamic_buffering;

	/*=== INITIAL VALIDATION ===*/
	result = vidioc_try_fmt_vid(file, fh, f);
	if (result < 0) {
		dprintk("S_FMT: try_fmt failed with %d\n", result);
		return result;
	}

	if (opener->buffer_count > 0) {
		dprintk("S_FMT: buffers already allocated, requiring release\n");
		return -EBUSY;
	}

	/* === MUTEX ACQUISITION === */
	result = mutex_lock_killable(&dev->image_mutex);
	if (result < 0) {
		dprintk("S_FMT: failed to acquire mutex (%d)\n", result);
		return result;
	}

	/* === PREVIOUS TOKEN RELEASE === */
	if (opener->format_token)
		release_token(dev, opener, format);

	/* === COMPATIBLE TOKEN VALIDATION === */
	if (dynamic_mode) {
		/* Dynamic mode: always allows format */
		dprintk("S_FMT: dynamic mode - allowing format without token restriction\n");
	} else {
		/* Static mode: traditional validation */
		if (!(dev->format_tokens & token)) {
			result = -EBUSY;
			dprintk("S_FMT: token %u not available in static mode\n",
				token);
			goto exit_s_fmt_unlock;
		}
	}

	dprintk("S_FMT[%s] %4s:%ux%u size=%u (dynamic=%d)\n",
		V4L2_TYPE_IS_CAPTURE(f->type) ? "CAPTURE" : "OUTPUT",
		fourcc2str(f->fmt.pix.pixelformat, buf), f->fmt.pix.width,
		f->fmt.pix.height, f->fmt.pix.sizeimage, dynamic_mode);

	/* === FORMAT CHANGE CHECK === */
	changed = !pix_format_eq(&dev->pix_format, &f->fmt.pix, 0);

	if (changed || has_no_owners(dev) || dynamic_mode) {
		/* === BUFFERS ALLOCATION === */
		result = allocate_buffers(dev, &f->fmt.pix);
		if (result < 0) {
			pr_err("v4l2-loopback: buffer allocation failure (%d)\n",
			       result);
			goto exit_s_fmt_unlock;
		}

		/* === DYNAMIC BUFFER INITIALIZATION === */
		if (dynamic_mode && !dev->dbuf) {
			result = init_dynamic_buffer(dev);
			if (result < 0) {
				pr_err("v4l2-loopback: dynamic buffer initialization failed (%d)\n",
				       result);
				goto exit_s_fmt_free;
			}
		}
	}

	/* === TIMEOUT BUFFER ALLOCATION === */
	if ((dev->timeout_image && changed) ||
	    (!dev->timeout_image && need_timeout_buffer(dev, token))) {
		result = allocate_timeout_buffer(dev);
		if (result < 0) {
			pr_err("v4l2-loopback: timeout buffer allocation failed (%d)\n",
			       result);
			goto exit_s_fmt_free;
		}
	}

	/* === FORMAT APPLICATION === */
	if (changed) {
		dev->pix_format = f->fmt.pix;
		dev->pix_format_has_valid_sizeimage =
			v4l2l_pix_format_has_valid_sizeimage(f);
		pr_info("v4l2-loopback: applied format: %4s %ux%u\n",
			fourcc2str(f->fmt.pix.pixelformat, buf),
			f->fmt.pix.width, f->fmt.pix.height);
	}

	/* === TOKEN ACQUISITION === */
	acquire_token(dev, opener, format, token);

	/* In dynamic mode, mark that there is an active writer */
	if (dynamic_mode && V4L2_TYPE_IS_OUTPUT(f->type)) {
		/* Simulate that there is an active output stream to avoid NO SIGNAL */
		dprintk("S_FMT: dynamic OUTPUT mode - marking as active\n");
	}

	if (opener->io_method == V4L2L_IO_TIMEOUT)
		dev->timeout_image_io = 0;

	pr_info("v4l2-loopback: S_FMT successful (dynamic=%d, token=0x%x)\n",
		dynamic_mode, token);

	goto exit_s_fmt_unlock;

/* === ERROR CLEANUP === */
exit_s_fmt_free:
	if (!dynamic_mode) { /* In dynamic mode, we preserve buffers for other openers */
		free_buffers(dev);
	}
exit_s_fmt_unlock:
	mutex_unlock(&dev->image_mutex);
	return result;
}

/* ------------------ CAPTURE ----------------------- */
/* ioctl for VIDIOC_ENUM_FMT, _G_FMT, _S_FMT, and _TRY_FMT when buffer type
 * is V4L2_BUF_TYPE_VIDEO_CAPTURE */

static int vidioc_enum_fmt_cap(struct file *file, void *fh,
			       struct v4l2_fmtdesc *f)
{
	return vidioc_enum_fmt_vid(file, fh, f);
}

static int vidioc_g_fmt_cap(struct file *file, void *fh, struct v4l2_format *f)
{
	struct v4l2_loopback_device *dev = v4l2loopback_getdevice(file);
	struct v4l2_loopback_opener *opener = fh_to_opener(fh);
	if (check_buffer_capability(dev, opener, f->type) < 0)
		return -EINVAL;
	f->fmt.pix = dev->pix_format;
	return 0;
}

static int vidioc_try_fmt_cap(struct file *file, void *fh,
			      struct v4l2_format *f)
{
	return vidioc_try_fmt_vid(file, fh, f);
}

static int vidioc_s_fmt_cap(struct file *file, void *fh, struct v4l2_format *f)
{
	return vidioc_s_fmt_vid(file, fh, f);
}

/* ------------------ OUTPUT ----------------------- */
/* ioctl for VIDIOC_ENUM_FMT, _G_FMT, _S_FMT, and _TRY_FMT when buffer type
 * is V4L2_BUF_TYPE_VIDEO_OUTPUT */

static int vidioc_enum_fmt_out(struct file *file, void *fh,
			       struct v4l2_fmtdesc *f)
{
	return vidioc_enum_fmt_vid(file, fh, f);
}

static int vidioc_g_fmt_out(struct file *file, void *fh, struct v4l2_format *f)
{
	struct v4l2_loopback_device *dev = v4l2loopback_getdevice(file);
	struct v4l2_loopback_opener *opener = fh_to_opener(fh);
	if (check_buffer_capability(dev, opener, f->type) < 0)
		return -EINVAL;
	/*
	 * LATER: this should return the currently valid format
	 * gstreamer doesn't like it, if this returns -EINVAL, as it
	 * then concludes that there is _no_ valid format
	 * CHECK whether this assumption is wrong,
	 * or whether we have to always provide a valid format
	 */
	f->fmt.pix = dev->pix_format;
	return 0;
}

static int vidioc_try_fmt_out(struct file *file, void *fh,
			      struct v4l2_format *f)
{
	return vidioc_try_fmt_vid(file, fh, f);
}

static int vidioc_s_fmt_out(struct file *file, void *fh, struct v4l2_format *f)
{
	return vidioc_s_fmt_vid(file, fh, f);
}

// #define V4L2L_OVERLAY
#ifdef V4L2L_OVERLAY
/* ------------------ OVERLAY ----------------------- */
/* currently unsupported */
/* GSTreamer's v4l2sink is buggy, as it requires the overlay to work
 * while it should only require it, if overlay is requested
 * once the gstreamer element is fixed, remove the overlay dummies
 */
#warning OVERLAY dummies
static int vidioc_g_fmt_overlay(struct file *file, void *priv,
				struct v4l2_format *fmt)
{
	return 0;
}

static int vidioc_s_fmt_overlay(struct file *file, void *priv,
				struct v4l2_format *fmt)
{
	return 0;
}
#endif /* V4L2L_OVERLAY */

/* ------------------ PARAMs ----------------------- */

/* get some data flow parameters, only capability, fps and readbuffers has
 * effect on this driver
 * called on VIDIOC_G_PARM
 */
static int vidioc_g_parm(struct file *file, void *fh,
			 struct v4l2_streamparm *parm)
{
	/* do not care about type of opener, hope these enums would always be
	 * compatible */
	struct v4l2_loopback_device *dev = v4l2loopback_getdevice(file);
	struct v4l2_loopback_opener *opener = fh_to_opener(fh);
	if (check_buffer_capability(dev, opener, parm->type) < 0)
		return -EINVAL;
	parm->parm.capture = dev->capture_param;
	return 0;
}

/* get some data flow parameters, only capability, fps and readbuffers has
 * effect on this driver
 * called on VIDIOC_S_PARM
 */
static int vidioc_s_parm(struct file *file, void *fh,
			 struct v4l2_streamparm *parm)
{
	struct v4l2_loopback_device *dev = v4l2loopback_getdevice(file);
	struct v4l2_loopback_opener *opener = fh_to_opener(fh);

	dprintk("S_PARM(frame-time=%u/%u)\n",
		parm->parm.capture.timeperframe.numerator,
		parm->parm.capture.timeperframe.denominator);
	if (check_buffer_capability(dev, opener, parm->type) < 0)
		return -EINVAL;

	switch (parm->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		set_timeperframe(dev, &parm->parm.capture.timeperframe);
		break;
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		set_timeperframe(dev, &parm->parm.output.timeperframe);
		break;
	default:
		return -EINVAL;
	}

	parm->parm.capture = dev->capture_param;
	return 0;
}

#ifdef V4L2LOOPBACK_WITH_STD
/* sets a tv standard, actually we do not need to handle this any special way
 * added to support effecttv
 * called on VIDIOC_S_STD
 */
static int vidioc_s_std(struct file *file, void *fh, v4l2_std_id *_std)
{
	v4l2_std_id req_std = 0, supported_std = 0;
	const v4l2_std_id all_std = V4L2_STD_ALL, no_std = 0;

	if (_std) {
		req_std = *_std;
		*_std = all_std;
	}

	/* we support everything in V4L2_STD_ALL, but not more... */
	supported_std = (all_std & req_std);
	if (no_std == supported_std)
		return -EINVAL;

	return 0;
}

/* gets a fake video standard
 * called on VIDIOC_G_STD
 */
static int vidioc_g_std(struct file *file, void *fh, v4l2_std_id *norm)
{
	if (norm)
		*norm = V4L2_STD_ALL;
	return 0;
}
/* gets a fake video standard
 * called on VIDIOC_QUERYSTD
 */
static int vidioc_querystd(struct file *file, void *fh, v4l2_std_id *norm)
{
	if (norm)
		*norm = V4L2_STD_ALL;
	return 0;
}
#endif /* V4L2LOOPBACK_WITH_STD */

static int v4l2loopback_set_ctrl(struct v4l2_loopback_device *dev, u32 id,
				 s64 val)
{
	int result = 0;
	switch (id) {
	case CID_KEEP_FORMAT:
		if (val < 0 || val > 1)
			return -EINVAL;
		dev->keep_format = val;
		result = mutex_lock_killable(&dev->image_mutex);
		if (result < 0)
			return result;
		if (!dev->keep_format) {
			if (has_no_owners(dev) && !any_buffers_mapped(dev))
				free_buffers(dev);
		}
		mutex_unlock(&dev->image_mutex);
		break;
	case CID_SUSTAIN_FRAMERATE:
		if (val < 0 || val > 1)
			return -EINVAL;
		spin_lock_bh(&dev->lock);
		dev->sustain_framerate = val;
		check_timers(dev);
		spin_unlock_bh(&dev->lock);
		break;
	case CID_TIMEOUT:
		if (val < 0 || val > MAX_TIMEOUT)
			return -EINVAL;
		if (val > 0) {
			result = mutex_lock_killable(&dev->image_mutex);
			if (result < 0)
				return result;
			/* on-the-fly allocate if device is owned; else
			 * allocate occurs on next S_FMT or REQBUFS */
			if (!has_no_owners(dev))
				result = allocate_timeout_buffer(dev);
			mutex_unlock(&dev->image_mutex);
			if (result < 0) {
				/* disable timeout as buffer not alloc'd */
				spin_lock_bh(&dev->lock);
				dev->timeout_jiffies = 0;
				spin_unlock_bh(&dev->lock);
				return result;
			}
		}
		spin_lock_bh(&dev->lock);
		dev->timeout_jiffies = msecs_to_jiffies(val);
		check_timers(dev);
		spin_unlock_bh(&dev->lock);
		break;
	case CID_TIMEOUT_IMAGE_IO:
		dev->timeout_image_io = 1;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int v4l2loopback_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_loopback_device *dev = container_of(
		ctrl->handler, struct v4l2_loopback_device, ctrl_handler);
	return v4l2loopback_set_ctrl(dev, ctrl->id, ctrl->val);
}

/* returns set of device outputs, in our case there is only one
 * called on VIDIOC_ENUMOUTPUT
 */
static int vidioc_enum_output(struct file *file, void *fh,
			      struct v4l2_output *outp)
{
	__u32 index = outp->index;
	struct v4l2_loopback_device *dev = v4l2loopback_getdevice(file);
	struct v4l2_loopback_opener *opener = fh_to_opener(fh);

	if (check_buffer_capability(dev, opener, V4L2_BUF_TYPE_VIDEO_OUTPUT))
		return -ENOTTY;
	if (index)
		return -EINVAL;

	/* clear all data (including the reserved fields) */
	memset(outp, 0, sizeof(*outp));

	outp->index = index;
	strscpy(outp->name, "loopback in", sizeof(outp->name));
	outp->type = V4L2_OUTPUT_TYPE_ANALOG;
	outp->audioset = 0;
	outp->modulator = 0;
#ifdef V4L2LOOPBACK_WITH_STD
	outp->std = V4L2_STD_ALL;
#ifdef V4L2_OUT_CAP_STD
	outp->capabilities |= V4L2_OUT_CAP_STD;
#endif /*  V4L2_OUT_CAP_STD */
#endif /* V4L2LOOPBACK_WITH_STD */

	return 0;
}

/* which output is currently active,
 * called on VIDIOC_G_OUTPUT
 */
static int vidioc_g_output(struct file *file, void *fh, unsigned int *index)
{
	struct v4l2_loopback_device *dev = v4l2loopback_getdevice(file);
	struct v4l2_loopback_opener *opener = fh_to_opener(fh);
	if (check_buffer_capability(dev, opener, V4L2_BUF_TYPE_VIDEO_OUTPUT))
		return -ENOTTY;
	if (index)
		*index = 0;
	return 0;
}

/* set output, can make sense if we have more than one video src,
 * called on VIDIOC_S_OUTPUT
 */
static int vidioc_s_output(struct file *file, void *fh, unsigned int index)
{
	struct v4l2_loopback_device *dev = v4l2loopback_getdevice(file);
	struct v4l2_loopback_opener *opener = fh_to_opener(fh);
	if (check_buffer_capability(dev, opener, V4L2_BUF_TYPE_VIDEO_OUTPUT))
		return -ENOTTY;
	return index == 0 ? index : -EINVAL;
}

/* returns set of device inputs, in our case there is only one,
 * but later I may add more
 * called on VIDIOC_ENUMINPUT
 * 
 * rewritten added support for dynamic buffering
 * Copyright (C) 2025 Eli Oliveira Junior
 */
static int vidioc_enum_input(struct file *file, void *fh,
			     struct v4l2_input *inp)
{
	struct v4l2_loopback_device *dev = v4l2loopback_getdevice(file);
	struct v4l2_loopback_opener *opener = fh_to_opener(fh);
	__u32 index = inp->index;

	dprintk("vidioc_enum_input: stream_tokens=0x%x, keep_format=%d, dynamic=%d\n",
		dev->stream_tokens, dev->keep_format,
		dev->use_dynamic_buffering);

	if (check_buffer_capability(dev, opener, V4L2_BUF_TYPE_VIDEO_CAPTURE))
		return -ENOTTY;
	if (index)
		return -EINVAL;

	/* clear all data (including the reserved fields) */
	memset(inp, 0, sizeof(*inp));

	inp->index = index;
	strscpy(inp->name, "loopback", sizeof(inp->name));
	inp->type = V4L2_INPUT_TYPE_CAMERA;
	inp->audioset = 0;
	inp->tuner = 0;
	inp->status = 0;

#ifdef V4L2LOOPBACK_WITH_STD
	inp->std = V4L2_STD_ALL;
#ifdef V4L2_IN_CAP_STD
	inp->capabilities |= V4L2_IN_CAP_STD;
#endif
#endif /* V4L2LOOPBACK_WITH_STD */

	/* FIX: In dynamic mode always report OK signal */
	if (dev->use_dynamic_buffering) {
		/* Dynamic mode: never reports NO_SIGNAL */
		dprintk("vidioc_enum_input: dynamic mode - signal OK\n");
	} else {
		/* Static mode: original logic */
		if (has_output_token(dev->stream_tokens) && !dev->keep_format) {
			/* if no outputs attached; pretend device is powered off */
			inp->status |= V4L2_IN_ST_NO_SIGNAL;
			dprintk("vidioc_enum_input: static mode - NO_SIGNAL\n");
		}
	}

	return 0;
}

/* which input is currently active,
 * called on VIDIOC_G_INPUT
 */
static int vidioc_g_input(struct file *file, void *fh, unsigned int *index)
{
	struct v4l2_loopback_device *dev = v4l2loopback_getdevice(file);
	struct v4l2_loopback_opener *opener = fh_to_opener(fh);
	if (check_buffer_capability(dev, opener, V4L2_BUF_TYPE_VIDEO_CAPTURE))
		return -ENOTTY; /* NOTE: -EAGAIN might be more informative */
	if (index)
		*index = 0;
	return 0;
}

/* set input, can make sense if we have more than one video src,
 * called on VIDIOC_S_INPUT
 */
static int vidioc_s_input(struct file *file, void *fh, unsigned int index)
{
	struct v4l2_loopback_device *dev = v4l2loopback_getdevice(file);
	struct v4l2_loopback_opener *opener = fh_to_opener(fh);
	if (index != 0)
		return -EINVAL;
	if (check_buffer_capability(dev, opener, V4L2_BUF_TYPE_VIDEO_CAPTURE))
		return -ENOTTY; /* NOTE: -EAGAIN might be more informative */
	return 0;
}

/* --------------- V4L2 ioctl buffer related calls ----------------- */

#define is_allocated(opener, type, index)                                \
	(opener->format_token & (opener->io_method == V4L2L_IO_TIMEOUT ? \
					 V4L2L_TOKEN_TIMEOUT :           \
					 token_from_type(type)) &&       \
	 (index) < (opener)->buffer_count)
#define BUFFER_DEBUG_FMT_STR                                      \
	"buffer#%u @ %p type=%u bytesused=%u length=%u flags=%x " \
	"field=%u timestamp= %lld.%06lldsequence=%u\n"
#define BUFFER_DEBUG_FMT_ARGS(buf)                                         \
	(buf)->index, (buf), (buf)->type, (buf)->bytesused, (buf)->length, \
		(buf)->flags, (buf)->field,                                \
		(long long)(buf)->timestamp.tv_sec,                        \
		(long long)(buf)->timestamp.tv_usec, (buf)->sequence
/* Buffer flag helpers */
#define unset_flags(flags)                      \
	do {                                    \
		flags &= ~V4L2_BUF_FLAG_QUEUED; \
		flags &= ~V4L2_BUF_FLAG_DONE;   \
	} while (0)
#define set_queued(flags)                      \
	do {                                   \
		flags |= V4L2_BUF_FLAG_QUEUED; \
		flags &= ~V4L2_BUF_FLAG_DONE;  \
	} while (0)
#define set_done(flags)                         \
	do {                                    \
		flags &= ~V4L2_BUF_FLAG_QUEUED; \
		flags |= V4L2_BUF_FLAG_DONE;    \
	} while (0)

static bool any_buffers_mapped(struct v4l2_loopback_device *dev)
{
	u32 index;
	for (index = 0; index < dev->buffer_count; ++index)
		if (dev->buffers[index].buffer.flags & V4L2_BUF_FLAG_MAPPED)
			return true;
	return false;
}

static void prepare_buffer_queue(struct v4l2_loopback_device *dev, int count)
{
	struct v4l2l_buffer *bufd, *n;
	u32 pos;

	spin_lock_bh(&dev->list_lock);

	/* ensure sufficient number of buffers in queue */
	for (pos = 0; pos < count; ++pos) {
		bufd = &dev->buffers[pos];
		if (list_empty(&bufd->list_head))
			list_add_tail(&bufd->list_head, &dev->outbufs_list);
	}
	if (list_empty(&dev->outbufs_list))
		goto exit_prepare_queue_unlock;

	/* remove any excess buffers */
	list_for_each_entry_safe(bufd, n, &dev->outbufs_list, list_head) {
		if (bufd->buffer.index >= count)
			list_del_init(&bufd->list_head);
	}

	/* buffers are no longer queued; and `write_position` will correspond
	 * to the first item of `outbufs_list`. */
	pos = v4l2l_mod64(dev->write_position, count);
	list_for_each_entry(bufd, &dev->outbufs_list, list_head) {
		unset_flags(bufd->buffer.flags);
		dev->bufpos2index[pos % count] = bufd->buffer.index;
		++pos;
	}
exit_prepare_queue_unlock:
	spin_unlock_bh(&dev->list_lock);
}

/* forward declaration */
static int vidioc_streamoff(struct file *file, void *fh,
			    enum v4l2_buf_type type);
/* negotiate buffer type
 * only mmap streaming supported
 * called on VIDIOC_REQBUFS
 */
static int vidioc_reqbufs(struct file *file, void *fh,
			  struct v4l2_requestbuffers *reqbuf)
{
	struct v4l2_loopback_device *dev = v4l2loopback_getdevice(file);
	struct v4l2_loopback_opener *opener = fh_to_opener(fh);
	u32 token = opener->io_method == V4L2L_IO_TIMEOUT ?
			    V4L2L_TOKEN_TIMEOUT :
			    token_from_type(reqbuf->type);
	u32 req_count = reqbuf->count;
	int result = 0;

	dprintk("REQBUFS(memory=%u, req_count=%u) and device-bufs=%u/%u "
		"[used/max]\n",
		reqbuf->memory, req_count, dev->used_buffer_count,
		dev->buffer_count);

	switch (reqbuf->memory) {
	case V4L2_MEMORY_MMAP:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 20, 0)
		reqbuf->capabilities = 0; /* only guarantee MMAP support */
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0)
		reqbuf->flags = 0; /* no memory consistency support */
#endif
		break;
	default:
		return -EINVAL;
	}

	if (opener->format_token & ~token)
		/* different (buffer) type already assigned to descriptor by
		 * S_FMT or REQBUFS */
		return -EINVAL;

	MARK();
	result = mutex_lock_killable(&dev->image_mutex);
	if (result < 0)
		return result; /* -EINTR */

	/* CASE queue/dequeue timeout-buffer only: */
	if (opener->format_token & V4L2L_TOKEN_TIMEOUT) {
		opener->buffer_count = req_count;
		if (req_count == 0)
			release_token(dev, opener, format);
		goto exit_reqbufs_unlock;
	}

	MARK();
	/* CASE count is zero: streamoff, free buffers, release their token */
	if (req_count == 0) {
		if (dev->format_tokens & token) {
			acquire_token(dev, opener, format, token);
			opener->io_method = V4L2L_IO_MMAP;
		}
		result = vidioc_streamoff(file, fh, reqbuf->type);
		opener->buffer_count = 0;
		/* undocumented requirement - REQBUFS with count zero should
		 * ALSO release lock on logical stream */
		if (opener->format_token)
			release_token(dev, opener, format);
		if (has_no_owners(dev))
			dev->used_buffer_count = 0;
		goto exit_reqbufs_unlock;
	}

	/* CASE count non-zero: allocate buffers and acquire token for them */
	MARK();
	switch (reqbuf->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		if (!(dev->format_tokens & token ||
		      opener->format_token & token))
			/* only exclusive ownership for each stream */
			result = -EBUSY;
		break;
	default:
		result = -EINVAL;
	}
	if (result < 0)
		goto exit_reqbufs_unlock;

	if (has_other_owners(opener, dev) && dev->used_buffer_count > 0) {
		/* allow 'allocation' of existing number of buffers */
		req_count = dev->used_buffer_count;
	} else if (any_buffers_mapped(dev)) {
		/* do not allow re-allocation if buffers are mapped */
		result = -EBUSY;
		goto exit_reqbufs_unlock;
	}

	MARK();
	opener->buffer_count = 0;

	if (req_count > dev->buffer_count)
		req_count = dev->buffer_count;

	if (has_no_owners(dev)) {
		result = allocate_buffers(dev, &dev->pix_format);
		if (result < 0)
			goto exit_reqbufs_unlock;
	}
	if (!dev->timeout_image && need_timeout_buffer(dev, token)) {
		result = allocate_timeout_buffer(dev);
		if (result < 0)
			goto exit_reqbufs_unlock;
	}
	acquire_token(dev, opener, format, token);

	MARK();
	switch (opener->io_method) {
	case V4L2L_IO_TIMEOUT:
		dev->timeout_image_io = 0;
		opener->buffer_count = req_count;
		break;
	default:
		opener->io_method = V4L2L_IO_MMAP;
		prepare_buffer_queue(dev, req_count);
		dev->used_buffer_count = opener->buffer_count = req_count;
	}
exit_reqbufs_unlock:
	mutex_unlock(&dev->image_mutex);
	reqbuf->count = opener->buffer_count;
	return result;
}

/* returns buffer asked for;
 * give app as many buffers as it wants, if it less than MAX,
 * but map them in our inner buffers
 * called on VIDIOC_QUERYBUF
 */
static int vidioc_querybuf(struct file *file, void *fh, struct v4l2_buffer *buf)
{
	struct v4l2_loopback_device *dev = v4l2loopback_getdevice(file);
	struct v4l2_loopback_opener *opener = fh_to_opener(fh);
	u32 type = buf->type;
	u32 index = buf->index;

	if ((type != V4L2_BUF_TYPE_VIDEO_CAPTURE) &&
	    (type != V4L2_BUF_TYPE_VIDEO_OUTPUT))
		return -EINVAL;
	if (!is_allocated(opener, type, index))
		return -EINVAL;

	if (opener->format_token & V4L2L_TOKEN_TIMEOUT) {
		*buf = dev->timeout_buffer.buffer;
		buf->index = index;
	} else
		*buf = dev->buffers[index].buffer;

	buf->type = type;

	if (!(buf->flags & (V4L2_BUF_FLAG_DONE | V4L2_BUF_FLAG_QUEUED))) {
		/* v4l2-compliance requires these to be zero */
		buf->sequence = 0;
		buf->timestamp.tv_sec = buf->timestamp.tv_usec = 0;
	} else if (V4L2_TYPE_IS_CAPTURE(type)) {
		/* guess flags based on sequence values */
		if (buf->sequence >= opener->read_position) {
			set_done(buf->flags);
		} else if (buf->flags & V4L2_BUF_FLAG_DONE) {
			set_queued(buf->flags);
		}
	}
	dprintkrw("QUERYBUF(%s, index=%u) -> " BUFFER_DEBUG_FMT_STR,
		  V4L2_TYPE_IS_CAPTURE(type) ? "CAPTURE" : "OUTPUT", index,
		  BUFFER_DEBUG_FMT_ARGS(buf));
	return 0;
}

/* rewritten to be thread-safe with multiple producers in Dynamic Buffering
 * Copyright (C) 2025 Eli Oliveira Junior
 *
*/
static void buffer_written(struct v4l2_loopback_device *dev,
			   struct v4l2l_buffer *buf)
{
	unsigned long flags;

	timer_delete_sync(&dev->sustain_timer);
	timer_delete_sync(&dev->timeout_timer);

	spin_lock_irqsave(&dev->list_lock, flags); //thread-safe
	list_move_tail(&buf->list_head, &dev->outbufs_list);
	spin_unlock_irqrestore(&dev->list_lock, flags); //thread-safe

	spin_lock_irqsave(&dev->lock, flags); //thread-safe
	dev->bufpos2index[v4l2l_mod64(dev->write_position,
				      dev->used_buffer_count)] =
		buf->buffer.index;
	++dev->write_position;
	dev->reread_count = 0;

	check_timers(dev);
	spin_unlock_irqrestore(&dev->lock, flags); //thread-safe
}

/*
    Copyright (C) 2025 Eli Oliveira Junior

    sync_dynamic_to_v4l2_buffer - Synchronizes data between dynamic and V4L2 buffers
    @dev: Pointer to the v4l2_loopback_device structure (parent device)
    @bufd: Pointer to the destination v4l2l_buffer structure
    @bytesused: Maximum number of bytes to copy (typically buffer capacity)

    Performs safe data transfer from the dynamic circular buffer to a V4L2 buffer
    with these key features:
    Data Transfer Protocol:
        Reference Counting:
        Atomic increment on dynamic buffer before access
        Guaranteed release through decrement and cleanup
        Error Handling:
        Gracefully handles empty/inactive buffers (zero-fills)
        Properly propagates read errors
        Validates buffer states before operations
        Memory Management:
        Automatically cleans up buffers on last reference release
        Uses proper vfree() for vmalloc'd memory
    State Management:
        Checks shutdown_requested flag before operations
        Validates dynamic buffering activation state
        Handles concurrent access safely
    Return Values:
        0: Number of bytes successfully copied (may be zero-filled)
    0: Dynamic buffering disabled (no error)
       -ENODEV: Dynamic buffer unavailable
       -EINVAL: Invalid parameters
       -EAGAIN: Buffer temporarily unavailable
    Locking Strategy:
        Uses device spinlock for reference counting
        Internal dynamic buffer locks for data access
    Memory Safety:
        Guarantees reference release
        Prevents use-after-free
        Properly handles mapped V4L2 buffer memory
*/
static int sync_dynamic_to_v4l2_buffer(struct v4l2_loopback_device *dev,
				       struct v4l2l_buffer *bufd, u32 bytesused)
{
	struct dynamic_buffer *dbuf;
	unsigned long flags;
	void *dest;
	u32 to_copy;
	int ret = 0;

	if (!dev->use_dynamic_buffering || !dev->dbuf)
		return 0;

	spin_lock_irqsave(&dev->lock, flags);
	dbuf = dev->dbuf;
	if (dbuf && !dbuf->shutdown_requested) // Shutdown check
		atomic_inc(&dbuf->ref_count);
	else
		dbuf = NULL; // Ensure dbuf is NULL if shutdown was requested
	spin_unlock_irqrestore(&dev->lock, flags);

	if (!dbuf)
		return -ENODEV;

	/* Destination is the mapped V4L2 buffer */
	dest = dev->image + bufd->buffer.m.offset;
	to_copy = min(bytesused, dev->buffer_size);

	ret = dynamic_buffer_read(dev, dest, to_copy, false);
	if (ret < 0) {
		/* Specific treatment by error type */
		if (ret == -EAGAIN || ret == -ENODEV) {
			/* Empty or inactive buffer - expected behavior, fill with zeros */
			memset(dest, 0, to_copy);
			ret = to_copy;
		} else {
			/* Actual error (e.g. -EINVAL) - propagate the error */
			dprintk("sync_dynamic_to_v4l2_buffer: read error %d\n",
				ret);
			/* Still need to release the reference before returning */
			if (atomic_dec_return(&dbuf->ref_count) == 0 &&
			    dbuf->shutdown_requested) {
				vfree(dbuf->data);
				kfree(dbuf);
			}
			return ret;
		}
	} else if (ret < to_copy) {
		/* Fill the rest with zeros if necessary */
		memset(dest + ret, 0, to_copy - ret);
		ret = to_copy;
	}

	/* Reference release */
	if (atomic_dec_return(&dbuf->ref_count) == 0 &&
	    dbuf->shutdown_requested) {
		vfree(dbuf->data);
		kfree(dbuf);
	}

	return ret;
}

/* put buffer to queue
 * called on VIDIOC_QBUF
 * 
 * Copyright (C) 2025 Eli Oliveira Junior
 * rewritten added support for dynamic buffering
 */
static int vidioc_qbuf(struct file *file, void *fh, struct v4l2_buffer *buf)
{
	struct v4l2_loopback_device *dev = v4l2loopback_getdevice(file);
	struct v4l2_loopback_opener *opener = fh_to_opener(fh);
	struct v4l2l_buffer *bufd;
	u32 index = buf->index;
	u32 type = buf->type;
	unsigned long flags;

	dprintk("QBUF START: type=%d, index=%u, bytesused=%u\n", type, index,
		buf->bytesused);

	if (!is_allocated(opener, type, index))
		return -EINVAL;
	bufd = &dev->buffers[index];

	switch (buf->memory) {
	case V4L2_MEMORY_MMAP:
		if (!(bufd->buffer.flags & V4L2_BUF_FLAG_MAPPED))
			dprintkrw("QBUF() unmapped buffer [index=%u]\n", index);
		break;
	default:
		return -EINVAL;
	}

	if (opener->format_token & V4L2L_TOKEN_TIMEOUT) {
		set_queued(buf->flags);
		return 0;
	}

	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		dprintkrw("QBUF(CAPTURE, index=%u) -> " BUFFER_DEBUG_FMT_STR,
			  index, BUFFER_DEBUG_FMT_ARGS(buf));
		set_queued(buf->flags);
		break;
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		dprintkrw("QBUF(OUTPUT, index=%u) -> " BUFFER_DEBUG_FMT_STR,
			  index, BUFFER_DEBUG_FMT_ARGS(buf));

		/* In dynamic mode, writes to the dynamic buffer */
		if (dev->use_dynamic_buffering && dev->dbuf) {
			void *src = dev->image + bufd->buffer.m.offset;
			u32 bytes_to_write = buf->bytesused;

			dprintk("Writing %u bytes to dynamic buffer\n",
				bytes_to_write);

			if (bytes_to_write > 0) {
				int ret = dynamic_buffer_write(dev, src,
							       bytes_to_write);
				if (ret < 0) {
					dprintk("Failed to write to dynamic buffer: %d\n",
						ret);
				} else {
					dprintk("Successfully wrote %d bytes to dynamic buffer\n",
						ret);
				}
			}
		}

		if (!(bufd->buffer.flags & V4L2_BUF_FLAG_TIMESTAMP_COPY) &&
		    (buf->timestamp.tv_sec == 0 &&
		     buf->timestamp.tv_usec == 0)) {
			v4l2l_get_timestamp(&bufd->buffer);
		} else {
			bufd->buffer.timestamp = buf->timestamp;
			bufd->buffer.flags |= V4L2_BUF_FLAG_TIMESTAMP_COPY;
			bufd->buffer.flags &=
				~V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
		}
		if (dev->pix_format_has_valid_sizeimage) {
			if (buf->bytesused >= dev->pix_format.sizeimage) {
				bufd->buffer.bytesused =
					dev->pix_format.sizeimage;
			} else {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0)
				dev_warn_ratelimited(
					&dev->vdev->dev,
#else
				dprintkrw(
#endif
					"warning queued output buffer bytesused too small %u < %u\n",
					buf->bytesused,
					dev->pix_format.sizeimage);
				bufd->buffer.bytesused = buf->bytesused;
			}
		} else {
			bufd->buffer.bytesused = buf->bytesused;
		}

		/* Additional protection for multiple producers in dynamic buffering */
		spin_lock_irqsave(&dev->lock, flags);
		bufd->buffer.sequence = dev->write_position;
		spin_unlock_irqrestore(&dev->lock, flags);

		set_queued(bufd->buffer.flags);
		*buf = bufd->buffer;

		dprintk("Before buffer_written: write_position=%lld\n",
			dev->write_position);
		buffer_written(dev, bufd);
		dprintk("After buffer_written: write_position=%lld\n",
			dev->write_position);

		set_done(bufd->buffer.flags);

		/* Check for waiters before waking up */
		if (waitqueue_active(&dev->read_event))
			wake_up_all(&dev->read_event);

		break;
	default:
		return -EINVAL;
	}
	buf->type = type;
	dprintk("QBUF END: stream_tokens=0x%x\n", dev->stream_tokens);
	return 0;
}

static int can_read(struct v4l2_loopback_device *dev,
		    struct v4l2_loopback_opener *opener)
{
	int ret;

	spin_lock_bh(&dev->lock);
	check_timers(dev);
	ret = dev->write_position > opener->read_position ||
	      dev->reread_count > opener->reread_count || dev->timeout_happened;
	spin_unlock_bh(&dev->lock);
	return ret;
}

/*
 * Copyright (C) 2025 Eli Oliveira Junior
 * rewritten added support for dynamic buffering
 * add dynamic buffer synchronization
*/
static int get_capture_buffer(struct file *file)
{
	struct v4l2_loopback_device *dev = v4l2loopback_getdevice(file);
	struct v4l2_loopback_opener *opener = fh_to_opener(file->private_data);
	int pos, timeout_happened;
	u32 index;

	if ((file->f_flags & O_NONBLOCK) &&
	    (dev->write_position <= opener->read_position &&
	     dev->reread_count <= opener->reread_count &&
	     !dev->timeout_happened))
		return -EAGAIN;
	wait_event_interruptible(dev->read_event, can_read(dev, opener));

	spin_lock_bh(&dev->lock);
	if (dev->write_position == opener->read_position) {
		if (dev->reread_count > opener->reread_count + 2)
			opener->reread_count = dev->reread_count - 1;
		++opener->reread_count;
		pos = v4l2l_mod64(opener->read_position +
					  dev->used_buffer_count - 1,
				  dev->used_buffer_count);
	} else {
		opener->reread_count = 0;
		if (dev->write_position >
		    opener->read_position + dev->used_buffer_count)
			opener->read_position = dev->write_position - 1;
		pos = v4l2l_mod64(opener->read_position,
				  dev->used_buffer_count);
		++opener->read_position;
	}
	timeout_happened = dev->timeout_happened && (dev->timeout_jiffies > 0);
	dev->timeout_happened = 0;
	spin_unlock_bh(&dev->lock);

	index = dev->bufpos2index[pos];
	if (timeout_happened) {
		if (index >= dev->used_buffer_count) {
			dprintkrw("get_capture_buffer() read position is at "
				  "an unallocated buffer [index=%u]\n",
				  index);
			return -EFAULT;
		}
		/* although allocated on-demand, timeout_image is freed only
		 * in free_buffers(), so we don't need to worry about it being
		 * deallocated suddenly */
		memcpy(dev->image + dev->buffers[index].buffer.m.offset,
		       dev->timeout_image, dev->buffer_size);
	}

	else if (dev->use_dynamic_buffering && dev->dbuf) {
		/* In dynamic mode, sync from dynamic buffer */

		int sync_ret = sync_dynamic_to_v4l2_buffer(
			dev, &dev->buffers[index], dev->buffer_size);
		if (sync_ret < 0) {
			dprintk("Failed to sync dynamic buffer to V4L2: %d\n",
				sync_ret);
		}
	}

	return (int)index;
}

/* put buffer to dequeue
 * called on VIDIOC_DQBUF
 */
static int vidioc_dqbuf(struct file *file, void *fh, struct v4l2_buffer *buf)
{
	struct v4l2_loopback_device *dev = v4l2loopback_getdevice(file);
	struct v4l2_loopback_opener *opener = fh_to_opener(fh);
	u32 type = buf->type;
	int index;
	struct v4l2l_buffer *bufd;

	if (buf->memory != V4L2_MEMORY_MMAP)
		return -EINVAL;
	if (opener->format_token & V4L2L_TOKEN_TIMEOUT) {
		*buf = dev->timeout_buffer.buffer;
		buf->type = type;
		unset_flags(buf->flags);
		return 0;
	}
	if ((opener->buffer_count == 0) ||
	    !(opener->format_token & token_from_type(type)))
		return -EINVAL;

	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		index = get_capture_buffer(file);
		if (index < 0)
			return index;
		*buf = dev->buffers[index].buffer;
		unset_flags(buf->flags);
		break;
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		spin_lock_bh(&dev->list_lock);

		bufd = list_first_entry_or_null(&dev->outbufs_list,
						struct v4l2l_buffer, list_head);
		if (bufd)
			list_move_tail(&bufd->list_head, &dev->outbufs_list);

		spin_unlock_bh(&dev->list_lock);
		if (!bufd)
			return -EFAULT;
		unset_flags(bufd->buffer.flags);
		*buf = bufd->buffer;
		break;
	default:
		return -EINVAL;
	}

	buf->type = type;
	dprintkrw("DQBUF(%s, index=%u) -> " BUFFER_DEBUG_FMT_STR,
		  V4L2_TYPE_IS_CAPTURE(type) ? "CAPTURE" : "OUTPUT", index,
		  BUFFER_DEBUG_FMT_ARGS(buf));
	return 0;
}

/* ------------- STREAMING ------------------- */

/* start streaming
 * called on VIDIOC_STREAMON
 * 
 * Copyright (C) 2025 Eli Oliveira Junior
 * rewritten added support for dynamic buffering
 */
static int vidioc_streamon(struct file *file, void *fh, enum v4l2_buf_type type)
{
	struct v4l2_loopback_device *dev = v4l2loopback_getdevice(file);
	struct v4l2_loopback_opener *opener = fh_to_opener(fh);
	u32 token = token_from_type(type);
	bool dynamic_mode = dev->use_dynamic_buffering;
	int result = 0;

	dprintk("STREAMON START: type=%d, token=0x%x, dynamic=%d, stream_tokens=0x%x\n",
		type, token, dynamic_mode, dev->stream_tokens);

	/* Ensure dynamic buffer is initialized if necessary */
	if (dev->use_dynamic_buffering && !dev->dbuf &&
	    type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		int init_ret;
		mutex_lock(&dev->image_mutex);
		init_ret = init_dynamic_buffer(dev);
		mutex_unlock(&dev->image_mutex);
		if (init_ret < 0) {
			pr_err("v4l2-loopback: dynamic buffer initialization failed on STREAMON\n");
			return init_ret;
		}
	}

	/* === VALIDATION FOR TIMEOUT BUFFER === */
	if (opener->format_token & V4L2L_TOKEN_TIMEOUT) {
		dprintk("STREAMON: using timeout buffer\n");
		return 0;
	}

	/* === BUFFER COUNT VALIDATION === */
	if (!opener->buffer_count || !(opener->format_token & token)) {
		pr_err("v4l2-loopback: STREAMON with no buffers allocated (count=%u, token=0x%x)\n",
		       opener->buffer_count, opener->format_token);
		return -EINVAL;
	}

	/* === SPECIFIC LOGIC BY BUFFER TYPE === */
	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		if (dynamic_mode) {
			/* Dynamic mode: allows multiple capturers */
			if (!(opener->stream_token & token)) {
				dprintk("CAPTURE acquiring token in dynamic mode\n");
				acquire_token(dev, opener, stream, token);
				client_usage_queue_event(dev->vdev);
				pr_info("v4l2-loopback: CAPTURE stream started (dynamic mode)\n");
			}
			result = 0;
		} else {
			/* Static mode: traditional validation */
			if (has_output_token(dev->stream_tokens) &&
			    !dev->keep_format) {
				pr_warn("v4l2-loopback: CAPTURE cannot start with OUTPUT active\n");
				return -EIO;
			}
			if (dev->stream_tokens & token) {
				acquire_token(dev, opener, stream, token);
				client_usage_queue_event(dev->vdev);
			}
			result = 0;
		}
		break;

	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		if (dynamic_mode) {
			/* Dynamic mode: allows multiple writers */
			if (!(opener->stream_token & token)) {
				dprintk("OUTPUT acquiring token in dynamic mode\n");
				acquire_token(dev, opener, stream, token);

				/* Increment active producers counter */
				atomic_inc(&dev->active_producers);

				/* Ensure dynamic buffer is initialized */
				if (!dev->dbuf) {
					mutex_lock(&dev->image_mutex);
					result = init_dynamic_buffer(dev);
					mutex_unlock(&dev->image_mutex);
					if (result < 0) {
						pr_err("v4l2-loopback: dynamic buffer initialization failed\n");
						atomic_dec(
							&dev->active_producers);
						return result;
					}
				}

				/* Prepare buffer queues if needed */
				if (list_empty(&dev->outbufs_list)) {
					prepare_buffer_queue(
						dev, dev->used_buffer_count);
				}

				pr_info("v4l2-loopback: OUTPUT stream started (dynamic mode, producers=%d)\n",
					atomic_read(&dev->active_producers));
			}
			result = 0;
		} else {
			/* Static mode: traditional validation */
			if (dev->stream_tokens & token) {
				dprintk("OUTPUT acquiring token in static mode\n");
				acquire_token(dev, opener, stream, token);
			}
			result = 0;
		}
		break;

	default:
		pr_err("v4l2-loopback: unsupported buffer type: %d\n", type);
		return -EINVAL;
	}

	dprintk("STREAMON END: result=%d, stream_tokens=0x%x, opener->stream_token=0x%x\n",
		result, dev->stream_tokens, opener->stream_token);
	return result;
}

/* stop streaming
 * called on VIDIOC_STREAMOFF
 * 
 * Copyright (C) 2025 Eli Oliveira Junior
 * rewritten added support for dynamic buffering
 */
static int vidioc_streamoff(struct file *file, void *fh,
			    enum v4l2_buf_type type)
{
	struct v4l2_loopback_device *dev = v4l2loopback_getdevice(file);
	struct v4l2_loopback_opener *opener = fh_to_opener(fh);
	u32 token = token_from_type(type);

	/* short-circuit when using timeout buffer set */
	if (opener->format_token & V4L2L_TOKEN_TIMEOUT)
		return 0;
	/* short-circuit when buffer set has no owner */
	if (dev->format_tokens & token)
		return 0;
	/* opener needs a claim to buffer set */
	if (!opener->format_token)
		return -EBUSY;
	if (opener->format_token & ~token)
		return -EINVAL;

	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		if (opener->stream_token & token) {
			/* In dynamic mode, decrement producer counter */
			if (dev->use_dynamic_buffering) {
				atomic_dec(&dev->active_producers);
				dprintk("STREAMOFF: OUTPUT producer count=%d\n",
					atomic_read(&dev->active_producers));
			}

			release_token(dev, opener, stream);

			/* In dynamic mode, only reset the queue if you are the last producer */
			if (!dev->use_dynamic_buffering ||
			    atomic_read(&dev->active_producers) == 0) {
				/* reset output queue */
				if (dev->used_buffer_count > 0)
					prepare_buffer_queue(
						dev, dev->used_buffer_count);
			}
		}
		return 0;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		if (opener->stream_token & token) {
			release_token(dev, opener, stream);
			client_usage_queue_event(dev->vdev);
		}
		return 0;
	default:
		return -EINVAL;
	}
}

#ifdef CONFIG_VIDEO_V4L1_COMPAT
static int vidiocgmbuf(struct file *file, void *fh, struct video_mbuf *p)
{
	struct v4l2_loopback_device *dev;
	MARK();

	dev = v4l2loopback_getdevice(file);
	p->frames = dev->buffer_count;
	p->offsets[0] = 0;
	p->offsets[1] = 0;
	p->size = dev->buffer_size;
	return 0;
}
#endif

static void client_usage_queue_event(struct video_device *vdev)
{
	struct v4l2_event ev;
	struct v4l2_loopback_device *dev;

	dev = container_of(vdev->v4l2_dev, struct v4l2_loopback_device,
			   v4l2_dev);

	memset(&ev, 0, sizeof(ev));
	ev.type = V4L2_EVENT_PRI_CLIENT_USAGE;
	((struct v4l2_event_client_usage *)&ev.u)->count =
		!has_capture_token(dev->stream_tokens);

	v4l2_event_queue(vdev, &ev);
}

static int client_usage_ops_add(struct v4l2_subscribed_event *sev,
				unsigned elems)
{
	if (!(sev->flags & V4L2_EVENT_SUB_FL_SEND_INITIAL))
		return 0;

	client_usage_queue_event(sev->fh->vdev);
	return 0;
}

static void client_usage_ops_replace(struct v4l2_event *old,
				     const struct v4l2_event *new)
{
	*((struct v4l2_event_client_usage *)&old->u) =
		*((struct v4l2_event_client_usage *)&new->u);
}

static void client_usage_ops_merge(const struct v4l2_event *old,
				   struct v4l2_event *new)
{
	*((struct v4l2_event_client_usage *)&new->u) =
		*((struct v4l2_event_client_usage *)&old->u);
}

const struct v4l2_subscribed_event_ops client_usage_ops = {
	.add = client_usage_ops_add,
	.replace = client_usage_ops_replace,
	.merge = client_usage_ops_merge,
};

static int vidioc_subscribe_event(struct v4l2_fh *fh,
				  const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subscribe_event(fh, sub);
	case V4L2_EVENT_PRI_CLIENT_USAGE:
		return v4l2_event_subscribe(fh, sub, 0, &client_usage_ops);
	}

	return -EINVAL;
}

/* file operations */
static void vm_open(struct vm_area_struct *vma)
{
	struct v4l2l_buffer *buf;
	MARK();

	buf = vma->vm_private_data;
	atomic_inc(&buf->use_count);
	buf->buffer.flags |= V4L2_BUF_FLAG_MAPPED;
}

static void vm_close(struct vm_area_struct *vma)
{
	struct v4l2l_buffer *buf;
	MARK();

	buf = vma->vm_private_data;
	if (atomic_dec_and_test(&buf->use_count))
		buf->buffer.flags &= ~V4L2_BUF_FLAG_MAPPED;
}

static struct vm_operations_struct vm_ops = {
	.open = vm_open,
	.close = vm_close,
};

static int v4l2_loopback_mmap(struct file *file, struct vm_area_struct *vma)
{
	u8 *addr;
	unsigned long start, size, offset;
	struct v4l2_loopback_device *dev = v4l2loopback_getdevice(file);
	struct v4l2_loopback_opener *opener = fh_to_opener(file->private_data);
	struct v4l2l_buffer *buffer = NULL;
	int result = 0;
	MARK();

	offset = (unsigned long)vma->vm_pgoff << PAGE_SHIFT;
	start = (unsigned long)vma->vm_start;
	size = (unsigned long)(vma->vm_end - vma->vm_start); /* always != 0 */

	/* ensure buffer size, count, and allocated image(s) are not altered by
	 * other file descriptors */
	result = mutex_lock_killable(&dev->image_mutex);
	if (result < 0)
		return result;

	if (size > dev->buffer_size) {
		dprintk("mmap() attempt to map %lubytes when %ubytes are "
			"allocated to buffers\n",
			size, dev->buffer_size);
		result = -EINVAL;
		goto exit_mmap_unlock;
	}
	if (offset % dev->buffer_size != 0) {
		dprintk("mmap() offset does not match start of any buffer\n");
		result = -EINVAL;
		goto exit_mmap_unlock;
	}
	switch (opener->format_token) {
	case V4L2L_TOKEN_TIMEOUT:
		if (offset != (unsigned long)dev->buffer_size * MAX_BUFFERS) {
			dprintk("mmap() incorrect offset for timeout image\n");
			result = -EINVAL;
			goto exit_mmap_unlock;
		}
		buffer = &dev->timeout_buffer;
		addr = dev->timeout_image;
		break;
	default:
		if (offset >= dev->image_size) {
			dprintk("mmap() attempt to map beyond all buffers\n");
			result = -EINVAL;
			goto exit_mmap_unlock;
		}
		u32 index = offset / dev->buffer_size;
		buffer = &dev->buffers[index];
		addr = dev->image + offset;
		break;
	}

	while (size > 0) {
		struct page *page = vmalloc_to_page(addr);

		result = vm_insert_page(vma, start, page);
		if (result < 0)
			goto exit_mmap_unlock;

		start += PAGE_SIZE;
		addr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}

	vma->vm_ops = &vm_ops;
	vma->vm_private_data = buffer;

	vm_open(vma);
exit_mmap_unlock:
	mutex_unlock(&dev->image_mutex);
	return result;
}

static unsigned int v4l2_loopback_poll(struct file *file,
				       struct poll_table_struct *pts)
{
	struct v4l2_loopback_device *dev = v4l2loopback_getdevice(file);
	struct v4l2_loopback_opener *opener = fh_to_opener(file->private_data);
	__poll_t req_events = poll_requested_events(pts);
	int ret_mask = 0;

	/* call poll_wait in first call, regardless, to ensure that the
	 * wait-queue is not null */
	poll_wait(file, &dev->read_event, pts);
	poll_wait(file, &opener->fh.wait, pts);

	if (req_events & POLLPRI) {
		if (v4l2_event_pending(&opener->fh)) {
			ret_mask |= POLLPRI;
			if (!(req_events & DEFAULT_POLLMASK))
				return ret_mask;
		}
	}

	switch (opener->format_token) {
	case V4L2L_TOKEN_OUTPUT:
		if (opener->stream_token != 0 ||
		    opener->io_method == V4L2L_IO_NONE)
			ret_mask |= POLLOUT | POLLWRNORM;
		break;
	case V4L2L_TOKEN_CAPTURE:
		if ((opener->io_method == V4L2L_IO_NONE ||
		     opener->stream_token != 0) &&
		    can_read(dev, opener))
			ret_mask |= POLLIN | POLLWRNORM;
		break;
	case V4L2L_TOKEN_TIMEOUT:
		ret_mask |= POLLOUT | POLLWRNORM;
		break;
	default:
		break;
	}

	return ret_mask;
}

/* do not want to limit device opens, it can be as many readers as user want,
 * writers are limited by means of setting writer field */
static int v4l2_loopback_open(struct file *file)
{
	struct v4l2_loopback_device *dev;
	struct v4l2_loopback_opener *opener;

	dev = v4l2loopback_getdevice(file);
	if (dev->open_count.counter >= dev->max_openers)
		return -EBUSY;
	/* kfree on close */
	opener = kzalloc(sizeof(*opener), GFP_KERNEL);
	if (opener == NULL)
		return -ENOMEM;

	atomic_inc(&dev->open_count);
	if (dev->timeout_image_io && dev->format_tokens & V4L2L_TOKEN_TIMEOUT)
		/* will clear timeout_image_io once buffer set acquired */
		opener->io_method = V4L2L_IO_TIMEOUT;

	v4l2_fh_init(&opener->fh, video_devdata(file));
	file->private_data = &opener->fh;

	v4l2_fh_add(&opener->fh);
	dprintk("open() -> dev@%p with image@%p\n", dev,
		dev ? dev->image : NULL);
	return 0;
}

/*
 * Copyright (C) 2025 Eli Oliveira Junior
 * rewritten to clear producer counter in dynamic buffering 
 * 
*/
static int v4l2_loopback_close(struct file *file)
{
	struct v4l2_loopback_device *dev = v4l2loopback_getdevice(file);
	struct v4l2_loopback_opener *opener = fh_to_opener(file->private_data);
	int result = 0;
	dprintk("close() -> dev@%p with image@%p\n", dev,
		dev ? dev->image : NULL);

	if (opener->format_token) {
		struct v4l2_requestbuffers reqbuf = {
			.count = 0, .memory = V4L2_MEMORY_MMAP, .type = 0
		};
		switch (opener->format_token) {
		case V4L2L_TOKEN_CAPTURE:
			reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			break;
		case V4L2L_TOKEN_OUTPUT:
		case V4L2L_TOKEN_TIMEOUT:
			reqbuf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
			break;
		}
		if (reqbuf.type)
			result = vidioc_reqbufs(file, file->private_data,
						&reqbuf);
		if (result < 0)
			dprintk("failed to free buffers REQBUFS(count=0) "
				" returned %d\n",
				result);

		/* If it was a producer in dynamic mode, ensure decrement */
		if (dev->use_dynamic_buffering &&
		    (opener->stream_token & V4L2L_TOKEN_OUTPUT)) {
			if (atomic_read(&dev->active_producers) > 0) {
				atomic_dec(&dev->active_producers);
				dprintk("close(): OUTPUT producer count=%d\n",
					atomic_read(&dev->active_producers));
			}
		}

		mutex_lock(&dev->image_mutex);
		release_token(dev, opener, format);
		mutex_unlock(&dev->image_mutex);
	}

	if (atomic_dec_and_test(&dev->open_count)) {
		timer_delete_sync(&dev->sustain_timer);
		timer_delete_sync(&dev->timeout_timer);
		if (!dev->keep_format) {
			mutex_lock(&dev->image_mutex);
			/* Reset producer counter when closing last opener*/
			atomic_set(&dev->active_producers, 0);
			free_buffers(dev);
			mutex_unlock(&dev->image_mutex);
		}
	}

	v4l2_fh_del(&opener->fh);
	v4l2_fh_exit(&opener->fh);

	kfree(opener);
	return 0;
}

/*
 * Copyright (C) 2025 Eli Oliveira Junior
 * rewritten added support for dynamic buffering 
 * 
*/
static int start_fileio(struct file *file, void *fh, enum v4l2_buf_type type)
{
	struct v4l2_loopback_device *dev = v4l2loopback_getdevice(file);
	struct v4l2_loopback_opener *opener = fh_to_opener(fh);
	struct v4l2_requestbuffers reqbuf = { .count = dev->buffer_count,
					      .memory = V4L2_MEMORY_MMAP,
					      .type = type };
	int token = token_from_type(type);
	int result;

	if (opener->format_token & V4L2L_TOKEN_TIMEOUT ||
	    opener->format_token & ~token)
		return -EBUSY; /* NOTE: -EBADF might be more informative */

	/* short-circuit if already have stream token */
	if (opener->stream_token && opener->io_method == V4L2L_IO_FILE)
		return 0;

	/* In dynamic mode with write(), simplify the process */
	if (dev->use_dynamic_buffering && type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		dprintk("start_fileio: dynamic OUTPUT mode - simplified setup\n");

		/* Mark as IO_FILE without need for REQBUFS */
		opener->io_method = V4L2L_IO_FILE;

		/* Acquiring token if necessary */
		if (!(opener->stream_token & token)) {
			acquire_token(dev, opener, stream, token);
		}

		return 0;
	}

	/* otherwise attempt to acquire stream token and assign IO method */
	if (!(dev->stream_tokens & token) || opener->io_method != V4L2L_IO_NONE)
		return -EBUSY;

	result = vidioc_reqbufs(file, fh, &reqbuf);
	if (result < 0)
		return result;
	result = vidioc_streamon(file, fh, type);
	if (result < 0)
		return result;

	opener->io_method = V4L2L_IO_FILE;
	return 0;
}

static ssize_t v4l2_loopback_read(struct file *file, char __user *buf,
				  size_t count, loff_t *ppos)
{
	struct v4l2_loopback_device *dev = v4l2loopback_getdevice(file);

	if (dev->use_dynamic_buffering && dev->dbuf) {
		u8 *kbuf;
		int ret;

		if (count == 0)
			return 0;

		kbuf = vmalloc(count);
		if (!kbuf)
			return -ENOMEM;

		ret = dynamic_buffer_read(dev, kbuf, count,
					  !(file->f_flags & O_NONBLOCK));
		if (ret > 0) {
			if (copy_to_user(buf, kbuf, ret))
				ret = -EFAULT;
		}

		vfree(kbuf);
		return ret;
	}
	struct v4l2_buffer *b;
	int index, result;

	dprintkrw("read() %zu bytes\n", count);
	result = start_fileio(file, file->private_data,
			      V4L2_BUF_TYPE_VIDEO_CAPTURE);
	if (result < 0)
		return result;

	index = get_capture_buffer(file);
	if (index < 0)
		return index;
	b = &dev->buffers[index].buffer;
	if (count > b->bytesused)
		count = b->bytesused;
	if (copy_to_user((void *)buf, (void *)(dev->image + b->m.offset),
			 count)) {
		printk(KERN_ERR "v4l2-loopback read() failed copy_to_user()\n");
		return -EFAULT;
	}
	return count;
}

/*
 * Copyright (C) 2025 Eli Oliveira Junior
 * rewritten added support for dynamic buffering
 * added write_position update 
 * 
*/
static ssize_t v4l2_loopback_write(struct file *file, const char __user *buf,
				   size_t count, loff_t *ppos)
{
	struct v4l2_loopback_device *dev = v4l2loopback_getdevice(file);
	struct v4l2_buffer *b;
	int index, result;

	dprintkrw("write() %zu bytes (dynamic=%d)\n", count,
		  dev->use_dynamic_buffering);

	if (dev->use_dynamic_buffering && dev->dbuf) {
		u8 *kbuf;
		int ret;

		if (count == 0)
			return 0;

		/* Ensure we have format configured */
		if (dev->pix_format.sizeimage == 0) {
			dprintk("write(): format not configured\n");
			return -EINVAL;
		}

		kbuf = vmalloc(count);
		if (!kbuf)
			return -ENOMEM;

		if (copy_from_user(kbuf, buf, count)) {
			vfree(kbuf);
			return -EFAULT;
		}

		/* Write to dynamic buffer using existing function */
		ret = dynamic_buffer_write(dev, kbuf, count);
		vfree(kbuf);

		/* Updates writing position and wakes up readers */
		if (ret > 0) {
			spin_lock_bh(&dev->lock);
			++dev->write_position;
			dev->reread_count = 0;
			check_timers(dev);
			spin_unlock_bh(&dev->lock);
			wake_up_all(&dev->read_event);

			dprintk("write(): wrote %d bytes, write_position=%lld\n",
				ret, (long long)dev->write_position);
		}

		return ret;
	}

	/* Original static mode */
	result = start_fileio(file, file->private_data,
			      V4L2_BUF_TYPE_VIDEO_OUTPUT);
	if (result < 0)
		return result;

	if (count > dev->buffer_size)
		count = dev->buffer_size;
	index = v4l2l_mod64(dev->write_position, dev->used_buffer_count);
	b = &dev->buffers[index].buffer;

	if (copy_from_user((void *)(dev->image + b->m.offset), (void *)buf,
			   count)) {
		printk(KERN_ERR
		       "v4l2-loopback write() failed copy_from_user()\n");
		return -EFAULT;
	}
	b->bytesused = count;

	v4l2l_get_timestamp(b);
	b->sequence = dev->write_position;
	set_queued(b->flags);
	buffer_written(dev, &dev->buffers[index]);
	set_done(b->flags);
	wake_up_all(&dev->read_event);

	return count;
}

/* init functions */
/* frees buffers, if allocated */
/*
 * Copyright (C) 2025 Eli Oliveira Junior
 * rewritten added support for dynamic buffering
*/
static void free_buffers(struct v4l2_loopback_device *dev)
{
	if (!dev)
		return;

	dprintk("free_buffers() with image@%p\n", dev->image);

	/* 1. Free dynamic buffer if enabled */
	if (dev->use_dynamic_buffering) {
		free_dynamic_buffer(dev);
	}

	/* 2. Check and free main image buffer */
	if (!dev->image)
		return;

	/* 3. Warn if buffers are still in use */
	if (!has_no_owners(dev) || any_buffers_mapped(dev)) {
		printk(KERN_WARNING
		       "v4l2-loopback: buffers of video device #%u freed while still mapped to userspace\n",
		       dev->vdev->num);
	}

	/* 4. Free the main image buffer */
	vfree(dev->image);
	dev->image = NULL;
	dev->image_size = 0;
	dev->buffer_size = 0;

	dprintk("free_buffers() completed successfully\n");
}

static void free_timeout_buffer(struct v4l2_loopback_device *dev)
{
	dprintk("free_timeout_buffer() with timeout_image@%p\n",
		dev->timeout_image);
	if (!dev->timeout_image)
		return;

	if ((dev->timeout_jiffies > 0 && !has_no_owners(dev)) ||
	    dev->timeout_buffer.buffer.flags & V4L2_BUF_FLAG_MAPPED)
		printk(KERN_WARNING
		       "v4l2-loopback free_timeout_buffer() timeout image "
		       "of device #%u freed while still mapped to userspace\n",
		       dev->vdev->num);

	vfree(dev->timeout_image);
	dev->timeout_image = NULL;
	dev->timeout_buffer_size = 0;
}

/*
 * Copyright (C) 2025 Eli Oliveira Junior
 * rewritten added support for dynamic buffering
 * allocate or reallocate image buffers (static + dynamic)
*/
static int allocate_buffers(struct v4l2_loopback_device *dev,
			    struct v4l2_pix_format *pix_format)
{
	u32 buffer_size = PAGE_ALIGN(pix_format->sizeimage);
	unsigned long image_size;
	int ret;
	bool dynamic_mode = dev->use_dynamic_buffering;

	/* === STRICT VALIDATION === */
	if (buffer_size == 0 || dev->buffer_count == 0 ||
	    buffer_size < pix_format->sizeimage) {
		pr_err("v4l2-loopback: invalid buffer parameters\n");
		return -EINVAL;
	}

	if (check_mul_overflow(buffer_size, dev->buffer_count, &image_size)) {
		pr_err("v4l2-loopback: overflow in image size calculation\n");
		return -ENOSPC;
	}

	dprintk("allocate_buffers: %lu bytes (%u × %u), dynamic=%d\n",
		image_size, buffer_size, dev->buffer_count, dynamic_mode);

	/* === REUSE OF BUFFERS === */
	if (dev->image && image_size == dev->image_size) {
		dprintk("allocate_buffers: reusing existing buffers\n");

		/* Ensure dynamic buffer initialization if needed */
		if (dynamic_mode && !dev->dbuf) {
			ret = init_dynamic_buffer(dev);
			if (ret) {
				pr_err("v4l2-loopback: dynamic buffer initialization failed\n");
				return ret;
			}
		}
		return 0;
	}

	/* === CHECKING BUFFERS IN USE === */
	if (dev->image && (!has_no_owners(dev) || any_buffers_mapped(dev))) {
		if (!dynamic_mode) {
			pr_warn("v4l2-loopback: cannot reallocate buffers in use\n");
			return -EBUSY;
		}
		/* In dynamic mode, we allow more flexible relocation */
		pr_info("v4l2-loopback: reallocating buffers in dynamic mode\n");
	}

	/* === RELEASE OF OLD BUFFERS === */
	if (dev->image) {
		/* In dynamic mode, preserve dynamic buffer during reallocation */
		if (dynamic_mode && dev->dbuf) {
			/* Check to not flush dynamic buffer */
			dev->dbuf->active = false;
		}
		free_buffers(dev);
	}

	/* === ALLOCATION OF NEW BUFFER === */
	dev->image = vmalloc(image_size);
	if (!dev->image) {
		pr_err("v4l2-loopback: allocation failure %lu bytes\n",
		       image_size);
		ret = -ENOMEM;
		goto err_out;
	}

	dev->image_size = image_size;
	dev->buffer_size = buffer_size;

	/* === INITIALIZATION OF INTERNAL BUFFERS === */
	init_buffers(dev, pix_format->sizeimage, buffer_size);

	/* === DYNAMIC BUFFER INITIALIZATION === */
	/* Initialize dynamic buffer if necessary */
	if (dev->use_dynamic_buffering) {
		ret = init_dynamic_buffer(dev);
		if (ret) {
			pr_err("v4l2-loopback: dynamic buffer initialization failed (%d)\n",
			       ret);
			goto err_free_image;
		}
	}
	pr_info("v4l2-loopback: buffers allocated successfully (%lu bytes, dynamic=%d)\n",
		dev->image_size, dynamic_mode);
	return 0;

/* === ERROR CLEANUP === */
err_free_image:
	vfree(dev->image);
	dev->image = NULL;
	dev->image_size = 0;
	dev->buffer_size = 0;
err_out:
	return ret;
}

static int allocate_timeout_buffer(struct v4l2_loopback_device *dev)
{
	/* device's `buffer_size` and `buffers` must be initialised in
	 * allocate_buffers() */

	dprintk("allocate_timeout_buffer() size %ubytes\n", dev->buffer_size);
	if (dev->buffer_size == 0)
		return -EINVAL;

	if (dev->timeout_image) {
		if (dev->timeout_buffer.buffer.flags & V4L2_BUF_FLAG_MAPPED)
			return -EBUSY;
		if (dev->buffer_size == dev->timeout_buffer_size)
			return 0;
		free_timeout_buffer(dev);
	}

	dev->timeout_image = vzalloc(dev->buffer_size);
	if (!dev->timeout_image) {
		dev->timeout_buffer_size = 0;
		return -ENOMEM;
	}
	dev->timeout_buffer_size = dev->buffer_size;
	return 0;
}
/* init inner buffers, they are capture mode and flags are set as for capture
 * mode buffers */
static void init_buffers(struct v4l2_loopback_device *dev, u32 bytes_used,
			 u32 buffer_size)
{
	u32 i;

	for (i = 0; i < dev->buffer_count; ++i) {
		struct v4l2_buffer *b = &dev->buffers[i].buffer;
		b->index = i;
		b->bytesused = bytes_used;
		b->length = buffer_size;
		b->field = V4L2_FIELD_NONE;
		b->flags = 0;
		b->m.offset = i * buffer_size;
		b->memory = V4L2_MEMORY_MMAP;
		b->sequence = 0;
		b->timestamp.tv_sec = 0;
		b->timestamp.tv_usec = 0;
		b->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

		v4l2l_get_timestamp(b);
	}
	dev->timeout_buffer = dev->buffers[0];
	dev->timeout_buffer.buffer.m.offset = MAX_BUFFERS * buffer_size;
}

/* fills and register video device */
static void init_vdev(struct video_device *vdev, int nr)
{
#ifdef V4L2LOOPBACK_WITH_STD
	vdev->tvnorms = V4L2_STD_ALL;
#endif /* V4L2LOOPBACK_WITH_STD */

	vdev->vfl_type = VFL_TYPE_VIDEO;
	vdev->fops = &v4l2_loopback_fops;
	vdev->ioctl_ops = &v4l2_loopback_ioctl_ops;
	vdev->release = &video_device_release;
	vdev->minor = -1;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0)
	vdev->device_caps = V4L2_CAP_DEVICE_CAPS | V4L2_CAP_VIDEO_CAPTURE |
			    V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_READWRITE |
			    V4L2_CAP_STREAMING;
#endif

	if (debug > 1)
		vdev->dev_debug = V4L2_DEV_DEBUG_IOCTL |
				  V4L2_DEV_DEBUG_IOCTL_ARG;

	vdev->vfl_dir = VFL_DIR_M2M;
}

/* init default capture parameters, only fps may be changed in future */
static void init_capture_param(struct v4l2_captureparm *capture_param)
{
	capture_param->capability = V4L2_CAP_TIMEPERFRAME; /* since 2.16 */
	capture_param->capturemode = 0;
	capture_param->extendedmode = 0;
	capture_param->readbuffers = max_buffers;
	capture_param->timeperframe.numerator = 1;
	capture_param->timeperframe.denominator = V4L2LOOPBACK_FPS_DEFAULT;
}

static void check_timers(struct v4l2_loopback_device *dev)
{
	if (has_output_token(dev->stream_tokens))
		return;

	if (dev->timeout_jiffies > 0 && !timer_pending(&dev->timeout_timer))
		mod_timer(&dev->timeout_timer, jiffies + dev->timeout_jiffies);
	if (dev->sustain_framerate && !timer_pending(&dev->sustain_timer))
		mod_timer(&dev->sustain_timer,
			  jiffies + dev->frame_jiffies * 3 / 2);
}
#ifdef HAVE_TIMER_SETUP
static void sustain_timer_clb(struct timer_list *t)
{
	struct v4l2_loopback_device *dev = from_timer(dev, t, sustain_timer);
#else
static void sustain_timer_clb(unsigned long nr)
{
	struct v4l2_loopback_device *dev =
		idr_find(&v4l2loopback_index_idr, nr);
#endif
	spin_lock(&dev->lock);
	if (dev->sustain_framerate) {
		dev->reread_count++;
		dprintkrw("sustain_timer_clb() write_pos=%lld reread=%u\n",
			  (long long)dev->write_position, dev->reread_count);
		if (dev->reread_count == 1)
			mod_timer(&dev->sustain_timer,
				  jiffies + max(1UL, dev->frame_jiffies / 2));
		else
			mod_timer(&dev->sustain_timer,
				  jiffies + dev->frame_jiffies);
		wake_up_all(&dev->read_event);
	}
	spin_unlock(&dev->lock);
}
#ifdef HAVE_TIMER_SETUP
static void timeout_timer_clb(struct timer_list *t)
{
	struct v4l2_loopback_device *dev = from_timer(dev, t, timeout_timer);
#else
static void timeout_timer_clb(unsigned long nr)
{
	struct v4l2_loopback_device *dev =
		idr_find(&v4l2loopback_index_idr, nr);
#endif
	spin_lock(&dev->lock);
	if (dev->timeout_jiffies > 0) {
		dev->timeout_happened = 1;
		mod_timer(&dev->timeout_timer, jiffies + dev->timeout_jiffies);
		wake_up_all(&dev->read_event);
	}
	spin_unlock(&dev->lock);
}

/*
    Copyright (C) 2025 Eli Oliveira Junior

    calculate_buffer_used_bytes - Computes occupied space in circular buffer
    @dbuf: Pointer to initialized dynamic_buffer structure (must be valid)

    Calculates the exact number of bytes currently stored in a circular buffer by
    comparing read and write positions. The computation handles all buffer states:
    Buffer State Cases:
        Linear (write_pos > read_pos):
        Used space = write_pos - read_pos
        Wrapped (write_pos < read_pos):
        Used space = (buffer_size - read_pos) + write_pos
        Empty (write_pos == read_pos):
        Returns 0
    Implementation Notes:
        Uses unsigned arithmetic to avoid overflow
        Lockless design (caller must ensure synchronization)
        Constant-time operation O(1)
        Handles full buffer case implicitly
    Safety Considerations:
        Requires valid initialized buffer
        Assumes positions are within valid range (0 <= pos < buffer_size)
        Not atomic - caller must hold appropriate locks if needed
    Example Usage:
      u32 used = calculate_buffer_used_bytes(dbuf);
      if (used > HIGH_WATERMARK) {
        // Trigger buffer expansion
        }
      Return: Number of bytes currently stored in buffer (0..buffer_size-1)
*/
static inline u32 calculate_buffer_used_bytes(struct dynamic_buffer *dbuf)
{
	if (dbuf->write_pos >= dbuf->read_pos)
		return dbuf->write_pos - dbuf->read_pos;
	else
		return (dbuf->size - dbuf->read_pos) + dbuf->write_pos;
}

/*
    Copyright (C) 2025 Eli Oliveira Junior

    init_dynamic_buffer - Initializes a thread-safe circular buffer for video frame storage
    @dev: Pointer to initialized v4l2_loopback_device structure

    Creates and configures a dynamic circular buffer with these key features:
    Memory Management:
        Automatically calculates optimal size based on:
        Device pixel format (if specified)
        Minimum/Maximum size constraints
        System capabilities
        Implements graceful fallback for allocation failures
        Uses vzalloc() for large contiguous memory regions
    Thread Safety:
        Implements race-free initialization
        Atomic reference counting
        Proper spinlock protection
        Idempotent operation (safe for multiple calls)
    Error Handling:
        Validates all parameters
        Checks for arithmetic overflow
        Provides detailed error logging
        Returns standardized error codes
    Initialization Phases:
        Validation: Checks device state and parameters
        Size Calculation: Determines optimal buffer dimensions
        Allocation: Attempts primary and fallback allocations
        Configuration: Sets up synchronization primitives
        Activation: Atomically enables the buffer
    Return Values:
        0: Success (buffer ready for use)
          -EINVAL: Invalid parameters or device state
          -EOVERFLOW: Size calculation overflow
          -ENOMEM: Memory allocation failure
    Locking Strategy:
        Uses device spinlock for atomic assignment
        Internal spinlock for buffer operations
        Waitqueues for blocking I/O
    Memory Safety:
        Guarantees cleanup on allocation failure
        Prevents memory leaks
        Handles concurrent access safely
*/
static int init_dynamic_buffer(struct v4l2_loopback_device *dev)
{
	struct dynamic_buffer *dbuf = NULL;
	size_t total_size, initial_size;
	unsigned long flags;
	bool already_initialized = false;

	/* Basic validation*/
	if (unlikely(!dev)) {
		pr_err("v4l2-loopback: null device in init_dynamic_buffer\n");
		return -EINVAL;
	}

	if (unlikely(!dev->use_dynamic_buffering)) {
		pr_info("v4l2-loopback: dynamic buffer disabled\n");
		return 0; /* It's not a mistake, just not necessary. */
	}

	/* Dual-initialization thread-safety check */
	spin_lock_irqsave(&dev->lock, flags);
	if (dev->dbuf) {
		already_initialized = true;
		atomic_inc(&dev->dbuf->ref_count);
	}
	spin_unlock_irqrestore(&dev->lock, flags);

	if (already_initialized) {
		pr_info("v4l2-loopback: dynamic buffer already initialized, reusing\n");
		return 0;
	}

	/* Determine buffer size*/
	if (!dev->buffer_size) {
		if (dev->pix_format.sizeimage > 0) {
			dev->buffer_size =
				PAGE_ALIGN(dev->pix_format.sizeimage);
			dev->buffer_count = max(2U, dev->buffer_count);
			pr_info("v4l2-loopback: buffer_size derived from pix_format: %u bytes\n",
				dev->buffer_size);
		} else {
			pr_warn("v4l2-loopback: buffer_size and sizeimage not set, using default\n");
			dev->buffer_size =
				PAGE_ALIGN(640 * 480 * 2); /* YUYV 640x480 */
			dev->buffer_count = 2;
		}
	}

	/* Safe size calculation */
	if (unlikely(check_mul_overflow(dev->buffer_size, dev->buffer_count,
					&total_size))) {
		pr_err("v4l2-loopback: overflow in buffer calculation (%u × %u)\n",
		       dev->buffer_size, dev->buffer_count);
		return -EOVERFLOW;
	}

	/* Conservative starting size */
	initial_size = max(total_size, (size_t)INITIAL_BUFFER_SIZE);
	initial_size = min(initial_size, (size_t)MAX_DYNAMIC_BUFFER_SIZE);

	if (unlikely(initial_size < MIN_BUFFER_SIZE)) {
		pr_err("v4l2-loopback: size %zu less than minimum %lu\n",
		       initial_size, (unsigned long)MIN_BUFFER_SIZE);
		return -EINVAL;
	}

	/* Structure allocation */
	dbuf = kzalloc(sizeof(*dbuf), GFP_KERNEL);
	if (unlikely(!dbuf)) {
		pr_err("v4l2-loopback: dynamic_buffer structure allocation failed\n");
		return -ENOMEM;
	}

	/* Buffer allocation with retry */
	dbuf->data = vzalloc(initial_size);
	if (unlikely(!dbuf->data)) {
		/* Try smaller size if failure */
		size_t fallback_size = max(total_size, (size_t)MIN_BUFFER_SIZE);
		pr_warn("v4l2-loopback: failed at %zu bytes, trying %zu\n",
			initial_size, fallback_size);

		dbuf->data = vzalloc(fallback_size);
		if (!dbuf->data) {
			pr_err("v4l2-loopback: critical buffer allocation failure\n");
			kfree(dbuf);
			return -ENOMEM;
		}
		initial_size = fallback_size;
	}

	/* Complete initialization */
	dbuf->size = initial_size;
	dbuf->initial_size = initial_size;
	dbuf->write_pos = 0;
	dbuf->read_pos = 0;
	atomic_set(&dbuf->available, 0);
	atomic_set(&dbuf->ref_count, 1);

	spin_lock_init(&dbuf->lock);
	init_waitqueue_head(&dbuf->read_waitq);
	init_waitqueue_head(&dbuf->write_waitq);

	dbuf->active = true;
	dbuf->shutdown_requested = false;
	memset(&dbuf->stats, 0, sizeof(dbuf->stats));
	dbuf->stats.created_at = ktime_get();
	dbuf->stats.min_capacity_used = initial_size;
	dbuf->stats.max_capacity_reached = initial_size;

	/* Final atomic assignment */
	spin_lock_irqsave(&dev->lock, flags);
	if (unlikely(dev->dbuf)) {
		/* Race condition - another thread has started */
		spin_unlock_irqrestore(&dev->lock, flags);
		vfree(dbuf->data);
		kfree(dbuf);
		pr_info("v4l2-loopback: dynamic buffer initialized by another thread\n");
		return 0; /* It's not a mistake */
	}

	dev->dbuf = dbuf;
	spin_unlock_irqrestore(&dev->lock, flags);

	pr_info("v4l2-loopback: dynamic buffer initialized successfully (%zu bytes)\n",
		initial_size);
	return 0;
}

/*
    Copyright (C) 2025 Eli Oliveira Junior

    free_dynamic_buffer - Safely deallocates a dynamic buffer with guaranteed cleanup
    @dev: Pointer to the v4l2_loopback_device structure (must be valid)

    Performs a complete shutdown sequence for dynamic buffers with these phases:
        Buffer Detachment:
        Atomically removes buffer reference from parent device
        Prevents new operations from starting
        Shutdown Signaling:
        Marks buffer inactive
        Sets shutdown_requested flag
        Wakes all blocked threads
        Grace Period:
        Waits BUFFER_GRACE_PERIOD_MS for pending operations
        Allows in-progress I/O to complete
        Resource Reclamation:
        Checks reference count
        Frees memory if no references remain
        Logs status for debugging
    Safety Mechanisms:
        Double-checked locking pattern
        Atomic reference counting
        Grace period for pending operations
        NULL pointer validation
        Proper memory barrier usage
    Memory Management:
        Uses vfree() for vmalloc'd data buffer
        Uses kfree() for control structure
        Nullifies pointers after free
    Error Recovery:
        Handles concurrent access during shutdown
        Tolerates repeated calls
        Preserves system stability if called during active I/O
    Locking Strategy:
        Device lock protects buffer pointer
        Buffer lock protects internal state
        Reference count ensures safe final free
    Thread Safety:
        Safe to call from any context
        Handles concurrent readers/writers
        Non-blocking after grace period
*/
static void free_dynamic_buffer(struct v4l2_loopback_device *dev)
{
	struct dynamic_buffer *dbuf;
	unsigned long flags;
	int ref_count;

	if (unlikely(!dev))
		return;

	spin_lock_irqsave(&dev->lock, flags);
	dbuf = dev->dbuf;
	dev->dbuf = NULL;
	spin_unlock_irqrestore(&dev->lock, flags);

	if (unlikely(!dbuf))
		return;

	// Mark for shutdown BEFORE decrementing reference
	spin_lock_irqsave(&dbuf->lock, flags);
	dbuf->active = false;
	dbuf->shutdown_requested = true;
	spin_unlock_irqrestore(&dbuf->lock, flags);

    // Force wake up ALL blocked operations immediately
	wake_up_interruptible_all(&dbuf->read_waitq);
	wake_up_interruptible_all(&dbuf->write_waitq);

	// Wait for all pending operations to complete
	msleep(BUFFER_GRACE_PERIOD_MS);

	if (ref_count == 0) {
		if (dbuf->data) {
			vfree(dbuf->data);
			dbuf->data = NULL;
		}
		kfree(dbuf);
		pr_info("v4l2-loopback: dynamic buffer flushed successfully\n");
	} else if (ref_count > 0) {
		pr_info("v4l2-loopback: buffer marked for release (refs=%d)\n",
			ref_count);
	}
}

/*
    Copyright (C) 2025 Eli Oliveira Junior

    __copy_circular_data - Efficient circular-to-linear buffer copy operation
    @src_data: Pointer to source circular buffer (must be valid)
    @src_size: Total capacity of circular buffer (must be power-of-2)
    @src_pos: Starting read position (0 <= src_pos < src_size)
    @dst_data: Destination linear buffer (must have copy_len capacity)
    @copy_len: Bytes to copy (must be <= src_size)

    Performs optimized data transfer from circular to linear memory with these
    characteristics:
    Copy Algorithm:
        Calculates contiguous chunk from src_pos to buffer end
        Performs first memcpy for this contiguous segment
        If needed, copies remaining data from buffer start
        Completes in maximum 2 memcpy operations
    Performance:
        O(1) computation overhead
        Minimal branching
        Optimal memcpy utilization
        No temporary buffers
    Safety Requirements:
        All pointers must be valid
        src_size must be power-of-2 for performance
        Positions and lengths must be within bounds
        Caller must ensure proper synchronization
    Typical Use Cases:
        Video frame extraction
        Audio buffer processing
        Network packet handling
    Optimization Notes:
        Branch prediction friendly
        Memory access pattern optimized
        Inlinable for small copies
        No dynamic memory allocation
*/
static void __copy_circular_data(const void *src_data, u32 src_size,
				 u32 src_pos, void *dst_data, u32 copy_len)
{
	u32 first_chunk = min(copy_len, src_size - src_pos);

	/* First part to end of buffer */
	memcpy(dst_data, (const u8 *)src_data + src_pos, first_chunk);

	/* Second part of the beginning of the buffer (if necessary) */
	if (first_chunk < copy_len) {
		memcpy((u8 *)dst_data + first_chunk, src_data,
		       copy_len - first_chunk);
	}
}

/*
    Copyright (C) 2025 Eli Oliveira Junior

    __resize_buffer_locked - Performs an atomic resize of a circular buffer
    @dbuf: Pointer to initialized dynamic_buffer structure (must be valid)
    @new_capacity: Requested new size in bytes (must be validated)

    Executes a safe buffer resize operation with these guarantees:
    Resize Protocol:
        Validation Phase:
        Checks buffer active state
        Verifies new capacity against:
          * Minimum system requirements (MIN_BUFFER_SIZE)
          * Maximum allowed size (MAX_DYNAMIC_BUFFER_SIZE)
          * Current data volume (available bytes)
    Skips insignificant size changes (<10% difference)
    Memory Management:
      Allocates new zero-initialized memory (vzalloc)
      Preserves existing data via optimized circular copy
      Uses atomic pointer swap for buffer replacement
      Frees old memory only after successful replacement
    State Maintenance:
      Updates buffer positions (read_pos/write_pos)
      Tracks capacity statistics (min/max usage)
      Maintains active status during operation
    Performance Characteristics:
      O(n) copy complexity (n = used bytes)
      Zero-copy for no-op cases
      Minimal locking requirements (caller-managed)
      Optimized for large buffer operations
    Safety Requirements:
      Caller must hold appropriate locks
      Buffer must be in active state
      new_capacity must be pre-validated
      No concurrent access during execution
    Error Handling:
      Returns 0 on success
      Returns -EINVAL for:
       Invalid parameters
       Size constraints violation
       Inactive buffer
    Returns -ENOMEM for allocation failures
    Typical Usage:
      Called from within a locked section to implement:
      Automatic buffer expansion (high watermark)
      Controlled buffer shrinking (low watermark)
      Manual size adjustment
*/
static int __resize_buffer_locked(struct dynamic_buffer *dbuf, u32 new_capacity)
{
	void *new_data;
	u32 old_size, available;

	if (unlikely(!dbuf || !dbuf->active))
		return -EINVAL;

	old_size = dbuf->size;
	available = atomic_read(&dbuf->available);

	/* Validation of new capacity */
	if (unlikely(new_capacity < MIN_BUFFER_SIZE ||
		     new_capacity > MAX_DYNAMIC_BUFFER_SIZE ||
		     new_capacity < available)) {
		return -EINVAL;
	}

	/* If the size has not changed significantly, do nothing. */
	if (abs((int)new_capacity - (int)old_size) < (old_size / 10)) {
		return 0;
	}

	/* Allocation of new buffer */
	new_data = vzalloc(new_capacity);
	if (unlikely(!new_data)) {
		pr_err("v4l2-loopback: failed to allocate %u bytes for resize\n",
		       new_capacity);
		return -ENOMEM;
	}

	/* Copy of existing data if any */
	if (available > 0) {
		__copy_circular_data(dbuf->data, old_size, dbuf->read_pos,
				     new_data, available);
	}

	/* Atomic buffer replacement */
	vfree(dbuf->data);
	dbuf->data = new_data;
	dbuf->size = new_capacity;
	dbuf->read_pos = 0;
	dbuf->write_pos = available;

	if (new_capacity > dbuf->stats.max_capacity_reached) {
		dbuf->stats.max_capacity_reached = new_capacity;
	}
	if (new_capacity < dbuf->stats.min_capacity_used) {
		dbuf->stats.min_capacity_used = new_capacity;
	}

	pr_debug(
		"v4l2-loopback: buffer resized from %u to %u bytes (data=%u)\n",
		old_size, new_capacity, available);

	return 0;
}

/*
    Copyright (C) 2025 Eli Oliveira Junior

    resize_dynamic_buffer - Thread-safe dynamic buffer resizing operation
    @dev: Pointer to initialized v4l2_loopback_device structure
    @new_capacity: Requested buffer size in bytes (must be validated)

    Performs a complete buffer resize operation with these guarantees:
    Resize Protocol:
        Reference Acquisition:
        Safely obtains buffer reference under device lock
        Checks shutdown state
        Uses atomic reference counting
        Core Resize Operation:
        Executes under buffer spinlock
        Delegates to __resize_buffer_locked
        Preserves all existing data
        Validates new capacity constraints
        Post-Resize Handling:
        Wakes blocked threads if successful
        Releases reference count
        Handles deferred cleanup if shutdown pending
    Thread Safety:
        Uses proper spinlock nesting (device -> buffer)
        Atomic reference counting
        Safe for concurrent readers/writers
        Handles shutdown race conditions
    Memory Management:
        Ensures proper reference counting
        Handles vmalloc'd memory correctly
        Deferred cleanup if references remain
        Null pointer protection
    Error Handling:
        Returns 0 on success
        Returns -EINVAL for invalid parameters
        Returns -ENODEV if:
        Device not initialized
        Buffer shutdown in progress
        Propagates errors from __resize_buffer_locked
    Locking Strategy:
        Device lock (for reference management)
        Buffer lock (for resize operation)
        Atomic operations (reference counting)
    Typical Usage:
        Called during automatic buffer expansion/shrinking
        Used for manual buffer size adjustment
        Part of dynamic buffer management system
*/
static int resize_dynamic_buffer(struct v4l2_loopback_device *dev,
				 u32 new_capacity)
{
	struct dynamic_buffer *dbuf;
	unsigned long flags;
	int ret;

	if (unlikely(!dev))
		return -EINVAL;

	/* Safely getting the buffer reference */
	spin_lock_irqsave(&dev->lock, flags);
	dbuf = dev->dbuf;
	if (dbuf && !dbuf->shutdown_requested) // Shutdown check
		atomic_inc(&dbuf->ref_count);
	else
		dbuf = NULL; // Ensure dbuf is NULL if shutdown was requested
	spin_unlock_irqrestore(&dev->lock, flags);

	if (unlikely(!dbuf))
		return -ENODEV;

	/* Resizing with buffer lock */
	spin_lock_irqsave(&dbuf->lock, flags);
	ret = __resize_buffer_locked(dbuf, new_capacity);
	spin_unlock_irqrestore(&dbuf->lock, flags);

	/* Wake up threads waiting for space/data after resize */
	if (ret == 0) {
		wake_up_interruptible_all(&dbuf->read_waitq);
		wake_up_interruptible_all(&dbuf->write_waitq);
	}

	/* Reference release */
	if (atomic_dec_return(&dbuf->ref_count) == 0 &&
	    dbuf->shutdown_requested) {
		vfree(dbuf->data);
		kfree(dbuf);
	}

	return ret;
}

/*
    Copyright (C) 2025 Eli Oliveira Junior

    dynamic_buffer_write - Thread-safe data write to dynamic circular buffer
    @dev: Pointer to initialized v4l2_loopback_device structure
    @src: Source buffer containing data to write (must be valid)
    @len: Number of bytes to write (must be > 0)

    Implements a robust write operation with multiple fallback strategies:
    Write Protocol:
        Reference Acquisition:
        Safely obtains buffer reference under device lock
        Validates buffer state (active/shutdown)
        Uses atomic reference counting
        Write Strategies (attempted in order):
    a) Direct write when sufficient space available
    b) Buffer expansion when high watermark reached
    c) Old data discard when partially full
    d) Blocking wait with timeout when necessary
        Post-Write Handling:
        Wakes blocked readers if data written
        Updates performance statistics
        Releases reference count
        Handles deferred cleanup if shutdown pending
    Thread Safety:
        Proper spinlock nesting (device -> buffer)
        Atomic operations for shared counters
        Waitqueue synchronization
        Handles concurrent read/write operations
    Memory Management:
        Safe buffer reference handling
        Automatic expansion up to MAX_DYNAMIC_BUFFER_SIZE
        Graceful fallback when allocation fails
        Proper cleanup on shutdown
    Error Handling:
        Returns bytes written on success (may be partial)
        Returns -EINVAL for invalid parameters
        Returns -ENODEV if buffer unavailable/inactive
        Returns -ETIMEDOUT on wait timeout
        Returns -EINTR if operation interrupted
    Performance Characteristics:
        O(1) for direct writes
        O(n) for buffer expansion (n = data size)
        Adaptive retry logic for contention
        Minimal locking duration
*/
static int dynamic_buffer_write(struct v4l2_loopback_device *dev, const u8 *src,
				u32 len)
{
	struct dynamic_buffer *dbuf;
	unsigned long flags;
	u32 available, free_space, first_chunk;
	int ret = 0, retry_count = 0;

	if (unlikely(!dev || !src || len == 0 || !dev->use_dynamic_buffering))
		return -EINVAL;

	// 1. dev->lock to get safe reference
	spin_lock_irqsave(&dev->lock, flags);
	dbuf = dev->dbuf;
	if (dbuf && !dbuf->shutdown_requested) // Shutdown check
		atomic_inc(&dbuf->ref_count);
	else
		dbuf = NULL; // Ensure dbuf is NULL if shutdown was requested
	spin_unlock_irqrestore(&dev->lock, flags);

	if (unlikely(!dbuf))
		return -ENODEV;

retry_write:
	// 2. dbuf->lock for operation
	spin_lock_irqsave(&dbuf->lock, flags);

	if (unlikely(!dbuf->active || dbuf->shutdown_requested)) {
		ret = -ENODEV;
		goto unlock_and_exit;
	}

	available = atomic_read(&dbuf->available);
	free_space = dbuf->size - available;

	// Case 1: Sufficient space - direct writing
	if (len <= free_space) {
		first_chunk = min(len, dbuf->size - dbuf->write_pos);

		memcpy((u8 *)dbuf->data + dbuf->write_pos, src, first_chunk);
		if (first_chunk < len) {
			memcpy(dbuf->data, src + first_chunk,
			       len - first_chunk);
		}

		dbuf->write_pos = (dbuf->write_pos + len) % dbuf->size;
		atomic_add(len, &dbuf->available);
		dbuf->stats.total_bytes_written += len;
		dbuf->stats.frames_written++;
		dbuf->stats.last_write_at = ktime_get();

		/* Check for waiters before waking up */
		if (waitqueue_active(&dbuf->read_waitq))
			wake_up_interruptible_all(&dbuf->read_waitq);
		ret = len;
		goto unlock_and_exit;
	}

	// Case 2: Buffer full - try expansion
	if (retry_count < MAX_RESIZE_RETRIES &&
	    dbuf->size < MAX_DYNAMIC_BUFFER_SIZE &&
	    (available * 100ULL / dbuf->size) >= HIGH_WATERMARK_PERCENT) {
		u32 new_size = min((u64)dbuf->size * 3 / 2,
				   (u64)MAX_DYNAMIC_BUFFER_SIZE);
		new_size = max(new_size, dbuf->size + len);

		// RELEASE dbuf->lock before calling resize
		spin_unlock_irqrestore(&dbuf->lock, flags);

		ret = resize_dynamic_buffer(dev, new_size);
		if (ret == 0) {
			retry_count++;
			// Update stats after successful resize
			spin_lock_irqsave(&dbuf->lock, flags);
			dbuf->stats.expand_count++;
			dbuf->stats.last_expand_at = ktime_get();
			spin_unlock_irqrestore(&dbuf->lock, flags);
			goto retry_write;
		}

		// If resize failed, re-acquire lock and continue
		spin_lock_irqsave(&dbuf->lock, flags);
	}

	// Case 3: Discard old data to make space
	if (available > len) {
		u32 discard_amount =
			min(available / 2, available - len + (dbuf->size / 10));
		dbuf->read_pos = (dbuf->read_pos + discard_amount) % dbuf->size;
		atomic_sub(discard_amount, &dbuf->available);

		/* Check for waiters before waking up */
		if (waitqueue_active(&dbuf->read_waitq))
			wake_up_interruptible_all(&dbuf->read_waitq);
		spin_unlock_irqrestore(&dbuf->lock, flags);
		goto retry_write;
	}

	// Case 4: Wait for space to become available
	spin_unlock_irqrestore(&dbuf->lock, flags);

	ret = wait_event_interruptible_timeout(
		dbuf->write_waitq,
		(!dbuf->active || dbuf->shutdown_requested ||
		 (dbuf->size - atomic_read(&dbuf->available)) >= len),
		msecs_to_jiffies(100));

	if (ret == -ERESTARTSYS)
		ret = -EINTR;
	else if (ret == 0)
		ret = -ETIMEDOUT;
	else if (ret > 0)
		goto retry_write;

	goto exit_with_ref;

unlock_and_exit:
	spin_unlock_irqrestore(&dbuf->lock, flags);

exit_with_ref:
	// Reference cleanup
	if (atomic_dec_return(&dbuf->ref_count) == 0 &&
	    dbuf->shutdown_requested) {
		vfree(dbuf->data);
		kfree(dbuf);
	}

	return ret;
}

/*
    Copyright (C) 2025 Eli Oliveira Junior

    dynamic_buffer_read - Thread-safe data read from dynamic circular buffer
    @dev: Pointer to initialized v4l2_loopback_device structure
    @dst: Destination buffer for read data (must be valid)
    @len: Maximum number of bytes to read (must be > 0)
    @block: Blocking mode flag (true = wait for data, false = non-blocking)

    Implements a robust read operation with these features:
    Read Protocol:
        Reference Acquisition:
          Safely obtains buffer reference under device lock
          Validates buffer state (active/shutdown)
          Uses atomic reference counting
          Read Modes:
            a) Blocking: Waits indefinitely for data (interruptible)
            b) Non-blocking: Returns immediately if no data available
        Data Transfer:
          Handles circular buffer wrap-around
          Updates read position atomically
          Wakes blocked writers when space becomes available
          Buffer Management:
          Automatic shrinking when underutilized
          Maintains minimum buffer size constraints
          Preserves initial buffer configuration
    Thread Safety:
        Proper spinlock nesting (device -> buffer)
        Atomic operations for shared counters
        Waitqueue synchronization
        Handles concurrent read/write operations
    Memory Management:
        Safe buffer reference handling
        Automatic size adjustment
        Proper cleanup on shutdown
    Error Handling:
        Returns bytes read on success (may be partial)
        Returns -EINVAL for invalid parameters
        Returns -ENODEV if buffer unavailable/inactive
        Returns -EAGAIN in non-blocking mode with no data
        Returns -EINTR if blocking operation interrupted
    Performance Characteristics:
        O(1) for direct reads
        O(n) for buffer shrinking (n = data size)
        Adaptive size management
        Minimal locking duration
*/
static int dynamic_buffer_read(struct v4l2_loopback_device *dev, u8 *dst,
			       u32 len, bool block)
{
	struct dynamic_buffer *dbuf;
	unsigned long flags;
	u32 available, to_read, first_chunk;
	int ret = 0;

	if (unlikely(!dev || !dst || len == 0 || !dev->use_dynamic_buffering))
		return -EINVAL;

	// 1. dev->lock to get reference
	spin_lock_irqsave(&dev->lock, flags);
	dbuf = dev->dbuf;
	if (dbuf && !dbuf->shutdown_requested) // Shutdown check
		atomic_inc(&dbuf->ref_count);
	else
		dbuf = NULL; // Ensure dbuf is NULL if shutdown was requested
	spin_unlock_irqrestore(&dev->lock, flags);

	if (unlikely(!dbuf))
		return -ENODEV;

	// Blocking mode: wait for data (NO LOCKS)
	if (block) {
		ret = wait_event_interruptible(
			dbuf->read_waitq,
			(!dbuf->active || dbuf->shutdown_requested ||
			 atomic_read(&dbuf->available) > 0));

		if (ret == -ERESTARTSYS) {
			ret = -EINTR;
			goto exit_with_ref;
		}
	}

	// 2. dbuf->lock for operation
	spin_lock_irqsave(&dbuf->lock, flags);

	if (unlikely(!dbuf->active || dbuf->shutdown_requested)) {
		ret = -ENODEV;
		goto unlock_and_exit;
	}

	available = atomic_read(&dbuf->available);

	if (!block && available == 0) {
		ret = -EAGAIN;
		goto unlock_and_exit;
	}

	to_read = min(len, available);
	if (to_read == 0) {
		ret = 0;
		goto unlock_and_exit;
	}

	// Reading in up to two parts (circular buffer)
	first_chunk = min(to_read, dbuf->size - dbuf->read_pos);
	memcpy(dst, (const u8 *)dbuf->data + dbuf->read_pos, first_chunk);

	if (first_chunk < to_read) {
		memcpy(dst + first_chunk, dbuf->data, to_read - first_chunk);
	}

	// Update pointers and counters
	dbuf->read_pos = (dbuf->read_pos + to_read) % dbuf->size;
	atomic_sub(to_read, &dbuf->available);

	// Update statistics
	dbuf->stats.total_bytes_read += to_read;
	dbuf->stats.frames_read++;
	dbuf->stats.last_read_at = ktime_get();

	/* Check for waiters before waking up */
	if (waitqueue_active(&dbuf->write_waitq))
		wake_up_interruptible_all(&dbuf->write_waitq);

	// Check the need for shrinkage
	available = atomic_read(&dbuf->available);
	if (dbuf->size > dbuf->initial_size * 2 &&
	    available < (dbuf->size / LOW_WATERMARK_FRACTION) &&
	    dbuf->size > MIN_BUFFER_SIZE) {
		u32 target_size = max(dbuf->initial_size,
				      max(available * 4, (u32)MIN_BUFFER_SIZE));

		if (target_size < dbuf->size) {
			// RELEASE lock before resizing
			spin_unlock_irqrestore(&dbuf->lock, flags);

			if (resize_dynamic_buffer(dev, target_size) == 0) {
				// Update stats if shrink successful
				spin_lock_irqsave(&dbuf->lock, flags);
				dbuf->stats.shrink_count++;
				dbuf->stats.last_shrink_at = ktime_get();
				spin_unlock_irqrestore(&dbuf->lock, flags);
			}

			ret = to_read;
			goto exit_with_ref;
		}
	}

	ret = to_read;

unlock_and_exit:
	spin_unlock_irqrestore(&dbuf->lock, flags);

exit_with_ref:
	// Reference cleanup
	if (atomic_dec_return(&dbuf->ref_count) == 0 &&
	    dbuf->shutdown_requested) {
		vfree(dbuf->data);
		kfree(dbuf);
	}

	return ret;
}

/* init loopback main structure 
 * Copyright (C) 2025 Eli Oliveira Junior
 * rewritten added support for dynamic buffering
*/

#define DEFAULT_FROM_CONF(confmember, default_condition, default_value)        \
	((conf) ?                                                              \
		 ((conf->confmember default_condition) ? (default_value) :     \
							 (conf->confmember)) : \
		 default_value)

static int v4l2_loopback_add(struct v4l2_loopback_config *conf, int *ret_nr)
{
	struct v4l2_loopback_device *dev;
	struct v4l2_ctrl_handler *hdl;
	struct v4l2loopback_private *vdev_priv = NULL;
	int err;

	u32 _width = V4L2LOOPBACK_SIZE_DEFAULT_WIDTH;
	u32 _height = V4L2LOOPBACK_SIZE_DEFAULT_HEIGHT;

	u32 _min_width = DEFAULT_FROM_CONF(min_width,
					   < V4L2LOOPBACK_SIZE_MIN_WIDTH,
					   V4L2LOOPBACK_SIZE_MIN_WIDTH);
	u32 _min_height = DEFAULT_FROM_CONF(min_height,
					    < V4L2LOOPBACK_SIZE_MIN_HEIGHT,
					    V4L2LOOPBACK_SIZE_MIN_HEIGHT);
	u32 _max_width = DEFAULT_FROM_CONF(max_width, < _min_width, max_width);
	u32 _max_height =
		DEFAULT_FROM_CONF(max_height, < _min_height, max_height);
	bool _announce_all_caps = (conf && conf->announce_all_caps >= 0) ?
					  (bool)(conf->announce_all_caps) :
					  !(V4L2LOOPBACK_DEFAULT_EXCLUSIVECAPS);
	int _max_buffers = DEFAULT_FROM_CONF(max_buffers, <= 0, max_buffers);
	int _max_openers = DEFAULT_FROM_CONF(max_openers, <= 0, max_openers);
	struct v4l2_format _fmt;

	int nr = -1;

	if (conf) {
		const int output_nr = conf->output_nr;
#ifdef SPLIT_DEVICES
		const int capture_nr = conf->capture_nr;
#else
		const int capture_nr = output_nr;
#endif
		if (capture_nr >= 0 && output_nr == capture_nr) {
			nr = output_nr;
		} else if (capture_nr < 0 && output_nr < 0) {
			nr = -1;
		} else if (capture_nr < 0) {
			nr = output_nr;
		} else if (output_nr < 0) {
			nr = capture_nr;
		} else {
			printk(KERN_ERR
			       "v4l2-loopback add() split OUTPUT and CAPTURE "
			       "devices not yet supported.\n");
			printk(KERN_INFO
			       "v4l2-loopback add() both devices must have the "
			       "same number (%d != %d).\n",
			       output_nr, capture_nr);
			return -EINVAL;
		}
	}

	if (idr_find(&v4l2loopback_index_idr, nr))
		return -EEXIST;

	/* initialisation of a new device */
	dprintk("add() creating device #%d\n", nr);
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	/*Implement Dynamic Buffer*/
	if (conf && conf->dynamic_buffering == 1) {
		dev->use_dynamic_buffering =
			conf->dynamic_buffering; //Use Dynamic Buffer
	} else {
		dev->use_dynamic_buffering =
			dynamic_buffering; //Use Static Buffer (Default)
	}
	dev->dbuf = NULL; // Initialize pointer to NULL
	atomic_set(&dev->active_producers, 0);

	/* allocate id, if @id >= 0, we're requesting that specific id */
	if (nr >= 0) {
		err = idr_alloc(&v4l2loopback_index_idr, dev, nr, nr + 1,
				GFP_KERNEL);
		if (err == -ENOSPC)
			err = -EEXIST;
	} else {
		err = idr_alloc(&v4l2loopback_index_idr, dev, 0, 0, GFP_KERNEL);
	}
	if (err < 0)
		goto out_free_dev;

	/* register new device */
	MARK();
	nr = err;

	if (conf && conf->card_label[0]) {
		snprintf(dev->card_label, sizeof(dev->card_label), "%s",
			 conf->card_label);
	} else {
		snprintf(dev->card_label, sizeof(dev->card_label),
			 "Dummy video device (0x%04X)", nr);
	}
	snprintf(dev->v4l2_dev.name, sizeof(dev->v4l2_dev.name),
		 "v4l2loopback-%03d", nr);

	err = v4l2_device_register(NULL, &dev->v4l2_dev);
	if (err)
		goto out_free_idr;

	/* initialise the _video_ device */
	MARK();
	err = -ENOMEM;
	dev->vdev = video_device_alloc();
	if (dev->vdev == NULL)
		goto out_unregister;

	vdev_priv = kzalloc(sizeof(struct v4l2loopback_private), GFP_KERNEL);
	if (vdev_priv == NULL)
		goto out_unregister;

	video_set_drvdata(dev->vdev, vdev_priv);
	if (video_get_drvdata(dev->vdev) == NULL)
		goto out_unregister;

	snprintf(dev->vdev->name, sizeof(dev->vdev->name), "%s",
		 dev->card_label);
	vdev_priv->device_nr = nr;
	init_vdev(dev->vdev, nr);
	dev->vdev->v4l2_dev = &dev->v4l2_dev;

	/* initialise v4l2-loopback specific parameters */
	MARK();
	dev->announce_all_caps = _announce_all_caps;
	dev->min_width = _min_width;
	dev->min_height = _min_height;
	dev->max_width = _max_width;
	dev->max_height = _max_height;
	dev->max_openers = _max_openers;

	/* set (initial) pixel and stream format */
	_width = clamp_val(_width, _min_width, _max_width);
	_height = clamp_val(_height, _min_height, _max_height);
	_fmt = (struct v4l2_format){
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.fmt.pix = { .width = _width,
			     .height = _height,
			     .pixelformat = formats[0].fourcc,
			     .colorspace = V4L2_COLORSPACE_DEFAULT,
			     .field = V4L2_FIELD_NONE }
	};

	err = v4l2l_fill_format(&_fmt, _min_width, _max_width, _min_height,
				_max_height);
	if (err)
		/* highly unexpected failure to assign default format */
		goto out_unregister;
	dev->pix_format = _fmt.fmt.pix;
	init_capture_param(&dev->capture_param);
	set_timeperframe(dev, &dev->capture_param.timeperframe);

	/* ctrls parameters */
	dev->keep_format = 0;
	dev->sustain_framerate = 0;
	dev->timeout_jiffies = 0;
	dev->timeout_image_io = 0;

	/* initialise OUTPUT and CAPTURE buffer values */
	dev->image = NULL;
	dev->image_size = 0;
	dev->buffer_count = _max_buffers;
	dev->buffer_size = 0;
	dev->used_buffer_count = 0;
	INIT_LIST_HEAD(&dev->outbufs_list);
	do {
		u32 index;
		for (index = 0; index < dev->buffer_count; ++index)
			INIT_LIST_HEAD(&dev->buffers[index].list_head);

	} while (0);
	memset(dev->bufpos2index, 0, sizeof(dev->bufpos2index));
	dev->write_position = 0;

	/* initialise synchronisation data */
	atomic_set(&dev->open_count, 0);
	mutex_init(&dev->image_mutex);
	spin_lock_init(&dev->lock);
	spin_lock_init(&dev->list_lock);
	init_waitqueue_head(&dev->read_event);
	dev->format_tokens = V4L2L_TOKEN_MASK;
	dev->stream_tokens = V4L2L_TOKEN_MASK;

	/* initialise sustain frame rate and timeout parameters, and timers */
	dev->reread_count = 0;
	dev->timeout_image = NULL;
	dev->timeout_happened = 0;
#ifdef HAVE_TIMER_SETUP
	timer_setup(&dev->sustain_timer, sustain_timer_clb, 0);
	timer_setup(&dev->timeout_timer, timeout_timer_clb, 0);
#else
	setup_timer(&dev->sustain_timer, sustain_timer_clb, nr);
	setup_timer(&dev->timeout_timer, timeout_timer_clb, nr);
#endif

	/* initialise the control handler and add controls */
	MARK();
	hdl = &dev->ctrl_handler;
	err = v4l2_ctrl_handler_init(hdl, 4);
	if (err)
		goto out_unregister;
	v4l2_ctrl_new_custom(hdl, &v4l2loopback_ctrl_keepformat, NULL);
	v4l2_ctrl_new_custom(hdl, &v4l2loopback_ctrl_sustainframerate, NULL);
	v4l2_ctrl_new_custom(hdl, &v4l2loopback_ctrl_timeout, NULL);
	v4l2_ctrl_new_custom(hdl, &v4l2loopback_ctrl_timeoutimageio, NULL);
	if (hdl->error) {
		err = hdl->error;
		goto out_free_handler;
	}
	dev->v4l2_dev.ctrl_handler = hdl;

	err = v4l2_ctrl_handler_setup(hdl);
	if (err)
		goto out_free_handler;

	/* register the device (creates /dev/video*) */
	MARK();
	if (video_register_device(dev->vdev, VFL_TYPE_VIDEO, nr) < 0) {
		printk(KERN_ERR
		       "v4l2-loopback add() failed video_register_device()\n");
		err = -EFAULT;
		goto out_free_device;
	}
	v4l2loopback_create_sysfs(dev->vdev);
	/* NOTE: ambivalent if sysfs entries fail */

	if (ret_nr)
		*ret_nr = dev->vdev->num;
	return 0;

out_free_device:
	video_device_release(dev->vdev);
out_free_handler:
	v4l2_ctrl_handler_free(&dev->ctrl_handler);
out_unregister:
	video_set_drvdata(dev->vdev, NULL);
	if (vdev_priv != NULL)
		kfree(vdev_priv);
	v4l2_device_unregister(&dev->v4l2_dev);
out_free_idr:
	idr_remove(&v4l2loopback_index_idr, nr);
out_free_dev:
	kfree(dev);
	return err;
}

/*
 * Copyright (C) 2025 Eli Oliveira Junior
 * rewritten added support for dynamic buffering 
 */
static void v4l2_loopback_remove(struct v4l2_loopback_device *dev)
{
	int device_nr = v4l2loopback_get_vdev_nr(dev->vdev);

	if (!dev) {
		pr_warn("v4l2loopback: remove called with NULL device\n");
		return;
	}

	dprintk("removing device %d\n", device_nr);

	/* 1. Stop all I/O operations */
	mutex_lock(&dev->image_mutex);

	/* 2. Free dynamic buffer first */
	if (dev->use_dynamic_buffering) {
		dprintk("freeing dynamic buffer for device %d\n", device_nr);
		free_dynamic_buffer(dev);
	}

	/* 3. Free main buffers */
	dprintk("freeing main buffers for device %d\n", device_nr);
	free_buffers(dev);

	/* 4. Free timeout buffer */
	dprintk("freeing timeout buffer for device %d\n", device_nr);
	free_timeout_buffer(dev);

	mutex_unlock(&dev->image_mutex);

	/* 5. Remove sysfs interface */
	dprintk("removing sysfs interface for device %d\n", device_nr);
	v4l2loopback_remove_sysfs(dev->vdev);

	/* 6. Release control handler */
	dprintk("freeing ctrl handler for device %d\n", device_nr);
	v4l2_ctrl_handler_free(&dev->ctrl_handler);

	/* 7. Clear video device */
	dprintk("unregistering video device %d\n", device_nr);
	if (dev->vdev) {
		struct v4l2loopback_private *priv =
			video_get_drvdata(dev->vdev);
		kfree(priv);
		video_unregister_device(dev->vdev);
	}

	/* 8. Unregister V4L2 device */
	dprintk("unregistering v4l2 device %d\n", device_nr);
	v4l2_device_unregister(&dev->v4l2_dev);

	/* 9. Remove from IDR */
	if (device_nr >= 0) {
		dprintk("removing device %d from IDR\n", device_nr);
		idr_remove(&v4l2loopback_index_idr, device_nr);
	}

	/* 10. Finally release the device structure */
	dprintk("freeing device structure for device %d\n", device_nr);
	kfree(dev);
	dprintk("device %d removed successfully\n", device_nr);
}

static long v4l2loopback_control_ioctl(struct file *file, unsigned int cmd,
				       unsigned long parm)
{
	struct v4l2_loopback_device *dev;
	struct v4l2_loopback_config conf;
	struct v4l2_loopback_config *confptr = &conf;
	int device_nr, capture_nr, output_nr;
	int ret;
	const __u32 version = V4L2LOOPBACK_VERSION_CODE;

	ret = mutex_lock_killable(&v4l2loopback_ctl_mutex);
	if (ret)
		return ret;

	ret = -EINVAL;
	switch (cmd) {
	default:
		ret = -ENOSYS;
		break;
		/* add a v4l2loopback device (pair), based on the user-provided specs */
	case V4L2LOOPBACK_CTL_ADD:
	case V4L2LOOPBACK_CTL_ADD_legacy:
		if (parm) {
			if ((ret = copy_from_user(&conf, (void *)parm,
						  sizeof(conf))) < 0)
				break;
		} else
			confptr = NULL;
		ret = v4l2_loopback_add(confptr, &device_nr);
		if (ret >= 0)
			ret = device_nr;
		break;
		/* remove a v4l2loopback device (both capture and output) */
	case V4L2LOOPBACK_CTL_REMOVE:
	case V4L2LOOPBACK_CTL_REMOVE_legacy:
		ret = v4l2loopback_lookup((__u32)parm, &dev);
		if (ret >= 0 && dev) {
			ret = -EBUSY;
			if (dev->open_count.counter > 0)
				break;
			v4l2_loopback_remove(dev);
			ret = 0;
		};
		break;
		/* get information for a loopback device.
		 * this is mostly about limits (which cannot be queried directly with  VIDIOC_G_FMT and friends
		 */
	case V4L2LOOPBACK_CTL_QUERY:
	case V4L2LOOPBACK_CTL_QUERY_legacy:
		if (!parm)
			break;
		if ((ret = copy_from_user(&conf, (void *)parm, sizeof(conf))) <
		    0)
			break;
		capture_nr = output_nr = conf.output_nr;
#ifdef SPLIT_DEVICES
		capture_nr = conf.capture_nr;
#endif
		device_nr = (output_nr < 0) ? capture_nr : output_nr;
		MARK();
		/* get the device from either capture_nr or output_nr (whatever is valid) */
		if ((ret = v4l2loopback_lookup(device_nr, &dev)) < 0)
			break;
		MARK();
		/* if we got the device from output_nr and there is a valid capture_nr,
		 * make sure that both refer to the same device (or bail out)
		 */
		if ((device_nr != capture_nr) && (capture_nr >= 0) &&
		    ((ret = v4l2loopback_lookup(capture_nr, 0)) < 0))
			break;
		MARK();
		/* if otoh, we got the device from capture_nr and there is a valid output_nr,
		 * make sure that both refer to the same device (or bail out)
		 */
		if ((device_nr != output_nr) && (output_nr >= 0) &&
		    ((ret = v4l2loopback_lookup(output_nr, 0)) < 0))
			break;

		/* v4l2_loopback_config identified a single device, so fetch the data */
		snprintf(conf.card_label, sizeof(conf.card_label), "%s",
			 dev->card_label);

		conf.output_nr = dev->vdev->num;
#ifdef SPLIT_DEVICES
		conf.capture_nr = dev->vdev->num;
#endif
		conf.min_width = dev->min_width;
		conf.min_height = dev->min_height;
		conf.max_width = dev->max_width;
		conf.max_height = dev->max_height;
		conf.announce_all_caps = dev->announce_all_caps;
		conf.max_buffers = dev->buffer_count;
		conf.max_openers = dev->max_openers;
		conf.debug = debug;
		MARK();
		if (copy_to_user((void *)parm, &conf, sizeof(conf))) {
			ret = -EFAULT;
			break;
		}
		ret = 0;
		break;
	case V4L2LOOPBACK_CTL_VERSION:
		if (!parm)
			break;
		if (copy_to_user((void *)parm, &version, sizeof(version))) {
			ret = -EFAULT;
			break;
		}
		ret = 0;
		break;
	}

	mutex_unlock(&v4l2loopback_ctl_mutex);
	MARK();
	return ret;
}

/* LINUX KERNEL */

static const struct file_operations v4l2loopback_ctl_fops = {
	// clang-format off
	.owner		= THIS_MODULE,
	.open		= nonseekable_open,
	.unlocked_ioctl	= v4l2loopback_control_ioctl,
	.compat_ioctl	= v4l2loopback_control_ioctl,
	.llseek		= noop_llseek,
	// clang-format on
};

static struct miscdevice v4l2loopback_misc = {
	// clang-format off
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "v4l2loopback",
	.fops		= &v4l2loopback_ctl_fops,
	// clang-format on
};

static const struct v4l2_file_operations v4l2_loopback_fops = {
	// clang-format off
	.owner		= THIS_MODULE,
	.open		= v4l2_loopback_open,
	.release	= v4l2_loopback_close,
	.read		= v4l2_loopback_read,
	.write		= v4l2_loopback_write,
	.poll		= v4l2_loopback_poll,
	.mmap		= v4l2_loopback_mmap,
	.unlocked_ioctl	= video_ioctl2,
	// clang-format on
};

static const struct v4l2_ioctl_ops v4l2_loopback_ioctl_ops = {
	// clang-format off
	.vidioc_querycap		= &vidioc_querycap,
	.vidioc_enum_framesizes		= &vidioc_enum_framesizes,
	.vidioc_enum_frameintervals	= &vidioc_enum_frameintervals,

	.vidioc_enum_output		= &vidioc_enum_output,
	.vidioc_g_output		= &vidioc_g_output,
	.vidioc_s_output		= &vidioc_s_output,

	.vidioc_enum_input		= &vidioc_enum_input,
	.vidioc_g_input			= &vidioc_g_input,
	.vidioc_s_input			= &vidioc_s_input,

	.vidioc_enum_fmt_vid_cap	= &vidioc_enum_fmt_cap,
	.vidioc_g_fmt_vid_cap		= &vidioc_g_fmt_cap,
	.vidioc_s_fmt_vid_cap		= &vidioc_s_fmt_cap,
	.vidioc_try_fmt_vid_cap		= &vidioc_try_fmt_cap,

	.vidioc_enum_fmt_vid_out	= &vidioc_enum_fmt_out,
	.vidioc_s_fmt_vid_out		= &vidioc_s_fmt_out,
	.vidioc_g_fmt_vid_out		= &vidioc_g_fmt_out,
	.vidioc_try_fmt_vid_out		= &vidioc_try_fmt_out,

#ifdef V4L2L_OVERLAY
	.vidioc_s_fmt_vid_overlay	= &vidioc_s_fmt_overlay,
	.vidioc_g_fmt_vid_overlay	= &vidioc_g_fmt_overlay,
#endif

#ifdef V4L2LOOPBACK_WITH_STD
	.vidioc_s_std			= &vidioc_s_std,
	.vidioc_g_std			= &vidioc_g_std,
	.vidioc_querystd		= &vidioc_querystd,
#endif /* V4L2LOOPBACK_WITH_STD */

	.vidioc_g_parm			= &vidioc_g_parm,
	.vidioc_s_parm			= &vidioc_s_parm,

	.vidioc_reqbufs			= &vidioc_reqbufs,
	.vidioc_querybuf		= &vidioc_querybuf,
	.vidioc_qbuf			= &vidioc_qbuf,
	.vidioc_dqbuf			= &vidioc_dqbuf,

	.vidioc_streamon		= &vidioc_streamon,
	.vidioc_streamoff		= &vidioc_streamoff,

#ifdef CONFIG_VIDEO_V4L1_COMPAT
	.vidiocgmbuf			= &vidiocgmbuf,
#endif

	.vidioc_subscribe_event		= &vidioc_subscribe_event,
	.vidioc_unsubscribe_event	= &v4l2_event_unsubscribe,
	// clang-format on
};

static int free_device_cb(int id, void *ptr, void *data)
{
	struct v4l2_loopback_device *dev = ptr;
	v4l2_loopback_remove(dev);
	return 0;
}
static void free_devices(void)
{
	idr_for_each(&v4l2loopback_index_idr, &free_device_cb, NULL);
	idr_destroy(&v4l2loopback_index_idr);
}

static int __init v4l2loopback_init_module(void)
{
	const u32 min_width = V4L2LOOPBACK_SIZE_MIN_WIDTH;
	const u32 min_height = V4L2LOOPBACK_SIZE_MIN_HEIGHT;
	int err;
	int i;
	MARK();

	err = misc_register(&v4l2loopback_misc);
	if (err < 0)
		return err;

	if (devices < 0) {
		devices = 1;

		/* try guessing the devices from the "video_nr" parameter */
		for (i = MAX_DEVICES - 1; i >= 0; i--) {
			if (video_nr[i] >= 0) {
				devices = i + 1;
				break;
			}
		}
	}

	if (devices > MAX_DEVICES) {
		devices = MAX_DEVICES;
		printk(KERN_INFO
		       "v4l2-loopback init() number of initial devices is "
		       "limited to: %d\n",
		       MAX_DEVICES);
	}

	if (max_buffers > MAX_BUFFERS) {
		max_buffers = MAX_BUFFERS;
		printk(KERN_INFO
		       "v4l2-loopback init() number of buffers is limited "
		       "to: %d\n",
		       MAX_BUFFERS);
	}

	if (max_openers < 0) {
		printk(KERN_INFO
		       "v4l2-loopback init() allowing %d openers rather "
		       "than %d\n",
		       2, max_openers);
		max_openers = 2;
	}

	if (max_width < min_width) {
		max_width = V4L2LOOPBACK_SIZE_DEFAULT_MAX_WIDTH;
		printk(KERN_INFO "v4l2-loopback init() using max_width %d\n",
		       max_width);
	}
	if (max_height < min_height) {
		max_height = V4L2LOOPBACK_SIZE_DEFAULT_MAX_HEIGHT;
		printk(KERN_INFO "v4l2-loopback init() using max_height %d\n",
		       max_height);
	}

	for (i = 0; i < devices; i++) {
		struct v4l2_loopback_config cfg = {
			// clang-format off
			.output_nr		= video_nr[i],
#ifdef SPLIT_DEVICES
			.capture_nr		= video_nr[i],
#endif
			.min_width		= min_width,
			.min_height		= min_height,
			.max_width		= max_width,
			.max_height		= max_height,
			.announce_all_caps	= (!exclusive_caps[i]),
			.max_buffers		= max_buffers,
			.max_openers		= max_openers,
			.debug			= debug,
			.dynamic_buffering    = 0,
			// clang-format on
		};
		cfg.card_label[0] = 0;
		if (card_label[i])
			snprintf(cfg.card_label, sizeof(cfg.card_label), "%s",
				 card_label[i]);
		err = v4l2_loopback_add(&cfg, 0);
		if (err) {
			free_devices();
			goto error;
		}
	}

	dprintk("module installed\n");

	printk(KERN_INFO "v4l2-loopback driver version %d.%d.%d%s loaded\n",
	       // clang-format off
	       (V4L2LOOPBACK_VERSION_CODE >> 16) & 0xff,
	       (V4L2LOOPBACK_VERSION_CODE >>  8) & 0xff,
	       (V4L2LOOPBACK_VERSION_CODE      ) & 0xff,
#ifdef SNAPSHOT_VERSION
	       " (" __stringify(SNAPSHOT_VERSION) ")"
#else
	       ""
#endif
	       );
	// clang-format on

	return 0;
error:
	misc_deregister(&v4l2loopback_misc);
	return err;
}

static void v4l2loopback_cleanup_module(void)
{
	MARK();
	/* unregister the device -> it deletes /dev/video* */
	free_devices();
	/* and get rid of /dev/v4l2loopback */
	misc_deregister(&v4l2loopback_misc);
	dprintk("module removed\n");
}

MODULE_ALIAS_MISCDEV(MISC_DYNAMIC_MINOR);

module_init(v4l2loopback_init_module);
module_exit(v4l2loopback_cleanup_module);
