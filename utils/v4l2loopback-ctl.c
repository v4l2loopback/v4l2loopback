/* -*- c-file-style: "linux" -*- */
/*
 * v4l2loopback-ctl  --  An application to control v4l2loopback devices driver
 *
 * Copyright (C) 2020-2023 IOhannes m zmoelnig (zmoelnig@iem.at)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <ctype.h>
#include <getopt.h>
#include <glob.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/ioctl.h>
#include <linux/videodev2.h>

#include <errno.h>

#include "v4l2loopback.h"

#ifndef GLOB_ONLYDIR
/* Fix for musl libc and other libcs missing GLOB_ONLYDIR at glob.h */
/* (GLOB_ONLYDIR is not required by POSIX) */
#define GLOB_ONLYDIR 0
#endif

#define CONTROLDEVICE "/dev/v4l2loopback"

#if 0
#define MARK() dprintf(2, "%s:%d @ %s\n", __FILE__, __LINE__, __func__)
#else
#define MARK()
#endif

struct v4l2l_format {
	char *name;
	int fourcc; /* video4linux 2 */
	int depth; /* bit/pixel */
	int flags;
};
#define FORMAT_FLAGS_PLANAR 0x01
#define FORMAT_FLAGS_COMPRESSED 0x02
#include "../v4l2loopback_formats.h"

/********************/
/* helper functions */

/* running externals programs */
static char *which(char *outbuf, size_t bufsize, const char *filename)
{
	struct stat statbuf;
	char *paths, *saveptr = NULL;
	if (filename && '/' == *filename) {
		/* an absolute filename */
		int err = stat(filename, &statbuf);
		if (!err) {
			snprintf(outbuf, bufsize, "%s", filename);
			return outbuf;
		}
		return NULL;
	}
	for (paths = getenv("PATH");; paths = NULL) {
		char *path = strtok_r(paths, ":", &saveptr);
		int err;
		if (path == NULL)
			return NULL;
		snprintf(outbuf, bufsize, "%s/%s", path, filename);
		err = stat(outbuf, &statbuf);
		if (!err)
			return outbuf;
	}
	return NULL;
}

static pid_t pid;
void exec_cleanup(int signal)
{
	if (pid) {
		switch (signal) {
		default:
			break;
		case SIGINT:
			kill(pid, SIGTERM);
			break;
		}
	}

	while (waitpid((pid_t)(-1), 0, WNOHANG) > 0) {
	}
}
static int my_execv(char *const *cmdline)
{
	char exe[1024];
	int res = 0;
	if (!which(exe, 1024, cmdline[0])) {
		dprintf(2, "cannot find %s - is it installed???\n", cmdline[0]);
		return 1;
	}
#if 0
	do {
		char *const *argp = cmdline;
		dprintf(2, "%s:", exe);
		while (*argp) {
			dprintf(2, " %s", *argp++);
		};
		dprintf(2, "\n");
	} while(0);
#endif

	pid = fork();
	if (pid == 0) { /* this is the child-process */
		res = execv(exe, cmdline);
		if (res < 0) {
			dprintf(2, "ERROR running helper program (%d, %d)", res,
				errno);
			dprintf(2, "failed program was:\n\t");
			while (*cmdline)
				dprintf(2, " %s", *cmdline++);
			dprintf(2, "\n");
			exit(0);
		}
		exit(0);
	} else if (pid > 0) { /* we are parent: wait for child */
		int status = 0;
		int waitoptions = 0;
		signal(SIGCHLD, exec_cleanup);
		signal(SIGINT, exec_cleanup);
		waitpid(pid, &status, waitoptions);
		pid = 0;
		if (WIFEXITED(status))
			return WEXITSTATUS(status);
		return 0;
	} else { /* pid < 0, error */
		dprintf(2, "ERROR: child fork failed\n");
		exit(1);
	}
	return 0;
}

/* misc */
static int my_atoi(const char *name, const char *s)
{
	char *endptr = 0;
	int n = strtol(s, &endptr, 10);
	if (*endptr) {
		dprintf(2, "%s must be a number (got: '%s')\n", name, s);
		exit(1);
	}
	return n;
}

static void printf_raw(const char *str, int escape_level)
{
	const char *backslash = (escape_level > 1) ? "\\\\" : "\\";
	if (escape_level > 0)
		while (*str) {
			char c = *str++;
			switch (c) {
			case '\"':
				printf("%s\"", backslash);
				break;
			case '\'':
				printf("%s\'", backslash);
				break;
			case '\\':
				printf("%s\\", backslash);
				break;
			case '\a':
				printf("%sa", backslash);
				break;
			case '\b':
				printf("%sb", backslash);
				break;
			case '\n':
				printf("%sn", backslash);
				break;
			case '\t':
				printf("%st", backslash);
				break;
				// and so on
			default:
				if (iscntrl(c))
					printf("%s%03o", backslash, c);
				else
					printf("%c", c);
			}
		}
	else
		printf("%s", str);
}

static char *fourcc2str(unsigned int fourcc, char buf[4])
{
	buf[0] = (fourcc >> 0) & 0xFF;
	buf[1] = (fourcc >> 8) & 0xFF;
	buf[2] = (fourcc >> 16) & 0xFF;
	buf[3] = (fourcc >> 24) & 0xFF;

	return buf;
}
unsigned int str2fourcc(char buf[4])
{
	return (buf[0]) + (buf[1] << 8) + (buf[2] << 16) + (buf[3] << 24);
}

/* helper functions */
/********************/

static unsigned int _get_control_id(int fd, const char *control)
{
	const size_t length = strnlen(control, 1024);
	const unsigned next = V4L2_CTRL_FLAG_NEXT_CTRL;
	struct v4l2_queryctrl qctrl;
	int id;

	memset(&qctrl, 0, sizeof(qctrl));
	while (ioctl(fd, VIDIOC_QUERYCTRL, &qctrl) == 0) {
		if (!strncmp(qctrl.name, control, length))
			return qctrl.id;
		qctrl.id |= next;
	}
	for (id = V4L2_CID_USER_BASE; id < V4L2_CID_LASTP1; id++) {
		qctrl.id = id;
		if (ioctl(fd, VIDIOC_QUERYCTRL, &qctrl) == 0) {
			if (!strncmp(qctrl.name, control, length))
				return qctrl.id;
		}
	}
	for (qctrl.id = V4L2_CID_PRIVATE_BASE;
	     ioctl(fd, VIDIOC_QUERYCTRL, &qctrl) == 0; qctrl.id++) {
		if (!strncmp(qctrl.name, control, length)) {
			unsigned int id = qctrl.id;
			return id;
		}
	}
	return 0;
}

static int set_control_i(int fd, const char *control, int value)
{
	struct v4l2_control ctrl;
	memset(&ctrl, 0, sizeof(ctrl));
	ctrl.id = _get_control_id(fd, control);
	ctrl.value = value;
	if (ctrl.id && ioctl(fd, VIDIOC_S_CTRL, &ctrl) == 0) {
		int value = ctrl.value;
		return value;
	}
	return 0;
}
static int get_control_i(int fd, const char *control)
{
	struct v4l2_control ctrl;
	memset(&ctrl, 0, sizeof(ctrl));
	ctrl.id = _get_control_id(fd, control);

	if (ctrl.id && ioctl(fd, VIDIOC_G_CTRL, &ctrl) == 0) {
		int value = ctrl.value;
		return value;
	}
	return 0;
}

/********************/
/* main logic       */
typedef enum {
	VERSION,
	HELP,
	ADD,
	DELETE,
	LIST,
	QUERY,
	SET_FPS,
	GET_FPS,
	SET_CAPS,
	GET_CAPS,
	SET_TIMEOUTIMAGE,
	MOO,
	_UNKNOWN
} t_command;

static void _help(int detail, const char *section, const char *program,
		  const char *verb, const char *argstring,
		  const char *description, const char *options)
{
	(void)section;
	if (!detail) {
		dprintf(2, "%s%s%s %s\n", program ? program : "",
			program ? " " : "", verb, argstring);
		return;
	}

	dprintf(2, "\n%s%s%s %s:\n", program ? program : "", program ? " " : "",
		verb, argstring);

	dprintf(2, " %s\n ", description);

	if (options) {
		dprintf(2, "%s\n", options);
		//dprintf(2, "\n\n'%s' OPTIONS:%s\n", verb, options);
	}
}

static void help_list(const char *program, int detail)
{
	_help(detail, "Listing Devices", program, "list", "[OPTIONS]",
	      "list all available loopback-devices",
	      "\n\t-e, --escape             escape control-characters in (device) names"
	      "\n\t-h, --help               print this help and exit"
	      "");
}
static void help_add(const char *program, int detail)
{
	_help(detail, "Adding Devices", program, "add",
	      "[OPTIONS] [<outputdevice> [<capturedevice>]]",
	      "create/add a new loopback-device",
	      "\n\t-b <num>, --buffers <num>     buffers to queue"
	      "\n\t-h <h>, --max-height <h>      maximum allowed frame height"
	      "\n\t-n <name>, --name <name>      pretty name for the device"
	      "\n\t-o <num>, --max-openers <num> maximum allowed concurrent openers"
	      "\n\t-v, --verbose                 verbose mode (print properties of device after successfully creating it)"
	      "\n\t-w <w>, --max-width <w>       maximum allowed frame width"
	      "\n\t-x <x>, --exclusive-caps <x>  whether to announce OUTPUT/CAPTURE capabilities exclusively"
	      "\n\t--min-width <w>               minimum allowed frame width"
	      "\n\t--min-height <w>              minimum allowed frame height"
	      "\n\t-?, --help                    print this help and exit"
	      "\n"
	      "\n  <outputdevice>\tif given, create a specific device (otherwise just create a free one)."
	      "\n          \teither specify a device name (e.g. '/dev/video1') or a device number ('1')."
	      "\n  <capturedevice>\tif given, use separate output & capture devices (otherwise they are the same).");
}
static void help_delete(const char *program, int detail)
{
	_help(detail, "Deleting Devices", program, "delete", "<device>",
	      "delete/remove an unused loopback device",
	      "\n  <device>\tcan be given one more more times (to delete multiple devices at once)."
	      "\n          \teither specify a device name (e.g. '/dev/video1') or a device number ('1').");
}
static void help_query(const char *program, int detail)
{
	_help(detail, "Querying Devices", program, "query",
	      "[OPTIONS] <device>", "query information about a loopback device",
	      "\n\t-e, --escape             escape control-characters in (device) names"
	      "\n\t-h, --help               print this help and exit"
	      "\n"
	      "\n  <device>\tcan be given one more more times (to query multiple devices at once)."
	      "\n         \teither specify a device name (e.g. '/dev/video1') or a device number ('1').");
}
static void help_setfps(const char *program, int detail)
{
	_help(detail, "Setting Framerate", program, "set-fps", "<device> <fps>",
	      "set the default framerate for a loopback device",
	      "\n  <device>\teither specify a device name (e.g. '/dev/video1') or a device number ('1')."
	      "\n     <fps>\tframes per second, either as integer ('30') or fraction ('50/2').");
}
static void help_getfps(const char *program, int detail)
{
	_help(detail, "Getting Framerate", program, "get-fps", "<device>",
	      "query the framerate of a loopback device", 0);
}
static void help_setcaps(const char *program, int detail)
{
	_help(detail, "Setting Capabilities", program, "set-caps",
	      "<device> <caps>",
	      "set format/dimension/framerate of a loopback device",
	      "\n  <device>\teither specify a device name (e.g. '/dev/video1') or a device number ('1')."
	      "\n    <caps>\tformat specification as '<fourcc>:<width>x<height>@<fps>' (e.g. 'UYVY:1024x768@60/1')"
	      "\n          \tunset the current caps with the special value 'any'");
	if (detail > 1) {
		dprintf(2, "\nknown fourcc-codes"
			   "\n=================="
			   "\nFOURCC\thex       \tdec         \tdescription"
			   "\n------\t----------\t------------\t-----------"
			   "");
		char fourcc[5];
		const size_t num_formats = sizeof(formats) / sizeof(*formats);
		size_t i = 0;
		for (i = 0; i < num_formats; i++) {
			const struct v4l2l_format *fmt = formats + i;
			memset(fourcc, 0, 5);
			dprintf(2, "'%4s'\t0x%08X\t%12d\t%s\n",
				fourcc2str(fmt->fourcc, fourcc), fmt->fourcc,
				fmt->fourcc, fmt->name);
		}
	}
}
static void help_getcaps(const char *program, int detail)
{
	_help(detail, "Getting Capabilities", program, "get-caps", "<device>",
	      "get current format/dimension/framerate of a loopback device", 0);
}
static void help_settimeoutimage(const char *program, int detail)
{
	_help(detail, "Setting Timeout Image", program, "set-timeout-image",
	      "[OPTIONS] <device> <image>",
	      "set a fallback image to be used if a video producer does not send new frames in time.",
	      "\n   <flags>\tany of the following flags may be present"
	      "\n\t-h, --help                         print this help and exit"
	      "\n\t-t <timeout>, --timeout <timeout>  timeout (in ms)"
	      "\n\t-v, --verbose                      raise verbosity (print what is being done)"
	      "\n"
	      "\n  <device>\teither specify a device name (e.g. '/dev/video1') or a device number ('1')."
	      "\n   <image>\timage file");
}
static void help_none(const char *program, int detail)
{
}
typedef void (*t_help)(const char *, int);
static t_help get_help(t_command cmd)
{
	switch (cmd) {
	default:
		break;
	case ADD:
		return help_add;
	case DELETE:
		return help_delete;
	case LIST:
		return help_list;
	case QUERY:
		return help_query;
	case SET_FPS:
		return help_setfps;
	case GET_FPS:
		return help_getfps;
	case SET_CAPS:
		return help_setcaps;
	case GET_CAPS:
		return help_getcaps;
	case SET_TIMEOUTIMAGE:
		return help_settimeoutimage;
	}
	return help_none;
}

static void help(const char *name, int status)
{
	t_command cmd;
	dprintf(2, "Usage: %s [OPTIONS]\n", name);
	for (cmd = ADD; cmd < _UNKNOWN; cmd++) {
		t_help hlp = get_help(cmd);
		if (help_none == hlp)
			continue;
		dprintf(2, "  or:  %s ", name);
		hlp(0, 0);
	}

	dprintf(2, "\nManage v4l2 loopback devices.");
	dprintf(2,
		"\n"
		"\nThe general invocation uses a verb (like 'add' or 'delete') that defines"
		"\nan action to be executed. Each verb has their own options and arguments."
		"\n"
		"\nOptions:"
		"\n\t-h, -?, --help: print this help and exit"
		"\n\t-v, --version: print version and exit"
		"\n\n");

	/* long helps */
	dprintf(2, "*Verbs and their arguments*\n");
	for (cmd = ADD; cmd < _UNKNOWN; cmd++) {
		t_help hlp = get_help(cmd);
		if (help_none == hlp)
			continue;
		hlp("v4l2loopback-ctl", 1);
		dprintf(2, "\n\n");
	}

	dprintf(2,
		"*Reporting Bugs*\n"
		"\nIssue tracker: https://github.com/umlaeute/v4l2loopback/issues"
		"\nSecurity Issue tracker: https://git.iem.at/zmoelnig/v4l2loopback/-/issues"
		"\n\n");
	exit(status);
}
static void usage(const char *name)
{
	help(name, 1);
}
static void usage_topic(const char *name, t_command cmd, int argc, char **argv)
{
	t_help hlp = get_help(cmd);
	if (help_none == hlp)
		usage(name);
	else
		hlp(name, 2);
	dprintf(2, "\n");
	exit(1);
}

static const char *my_realpath(const char *path, char *resolved_path)
{
	char *str = realpath(path, resolved_path);
	return str ? str : path;
}
static int parse_device(const char *devicename_)
{
	char devicenamebuf[4096];
	const char *devicename = my_realpath(devicename_, devicenamebuf);
	int ret = strncmp(devicename, "/dev/video", 10);
	const char *device = (ret) ? devicename : (devicename + 10);
	char *endptr = 0;
	int dev = strtol(device, &endptr, 10);
	if (!*endptr)
		return dev;

	return -1;
}

static void print_conf(struct v4l2_loopback_config *cfg, int escape_level)
{
	int output_nr, capture_nr;
	MARK();
	if (!cfg) {
		printf("configuration: %p\n", cfg);
		return;
	}
	output_nr = capture_nr = cfg->output_nr;
#ifdef SPLIT_DEVICES
	capture_nr = cfg->capture_nr;
#endif
	MARK();
	printf("\tcapture_device#  : %d"
	       "\n\toutput_device#   : %d"
	       "\n\tcard_label       : ",
	       capture_nr, output_nr);
	printf_raw(cfg->card_label, escape_level);
	printf("\n\tmin_width        : %d"
	       "\n\tmax_width        : %d"
	       "\n\tmin_height       : %d"
	       "\n\tmax_height       : %d"
	       "\n\tannounce_all_caps: %d"
	       "\n\tmax_buffers      : %d"
	       "\n\tmax_openers      : %d"
	       "\n\tdebug            : %d"
	       "\n",
	       cfg->min_width, cfg->max_width, cfg->min_height, cfg->max_height,
	       cfg->announce_all_caps, cfg->max_buffers, cfg->max_openers,
	       cfg->debug);
	MARK();
}

static struct v4l2_loopback_config *
make_conf(struct v4l2_loopback_config *cfg, const char *label, int min_width,
	  int max_width, int min_height, int max_height, int exclusive_caps,
	  int buffers, int openers, int capture_device, int output_device)
{
	if (!cfg)
		return 0;
	/* check if at least one of the args are non-default */
	if (!label && min_width <= 0 && max_width <= 0 && min_height <= 0 &&
	    max_height <= 0 && exclusive_caps < 0 && buffers <= 0 &&
	    openers <= 0 && capture_device < 0 && output_device < 0)
		return 0;
#ifdef SPLIT_DEVICES
	cfg->capture_nr = capture_device;
#endif
	cfg->output_nr = output_device;
	cfg->card_label[0] = 0;
	if (label)
		snprintf(cfg->card_label, 32, "%s", label);
	cfg->min_width = (min_width < 0) ? 0 : min_width;
	cfg->max_width = (max_width < 0) ? 0 : max_width;
	cfg->min_height = (min_height < 0) ? 0 : min_height;
	cfg->max_height = (max_height < 0) ? 0 : max_height;
	cfg->announce_all_caps = (exclusive_caps < 0) ? -1 : !exclusive_caps;
	cfg->max_buffers = buffers;
	cfg->max_openers = openers;
	cfg->debug = 0;
	return cfg;
}

static int add_device(int fd, struct v4l2_loopback_config *cfg, int verbose)
{
	int err = 0;
	MARK();
	int ret = ioctl(fd, V4L2LOOPBACK_CTL_ADD, cfg);
	MARK();
	if (ret < 0) {
		err = errno;
		perror("failed to create device");
		return err;
	}
	MARK();

	printf("/dev/video%d\n", ret);

	if (verbose > 0) {
		MARK();
		struct v4l2_loopback_config config;
		memset(&config, 0, sizeof(config));
		config.output_nr = ret;
#ifdef SPLIT_DEVICES
		config.capture_nr = ret;
#endif
		ret = ioctl(fd, V4L2LOOPBACK_CTL_QUERY, &config);
		if (ret < 0) {
			err = errno;
			perror("failed querying newly added device");
		}
		MARK();
		print_conf(&config, 0);
		MARK();
	}
	return err;
}

static int delete_device(int fd, const char *devicename)
{
	int err = 0;
	int dev = parse_device(devicename);
	if (dev < 0) {
		dprintf(2, "ignoring illegal devicename '%s'\n", devicename);
		return 1;
	}
	if (ioctl(fd, V4L2LOOPBACK_CTL_REMOVE, dev) < 0) {
		err = errno;
		perror(devicename);
	}

	return err;
}

static int query_device(int fd, const char *devicename, int escape)
{
	int err;
	struct v4l2_loopback_config config;
	int dev = parse_device(devicename);
	if (dev < 0) {
		dprintf(2, "ignoring illegal devicename '%s'\n", devicename);
		return 1;
	}

	memset(&config, 0, sizeof(config));
	config.output_nr = dev;
#ifdef SPLIT_DEVICES
	config.capture_nr = dev;
#endif
	err = ioctl(fd, V4L2LOOPBACK_CTL_QUERY, &config);
	if (err)
		perror("query failed");
	else {
		printf("%s\n", devicename);
		print_conf(&config, escape);
		return 0;
	}
	return err;
}
static int list_devices(int fd, int escape)
{
	struct devnode_ {
		int output, capture;
		char name[32];
	} *devices = 0;
	size_t numdevices = 0, i;
	glob_t globbuf = { 0 };
	int output_nr, capture_nr;
	glob("/sys/devices/virtual/video4linux/video*", GLOB_ONLYDIR, 0,
	     &globbuf);
	if (globbuf.gl_pathc) {
		devices = malloc(globbuf.gl_pathc * sizeof(*devices));
	}
	for (i = 0; i < globbuf.gl_pathc; i++) {
		size_t j;
		struct v4l2_loopback_config config = { 0 };
		int dev = -1;
		char *endptr;
		struct stat sb;
		const char *path = globbuf.gl_pathv[i];
		if (lstat(path, &sb)) {
			//perror("stat");
			continue;
		}
		if (!S_ISDIR(sb.st_mode)) {
			//dprintf(2, "not a directory\n");
			continue;
		}
		dev = strtol(path + 38, &endptr, 10);
		if (*endptr) {
			//dprintf(2, "unable to parse device-name\n");
			continue;
		}
		/* check if this is a loopback device */
		config.output_nr = dev;
#ifdef SPLIT_DEVICES
		config.capture_nr = -1;
		if (ioctl(fd, V4L2LOOPBACK_CTL_QUERY, &config)) {
			memset(&config, 0, sizeof(config));
			config.output_nr = -1;
			config.capture_nr = dev;
			if (ioctl(fd, V4L2LOOPBACK_CTL_QUERY, &config)) {
				//dprintf(2, "not a loopback device\n");
				continue;
			}
		}
		capture_nr = config.capture_nr;
#else
		if (ioctl(fd, V4L2LOOPBACK_CTL_QUERY, &config)) {
			//dprintf(2, "not a loopback device\n");
			continue;
		}
		capture_nr = config.output_nr;
#endif
		output_nr = config.output_nr;
		/* check if we already have this device */
		for (j = 0; j < numdevices; j++) {
			if ((devices[j].output == output_nr) &&
			    (devices[j].capture == capture_nr)) {
				//dprintf(2, "duplicate device\n");
				output_nr = capture_nr = -1;
				break;
			}
		}
		if ((output_nr < 0) || (capture_nr < 0))
			continue;

		devices[numdevices].output = output_nr;
		devices[numdevices].capture = capture_nr;
		snprintf(devices[numdevices].name,
			 sizeof(devices[numdevices].name), "%s",
			 config.card_label);
		numdevices++;
	}
	if (numdevices) {
		dprintf(2, "OUTPUT       \tCAPTURE      \tNAME\n");
	} else {
		dprintf(2, "no loopback devices found\n");
	}
	for (i = 0; i < numdevices; i++) {
		const char *str = devices[i].name;
		printf("/dev/video%-3d\t/dev/video%-3d\t", devices[i].output,
		       devices[i].capture);
		printf_raw(str, escape);
		printf("\n");
	}
	globfree(&globbuf);
	free(devices);
	return 0;
}
static int open_videodevice(const char *devicename, int mode)
{
	int fd = open(devicename, mode);
	if (fd < 0) {
		int devnr = parse_device(devicename);
		if (devnr >= 0) {
			char devname[100];
			snprintf(devname, 99, "/dev/video%d", devnr);
			devname[99] = 0;
			fd = open(devname, mode);
		}
	}
	return fd;
}
static int open_controldevice()
{
	int fd = open(CONTROLDEVICE, 0);
	if (fd < 0) {
		perror("unable to open control device '" CONTROLDEVICE "'");
		exit(1);
	}
	return fd;
}

static int open_sysfs_file(const char *devicename, const char *filename,
			   int flags)
{
	int fd = -1;
	char sysdev[100];
	int dev = parse_device(devicename);
	if (dev < 0) {
		dprintf(2, "ignoring illegal devicename '%s'\n", devicename);
		return -1;
	}
	snprintf(sysdev, sizeof(sysdev) - 1,
		 "/sys/devices/virtual/video4linux/video%d/%s", dev, filename);
	sysdev[sizeof(sysdev) - 1] = 0;
	fd = open(sysdev, flags);
	if (fd < 0) {
		perror(sysdev);
		return -1;
	}
	//dprintf(2, "%s\n", sysdev);
	return fd;
}

static int parse_fps(const char *fps, int *numerator, int *denominator)
{
	int num = 0;
	int denom = 1;
	if (sscanf(fps, "%d/%d", &num, &denom) <= 0) {
		return 1;
	}
	if (numerator)
		*numerator = num;
	if (denominator)
		*denominator = denom;
	return 0;
}
static int is_fps(const char *fps)
{
	return parse_fps(fps, 0, 0);
}

static int set_fps(const char *devicename, const char *fps)
{
	int result = 1;
	char _fps[100];
	int fd = open_sysfs_file(devicename, "format", O_WRONLY);
	if (fd < 0)
		return 1;
	snprintf(_fps, sizeof(_fps) - 1, "@%s", fps);
	_fps[sizeof(_fps) - 1] = 0;

	if (write(fd, _fps, strnlen(_fps, sizeof(_fps))) < 0) {
		perror("failed to set fps");
		goto done;
	}

	result = 0;
done:
	close(fd);
	return result;
}

typedef struct _caps {
	unsigned int fourcc;
	int width, height;
	int fps_num, fps_denom;
} t_caps;

static void print_caps(t_caps *caps)
{
	char fourcc[4];
	if (!caps) {
		dprintf(2, "no caps\n");
		return;
	}
	dprintf(2, "FOURCC : %.4s\n", fourcc2str(caps->fourcc, fourcc));
	dprintf(2, "dimen  : %dx%d\n", caps->width, caps->height);
	dprintf(2, "fps    : %d/%d\n", caps->fps_num, caps->fps_denom);
}
static int parse_caps(const char *buffer, t_caps *caps)
{
	char fourcc[5];
	memset(caps, 0, sizeof(*caps));
	memset(fourcc, 0, sizeof(*fourcc));
	caps->fps_denom = 1;

	if (!(buffer && *buffer))
		return 1;

	if (sscanf(buffer, "%4c:%dx%d@%d/%d", fourcc, &caps->width,
		   &caps->height, &caps->fps_num, &caps->fps_denom) <= 0) {
	}
	caps->fourcc = str2fourcc(fourcc);
	return (0 == caps->fourcc);
}
static int read_caps(const char *devicename, t_caps *caps)
{
	int result = 1;
	char _caps[100];
	int len;
	int fd = open_sysfs_file(devicename, "format", O_RDONLY);
	if (fd < 0)
		return 1;

	len = read(fd, _caps, 100);
	if (len <= 0) {
		if (len)
			perror("failed to read fps");
		goto done;
	}
	_caps[100 - 1] = 0;
	if (caps) {
		if (parse_caps(_caps, caps)) {
			dprintf(2, "unable to parse format '%s'\n", _caps);
			goto done;
		}
	}
	result = 0;
done:
	close(fd);
	return result;
}

static int get_fps(const char *devicename)
{
	t_caps caps;
	struct v4l2_streamparm param;
	int fd = -1;
	int num = -1, denom = -1;
	int ret = 0;

	if (!read_caps(devicename, &caps)) {
		num = caps.fps_num;
		denom = caps.fps_denom;
		goto done;
	}

	/* get the framerate via ctls */
	fd = open_videodevice(devicename, O_RDWR);
	if (fd < 0) {
		ret = 1;
		goto done;
	}

	memset(&param, 0, sizeof(param));
	param.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	if (ioctl(fd, VIDIOC_G_PARM, &param) == 0) {
		const struct v4l2_fract *tf = &param.parm.output.timeperframe;
		num = tf->numerator;
		denom = tf->denominator;
		goto done;
	}

	memset(&param, 0, sizeof(param));
	param.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (ioctl(fd, VIDIOC_G_PARM, &param) == 0) {
		const struct v4l2_fract *tf = &param.parm.output.timeperframe;
		num = tf->numerator;
		denom = tf->denominator;
		goto done;
	}

	ret = 1;
done:
	if (fd >= 0)
		close(fd);
	printf("%d/%d\n", num, denom);
	return ret;
}
static int set_caps(const char *devicename, const char *capsstring)
{
	int result = 1;
	int fd = open_videodevice(devicename, O_RDWR);
	struct v4l2_format vid_format;
	struct v4l2_capability vid_caps;
	t_caps caps;

	/* now open up the device */
	if (fd < 0)
		goto done;

	if (!strncmp("any", capsstring, 4)) {
		/* skip caps-parsing */
	} else if (!strncmp("video/", capsstring, 6)) {
		dprintf(2,
			"ERROR: GStreamer-style caps are no longer supported!\n");
		dprintf(2,
			"ERROR: use '<FOURCC>:<width>x<height>[@<fps>] instead\n");
		dprintf(2,
			"       e.g. 'UYVY:640x480@30/1' or 'RGBA:1024x768'\n");
		goto done;
	} else if (parse_caps(capsstring, &caps)) {
		dprintf(2, "unable to parse format '%s'\n", capsstring);
		goto done;
	}
	//print_caps(&caps);

	/* check whether this is actually a video-device */
	if (ioctl(fd, VIDIOC_QUERYCAP, &vid_caps) == -1) {
		perror("VIDIOC_QUERYCAP");
		goto done;
	}

	if (!strncmp("any", capsstring, 4)) {
		set_control_i(fd, "keep_format", 0);
		//set_control_i(fd, "sustain_framerate", 0);
		result = 0;
		goto done;
	}

	/* try to get the default values for the format first */
	memset(&vid_format, 0, sizeof(vid_format));

	vid_format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	if (ioctl(fd, VIDIOC_G_FMT, &vid_format) == -1) {
		perror("VIDIOC_G_FMT");
	}

	/* and set those caps that we have */
	if (caps.width)
		vid_format.fmt.pix.width = caps.width;
	if (caps.height)
		vid_format.fmt.pix.height = caps.height;
	if (caps.fourcc)
		vid_format.fmt.pix.pixelformat = caps.fourcc;

	if (ioctl(fd, VIDIOC_S_FMT, &vid_format) == -1) {
		perror("unable to set requested format");
		goto done;
	}

	set_control_i(fd, "keep_format", 1);

	/* finally, try setting the fps */
	if (caps.fps_num && caps.fps_denom) {
		char fps[100];
		int didit;
		snprintf(fps, 100, "%d/%d", caps.fps_num, caps.fps_denom);
		didit = set_fps(devicename, fps);
		if (!didit) {
			set_control_i(fd, "sustain_framerate", 1);
		}
		close(fd);
		fd = -1;
		return didit;
	}

	result = 0;
done:
	if (fd >= 0)
		close(fd);
	return result;
}
static int get_caps(const char *devicename)
{
	int format = 0;
	t_caps caps;
	char fourcc[4];
	if (read_caps(devicename, &caps))
		return 1;
	switch (format) {
	default:
		printf("%.4s:%dx%d@%d/%d\n", fourcc2str(caps.fourcc, fourcc),
		       caps.width, caps.height, caps.fps_num, caps.fps_denom);
		break;
	case 1: /* GStreamer-1.0 */

		/* FOURCC is different everywhere... */
		switch (caps.fourcc) {
		default:
			break;
		case 0x56595559: /* YUYV */
			caps.fourcc = str2fourcc("YUY2");
			break;
		}

		printf("video/x-raw,format=%.4s,width=%d,height=%d,framerate=%d/%d\n",
		       fourcc2str(caps.fourcc, fourcc), caps.width, caps.height,
		       caps.fps_num, caps.fps_denom);
		break;
	}
	return 0;
}
static int set_timeoutimage(const char *devicename, const char *imagefile,
			    int timeout, int verbose)
{
	int err = 0;
	int fd = -1;
	char imagearg[4096], imagefile2[4096], devicearg[4096];
	char *args[] = { "gst-launch-1.0",
			 "uridecodebin",
			 0,
			 "!",
			 "videoconvert",
			 "!",
			 "videoscale",
			 "!",
			 "imagefreeze",
			 "!",
			 "identity",
			 "eos-after=2",
			 "drop-allocation=1",
			 "!",
			 "v4l2sink",
			 "show-preroll-frame=false",
			 0,
			 0 };
	if (verbose)
		printf("set-timeout-image '%s' for '%s' with %dms timeout\n",
		       imagefile, devicename, timeout);

	snprintf(imagearg, 4096, "uri=file://%s",
		 realpath(imagefile, imagefile2));
	snprintf(devicearg, 4096, "device=%s", devicename);
	imagearg[4095] = devicearg[4095] = 0;
	args[2] = imagearg;
	args[16] = devicearg;

	fd = open_videodevice(devicename, O_RDWR);
	if (fd >= 0) {
		dprintf(2, "v4l2-ctl -d %s -c timeout_image_io=1\n",
			devicename);
		set_control_i(fd, "timeout_image_io", 1);
		close(fd);
	} else {
		err = errno;
	}

	if (verbose > 1) {
		char **ap = args;
		while (*ap) {
			dprintf(2, "%s", *ap);
			if (*ap++)
				dprintf(2, " ");
			else
				dprintf(2, "\n");
		}
	}

	dprintf(2,
		"v======================================================================v\n");
	if (my_execv(args)) {
		dprintf(2, "ERROR: setting time-out image failed\n");
		err = 1;
	}
	dprintf(2,
		"^======================================================================^\n");

	fd = open_videodevice(devicename, O_RDWR);
	if (fd >= 0) {
		/* finally check the timeout */
		if (timeout < 0) {
			timeout = get_control_i(fd, "timeout");
		} else {
			dprintf(2, "v4l2-ctl -d %s -c timeout=%d\n", devicename,
				timeout);
			timeout = set_control_i(fd, "timeout", timeout);
		}
		if (timeout <= 0) {
			dprintf(2,
				"Timeout is currently disabled; you can set it to some positive value, e.g.:\n");
			dprintf(2, "    $  v4l2-ctl -d %s -c timeout=3000\n",
				devicename);
		}

		close(fd);
	} else {
		err = errno;
	}
	return err;
}

static t_command get_command(const char *command)
{
	if (!strncmp(command, "-h", 3))
		return HELP;
	if (!strncmp(command, "-?", 3))
		return HELP;
	if (!strncmp(command, "--help", 7))
		return HELP;
	if (!strncmp(command, "-v", 3))
		return VERSION;
	if (!strncmp(command, "--version", 10))
		return VERSION;
	if (!strncmp(command, "list", 5))
		return LIST;
	if (!strncmp(command, "add", 4))
		return ADD;
	if (!strncmp(command, "del", 3)) /* also allow delete */
		return DELETE;
	if (!strncmp(command, "query", 6))
		return QUERY;
	if (!strncmp(command, "set-fps", 8))
		return SET_FPS;
	if (!strncmp(command, "get-fps", 8))
		return GET_FPS;
	if (!strncmp(command, "set-caps", 9))
		return SET_CAPS;
	if (!strncmp(command, "get-caps", 9))
		return GET_CAPS;
	if (!strncmp(command, "set-timeout-image", 18))
		return SET_TIMEOUTIMAGE;
	if (!strncmp(command, "moo", 10))
		return MOO;
	return _UNKNOWN;
}

typedef int (*t_argcheck)(const char *);
static int called_deprecated(const char *device, const char *argument,
			     const char *programname, const char *cmdname,
			     const char *argname, t_argcheck argcheck)
{
	/* check if <device> does not look like a device, but <argument> does
	 * if so, assume that the user swapped the two */
	/* if the <device> looks about right, optionally do some extra
	 * <argument>-check, to see if it can be used
	 */

	int deviceswapped = 0;
	int argswapped = 0;

	if (argcheck)
		argswapped =
			((argcheck(argument) != 0) && (argcheck(device) == 0));

	if (!argswapped)
		deviceswapped = (parse_device(device) < 0 &&
				 parse_device(argument) >= 0);

	if (argswapped || deviceswapped) {
		dprintf(2, "WARNING: '%s %s <%s> <image>' is deprecated!\n",
			programname, cmdname, argname);
		dprintf(2, "WARNING: use '%s %s <device> <%s>' instead.\n",
			programname, cmdname, argname);
		return 1;
	}
	return 0;
}

static int do_defaultargs(const char *progname, t_command cmd, int argc,
			  char **argv)
{
	static const char options_short[] = "?h";
	static const struct option options_long[] = {
		{ "help", no_argument, NULL, 'h' }, { 0, 0, 0, 0 }
	};
	for (;;) {
		int c;
		int idx;
		c = getopt_long(argc - 1, argv + 1, options_short, options_long,
				&idx);
		if (-1 == c)
			break;
		switch (c) {
		case 'h':
			usage_topic(argv[0], cmd, argc - 1, argv + 1);
			exit(0);
		default:
			usage_topic(argv[0], cmd, argc - 1, argv + 1);
			exit(1);
		}
	}
	return optind;
}

int main(int argc, char **argv)
{
	const char *progname = argv[0];
	int i;
	int fd = -1;
	int verbose = 0;
	t_command cmd;

	char *label = 0;
	int min_width = -1;
	int max_width = -1;
	int min_height = -1;
	int max_height = -1;
	int exclusive_caps = -1;
	int buffers = -1;
	int openers = -1;
	int escape_strings = 0;

	int ret = 0;

	static const char add_options_short[] = "?vn:w:h:x:b:o:";
	static const struct option add_options_long[] = {
		{ "help", no_argument, NULL, '?' },
		{ "verbose", no_argument, NULL, 'v' },
		{ "name", required_argument, NULL, 'n' },
		{ "min-width", required_argument, NULL, 'w' + 0xFFFF },
		{ "max-width", required_argument, NULL, 'w' },
		{ "min-height", required_argument, NULL, 'h' + 0xFFFF },
		{ "max-height", required_argument, NULL, 'h' },
		{ "exclusive-caps", required_argument, NULL, 'x' },
		{ "buffers", required_argument, NULL, 'b' },
		{ "max-openers", required_argument, NULL, 'o' },
		{ 0, 0, 0, 0 }
	};
	static const char list_options_short[] = "?he";
	static const struct option list_options_long[] = {
		{ "help", no_argument, NULL, 'h' },
		{ "escape", no_argument, NULL, 'e' },
		{ 0, 0, 0, 0 }
	};
	static const char timeoutimg_options_short[] = "?ht:v";
	static const struct option timeoutimg_options_long[] = {
		{ "help", no_argument, NULL, 'h' },
		{ "timeout", required_argument, NULL, 't' },
		{ "verbose", no_argument, NULL, 'v' },
		{ 0, 0, 0, 0 }
	};

	if (argc < 2)
		usage(progname);
	cmd = get_command(argv[1]);
	if (_UNKNOWN == cmd) {
		dprintf(2, "unknown command '%s'\n\n", argv[1]);
		usage(progname);
		return 1;
	}
	argc--;
	argv++;
	switch (cmd) {
	case HELP:
		help(progname, 0);
		break;
	case LIST:
		for (;;) {
			int c;
			int idx;
			c = getopt_long(argc, argv, list_options_short,
					list_options_long, &idx);
			if (-1 == c)
				break;
			switch (c) {
			case 'e':
				escape_strings++;
				break;
			default:
				usage_topic(progname, cmd, argc - 1, argv + 1);
				return 1;
			}
		}
		argc -= optind;
		argv += optind;

		if (argc) {
			dprintf(2, "'list' does not take any arguments\n");
			return 1;
		}
		fd = open_controldevice();
		if (fd >= 0)
			list_devices(fd, escape_strings);
		else
			return 1;
		break;
	case ADD:
		for (;;) {
			int c;
			int idx;
			c = getopt_long(argc, argv, add_options_short,
					add_options_long, &idx);
			if (-1 == c)
				break;
			switch (c) {
			case 'v':
				verbose++;
				break;
			case 'n':
				label = optarg;
				break;
			case 'w' + 0xFFFF:
				min_width = my_atoi("min_width", optarg);
				break;
			case 'h' + 0xFFFF:
				min_height = my_atoi("min_height", optarg);
				break;
			case 'w':
				max_width = my_atoi("max_width", optarg);
				break;
			case 'h':
				max_height = my_atoi("max_height", optarg);
				break;
			case 'x':
				exclusive_caps =
					my_atoi("exclusive_caps", optarg);
				break;
			case 'b':
				buffers = my_atoi("buffers", optarg);
				break;
			case 'o':
				openers = my_atoi("openers", optarg);
				break;
			default:
				usage_topic(progname, cmd, argc - 1, argv + 1);
				return 1;
			}
		}
		argc -= optind;
		argv += optind;
		fd = open_controldevice();
		if (min_width > max_width && max_width > 0) {
			dprintf(2,
				"min_width (%d) must not be greater than max_width (%d)\n",
				min_width, max_width);
			return 1;
		}
		if (min_height > max_height && max_height > 0) {
			dprintf(2,
				"min_height (%d) must not be greater than max_height (%d)\n",
				min_height, max_height);
			return 1;
		}
		do {
			struct v4l2_loopback_config cfg;
			int capture_nr = -1, output_nr = -1;
			switch (argc) {
			case 0:
				/* no device given: pick some */
				break;
			case 2:
				/* two devices given: capture_device and output_device */
				output_nr = parse_device(argv[0]);
				capture_nr = parse_device(argv[1]);
#ifndef SPLIT_DEVICES
				if (capture_nr != output_nr)
					dprintf(2,
						"split output/capture devices currently not supported...ignoring capture device\n");
				capture_nr = output_nr;
#endif
				break;
			case 1:
				/* single device given: use it for both input and output */
				capture_nr = output_nr = parse_device(argv[0]);
				break;
			default:
				usage_topic(progname, cmd, argc, argv);
				return 1;
			}
			ret = add_device(fd,
					 make_conf(&cfg, label, min_width,
						   max_width, min_height,
						   max_height, exclusive_caps,
						   buffers, openers, capture_nr,
						   output_nr),
					 verbose);
		} while (0);
		break;
	case DELETE:
		optind = do_defaultargs(progname, cmd, argc, argv);
		argc -= optind;
		argv += optind;

		if (!argc)
			usage_topic(progname, cmd, argc, argv);
		fd = open_controldevice();
		for (i = 0; i < argc; i++) {
			int err = delete_device(fd, argv[i]);
			if (err)
				ret = err;
		}
		break;
	case QUERY:
		for (;;) {
			int c;
			int idx;
			c = getopt_long(argc, argv, list_options_short,
					list_options_long, &idx);
			if (-1 == c)
				break;
			switch (c) {
			case 'e':
				escape_strings++;
				break;
			default:
				usage_topic(progname, cmd, argc - 1, argv + 1);
				return 1;
			}
		}
		argc -= optind;
		argv += optind;

		if (!argc)
			usage_topic(progname, cmd, argc, argv);
		fd = open_controldevice();
		for (i = 0; i < argc; i++) {
			ret += query_device(fd, argv[i], escape_strings);
		}
		ret = (ret > 0);
		break;
	case SET_FPS:
		optind = do_defaultargs(progname, cmd, argc, argv);
		argc -= optind;
		argv += optind;
		if (argc != 2)
			usage_topic(progname, cmd, argc, argv);
		if (called_deprecated(argv[0], argv[1], progname, "set-fps",
				      "fps", is_fps)) {
			ret = set_fps(argv[1], argv[0]);
		} else
			ret = set_fps(argv[0], argv[1]);
		break;
	case GET_FPS:
		optind = do_defaultargs(progname, cmd, argc, argv);
		argc -= optind;
		argv += optind;
		if (argc != 1)
			usage_topic(progname, cmd, argc, argv);
		ret = get_fps(argv[0]);
		break;
	case SET_CAPS:
		optind = do_defaultargs(progname, cmd, argc, argv);
		argc -= optind;
		argv += optind;
		if (argc != 2)
			usage_topic(progname, cmd, argc, argv);
		if (called_deprecated(argv[0], argv[1], progname, "set-caps",
				      "caps", 0)) {
			ret = set_caps(argv[1], argv[0]);
		} else {
			ret = set_caps(argv[0], argv[1]);
		}
		break;
	case GET_CAPS:
		optind = do_defaultargs(progname, cmd, argc, argv);
		argc -= optind;
		argv += optind;
		if (argc != 1)
			usage_topic(progname, cmd, argc, argv);
		ret = get_caps(argv[0]);
		break;
	case SET_TIMEOUTIMAGE:
		if ((3 == argc) && (strncmp("-t", argv[1], 3)) &&
		    (strncmp("--timeout", argv[1], 10)) &&
		    (called_deprecated(argv[1], argv[2], progname,
				       "set-timeout-image", "image", 0))) {
			ret = set_timeoutimage(argv[2], argv[1], -1, verbose);
		} else {
			int timeout = -1;
			for (;;) {
				int c, idx;
				c = getopt_long(argc, argv,
						timeoutimg_options_short,
						timeoutimg_options_long, &idx);
				if (-1 == c)
					break;
				switch (c) {
				case 't':
					timeout = my_atoi("timeout", optarg);
					break;
				case 'v':
					verbose++;
					break;
				default:
					usage_topic(progname, cmd, argc, argv);
				}
			}
			argc -= optind;
			argv += optind;
			if (argc != 2)
				usage_topic(progname, cmd, argc, argv);
			ret = set_timeoutimage(argv[0], argv[1], timeout,
					       verbose);
		}
		break;
	case VERSION:
#ifdef SNAPSHOT_VERSION
		printf("%s v%s\n", progname, SNAPSHOT_VERSION);
#else
		printf("%s v%d.%d.%d\n", progname, V4L2LOOPBACK_VERSION_MAJOR,
		       V4L2LOOPBACK_VERSION_MINOR, V4L2LOOPBACK_VERSION_BUGFIX);
#endif
		fd = open("/sys/module/v4l2loopback/version", O_RDONLY);
		if (fd >= 0) {
			char buf[1024];
			int len = read(fd, buf, sizeof(buf));
			if (len > 0) {
				if (len < sizeof(buf))
					buf[len] = 0;
				printf("v4l2loopback sysfs v%s", buf);
			}
			close(fd);
		}
		fd = open_controldevice();
		if (fd >= 0) {
			__u32 version = 0;
			if (ioctl(fd, V4L2LOOPBACK_CTL_VERSION, &version) ==
			    0) {
				printf("v4l2loopback module v%d.%d.%d\n",
				       (version >> 16) & 0xff,
				       (version >> 8) & 0xff,
				       (version >> 0) & 0xff);
			}
		}
		break;
	default:
		dprintf(2, "not implemented: '%s'\n", argv[0]);
		break;
	}

	if (fd >= 0)
		close(fd);

	return ret;
}
