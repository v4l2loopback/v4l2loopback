ifneq ($(wildcard .gitversion),)
# building a snapshot version
V4L2LOOPBACK_SNAPSHOT_VERSION=$(patsubst v%,%,$(shell git describe --always --dirty 2>/dev/null || shell git describe --always 2>/dev/null || echo snapshot))
override KCPPFLAGS += -DSNAPSHOT_VERSION='"$(V4L2LOOPBACK_SNAPSHOT_VERSION)"'
endif

include Kbuild
ifeq ($(KBUILD_MODULES),)

KERNELRELEASE	?= `uname -r`
KERNEL_DIR	?= /lib/modules/$(KERNELRELEASE)/build
PWD		:= $(shell pwd)

PREFIX ?= /usr/local
BINDIR  = $(PREFIX)/bin
INCLUDEDIR = $(PREFIX)/include
MANDIR  = $(PREFIX)/share/man
MAN1DIR = $(MANDIR)/man1
INSTALL = install
INSTALL_PROGRAM = $(INSTALL) -p -m 755
INSTALL_DIR     = $(INSTALL) -p -m 755 -d
INSTALL_DATA    = $(INSTALL) -m 644

MODULE_OPTIONS = devices=2

##########################################
# note on build targets
#
# module-assistant makes some assumptions about targets, namely
#  <modulename>: must be present and build the module <modulename>
#                <modulename>.ko is not enough
# install: must be present (and should only install the module)
#
# we therefore make <modulename> a .PHONY alias to <modulename>.ko
# and remove utils-installation from 'install'
# call 'make install-all' if you want to install everything
##########################################


.PHONY: all install clean distclean
.PHONY: install-all install-extra install-utils install-man install-headers
.PHONY: modprobe v4l2loopback

# we don't control the .ko file dependencies, as it is done by kernel
# makefiles. therefore v4l2loopback.ko is a phony target actually
.PHONY: v4l2loopback.ko utils

all: v4l2loopback.ko utils

v4l2loopback: v4l2loopback.ko
v4l2loopback.ko:
	@echo "Building v4l2-loopback driver..."
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) KCPPFLAGS="$(KCPPFLAGS)" modules

install-all: install install-extra
install:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules_install
	@echo ""
	@echo "SUCCESS (if you got 'SSL errors' above, you can safely ignore them)"
	@echo ""

install-extra: install-utils install-man install-headers
install-utils: utils/v4l2loopback-ctl
	$(INSTALL_DIR) "$(DESTDIR)$(BINDIR)"
	$(INSTALL_PROGRAM) $< "$(DESTDIR)$(BINDIR)"

install-man: man/v4l2loopback-ctl.1
	$(INSTALL_DIR) "$(DESTDIR)$(MAN1DIR)"
	$(INSTALL_DATA) $< "$(DESTDIR)$(MAN1DIR)"

install-headers: v4l2loopback.h
	$(INSTALL_DIR) "$(DESTDIR)$(INCLUDEDIR)/linux"
	$(INSTALL_DATA) $< "$(DESTDIR)$(INCLUDEDIR)/linux"

clean:
	rm -f *~
	rm -f Module.symvers Module.markers modules.order
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) clean
	-$(MAKE) -C utils clean

distclean: clean
	rm -f man/v4l2loopback-ctl.1

modprobe: v4l2loopback.ko
	-sudo chmod a+r $<
	-sudo modprobe videodev
	-sudo rmmod $<
	sudo insmod ./$< $(MODULE_OPTIONS)

man/v4l2loopback-ctl.1: utils/v4l2loopback-ctl
	help2man -N --name "control v4l2 loopback devices" \
		--no-discard-stderr --help-option=-h --version-option=-v \
		$^ > $@

utils: utils/v4l2loopback-ctl
utils/v4l2loopback-ctl: utils/v4l2loopback-ctl.c v4l2loopback.h
	$(MAKE) -C utils V4L2LOOPBACK_SNAPSHOT_VERSION=$(V4L2LOOPBACK_SNAPSHOT_VERSION)

.clang-format:
	curl "https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/plain/.clang-format" > $@

.PHONY: clang-format
clang-format: .clang-format
	clang-format -i *.c *.h utils/*.c

.PHONY: sign
# try to read the default certificate/key from the dkms config
dkms_framework=/etc/dkms/framework.conf /etc/dkms/framework.conf.d/*.conf
KBUILD_SIGN_KEY=$(patsubst mok_signing_key=%,%,$(lastword $(shell grep -h -E "^[[:space:]]*mok_signing_key=" $(dkms_framework))))
KBUILD_SIGN_CERT=$(patsubst mok_certificate=%,%,$(lastword $(shell grep -h -E "^[[:space:]]*mok_certificate=" $(dkms_framework))))

ifeq ($(KBUILD_SIGN_PIN),)
define usage_kbuildsignpin
$(info )
$(info ++++++ If your certificate requires a password, pass it via the KBUILD_SIGN_PIN env-var!)
$(info ++++++ E.g. using 'export KBUILD_SIGN_PIN; read -s -p "Passphrase for signing key $(KBUILD_SIGN_KEY): " KBUILD_SIGN_PIN; sudo --preserve-env=KBUILD_SIGN_PIN make sign')
$(info )
endef
endif

define usage_kbuildsign
sign: v4l2loopback.ko
	$(info )
	$(info ++++++ To sign the $< module, you must set KBUILD_SIGN_KEY/KBUILD_SIGN_CERT to point to the signing key/certificate!)
	$(info ++++++ For your convenience, we try to read these variables as 'mok_signing_key' resp. 'mok_certificate' from $(dkms_framework))
	$(call usage_kbuildsignpin)
endef

ifeq ($(wildcard $(KBUILD_SIGN_KEY)),)
$(call usage_kbuildsign)
else ifeq ($(wildcard $(KBUILD_SIGN_CERT)),)
$(call usage_kbuildsign)
else
sign: v4l2loopback.ko
	$(call usage_kbuildsignpin)
	"$(KERNEL_DIR)"/scripts/sign-file sha256 $(KBUILD_SIGN_KEY) $(KBUILD_SIGN_CERT) $<
endif

endif # !kbuild
