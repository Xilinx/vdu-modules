SRC := $(shell pwd)

ifeq ($(O),)
	include $(KERNEL_SRC)/.config
else
	include $(O)/.config
endif

obj-m	+= common/
obj-m	+= al5d/

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) O=$(O) modules

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
	rm -f Module.markers Module.symvers modules.order modules.builtin
	rm -f */*.ko */*.mod.c */.*.mod.c */.*.cmd */*.o
	rm -f */modules.order */modules.builtin
	rm -rf .tmp_versions Modules.symvers
