TARGET := max30100.ko

MDIR := arch/arm/max30100

CURRENT := $(shell uname -r)
KDIR := /lib/modules/$(CURRENT)/build
PWD := $(shell pwd)
DEST := /lib/modules/$(CURRENT)/kernel/$(MDIR)

obj-m := max30100.o

default:
	$(MAKE) -I $(KDIR)/arch/arm/include/asm/ -C $(KDIR) M=$(PWD)

install:
	#@if test -f $(DEST)/$(TARGET).orig; then \
	#       echo "Backup of .ko already exists."; \
	#else \
	#       echo "Creating a backup of .ko."; \
	#       mv -v $(DEST)/$(TARGET) $(DEST)/$(TARGET).orig; \
	#fi
	su -c "cp $(TARGET) $(DEST) && /sbin/depmod -a"

revert:
	@echo "Reverting to the original .ko."
	@mv -v $(DEST)/$(TARGET).orig $(DEST)/$(TARGET)

clean:
	rm -f *.o $(TARGET) .*.cmd .*.flags *.mod.c

-include $(KDIR)/Rules.make
