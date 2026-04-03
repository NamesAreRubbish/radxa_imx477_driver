obj-m := imx477.o

SRC := $(shell pwd)

all:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(SRC) modules

clean:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(SRC) clean
