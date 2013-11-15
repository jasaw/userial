ifneq ($(SRC),)
VPATH=$(SRC)
endif

CFLAGS += -I. -I$(SRC) -Wall -std=c99 -D_BSD_SOURCE=1 -D_GNU_SOURCE=1

all: userial

userial: userial.o
	$(CC) -o $@ $^

clean:
	rm -f *.o userial
