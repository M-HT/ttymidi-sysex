all: ttymidi-sysex

ttymidi-sysex: ttymidi-sysex.c
	gcc -O3 -Wall -o ttymidi-sysex ttymidi-sysex.c -lasound -lpthread

.PHONY: clean
clean:
	rm -f ttymidi-sysex
