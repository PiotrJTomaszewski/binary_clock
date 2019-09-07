CC=gcc
CFLAGS=-Wall -Wextra -Wpedantic

bc_update.out: update_time.c
	$(CC) -o $@ $< $(CFLAGS)

.PHONY: clean install

clean:
	rm -f bc_update.out

install:
	cp -i bc_update.out /usr/bin/bc_update
