CFLAGS = -g -Wall -std=c99
CFLAGS +=  -I urjtag/urjtag/include/ -L urjtag/urjtag/src/.libs/
LIBURJTAG=-Wl,-Bstatic -lurjtag -Wl,-Bdynamic -lftdi1 -lusb-1.0 -lreadline

libmwdmi.so: mwdmi.c
	$(CC) -shared -fPIC  -o $@ $^ $(CFLAGS) $(LIBURJTAG)
