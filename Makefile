CC?=gcc
CFLAGS?= -O2 -s -pipe
CFLAGS+= -DNDEBUG
CFLAGS=-W  -Wall -g -O0

CFLAGS+= -D_ENDIAN_LITTLE -I./include -I./util/codec -I./util/proto -I./pal

LDFLAGS+= -lm

DESTDIR?=/usr/local

OBJS=	sirf_codec_ssb.o \
	sirf_codec_ascii.o \
	sirf_codec_nmea.o \
	output_dump.o \
	output_nmea.o \
	output_rinex.o

all: sirfdump

clean:
	rm -f *.o sirfdump

sirfdump: ${OBJS} sirfdump.c sirfdump.h
	$(CC) $(CFLAGS) $(LDFLAGS) \
	sirfdump.c ${OBJS} \
	-o sirfdump

sirf_codec_ssb.o: util/codec/sirf_codec_ssb.c
	$(CC) $(CFLAGS) -c util/codec/sirf_codec_ssb.c

sirf_codec_ascii.o: util/codec/sirf_codec_ascii.c
	$(CC) $(CFLAGS) -c util/codec/sirf_codec_ascii.c

sirf_codec_nmea.o: util/codec/sirf_codec_nmea.c
	$(CC) $(CFLAGS) -DSIRF_CODEC_NMEA -c util/codec/sirf_codec_nmea.c

output_dump.o: output_dump.c sirfdump.h sirf_codec_ssb.o sirf_codec_ascii.o
	$(CC) $(CFLAGS) -c output_dump.c

output_nmea.o: output_dump.c sirfdump.h sirf_codec_ssb.o sirf_codec_nmea.o
	$(CC) $(CFLAGS) -c output_nmea.c

output_rinex.o: output_rinex.c sirfdump.h sirf_codec_ssb.o
	$(CC) $(CFLAGS) -c output_rinex.c

install:
	mkdir -p ${DESTDIR}/bin 2> /dev/null
	cp -p sirfdump ${DESTDIR}/bin

