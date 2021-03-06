CC?=gcc
CFLAGS?= -Os -pipe
CFLAGS+= -DNDEBUG
#CFLAGS=-W  -Wall -g -O0 -DVERBOSITY=LOG_RAW

CFLAGS+= -I./include -I./util/codec -I./util/proto -I./pal -I./stringlib

LDFLAGS+= -lm

DESTDIR?=/usr/local

UNAME_O:= $(shell uname -o)

ifeq ($(UNAME_O),GNU/Linux)
	NO_STRLCPY=1
endif

ifeq ($(UNAME_O),Msys)
	NO_STRLCPY=1
	CFLAGS+= -posix -D__USE_MINGW_ANSI_STDIO=1
endif

ifdef ENDIAN
	CFLAGS += -D_ENDIAN_$(ENDIAN)
endif

OBJS=	sirf_codec_ssb.o \
	sirf_codec_ascii.o \
	sirf_codec_nmea.o \
	sirf_proto_nmea.o \
	output_dump.o \
	output_nmea.o \
	output_csv.o \
	output_rinex.o \
	output_rinex_nav.o \
	output_rtcm.o \
	nav.o \
	isgps.o \
	crc24q.o \
	subframe.o

ifdef NO_STRLCPY
	OBJS += string_sif.o strnlen_sif.o
endif

all: sirfdump

clean:
	rm -f *.o sirfdump sirfsplitter

sirfdump: ${OBJS} sirfdump.c sirfdump.h
	$(CC) $(CFLAGS) \
	sirfdump.c ${OBJS} \
	-o sirfdump $(LDFLAGS)

sirf_codec_ssb.o: util/codec/sirf_codec_ssb.c
	$(CC) $(CFLAGS) -c util/codec/sirf_codec_ssb.c

sirf_codec_ascii.o: util/codec/sirf_codec_ascii.c
	$(CC) $(CFLAGS) -c util/codec/sirf_codec_ascii.c

sirf_codec_nmea.o: util/codec/sirf_codec_nmea.c
	$(CC) $(CFLAGS)  -c util/codec/sirf_codec_nmea.c

sirf_proto_nmea.o: util/proto/sirf_proto_nmea.c
	$(CC) $(CFLAGS) -c util/proto/sirf_proto_nmea.c

nav.o:  nav.c nav.h
	$(CC) $(CFLAGS) -c nav.c

output_dump.o: output_dump.c sirfdump.h
	$(CC) $(CFLAGS) -c output_dump.c

output_nmea.o: output_nmea.c sirfdump.h
	$(CC) $(CFLAGS) -c output_nmea.c

output_rinex.o: output_rinex.c sirfdump.h
	$(CC) $(CFLAGS) -c output_rinex.c

output_csv.o: output_csv.c sirfdump.h
	$(CC) $(CFLAGS) -c output_csv.c

output_rinex_nav.o: output_rinex_nav.c sirfdump.h
	$(CC) $(CFLAGS) -c output_rinex_nav.c

output_rtcm.o: output_rtcm.c sirfdump.h gpsd/crc24q.h
	$(CC) $(CFLAGS) -c output_rtcm.c

subframe.o: gpsd/gps.h gpsd/subframe.c
	$(CC) $(CFLAGS) -c gpsd/subframe.c

isgps.o: gpsd/gps.h gpsd/isgps.c
	$(CC) $(CFLAGS) -c gpsd/isgps.c

crc24q.o: gpsd/crc24q.h gpsd/crc24q.c
	$(CC) $(CFLAGS) -c gpsd/crc24q.c

string_sif.o: stringlib/string_sif.c
	$(CC) $(CFLAGS) -c stringlib/string_sif.c

strnlen_sif.o: stringlib/strnlen_sif.c
	$(CC) $(CFLAGS) -c stringlib/strnlen_sif.c

sirfsplitter: output_rinex.o sirf_codec_ssb.o sirfsplitter.c sirfdump.h
	$(CC) $(CFLAGS) \
	sirfsplitter.c output_rinex.o sirf_codec_ssb.o \
	-o sirfsplitter $(LDFLAGS)

install:
	mkdir -p ${DESTDIR}/bin 2> /dev/null
	cp -p sirfdump ${DESTDIR}/bin

