CFLAGS=-O3 -fPIC -fexpensive-optimizations -fomit-frame-pointer -funroll-loops -fno-stack-check -fno-stack-protector
CC=gcc
x86=$(shell gcc -E -dM - </dev/null | grep -c -e __x86_64 -e __i386)

ifneq ($(x86),0)
X86OBJS=ffdecsa_mmx.o ffdecsa_sse2.o
else
X86OBJS=
endif

libffdecsa.a: ffdecsa_int.o ffdecsa_long.o $(X86OBJS) ffdecsa.o
	ar rcu libffdecsa.a ffdecsa_int.o ffdecsa_long.o ffdecsa_mmx.o ffdecsa_sse2.o ffdecsa.o

ffdecsa_int.o:
	-make -C FFdecsa clean > /dev/null 2>&1
	make -C FFdecsa PARALLEL_MODE=PARALLEL_32_INT FLAGS="-DBUILD_PARALLEL_32_INT -include $(shell pwd)/libffdecsabuild.h $(CFLAGS)" COMPILER=$(CC) FFdecsa.o
	mv FFdecsa/FFdecsa.o ffdecsa_int.o

ffdecsa_long.o:
	-make -C FFdecsa clean > /dev/null 2>&1
	make -C FFdecsa PARALLEL_MODE=PARALLEL_64_LONG FLAGS="-DBUILD_PARALLEL_64_LONG -include $(shell pwd)/libffdecsabuild.h $(CFLAGS)" COMPILER=$(CC) FFdecsa.o
	mv FFdecsa/FFdecsa.o ffdecsa_long.o

ifneq ($(x86),0)
ffdecsa_mmx.o:
	-make -C FFdecsa clean > /dev/null 2>&1
	make -C FFdecsa PARALLEL_MODE=PARALLEL_64_MMX FLAGS="-DBUILD_PARALLEL_64_MMX -include $(shell pwd)/libffdecsabuild.h $(CFLAGS)" COMPILER=$(CC) FFdecsa.o
	mv FFdecsa/FFdecsa.o ffdecsa_mmx.o
endif

ifneq ($(x86),0)
ffdecsa_sse2.o:
	-make -C FFdecsa clean > /dev/null 2>&1
	make -C FFdecsa PARALLEL_MODE=PARALLEL_128_SSE2 FLAGS="-DBUILD_PARALLEL_128_SSE2 -include $(shell pwd)/libffdecsabuild.h $(CFLAGS)" COMPILER=$(CC) FFdecsa.o
	mv FFdecsa/FFdecsa.o ffdecsa_sse2.o
endif

ffdecsa.o: ffdecsa.c ffdecsa.h
	$(CC) $(CFLAGS) -c ffdecsa.c

clean:
	-make -C FFdecsa clean > /dev/null 2>&1
	rm -f libffdecsa.a *.o

