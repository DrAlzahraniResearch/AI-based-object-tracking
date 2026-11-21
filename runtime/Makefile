# Verified checker builds.
#   make            -> build/libutm_checker.so  (Python/ctypes; links the toolchain's libleanshared.so)
#                      build/libutm_checker.a   (static, for C/C++ executables, e.g. a ROS 2 monitor node)
#                      build/utm_smoke          (static executable linked from the .a: proves the embedding path)
#   make test       -> runs the C smoke test against both flavours
# LEAN_PREFIX defaults to the toolchain pinned by proofs/lean-toolchain (via elan).
#
# Why two flavours: Lean's static runtime (libleanrt.a) uses local-exec thread-local storage,
# which cannot go into a shared object; executables can link it statically, shared objects must use
# libleanshared.so.
PROOFS      ?= ../proofs
LEAN_PREFIX ?= $(shell cd $(PROOFS) && lean --print-prefix 2>/dev/null)
LEANC       := $(LEAN_PREFIX)/bin/leanc
LEAN        := $(LEAN_PREFIX)/bin/lean
LIBDIR      := $(LEAN_PREFIX)/lib/lean
STATIC_LIBS := -L$(LIBDIR) -Wl,--start-group -lInit -lleanrt -Wl,--end-group \
               -Wl,-Bstatic -lc++ -lc++abi -Wl,-Bdynamic -lgmp -luv -lpthread -ldl -lrt -lm

all: build/libutm_checker.so build/libutm_checker.a build/utm_smoke

build/UtmChecker.c: $(PROOFS)/UtmChecker.lean
	mkdir -p build
	cd $(PROOFS) && $(LEAN) UtmChecker.lean -c $(CURDIR)/build/UtmChecker.c

build/UtmChecker.o: build/UtmChecker.c
	$(LEANC) -c -O2 -fPIC -o $@ $<

build/utm_shim.o: utm_shim.c utm_checker.h
	$(LEANC) -c -O2 -fPIC -I. -o $@ utm_shim.c

build/libutm_checker.so: build/UtmChecker.o build/utm_shim.o utm.map
	$(LEANC) -shared -o $@ build/UtmChecker.o build/utm_shim.o \
	  -L$(LIBDIR) -lleanshared -Wl,-rpath,$(LIBDIR) -Wl,--version-script=utm.map

build/libutm_checker.a: build/UtmChecker.o build/utm_shim.o
	rm -f $@ && $(LEAN_PREFIX)/bin/llvm-ar rcs $@ $^

# Lean's bundled clang has no libc headers: compile ordinary C with the system compiler, link with leanc.
CC ?= cc
build/smoke.o: smoke.c utm_checker.h
	mkdir -p build
	$(CC) -c -O2 -I. -o $@ smoke.c

build/utm_smoke: build/smoke.o build/libutm_checker.a
	$(LEANC) -o $@ build/smoke.o build/libutm_checker.a $(STATIC_LIBS)

build/utm_smoke_shared: build/smoke.o build/libutm_checker.so
	$(CC) -o $@ build/smoke.o -Lbuild -lutm_checker -Wl,-rpath,'$$ORIGIN'

test: build/utm_smoke build/utm_smoke_shared
	./build/utm_smoke && ./build/utm_smoke_shared

clean:
	rm -rf build
.PHONY: all test clean
