# ac3dec - ATSC A/52 (AC-3) audio decoder
#
# Usage:
#   make            build the static library and the example decoder
#   make test       build and run the unit/API tests
#   make check      run the full test suite (needs python3, optionally ffmpeg)
#   make clean      remove build products
#
# The build is plain C99 and needs no dependencies beyond libm.  On Windows
# the same Makefile works from an MSYS2 shell and from cmd.exe (mingw32-make);
# CMake is also supported.

CC      ?= cc
AR      ?= ar
RANLIB  ?= ranlib

CFLAGS  ?= -O2
CFLAGS  += -std=c99 -Wall -Wextra
CPPFLAGS ?=
LDFLAGS ?=
LIBS    = -lm
PYTHON  ?= python3

ifeq ($(OS),Windows_NT)
EXEEXT  = .exe
MKDIR    = if not exist "$(subst /,\,$1)" mkdir "$(subst /,\,$1)"
RMDIR    = if exist "$(subst /,\,$1)" rmdir /S /Q "$(subst /,\,$1)"
else
EXEEXT  =
MKDIR    = mkdir -p "$1"
RMDIR    = rm -rf "$1"
endif

BUILD   ?= build
LIB     = $(BUILD)/libac3dec.a
EXAMPLE = $(BUILD)/ac3dec_example$(EXEEXT)

SRC = src/ac3_tables.c \
      src/ac3_frame.c \
      src/ac3_bitalloc.c \
      src/ac3_block.c \
      src/ac3_imdct.c \
      src/ac3_downmix.c \
      src/eac3_dec.c \
      src/eac3_tables.c \
      src/ac3_decode.c \
      src/dolby_e.c \
      src/dolby_e_parse.c

OBJ = $(patsubst src/%.c,$(BUILD)/%.o,$(SRC))

.PHONY: all lib example dolby-e test check clean

all: lib example

lib: $(LIB)

example: $(EXAMPLE)

$(LIB): $(OBJ)
	$(AR) rcs $@ $(OBJ)
	-$(RANLIB) $@

$(BUILD)/%.o: src/%.c ac3dec.h dolby_e.h src/ac3_internal.h dolby_e_compat.h
	@$(call MKDIR,$(BUILD))
	$(CC) $(CPPFLAGS) $(CFLAGS) -I. -c $< -o $@

$(EXAMPLE): example_decoder.c $(LIB) ac3dec.h
	@$(call MKDIR,$(BUILD))
	$(CC) $(CPPFLAGS) $(CFLAGS) -I. $< $(LIB) $(LDFLAGS) $(LIBS) -o $@

DOLBY_E_EXAMPLE = $(BUILD)/dolby_e_example$(EXEEXT)

dolby-e: $(DOLBY_E_EXAMPLE)

$(DOLBY_E_EXAMPLE): dolby_e_example.c $(LIB) dolby_e.h
	@$(call MKDIR,$(BUILD))
	$(CC) $(CPPFLAGS) $(CFLAGS) -I. $< $(LIB) $(LDFLAGS) $(LIBS) -o $@

$(BUILD)/test_api$(EXEEXT): tests/test_api.c $(LIB) ac3dec.h
	@$(call MKDIR,$(BUILD))
	$(CC) $(CPPFLAGS) $(CFLAGS) -I. -Isrc $< $(LIB) $(LDFLAGS) $(LIBS) -o $@

$(BUILD)/test_imdct$(EXEEXT): tests/test_imdct.c $(LIB) src/ac3_internal.h
	@$(call MKDIR,$(BUILD))
	$(CC) $(CPPFLAGS) $(CFLAGS) -I. -Isrc $< $(LIB) $(LDFLAGS) $(LIBS) -o $@

test: $(BUILD)/test_api$(EXEEXT) $(BUILD)/test_imdct$(EXEEXT)
	$(BUILD)/test_imdct$(EXEEXT)
	$(BUILD)/test_api$(EXEEXT) tests/data

check: test
	$(PYTHON) tests/run_tests.py

clean:
	@$(call RMDIR,$(BUILD))


