warning_flags = -Wall -Wextra -Wshadow -Wno-unused-parameter
common_cflags = -O3 -ffast-math $(MFLAGS) $(warning_flags)
AM_CXXFLAGS = -std=c++14 $(common_cflags)

AM_CPPFLAGS = $(VapourSynth_CFLAGS)

lib_LTLIBRARIES = libdeblock.la

libdeblock_la_SOURCES = Deblock/Deblock.cpp

libdeblock_la_LDFLAGS = -no-undefined -avoid-version $(PLUGINLDFLAGS)
