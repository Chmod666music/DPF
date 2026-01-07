#!/usr/bin/make -f
# Makefile for DISTRHO Plugins #

NAME = d_drumcloud

FILES_DSP = \
        SendNoteExamplePlugin.cpp

# UI stub (required for CLAP)
FILES_UI = 

# Build only CLAP for now
TARGETS = clap

# Use C++17
CXXFLAGS += -std=gnu++17

# IMPORTANT: do not override BUILD_CXX_FLAGS / BASE_FLAGS here
# (DPF sets include paths for DistrhoPlugin.hpp automatically)
UI_TYPE = external

include ../../Makefile.plugins.mk

all: $(TARGETS)

