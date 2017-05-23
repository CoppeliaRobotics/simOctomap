BOOST_DIR ?= $(shell find /usr/local/Cellar/boost -mindepth 1 -maxdepth 1 -type d | sort | tail -n1)
BOOST_CFLAGS = -I$(BOOST_DIR)/include
BOOST_LDLIBS = -L$(BOOST_DIR)/lib -lboost_system

OCTOMAP_DIR ?= $(HOME)/octomap
OCTOMAP_CFLAGS = -I$(OCTOMAP_DIR)/octomap/include
OCTOMAP_LDLIBS = -L$(OCTOMAP_DIR)/lib -loctomath -loctomap

# to override these variables, call make OCTOMAP_DIR="/path/to/octomap" OCTOMAP_LDLIBS="..."

# for when $PWD is a symlink:
PARENT_DIR = $(shell sh -c 'cd $$PWD/..; pwd')

CXXFLAGS = -ggdb -O0 -I$(PARENT_DIR)/include -Igenerated -Wall -Wno-unused -Wno-overloaded-virtual -Wno-sign-compare -fPIC $(BOOST_CFLAGS) $(OCTOMAP_CFLAGS)
LDLIBS = -ggdb -O0 -lpthread -ldl $(BOOST_LDLIBS) $(OCTOMAP_LDLIBS)

.PHONY: clean all install doc

OS = $(shell uname -s)
ifeq ($(OS), Linux)
	CFLAGS += -D__linux
	EXT = so
	INSTALL_DIR ?= $(PARENT_DIR)/..
else
	CFLAGS += -D__APPLE__
	EXT = dylib
	INSTALL_DIR ?= $(PARENT_DIR)/../vrep.app/Contents/MacOS/
endif

all: libv_repExtOctomap.$(EXT)

v_repExtOctomap.o: generated/stubs.h

generated/stubs.o: generated/stubs.h generated/stubs.cpp

generated/stubs.h generated/stubs.cpp generated/reference.html: callbacks.xml
	python external/v_repStubsGen/generate.py --xml-file callbacks.xml --gen-all "$(PWD)/generated/"

libv_repExtOctomap.$(EXT): v_repExtOctomap.o generated/stubs.o $(PARENT_DIR)/common/v_repLib.o
	$(CXX) $^ $(LDLIBS) -shared -o $@

clean:
	rm -f libv_repExtOctomap.$(EXT)
	rm -f *.o
	rm -rf generated

install: all
	cp libv_repExtOctomap.$(EXT) $(INSTALL_DIR)
