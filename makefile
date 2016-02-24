BOOST_DIR ?= /usr/local/Cellar/boost/1.59.0
BOOST_CFLAGS = -I$(BOOST_DIR)/include
BOOST_LDLIBS = -L$(BOOST_DIR)/lib -lboost_system

OCTOMAP_DIR ?= $(HOME)/octomap
OCTOMAP_CFLAGS = -I$(OCTOMAP_DIR)/octomap/include
OCTOMAP_LDLIBS = -L$(OCTOMAP_DIR)/lib -loctomath -loctomap

# to override these variables, call make OCTOMAP_DIR="/path/to/octomap" OCTOMAP_LDLIBS="..."

CXXFLAGS = -ggdb -O0 -I../include -Wall -Wno-unused -Wno-overloaded-virtual -Wno-sign-compare -fPIC $(BOOST_CFLAGS) $(OCTOMAP_CFLAGS)
LDLIBS = -ggdb -O0 -lpthread -ldl $(BOOST_LDLIBS) $(OCTOMAP_LDLIBS)

.PHONY: clean all install doc

OS = $(shell uname -s)
ifeq ($(OS), Linux)
	CFLAGS += -D__linux
	EXT = so
	INSTALL_DIR ?= ../..
else
	CFLAGS += -D__APPLE__
	EXT = dylib
	INSTALL_DIR ?= ../../vrep.app/Contents/MacOS/
endif

all: libv_repExtOctomap.$(EXT) doc

doc: reference.html

reference.html: callbacks.xml callbacks.xsl
	saxon -s:callbacks.xml -a:on -o:$@

v_repExtOctomap.o: stubs.h

stubs.o: stubs.h stubs.cpp

stubs.h: callbacks.xml generate_stubs.py
	python generate_stubs.py -h callbacks.xml > stubs.h.tmp
	mv stubs.h.tmp stubs.h

stubs.cpp: callbacks.xml generate_stubs.py
	python generate_stubs.py -c callbacks.xml > stubs.cpp.tmp
	mv stubs.cpp.tmp stubs.cpp

libv_repExtOctomap.$(EXT): v_repExtOctomap.o stubs.o ../common/v_repLib.o ../common/luaFunctionData.o ../common/luaFunctionDataItem.o
	$(CXX) $^ $(LDLIBS) -shared -o $@

clean:
	rm -f libv_repExtOctomap.$(EXT)
	rm -f *.o
	rm -f stubs.cpp stubs.cpp.tmp stubs.h stubs.h.tmp
	rm -f reference.html

install: all
	cp libv_repExtOctomap.$(EXT) $(INSTALL_DIR)
