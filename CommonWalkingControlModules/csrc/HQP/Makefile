all: lib header



TEMPLATE =

SRC = \
	Soth.cpp

HEADER = \
	Soth.h Algebra.h

LIB_DIR = ../lib
LIB = ${LIB_DIR}/libsoth.so

# --- CFLAGS
CFLAGS = -I/usr/include -I${PWD}/../include 


# --- MAIN RULES

header: ${HEADER:%=../include/soth/%} ${TEMPLATE:%=../include/soth/%}

lib: ${LIB}

# --- HEADERS

../include/soth/%.h: %.h
	@ln -snf ${PWD}/$< $@

../include/soth/%.t.cpp: %.t.cpp
	@ln -snf ${PWD}/$< $@

# --- LIB

../.objs/%.o: %.cpp
	echo $@ $^ ${CFLAGS}
	g++ -o $@ -c $< ${CFLAGS}


${LIB}: ${SRC:%.cpp=../.objs/%.o}
	g++ -o $@ -shared $^
