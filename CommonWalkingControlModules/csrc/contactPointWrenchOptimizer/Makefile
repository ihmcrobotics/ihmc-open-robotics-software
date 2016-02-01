# Produced by CVXGEN, 2013-03-11 17:02:06 -0400.
# CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com.
# The code in this file is Copyright (C) 2006-2012 Jacob Mattingley.
# CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
# applications without prior written permission from Jacob Mattingley.

# Filename: Makefile.
# Description: Basic Makefile.
OPT = -Wall
# for valgrind.
#OPT=-g -O0
OPT=-Os
# libmath is needed for sqrt. can remove in final product.
LDLIBS = -lm
CFLAGS = $(OPT) $(INCLUDES)
CC = gcc
.PHONY: all
all: testsolver
testsolver: solver.o matrix_support.o ldl.o testsolver.o util.o
# Include util.o for random functions and easy matrix printing.
#testsolver: solver.o matrix_support.o ldl.o util.o testsolver.o
solver.o: solver.h
matrix_support.o: solver.h
ldl.o: solver.h
util.o: solver.h
testsolver.o: solver.h
.PHONY : clean
clean :
	-rm -f *.o testsolver
