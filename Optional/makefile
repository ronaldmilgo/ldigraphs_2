CC=gcc
CFLAGS=-Wall -pedantic -std=c17 -g3

HW6=/c/cs223/hw6
BIN=/c/cs223/bin

# Compiling

all: Paths
Paths: paths.o ldigraph.o
	${CC} -o $@ ${CFLAGS} $^

ldigraph.o: ldigraph.h
paths.o: ldigraph.h

clean:
	rm -f Paths *.o


test:
	${HW6}/Tests/test.Paths


singletest:
	${HW6}/Tests/$(TEST)


submit:
	${BIN}/submit 6 makefile paths.c ldigraph.c log

check:
	${BIN}/check 6

testit:
	${BIN}/testit 6 Paths