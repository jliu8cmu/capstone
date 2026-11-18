CC	= cc
CFLAGS = -Icore -Wall -Wextra -std=c11 -g -fsanitize=address,undefined

SRCS = $(wildcard core/*.c) test/test_core.c

HDRS = $(wildcard core/*.h)

test_core: $(SRCS) $(HDRS)
	$(CC) $(CFLAGS) $(SRCS) -o  test_core


test: test_core
	./test_core


clean:
	rm -rf test_core test_core.dSYM

stress: $(SRCS) test/test_stress.c
	$(CC) -Wall -Wextra -std=c11 -g -O1 -fsanitize=thread -pthread $(wildcard core/*.c) test/test_stress.c -o test_stress
	./test_stress

.PHONY: test clean