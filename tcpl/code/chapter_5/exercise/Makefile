
# # chapter 1
# 1_1: ./chapter_1/1-1.c
# # 	mkdir ./chapter_1/build
# 	gcc ./chapter_1/1-1.c -o ./chapter_1/build/1_1


# # exercise of chapter 1
# exercise_1-1: ./chapter_1/exercise/exercise_1-1.c
# 	gcc ./chapter_1/exercise/exercise_1-1.c -o ./chapter_1/build/exercise_1-1
SOURCE = $(wildcard *.c)
TARGETS = $(patsubst %.c, %, $(SOURCE))
 
CC = gcc
CFLAGS = -Wall -g
 
all:$(TARGETS)
 
$(TARGETS):%:%.c
	$(CC) $< $(CFLAGS) -o $@
 
.PHONY:clean all
clean:
	-rm -rf $(TARGETS)