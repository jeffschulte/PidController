# Makefile for U3 examples to build combined static library (.a)

U3FEEDBACK_SRC=u3Feedback.c u3.c
U3CONFIGU3_SRC=u3BasicConfigU3.c
U3ALLIO_SRC=u3allio.c u3.c
U3STREAM_SRC=u3Stream.c u3.c
U3EFUNCTIONS_SRC=u3EFunctions.c u3.c
U3LJTDAC_SRC=u3LJTDAC.c u3.c

# Gather all object files together for a single library
U3FEEDBACK_OBJ=$(U3FEEDBACK_SRC:.c=.o)
U3CONFIGU3_OBJ=$(U3CONFIGU3_SRC:.c=.o)
U3ALLIO_OBJ=$(U3ALLIO_SRC:.c=.o)
U3STREAM_OBJ=$(U3STREAM_SRC:.c=.o)
U3EFUNCTIONS_OBJ=$(U3EFUNCTIONS_SRC:.c=.o)
U3LJTDAC_OBJ=$(U3LJTDAC_SRC:.c=.o)

ALL_U3_OBJS=$(U3FEEDBACK_OBJ) $(U3CONFIGU3_OBJ) $(U3ALLIO_OBJ) $(U3STREAM_OBJ) $(U3EFUNCTIONS_OBJ) $(U3LJTDAC_OBJ)

SRCS=$(wildcard *.c)
HDRS=$(wildcard *.h)

CFLAGS +=-Wall -g
LIBS=-lm -llabjackusb

# Default target to build the combined static library
all: libu3.a

# Create a single static library containing all object files
libu3.a: $(ALL_U3_OBJS)
	ar rcs libu3.a $(ALL_U3_OBJS)

# Clean up the object files and library
clean:
	rm -f *.o *~ libu3.a