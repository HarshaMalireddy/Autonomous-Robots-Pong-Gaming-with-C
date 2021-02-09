# replace this with the top of your X11 tree
# X11 = /exp/rcf/share/X11R5
X11 = /usr/X11R6

############## do not change below this line ####################
ROGERINCDIR = ../include

XINCDIR = $(X11)/include
XLIBDIR = $(X11)/lib 
EDLAB_XINCDIR = /usr/include/X11

XAWLIB = -lXaw
XMULIB = -lXmu
XTOOLLIB = -lXt
XLIB = -lX11
XEXTLIB = -lXext
MATHLIB = -lm

LIBS =  -L$(XLIBDIR) $(XAWLIB) $(XMULIB) $(XTOOLLIB) $(XLIB) $(XEXTLIB) \
        $(MATHLIB)

RM = rm -f
CC = gcc
#CCFLAGS = -c -g $(OPT) -I. -I$(XINCDIR)
CCFLAGS = -c -g $(OPT) -I. -I$(XINCDIR) -I$(EDLAB_XINCDIR) -I$(ROGERINCDIR)

.SUFFIXES:      .c      .o

.c.o:
	$(CC) $(CCFLAGS) $<

############## do not change above this line ####################

PROG = x

PROJECTOFILES = project2.o

#HFILES =


all:	$(PROG)

$(PROG):	$(PROJECTOFILES)

clean:
	$(RM) $(PROJECTOFILES) $(PROG)

