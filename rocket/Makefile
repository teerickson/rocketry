# makefile for rc_mpu, builds example programs

# directories
SRCDIR		:= src
OBJDIR		:= obj
BINDIR		:= bin
INCLUDEDIR	:= library/include
LIBDIR		:= library/lib

# basic definitions for rules
OBJ_SOURCES	:= $(shell find $(SRCDIR) -mindepth 2 -type f -name *.c)
OBJ_SOURCES_CPP	:= $(shell find $(SRCDIR) -mindepth 2 -type f -name *.cpp)
EXE_SOURCES	:= $(shell find $(SRCDIR) -maxdepth 1 -type f -name *.c)
OBJ_TARGETS	:= $(OBJ_SOURCES:$(SRCDIR)/%.c=$(OBJDIR)%.o)
OBJ_TARGETS_CPP	:= $(OBJ_SOURCES_CPP:$(SRCDIR)/%.c=$(OBJDIR)%.o)
EXE_TARGETS	:= $(EXE_SOURCES:$(SRCDIR)/%.c=$(BINDIR)/%)
INCLUDES	:= $(shell find $(INCLUDEDIR) -name '*.h')

# compiler and linker programs
CC		:= gcc
CPP		:= g++
LINKER		:= gcc

# compile flags
WFLAGS		:= -Wall -Wextra -Werror=float-equal -Wuninitialized \
	-Wunused-variable -Wdouble-promotion -pedantic -Wmissing-prototypes \
	-Wmissing-declarations -Werror=undef
WFLAGS_CPP		:= -Wall -Wextra -Werror=float-equal -Wuninitialized \
	-Wunused-variable -Wdouble-promotion -pedantic \
	-Wmissing-declarations -Werror=undef
CFLAGS		:= -g -pthread -I $(INCLUDEDIR)
LDFLAGS		:= -lm -lrt -pthread -L $(LIBDIR) -l:librobotcontrol.so.1

# commands
RM		:= rm -rf
INSTALL		:= install -m 755
INSTALLDIR	:= install -d -m 755
LINK		:= ln -s -f
LINKDIR		:= /etc/librobotcontrol
LINKNAME	:= link_to_startup_program

# prefix variable in case this is used to make a deb package
prefix		?= /usr


all : $(OBJ_TARGETS) $(OBJ_TARGETS_CPP) $(EXE_TARGETS) 

debug :
	$(MAKE) $(MAKEFILE) DEBUGFLAG="-g -D DEBUG"
	@echo " "
	@echo "Make Debug Complete"
	@echo " "

clean :
	@$(RM) $(BINDIR)
	@$(RM) $(OBJDIR)
	@echo " "
	@echo "Make Clean Complete"

$(OBJ_TARGETS) : $(OBJ_SOURCES)
	@mkdir -p $(dir $@)
	$(CC) -c -std=c99 -o $@ $< $(CFLAGS) $(WFLAGS) $(DEBUGFLAG) $(LDFLAGS)
	@echo "made :$@"

$(OBJ_TARGETS_CPP) : $(OBJ_SOURCES_CPP)
	@mkdir -p $(dir $@)
	$(CPP) -c -std=c++0x -o $@ $< $(CFLAGS) $(WFLAGS_CPP) $(DEBUGFLAG) $(LDFLAGS)
	@echo "made :$@"

$(EXE_TARGETS) : $(EXE_SOURCES) $(OBJ_TARGETS) $(OBJ_TARGETS_CPP)
	mkdir -p $(BINDIR)
	$(CC) -o $@ $< $(CFLAGS) $(WFLAGS) $(DEBUGFLAG) $(LDFLAGS)
	@echo "made: $@"
