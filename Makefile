objects = src/main.o src/dfu.o

LIBUSB_CFLAGS = $(shell pkg-config --cflags libusb-1.0)
LIBUSB_LDFLAGS = $(shell pkg-config --libs libusb-1.0)

CXXFLAGS += $(LIBUSB_CFLAGS)
LDFLAGS += $(LIBUSB_LDFLAGS)

CXXFLAGS += -std=c++17
CXXFLAGS += -O1 -g -fsanitize=address -fno-omit-frame-pointer
LDFLAGS += -g -fsanitize=address

all: dfu

dfu: $(objects) compile_commands.json
	clang++ -o dfu $(LDFLAGS) $(objects)

%.o.json: %.cpp
	clang++ -MJ $@ $(CXXFLAGS) -c $<

src/main.o src/dfu.o: src/dfu.h

compile_commands.json: $(objects:.o=.o.json)
	sed -e '1s/^/[\'$$'\n''/' -e '$$s/,$$/\'$$'\n'']/' $^ > compile_commands.json

.PHONY: clean
clean:
	-rm -f dfu $(objects)
