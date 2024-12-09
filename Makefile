objects = src/main.o src/dfu.o

CXXFLAGS += -std=c++17 -O1 -g -fno-omit-frame-pointer
LDFLAGS += -g

LIBUSB_CFLAGS = $(shell pkg-config --cflags libusb-1.0)
LIBUSB_LDFLAGS = $(shell pkg-config --libs libusb-1.0)

CXXFLAGS += $(LIBUSB_CFLAGS)
LDFLAGS += $(LIBUSB_LDFLAGS)

ifdef ASAN
	CXXFLAGS += -fsanitize=address
	LDFLAGS += -fsanitize=address
endif

all: dfu

dfu: $(objects)
	$(CXX) -o dfu $(objects) $(LDFLAGS)

%.o.json: %.cpp
	clang++ -MJ $@ $(CXXFLAGS) -c $<

src/main.o src/dfu.o: src/dfu.h

compile_commands.json: $(objects:.o=.o.json)
	sed -e '1s/^/[\'$$'\n''/' -e '$$s/,$$/\'$$'\n'']/' $^ > compile_commands.json

.PHONY: clean
clean:
	-rm -f dfu $(objects)
