CXX := g++
CXXFLAGS := -std=c++14 -Wall -Wextra -I/usr/include/pugixml
LDFLAGS := -L/usr/lib64 -lpugixml
TARGET := xml_parser

SRCS := xml_reader_test.cxx
OBJS := $(SRCS:.cxx=.o)

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) -o $(TARGET) $(LDFLAGS)

%.o: %.cxx
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)
