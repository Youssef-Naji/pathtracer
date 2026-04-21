
CXX = clang++
OMP_PREFIX = $(shell brew --prefix libomp)
CXXFLAGS = -O2 -std=c++14 -Xpreprocessor -fopenmp -I$(OMP_PREFIX)/include
LDFLAGS = -L$(OMP_PREFIX)/lib -lomp
TARGET = pathtracer
SRC = main.cpp

all: $(TARGET)
$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC) $(LDFLAGS)
run: $(TARGET)
	./$(TARGET)
clean:
	rm -f $(TARGET)