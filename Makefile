PROJECT_NAME = particleSystem

NVCC = nvcc
CXX = g++
CC = gcc

CUDAPATH = /usr/local/cuda
BUILD_DIR = bin

CFLAGS = -c -m64 -I$(CUDAPATH)/include
NVCCFLAGS = -c -I$(CUDAPATH)/include

LFLAGS = -m64 -L$(CUDAPATH)/lib -lcuda -lcudart -lm -lGL -lGLU -lglut 

GL_INCLUDE = -I/usr/include -I./include
GL_LIB = ./lib/libGLEW.a -L/usr/lib64

all: build clean
build: build_dir cpu #gpu
	$(NVCC) $(LFLAGS) $(GL_LIB) -o $(BUILD_DIR)/$(PROJECT_NAME) *.o

build_dir:
	mkdir -p $(BUILD_DIR)

gpu:
	$(NVCC) $(NVCCFLAGS) *.cu
	
cpu:
	$(CXX) $(CFLAGS) $(GL_INCLUDE) *.cpp
	$(CC) $(CFLAGS) $(GL_INCLUDE) *.c
	
clean:
	rm -f *.o
	
run:
	./$(BUILD_DIR)/$(PROJECT_NAME)
