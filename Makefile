GL_INCLUDE = -I/usr/include -I./include
GL_LIB = -L/usr/lib64 ./lib/libGLEW.a

all: particleSystem

particleSystem: vec.o particle.o
	mkdir -p bin
	g++ -o bin/$@ $^ $(GL_LIB) -lGL -lglut -lm

.cpp.o:
	g++ -c -o $@ $< $(GL_INCLUDE)

clean:
	rm -f bin/hello-gl hello-gl.o
