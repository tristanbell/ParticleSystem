GL_INCLUDE = -I/usr/include -I./include
GL_LIB = ./lib/libGLEW.a -L/usr/lib64
CXXFLAGS = -std=c++0x

all: particleSystem

particleSystem: particle_system.o vec.o particle.o ParticleManager.o util.o
	mkdir -p bin
ifeq ($(shell uname), Darwin)
	g++ $(CXXFLAGS) -o bin/$@ $^ $(GL_LIB) -lGLEW -framework GLUT -framework OpenGL
else
	g++ $(CXXFLAGS) -o bin/$@ $^ $(GL_LIB) -lGL -lglut -lm -lGLU
endif

.cpp.o:
	g++ $(CXXFLAGS) -c -o $@ $< $(GL_INCLUDE)

clean:
	rm -f bin/particleSystem ParticleManager.o particle.o particle_system.o util.o vec.o

