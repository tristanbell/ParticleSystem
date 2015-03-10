GL_INCLUDE = -I/usr/include -I./include
GL_LIB = -L/usr/lib64 ./lib/libGLEW.a
CXXFLAGS = -std=c++0x

all: particleSystem

particleSystem: particle_system.o vec.o particle.o ParticleManager.o util.o
	mkdir -p bin
	g++ $(CXXFLAGS) -o bin/$@ $^ $(GL_LIB) -lGL -lglut -lm -lGLU

.cpp.o:
	g++ $(CXXFLAGS) -c -o $@ $< $(GL_INCLUDE)

clean:
	rm -f bin/particleSystem ParticleManager.o particle.o particle_system.o util.o vec.o

