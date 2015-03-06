GL_INCLUDE = -I/usr/include -I./include
GL_LIB = -L/usr/lib64 ./lib/libGLEW.a

hello-gl: hello-gl.o util.o
	mkdir -p bin
	gcc -o bin/hello-gl $^ $(GL_LIB) -lGL -lglut -lm

.c.o:
	gcc -c -o $@ $< $(GL_INCLUDE)

clean:
	rm -f bin/hello-gl hello-gl.o
