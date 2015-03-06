#include <stdlib.h>
#include <GL/glew.h>
#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif

int windowWidth = 640;
int windowHeight = 480;

static void render(void)
{
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    glutSwapBuffers();
}

void initGL(int argc, char **argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
	glutInitWindowSize(windowWidth, windowHeight);
	glutCreateWindow("Particle System");
//	glutIdleFunc(&idle);
	glutDisplayFunc(&render);

	glewInit();

	glutReportErrors();
}

int main(int argc, char **argv) {
	initGL(argc, argv);

	glutMainLoop();

	return 0;
}
