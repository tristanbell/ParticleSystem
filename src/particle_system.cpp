#include <stdlib.h>
#include <GL/glew.h>
#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#include <GL/freeglut.h>
#endif

#include <cmath>
#include "particle.h"
#include "ParticleManager.h"

// Camera parameters
int ox, oy;
int buttonState = 0;
float camera_trans[] = { 0, 0, -3 };
float camera_rot[] = { 0, 0, 0 };
float camera_trans_lag[] = { 0, 0, -3 };
float camera_rot_lag[] = { 0, 0, 0 };
const float inertia = 0.1f;

// Window dimensions
int windowWidth = 640;
int windowHeight = 640;

// Lighting information (taken from http://www.programming-techniques.com/2012/05/rendering-spheres-glutsolidsphere-and.html)
const GLfloat light_ambient[] = { 0.0f, 0.0f, 0.0f, 1.0f };
const GLfloat light_diffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f };
const GLfloat light_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
const GLfloat light_position[] = { 2.0f, 5.0f, 5.0f, 0.0f };

const GLfloat mat_ambient[] = { 0.7f, 0.7f, 0.7f, 1.0f };
const GLfloat mat_diffuse[] = { 0.8f, 0.8f, 0.8f, 1.0f };
const GLfloat mat_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
const GLfloat high_shininess[] = { 100.0f };

// FPS info
int frame = 0, eTime, timebase = 0;

ParticleManager *pManager;

void calculateFrameRate() {
	static float framesPerSecond = 0.0f; // This will store our fps
	static float lastTime = 0.0f; // This will hold the time from the last frame
	float currentTime = glutGet(GLUT_ELAPSED_TIME);
	++framesPerSecond;
	if (currentTime - lastTime > 1000.0f) {
		lastTime = currentTime;
		char fps[10];
		sprintf(fps, "FPS: %d",
				(int) framesPerSecond);
		glutSetWindowTitle(fps);
		framesPerSecond = 0;
	}
}

/**
 * Update particle system here. This is called by the GLUT
 * loop when nothing else is happening.
 */
static void idle(void) {
	calculateFrameRate();

	pManager->update();

	glutPostRedisplay();
}

/**
 * Renders to the screen.
 */
static void render(void) {
	// Clear the screen to begin with
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Move the camera
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// Update for inertia
	for (int i = 0; i < 3; ++i) {
		camera_trans_lag[i] += (camera_trans[i] - camera_trans_lag[i])
				* inertia;
		camera_rot_lag[i] += (camera_rot[i] - camera_rot_lag[i]) * inertia;
	}

	// Perform translation and rotation based on view parameters
	glTranslatef(camera_trans_lag[0], camera_trans_lag[1], camera_trans_lag[2]);
	glRotatef(camera_rot_lag[0], 1.0, 0.0, 0.0);
	glRotatef(camera_rot_lag[1], 0.0, 1.0, 0.0);

	// Draw the particles
	pManager->render();

	// Draw the wire cube for the particle enclosure
	glColor3f(1.0, 1.0, 1.0);
	glutWireCube(2.0);

	// Render to the screen
	glutSwapBuffers();
}

/**
 * Called when the window resizes. Reload the camera projection,
 * updating for the new aspect ratio.
 */
void reshape(int w, int h) {
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, (float) w / (float) h, 0.1, 100.0);

	glMatrixMode(GL_MODELVIEW);
	glViewport(0, 0, w, h);
}

/**
 * Called when the mouse changes state.
 */
void mouse(int button, int state, int x, int y) {
	int mods;

	if (state == GLUT_DOWN) {
		buttonState |= 1 << button;
	} else if (state == GLUT_UP) {
		buttonState = 0;
	}

	mods = glutGetModifiers();

	if (mods & GLUT_ACTIVE_SHIFT) {
		buttonState = 2;
	} else if (mods & GLUT_ACTIVE_CTRL) {
		buttonState = 3;
	}

	ox = x;
	oy = y;

	glutPostRedisplay();
}

/**
 * Called when the mouse is dragged. Updates the camera
 * parameters depending on the button state. (Taken from
 * the CUDA sample and commented out what we're currently
 * not using)
 */
void motion(int x, int y) {
	float dx, dy;
	dx = (float) (x - ox);
	dy = (float) (y - oy);

//    switch (mode)
//    {
//        case M_VIEW:
	if (buttonState == 3) {
		// left+middle = zoom
		camera_trans[2] += (dy / 100.0f) * 0.5f * fabs(camera_trans[2]);
	} else if (buttonState & 2) {
		// middle = translate
		camera_trans[0] += dx / 100.0f;
		camera_trans[1] -= dy / 100.0f;
	} else if (buttonState & 1) {
		// left = rotate
		camera_rot[0] += dy / 5.0f;
		camera_rot[1] += dx / 5.0f;
	}

//            break;

//        case M_MOVE:
//            {
//                float translateSpeed = 0.003f;
//                float3 p = psystem->getColliderPos();
//
//                if (buttonState==1)
//                {
//                    float v[3], r[3];
//                    v[0] = dx*translateSpeed;
//                    v[1] = -dy*translateSpeed;
//                    v[2] = 0.0f;
//                    ixform(v, r, modelView);
//                    p.x += r[0];
//                    p.y += r[1];
//                    p.z += r[2];
//                }
//                else if (buttonState==2)
//                {
//                    float v[3], r[3];
//                    v[0] = 0.0f;
//                    v[1] = 0.0f;
//                    v[2] = dy*translateSpeed;
//                    ixform(v, r, modelView);
//                    p.x += r[0];
//                    p.y += r[1];
//                    p.z += r[2];
//                }
//
//                psystem->setColliderPos(p);
//            }
//            break;
//    }

	ox = x;
	oy = y;

	glutPostRedisplay();
}

void initGL(int argc, char **argv) {
	glutInit(&argc, argv);
	glutInitDisplayMode(
			GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH /*| GLUT_MULTISAMPLE*/);
	glutInitWindowSize(windowWidth, windowHeight);
	glutCreateWindow("Particle System");
	glutIdleFunc(&idle);
	glutDisplayFunc(&render);
	glutReshapeFunc(&reshape);
	glutMotionFunc(&motion);
	glutMouseFunc(&mouse);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
//	glEnable(GL_MULTISAMPLE_ARB);

	glEnable(GL_LIGHT0);
	glEnable(GL_NORMALIZE);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHTING);
//
//	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
//	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
//	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
//	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
//
//	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
//	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
//	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
//	glMaterialfv(GL_FRONT, GL_SHININESS, high_shininess);

	glewInit();

	glutReportErrors();
}

int main(int argc, char **argv) {
	initGL(argc, argv);

	pManager = new ParticleManager(1000, Vec3(2, 2, 2));

	glutMainLoop();

	 delete pManager;

	return 0;
}
