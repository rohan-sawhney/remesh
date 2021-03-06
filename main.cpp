#ifdef __APPLE_CC__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "Mesh.h"

int gridX = 600;
int gridY = 600;
int gridZ = 600;

const double fovy = 50.;
const double clipNear = .01;
const double clipFar = 1000.;
double x = 0;
double y = 0;
double z = -2.5;

std::string path = "/Users/rohansawhney/Desktop/developer/C++/remesh/bunny.obj";

Mesh mesh;
bool first = true;
bool success = true;
bool project = false;
int iterations = 5;
double avgEdgeLength = 0;

void printInstructions()
{
    std::cerr << "' ': remesh original mesh\n"
              << "→/←: increase/decrease iterations\n"
              << "↑/↓: move in/out\n"
              << "w/s: move up/down\n"
              << "a/d: move left/right\n"
              << "p: toggle projection\n"
              << "r: reload\n"
              << "escape: exit program\n"
              << std::endl;
}

void init()
{
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glEnable(GL_DEPTH_TEST);
}

void draw()
{
    glColor4f(0.0, 0.0, 1.0, 0.6);
    
    glBegin(GL_LINES);
    for (EdgeCIter e = mesh.edges.begin(); e != mesh.edges.end(); e ++) {
       
        Eigen::Vector3d a = e->he->vertex->position;
        Eigen::Vector3d b = e->he->flip->vertex->position;
        
        glVertex3d(a.x(), a.y(), a.z());
        glVertex3d(b.x(), b.y(), b.z());
    }
    
    glEnd();
}

void display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    double aspect = (double)viewport[2] / (double)viewport[3];
    gluPerspective(fovy, aspect, clipNear, clipFar);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    gluLookAt(0, 0, z, x, y, 0, 0, 1, 0);
    
    if (success) {
        draw();
    }

    glutSwapBuffers();
}

void keyboard(unsigned char key, int x0, int y0)
{
    switch (key) {
        case 27 :
            exit(0);
        case ' ':
            if (!first) mesh.read(path);
            if (first) first = false;
            mesh.remesh(avgEdgeLength, 30.0, iterations, project);
            break;
        case 'a':
            x -= 0.03;
            break;
        case 'd':
            x += 0.03;
            break;
        case 'w':
            y += 0.03;
            break;
        case 's':
            y -= 0.03;
            break;
        case 'r':
            success = mesh.read(path);
            break;
        case 'p':
            project = !project;
            break;
    }
    
    glutPostRedisplay();
}

void special(int i, int x0, int y0)
{
    switch (i) {
        case GLUT_KEY_UP:
            z += 0.03;
            break;
        case GLUT_KEY_DOWN:
            z -= 0.03;
            break;
        case GLUT_KEY_LEFT:
            if (iterations > 1) iterations --;
            break;
        case GLUT_KEY_RIGHT:
            iterations ++;
            break;
    }
    
    std::string title = "Remesh, iterations: " + std::to_string(iterations);
    glutSetWindowTitle(title.c_str());
    
    glutPostRedisplay();
}

int main(int argc, char** argv) {

    success = mesh.read(path);
    for (EdgeCIter e = mesh.edges.begin(); e != mesh.edges.end(); e++) {
        avgEdgeLength += sqrt(e->lengthSquared());
    }
    avgEdgeLength /= (double)mesh.edges.size();
    
    printInstructions();
    glutInitWindowSize(gridX, gridY);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutInit(&argc, argv);
    std::string title = "Remesh, iterations: " + std::to_string(iterations);
    glutCreateWindow(title.c_str());
    init();
    glutDisplayFunc(display);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(special);
    glutMainLoop();
    
    return 0;
}
