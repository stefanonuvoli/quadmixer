#include "gldrawtext.h"

#include <GL/glut.h>

/* Draw a character string
*
* @param x        The x position
* @param y        The y position
* @param z        The z position
* @param string   The character string
*/
void drawTextGL(float x, float y, float z, const std::string& str)
{
    GLfloat color[3] = { 1.0, 0.0, 0.0 };

    glRasterPos3f(x, y, z);

    glColor3fv(color);
    for (const char& c : str)
    {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, c);
    }
}
