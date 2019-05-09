#include "gldrawtext.h"

/* Draw a character string
*
* @param x        The x position
* @param y        The y position
* @param z        The z position
* @param string   The character string
*/
void drawTextGL(float x, float y, float z, const std::string& str, void* font)
{
    glRasterPos3f(x, y, z);
    for (const char& c : str)
    {
        glutBitmapCharacter(font, c);
    }
}
