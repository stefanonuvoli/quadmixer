/* Copyright(C) 2019


The authors of

QuadMixer: Layout Preserving Blending of Quadrilateral Meshes
SIGGRAPH Asia 2019


All rights reserved.
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
****************************************************************************/

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
