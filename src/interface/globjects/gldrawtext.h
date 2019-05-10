#ifndef GLDRAWTEXT_H
#define GLDRAWTEXT_H

#include <string>

#include <GL/glut.h>

void drawTextGL(float x, float y, float z, const std::string& str, void* font);

#endif // GLDRAWTEXT_H
