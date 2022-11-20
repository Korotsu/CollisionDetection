#ifndef _GLOBJECT_H_
#define _GLOBJECT_H_

#include <GL/glew.h>
#include <vector>

#include "Maths.h"

class CGLObject
{
public:
	CGLObject();
	~CGLObject();

	std::vector<Vec2>	points;
protected:
	void				CreateBuffers();
	void				BindBuffers();
	void				DestroyBuffers();

	GLuint				m_vertexBufferId;
};

#endif // define _GLOBJECT_H_
