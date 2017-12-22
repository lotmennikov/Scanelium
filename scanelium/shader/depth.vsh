#version 330

uniform mat4 pMatrix;
uniform mat4 vmMatrix;
in vec4 vertex;
out vec3 v_vertex;

void main()
{
    gl_Position = pMatrix * vmMatrix * vertex;
	v_vertex = vec3(vmMatrix * vertex);
}
