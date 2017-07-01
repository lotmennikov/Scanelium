#version 140

uniform mat4 mvpMatrix;
in vec4 vertex;
in vec3 normal;

out vec3 v_normal;

void main()
{
    gl_Position = mvpMatrix * vertex;
    v_normal = normal;
}
