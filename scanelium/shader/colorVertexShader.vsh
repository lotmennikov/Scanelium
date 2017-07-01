#version 140

uniform mat4 mvpMatrix;
in vec4 vertex;
in vec3 color;

out vec3 v_color;

void main()
{
    gl_Position = mvpMatrix * vertex;
    v_color = color;
}
