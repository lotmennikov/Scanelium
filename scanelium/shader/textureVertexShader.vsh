#version 140

uniform mat4 mvpMatrix;
in vec4 vertex;
in vec2 texcoord;

out vec2 tex_coord;

void main()
{
    tex_coord = texcoord;
    gl_Position = mvpMatrix * vertex;
}
