#version 140

uniform mat4 mvpMatrix;
in vec4 vertex;




void main()
{
    gl_Position = mvpMatrix * vertex;
}
