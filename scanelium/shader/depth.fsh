#version 330

in vec3 v_vertex;
out float v_z;

void main(void)
{
    v_z = v_vertex.z;
}
