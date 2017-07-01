#version 140
uniform sampler2D colorTexture;

in vec2 tex_coord;
out vec4 fragColor;

void main(void)
{
    fragColor = texture(colorTexture, tex_coord);
}
