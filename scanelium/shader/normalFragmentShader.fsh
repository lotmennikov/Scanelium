#version 140

in vec3 v_normal;
out vec4 fragColor;

void main(void)
{
	vec3 v_normalized = normalize(v_normal);
	vec3 color = vec3(0.9, 0.9, 0.9);
	vec3 dirLight = vec3(0.0, 0.0, -1.0);
	float intens = max(0.0, dot(v_normalized, dirLight));
	
    fragColor = vec4(intens * color, 0.0);
}
