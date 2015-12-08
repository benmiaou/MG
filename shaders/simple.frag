#version 330 core
uniform vec3 color = vec3(1.f,0,0);

out vec4 out_color;


void main()
{
	out_color = vec4(color,1.f);
}
