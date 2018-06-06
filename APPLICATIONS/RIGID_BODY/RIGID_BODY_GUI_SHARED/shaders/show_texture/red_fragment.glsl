#version 330 core

out vec4 color;

uniform sampler2D uvmap;

in vec2 UV;

void main()
{
  float red = texture(uvmap, UV).r;
  color = vec4(red, red, red, 1);
}
