#version 330 core

out vec4 color;

uniform sampler2D uvmap;

in vec2 UV;

void main()
{
  color = texture(uvmap, UV);
}
