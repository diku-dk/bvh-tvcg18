#version 330 core

out vec4 color;

uniform sampler2D uvmap;

in vec2 UV;

void main()
{
  vec3 normal = texture(uvmap, UV).rgb;
  vec3 biased_normal = (normal + vec3(1,1,1)) * 0.5;

  color = vec4(biased_normal, 1);
  // color = vec4(normal, 1);
}
