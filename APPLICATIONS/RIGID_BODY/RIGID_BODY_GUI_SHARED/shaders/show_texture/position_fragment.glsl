#version 330 core

out vec4 color;

uniform sampler2D uvmap;

uniform float min_x;
uniform float min_y;
uniform float min_z;
uniform float max_x;
uniform float max_y;
uniform float max_z;

in vec2 UV;

void main()
{
  vec4 position_world = texture(uvmap, UV);

  float x = position_world.x;
  float y = position_world.y;
  float z = position_world.z;

  float biased_x = (position_world.x - min_x) / (max_x - min_x);
  float biased_y = (position_world.y - min_y) / (max_y - min_y);
  float biased_z = (position_world.z - min_z) / (max_z - min_z);

  color = vec4(biased_x, biased_y, biased_z, 1.0);
  //color = vec4(x, y, z, 1.0);
}
