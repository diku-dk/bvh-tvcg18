#version 330 core

out vec4 color;

uniform sampler2D uvmap;
uniform float z_near;
uniform float z_far;

in vec2 UV;

float linearize_depth(float z)
{
  float n = z_near;   // near plane
  float f = z_far; // far plane

  return(2.0 * n) / (f + n - z * (f - n));
}

void main()
{
  float depth = linearize_depth(texture(uvmap, UV).r );

//  bool test =  (0.0 < depth) &&   (depth< 1.0);

//  if(test)
    color = vec4(depth, depth, depth, 1.0);
//  else
//    color = vec4(1.0 ,1.0 , 1.0 ,1.0);
}
