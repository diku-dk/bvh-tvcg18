#ifndef RIGID_BODY_GUI_SHADOWMAP_OBJECT_H
#define RIGID_BODY_GUI_SHADOWMAP_OBJECT_H

#include <rigid_body_gui_lighting.h>

#include <gl3_glm.h>
#include <gl3_texture_2d.h>

namespace rigid_body
{
  namespace gui
  {

    class ShadowmapObject
    {
    public:

      LightInfo              m_light_source;
      gl3::Texture2D     m_depth_texture;

      glm::mat4              m_projection_matrix;
      glm::mat4              m_view_matrix;

    };

  }//namesapce gui
}//namespace rigid_body

// RIGID_BODY_GUI_SHADOWMAP_OBJECT_H
#endif