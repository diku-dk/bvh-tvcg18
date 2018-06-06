#ifndef RIGID_BODY_GUI_SCENE_OBJECT_H
#define RIGID_BODY_GUI_SCENE_OBJECT_H

#include <gl3_glm.h>

namespace rigid_body
{
  namespace gui
  {
    class SceneObject
    {
    public:

      size_t    m_rid;        ///< Rigid body identifier
      size_t    m_gid;        ///< Geometry identifier
      size_t    m_tid;        ///< Texture identifier
      size_t    m_pid;        ///< Shader program identifier

      glm::mat4 m_model_matrix;    ///< Position and orientation of rigid body
      glm::mat4 m_texture_matrix;  ///< texture scaling

      float     m_red;            ///< Object red color intensity
      float     m_green;          ///< Object green color intensity
      float     m_blue;           ///< Object blue color intensity

      float m_aabb_width;         ///< Width of AABB around object.
      float m_aabb_height;        ///< Height of AABB around object.
      float m_aabb_depth;         ///< Depth of AABB around object.

    };

  }//namespace gui
}//namespace rigid_body

// RIGID_BODY_GUI_SCENE_OBJECT_H
#endif