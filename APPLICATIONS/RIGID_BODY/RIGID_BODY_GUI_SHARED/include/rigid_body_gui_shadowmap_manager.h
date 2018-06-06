#ifndef RIGID_BODY_GUI_SHADOWMAP_MANAGER_H
#define RIGID_BODY_GUI_SHADOWMAP_MANAGER_H

#include <rigid_body_gui_shadowmap_object.h>

namespace rigid_body
{
  namespace gui
  {

    class ShadowmapManager
    {
    public:

      std::vector<ShadowmapObject> m_objects;

    public:

      void clear()
      {
        for(unsigned int i = 0u; i < m_objects.size(); ++i)
        {
          m_objects[i].m_depth_texture.clear();
        }
        m_objects.clear();
      }

      void add(ShadowmapObject const & object)
      {
        m_objects.push_back(object);
      }

    };

  }//namespace gui
}//namespace rigid_body

// RIGID_BODY_GUI_SHADOWMAP_MANAGER_H
#endif


