#ifndef RIGID_BODY_GUI_SCENE_MANAGER_H
#define RIGID_BODY_GUI_SCENE_MANAGER_H

#include <rigid_body_gui_scene_object.h>

namespace rigid_body
{
  namespace gui
  {

    class SceneManager
    {
    public:

      std::vector<SceneObject> m_objects;

    public:

      void clear()
      {
        m_objects.clear();
      }

      void add(SceneObject const & object)
      {
        m_objects.push_back(object);
      }

    };

  }//namespace gui
}//namespace rigid_body

// RIGID_BODY_GUI_SCENE_MANAGER_H
#endif


