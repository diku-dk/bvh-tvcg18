#include <util_log.h>

#include <string>

#ifndef __DEVILLIB__
namespace gl3
{
  namespace il
  {
    void framegrab(std::string const & filename)
    {
      // devIL not found, dummy function to avoid compilation error
    }
  }
}

#else

#include <gl3.h>

#include <IL/ilut.h>
#include <IL/ilu.h>
#include <IL/il.h>

#include <vector>

using namespace std;

namespace gl3
{
  namespace il
  {
    
    void gl_check_errors()
    {
      GLuint err;
      while ( (err = glGetError()) != GL_NO_ERROR )
      {
        util::Log logging;

        logging << "GL: " << glErrorString(err) << util::Log::newline();
      }
    }
    
    void ilu_check_errors()
    {
      ILenum err;
      while ( (err = ilGetError()) != IL_NO_ERROR )
      {
        util::Log logging;

        logging << "DEVIL: " << iluErrorString(err) << util::Log::newline();
      }
    }
    
    void framegrab(std::string const & filename)
    {
      gl_check_errors();
      ilu_check_errors();
      
      ilInit();
      iluInit();
      ilutInit();
      gl_check_errors();
      ilu_check_errors();
      
      ilEnable(IL_FILE_OVERWRITE);
      gl_check_errors();
      ilu_check_errors();
      
      ILuint imageID = ilGenImage();
      gl_check_errors();
      ilu_check_errors();
      
      ilBindImage(imageID);
      gl_check_errors();
      ilu_check_errors();
      
      GLint m_viewport[4];
      
      glGetIntegerv( GL_VIEWPORT, m_viewport );
      gl_check_errors();
      ilu_check_errors();
      
      int const x = m_viewport[0]; int const y = m_viewport[1];
      int const width = m_viewport[2]; int const height = m_viewport[3];
      
      std::vector<unsigned char> imData;
      imData.resize(width * height * sizeof(unsigned char) * 3);
      
      glReadPixels(x, y, width, height, GL_RGB, GL_UNSIGNED_BYTE, &imData[0]);
      gl_check_errors();
      ilu_check_errors();
      
      ilTexImage( width, height, 1, 3, IL_RGB, IL_UNSIGNED_BYTE, &imData[0]);
      gl_check_errors();
      ilu_check_errors();
      
      ilSetData(&imData[0]);
      gl_check_errors();
      ilu_check_errors();
      
      ilSaveImage(filename.c_str());
      gl_check_errors();
      ilu_check_errors();
      
      ilDeleteImage(imageID);
      gl_check_errors();
      ilu_check_errors();
      
      ilShutDown();
      gl_check_errors();
      ilu_check_errors();
    }
  }//namespace il
}//namespace GL3
#endif
