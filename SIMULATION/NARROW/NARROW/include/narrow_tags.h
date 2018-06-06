#ifndef NARROW_TAGS_H
#define	NARROW_TAGS_H

namespace narrow
{
  /**
   * Aggregate tags that are used for dispatching function calls to the
   * appropriate component. E.g. for NARROW handling of objects there is
   * currently the full-fledged sequential version, of which some algorithms
   * have been ported to OpenCL. 
   */
    
  struct sequential
  {};

  struct dikucl
  {};
  
} // namespace narrow

// NARROW_TAGS_H
#endif

