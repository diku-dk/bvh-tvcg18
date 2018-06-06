#ifndef KDOP_TAGS_H
#define	KDOP_TAGS_H

namespace kdop
{
  /**
   * Aggregate tags that are used for dispatching function calls to the
   * appropriate component. E.g. for KDOP handling there is currently the
   * full-fledged sequential version, of which some algorithms have been ported
   * to OpenCL, of which some are from the gProximity paper. 
   */
  
  struct sequential
  {};

  struct dikucl
  {
    struct gproximity {};
  };
  
} // namespace kdop

// KDOP_TAGS_H
#endif

