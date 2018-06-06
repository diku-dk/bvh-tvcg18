#ifndef NARROW_SPHERE_SPHERE_H
#define NARROW_SPHERE_SPHERE_H

#include <geometry.h>

#include <tiny.h>

namespace narrow
{
  
  namespace detail
  {

    /**
     * Dispatch pair-wise testing of shapes from two different objects.
     *
     * @tparam shape_containerA         A container type of shape types coming from the narrowphase object A.
     * @tparam shape_containerB         A container type of (possible another then A)  of shape types coming from the narrowphase object B.
     * @tparam M                        The math types used.
     * @tparam proximity_container      A container type of proximity info.
     * @tparam Params                   A Policy/Trait class holding parameter settings for the internal workings of the CONVEX algorithm.
     *
     * @param A        Shapes from object A.
     * @param B        Shapes from object B.
     * @param input    Current proximity information about the two narrowphase objects.
     * @param results  Upon return any new proximity information will have been added to this container.
     * @param tag      Tag dispatching used for transfering parameter settings to the internal settings.
     */
    
    template<typename M>
    inline void sphere_sphere(
                              typename Geometry<M>::sphere_container const & A
                              , typename Geometry<M>::sphere_container const & B
                              , typename M::vector3_type const & tA
                              , typename M::quaternion_type const & qA
                              , typename M::vector3_type const & tB
                              , typename M::quaternion_type const & qB
                              , typename M::real_type const & envelope
                              , typename geometry::ContactsCallback<typename M::vector3_type> & callback
                              )
    {
      using std::min;
      
      typedef typename M::value_traits    VT;
      typedef typename M::coordsys_type   C;
      typedef typename M::vector3_type    V;

      typedef typename Geometry<M>::sphere_container::const_iterator sphere_iterator;
      
      assert( envelope > VT::zero() || !"sphere_sphere(): collision envelope must be positive");
      
      if( A.empty() || B.empty())
        return;
      
      C bodyAtoWCS = C(tA, qA);
      C bodyBtoWCS = C(tB, qB);
      
      for( sphere_iterator a = A.begin(); a!=A.end(); ++a )
      {
        for( sphere_iterator b = B.begin(); b!=B.end(); ++b )
        {
          C shapeAtobodyA = C(a->transform().T(), a->transform().Q());
          C shapeBtobodyB = C(b->transform().T(), b->transform().Q());
          C shapeAtoWCS = tiny::prod(shapeAtobodyA, bodyAtoWCS);
          C shapeBtoWCS = tiny::prod(shapeBtobodyB, bodyBtoWCS);
          
          geometry::Sphere<V> const A = geometry::make_sphere( shapeAtoWCS.T(), a->radius());
          geometry::Sphere<V> const B = geometry::make_sphere( shapeBtoWCS.T(), b->radius());
          
          geometry::contacts_sphere_sphere(
                                           A
                                           , B
                                           , envelope * min(a->scale(), b->scale())
                                           , callback
                                           );
        }
      }
    }
    

  } // namespace detail

} //namespace narrow

// NARROW_SPHERE_SPHERE_H
#endif
