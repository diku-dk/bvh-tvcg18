#ifndef NARROW_DISPATCH_COLLISION_HANDLER_H
#define NARROW_DISPATCH_COLLISION_HANDLER_H

#include <narrow_test_pair.h>
#include <narrow_box_box.h>
#include <narrow_box_sphere.h>
#include <narrow_sphere_box.h>
#include <narrow_sphere_sphere.h>
#include <narrow_tetramesh_tetramesh.h>
#include <narrow_spheres_tetramesh.h>

#include <cassert>
#include <vector>

namespace narrow
{

  namespace details
  {
    template< typename M>
    inline void dispatch_primitives( System<M> const & system, std::vector<TestPair<M> > & test_pairs )
    {
      assert( ! test_pairs.empty() || !"dispatch_primitives : test_pairs are empty" );
      
      typedef typename std::vector<TestPair<M> >::iterator pair_iterator;
      typedef typename Geometry<M>::box_container          box_container;
      typedef typename Geometry<M>::sphere_container       sphere_container;
      typedef typename Geometry<M>::convex_container       convex_container;

      pair_iterator current = test_pairs.begin();
      pair_iterator end     = test_pairs.end();

      for(;current!=end;++current)
      {
        Geometry<M> const & geoA = system.get_geometry( current->obj_a().get_geometry_idx() );
        Geometry<M> const & geoB = system.get_geometry( current->obj_b().get_geometry_idx() );

        box_container      const & boxesA     = geoA.m_boxes;
        sphere_container   const & spheresA   = geoA.m_spheres;
        //convex_container   const & hullsA     = geoA.m_hulls;

        box_container      const & boxesB     = geoB.m_boxes;
        sphere_container   const & spheresB   = geoB.m_spheres;
        //convex_container   const & hullsB     = geoB.m_hulls;

        detail::box_box<M>(
                           boxesA
                           , boxesB
                           , current->t_a()
                           , current->Q_a()
                           , current->t_b()
                           , current->Q_b()
                           , system.params().get_envelope()
                           , current->callback()
                           );

        detail::box_sphere<M>(
                              boxesA
                              , spheresB
                              , current->t_a()
                              , current->Q_a()
                              , current->t_b()
                              , current->Q_b()
                              , system.params().get_envelope()
                              , current->callback()
                              );

        detail::sphere_box<M>(
                              spheresA
                              , boxesB
                              , current->t_a()
                              , current->Q_a()
                              , current->t_b()
                              , current->Q_b()
                              , system.params().get_envelope()
                              , current->callback()
                              );

        detail::sphere_sphere<M>(
                                 spheresA
                                 , spheresB
                                 , current->t_a()
                                 , current->Q_a()
                                 , current->t_b()
                                 , current->Q_b()
                                 , system.params().get_envelope()
                                 , current->callback()
                                 );
/*
        detail::convex_convex<M>(
                           hullsA
                           , hullsB
                           , current->t_a()
                           , current->Q_a()
                           , current->t_b()
                           , current->Q_b()
                           , system.params().get_envelope()
                           , current->callback()
                           );
 */


      }

    }


    template< typename M>
    inline void dispatch_mixed( System<M> const & system, std::vector<TestPair<M> > & test_pairs )
    {
      assert( ! test_pairs.empty() || !"dispatch_mixed : test_pairs are empty" );
      
      typedef typename std::vector<TestPair<M> >::iterator pair_iterator;
      typedef typename Geometry<M>::sphere_container       sphere_container;

      pair_iterator current = test_pairs.begin();
      pair_iterator end     = test_pairs.end();

      for(;current!=end;++current)
      {
        Geometry<M> const & geoA = system.get_geometry( current->obj_a().get_geometry_idx() );
        Geometry<M> const & geoB = system.get_geometry( current->obj_b().get_geometry_idx() );

        //box_container      const & boxesA     = geoA.m_boxes;
        sphere_container   const & spheresA   = geoA.m_spheres;

        //box_container      const & boxesB     = geoB.m_boxes;
        sphere_container   const & spheresB   = geoB.m_spheres;

        if ( geoA.m_tetramesh.has_data() )
        {
          detail::spheres_tetramesh<M>(
                                       spheresB
                                       , current->t_b()
                                       , current->Q_b()
                                       , current->obj_a()
                                       , geoA
                                       , current->callback()
                                        , true
                                       );
        }

        if ( geoB.m_tetramesh.has_data() )
        {
          detail::spheres_tetramesh<M>(
                                       spheresA
                                       , current->t_a()
                                       , current->Q_a()
                                       , current->obj_b()
                                       , geoB
                                       , current->callback()
                                       , false
                                       );
        }

      }
    }

  }//namespace details


  template< typename M>
  inline void dispatch_collision_handlers( System<M> const & system, std::vector<TestPair<M> > const & test_pairs )
  {
    // assert( ! test_pairs.empty() || !"dispatch_collision_handlers : test_pairs are empty" );
    
    typedef typename std::vector<TestPair<M> >::const_iterator pair_iterator;

    std::vector< TestPair<M> > tetramesh_pairs;
    std::vector< TestPair<M> > primitive_pairs;
    std::vector< TestPair<M> > mixed_pairs;

    pair_iterator current = test_pairs.begin();
    pair_iterator end     = test_pairs.end();

    for(;current!=end;++current)
    {
      bool const A_is_tetramesh = system.get_geometry( current->obj_a().get_geometry_idx() ).m_tetramesh.has_data();
      bool const B_is_tetramesh = system.get_geometry( current->obj_b().get_geometry_idx() ).m_tetramesh.has_data();

      if (A_is_tetramesh && B_is_tetramesh)
      {
        tetramesh_pairs.push_back( *current );
      }
      else if (A_is_tetramesh && !B_is_tetramesh )
      {
        mixed_pairs.push_back( *current );
      }
      else if ( !A_is_tetramesh && B_is_tetramesh )
      {
        mixed_pairs.push_back( *current );
      }
      else
      {
        primitive_pairs.push_back( *current );
      }
    }
    
    if ( ! primitive_pairs.empty() )
      details::dispatch_primitives( system, primitive_pairs );
    // if ( ! tetramesh_pairs.empty() )
      details::dispatch_tetramesh_tetramesh( system, tetramesh_pairs );
    if ( ! mixed_pairs.empty() )
      details::dispatch_mixed( system, mixed_pairs );
  }
  
} //namespace narrow

// NARROW_DISPATCH_COLLISION_HANDLER_H
#endif
