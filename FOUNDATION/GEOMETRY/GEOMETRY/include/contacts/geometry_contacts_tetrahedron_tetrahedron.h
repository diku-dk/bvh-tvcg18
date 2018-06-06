#ifndef GEOMETRY_CONTACTS_TETRAHEDRON_TETRAHEDRON_H
#define GEOMETRY_CONTACTS_TETRAHEDRON_TETRAHEDRON_H

#include <contacts/geometry_contacts_callback.h>

#include <types/geometry_tetrahedron.h>
#include <types/geometry_triangle.h>

#include <closest_points/geometry_closest_points_line_line.h>

#include <tiny_precision.h>        // needed for tiny::working_precision

#include <cmath>                   // needed for std::min and std::max
#include <vector>

namespace geometry
{
  namespace details
  {
    template<typename V>
    class UnscaledPlane
    {
    public:

      V m_normal;    ///< The plane normal NOT necessary a unit-normal
      V m_point;     ///< A point on the plane (any point)

    };

    template<typename V>
    inline UnscaledPlane<V> make_unscaled_plane(V const & p0,V const & p1,V const & p2)
    {
      UnscaledPlane<V> plane;

      typedef typename V::real_type T;

      V const m0 = cross(p1-p0, p2-p0);
      V const m1 = cross(p2-p1, p0-p1);
      V const m2 = cross(p0-p2, p1-p2);

      T const l0 = inner_prod(m0,m0);
      T const l1 = inner_prod(m1,m1);
      T const l2 = inner_prod(m2,m2);

      if (l0 >= l1 && l0 >= l2)
      {
        plane.m_normal = m0;
        plane.m_point  = p0;
      }
      if (l1 >= l0 && l1 >= l2)
      {
        plane.m_normal = m1;
        plane.m_point  = p1;
      }
      if (l2 >= l1 && l2 >= l0)
      {
        plane.m_normal = m2;
        plane.m_point  = p2;
      }

      return plane;
    }

    template<typename V>
    inline UnscaledPlane<V> make_unscaled_plane(Triangle<V> const & triangle)
    {
      return make_unscaled_plane(triangle.p(0),triangle.p(1),triangle.p(2));
    }

    template<typename V>
    inline UnscaledPlane<V> make_unscaled_plane(V const & normal, V const & point)
    {
      UnscaledPlane<V> plane;

      plane.m_normal = normal;
      plane.m_point  = point;

      return plane;
    }

    template<typename V>
    inline typename V::real_type get_signed_distance( V const & q, UnscaledPlane<V> const & plane)
    {
      return inner_prod( plane.m_normal, q - plane.m_point);
    }

    template<typename V>
    inline bool inside_planes(V const & q, std::vector<UnscaledPlane<V> > const & planes)
    {
      assert(planes.size() == 4u || !"inside_planes(): internal error");

      typedef typename V::value_traits  VT;
      typedef typename V::real_type     T;

      T const d0 = get_signed_distance( q, planes[0] );
      T const d1 = get_signed_distance( q, planes[1] );
      T const d2 = get_signed_distance( q, planes[2] );
      T const d3 = get_signed_distance( q, planes[3] );

      if (d0 > VT::zero())
        return false;
      if (d1 > VT::zero())
        return false;
      if (d2 > VT::zero())
        return false;
      if (d3 > VT::zero())
        return false;

      return true;
    }

    template<typename V>
    inline V make_intersection(V const & a, V const & b, UnscaledPlane<V> const & plane)
    {
      using std::fabs;

      typedef typename V::real_type     T;

      T const dA = fabs( get_signed_distance( a, plane )  );
      T const dB = fabs( get_signed_distance( b, plane )  );
      T const t = dA / (dA+dB);
      V const q = a + (b-a)*t;

      return q;
    }

    template<typename V>
    inline typename V::real_type make_intersection_parameter(V const & a, V const & b, UnscaledPlane<V> const & plane)
    {
      using std::fabs;

      typedef typename V::real_type     T;

      T const dA = fabs( get_signed_distance( a, plane )  );
      T const dB = fabs( get_signed_distance( b, plane )  );
      T const t = dA / (dA+dB);

      return t;
    }

    template<typename V>
    inline bool is_crossing_plane(V const & a, V const & b, UnscaledPlane<V> const & plane)
    {
      typedef typename V::value_traits  VT;
      typedef typename V::real_type     T;

      T const dA = get_signed_distance( a, plane );
      T const dB = get_signed_distance( b, plane );

      if( dA >= VT::zero() && dB >= VT::zero() )
        return false;

      if( dA <= VT::zero() && dB <= VT::zero() )
        return false;

      return true;
    }

    /**
     * Determines the SAT axis with minimum overlap and propose to use this as contact normal
     */
    template< typename V>
    inline bool pick_sat_normal(
                            Tetrahedron<V> const & tetA
                            , Tetrahedron<V> const & tetB
                            , V & n
                            )
    {
      using std::min;
      using std::max;

      typedef typename V::value_traits VT;
      typedef typename V::real_type     T;

      std::vector<V> A(4,V::zero());
      std::vector<V> B(4,V::zero());

      std::vector<V> axes;
      axes.reserve(44u);

      A[0] = tetA.p(0);
      A[1] = tetA.p(1);
      A[2] = tetA.p(2);
      A[3] = tetA.p(3);

      B[0] = tetB.p(0);
      B[1] = tetB.p(1);
      B[2] = tetB.p(2);
      B[3] = tetB.p(3);

      for(unsigned int p = 0u; p < 4u; ++p)
      {
        Triangle<V>               const triangleA = get_opposite_face(p, tetA);
        details::UnscaledPlane<V> const planA     = details::make_unscaled_plane(triangleA);

        axes.push_back ( tiny::unit( planA.m_normal ) );

        Triangle<V>               const triangleB = get_opposite_face(p, tetB);
        details::UnscaledPlane<V> const planB     = details::make_unscaled_plane(triangleB);

        axes.push_back ( tiny::unit( planB.m_normal ) );
      }

      unsigned int const edge_table[6][2] = {
        { 0, 1}
        , { 0, 2}
        , { 0, 3}
        , { 1, 2}
        , { 1, 3}
        , { 2, 3}
      };

      for(unsigned int a=0u;a < 6u;++a)
      {
        V const edgeA = A[edge_table[a][1]] - A[edge_table[a][0]];

        for(unsigned int b = 0u; b < 6u; ++b)
        {
          V const edgeB = B[edge_table[b][1]] - B[edge_table[b][0]];
          V const AxB   = tiny::cross( edgeA, edgeB );
          T const l     = tiny::norm(AxB);

          if(l > tiny::working_precision<T>() )
          {
            V const axis = AxB / l;

            axes.push_back( axis );
          }
        }
      }

      unsigned int const N = axes.size();

      std::vector<T> a_min( N, VT::highest() );
      std::vector<T> b_min( N, VT::highest() );
      std::vector<T> a_max( N, VT::lowest()  );
      std::vector<T> b_max( N, VT::lowest()  );

      T min_overlap = VT::lowest();

      for(size_t i=0u;i < N; ++i)
      {

        for( typename std::vector<V>::const_iterator p_a = A.begin(); p_a != A.end(); ++p_a)
        {
          T const d = inner_prod( (*p_a), axes[i] );
          a_min[i] = min( a_min[i], d);
          a_max[i] = max( a_max[i], d);
        }
        for( typename std::vector<V>::const_iterator p_b = B.begin(); p_b != B.end(); ++p_b)
        {
          T const d = inner_prod( (*p_b), axes[i] );
          b_min[i] = min( b_min[i], d);
          b_max[i] = max( b_max[i], d);
        }

        if(a_max[i] < b_min[i])
          return false;

        if(b_max[i] < a_min[i])
          return false;

        if(a_min[i] <= b_min[i] &&  b_min[i] <= a_max[i])
        {
          T const overlap = b_min[i] - a_max[i];

          if(overlap > min_overlap)
          {
            min_overlap = overlap;
            n = axes[i];
          }
        }
        
        if(b_min[i] <= a_min[i] &&  a_min[i] <= b_max[i])
        {
          T const overlap = a_min[i] - b_max[i];
          
          if(overlap > min_overlap)
          {
            min_overlap = overlap;
            n = -axes[i];
          }
        }
      }
      
      return (min_overlap <= VT::zero());
    }

    /**
     * This is just an overloaded version of pick_sat_normal that makes it
     * convenient to pass surface information arguments as dummy data.
     */
    template< typename V>
    inline bool pick_sat_normal(
                                Tetrahedron<V> const & tetA
                                , Tetrahedron<V> const & tetB
                                , std::vector<bool> const & surface_A
                                , std::vector<bool> const & surface_B
                                , V & n
                                )
    {
      return pick_sat_normal(tetA, tetB, n );
    }

    /**
     * Determines normal as the separation axis with minimum overlap
     * under the restricted that only separation axes generated from
     * surface information are considered valid.
     */
    template< typename V>
    inline bool pick_restricted_sat_normal(
                            Tetrahedron<V> const & tetA
                            , Tetrahedron<V> const & tetB
                            , std::vector<bool> const & surface_A
                            , std::vector<bool> const & surface_B
                            , V & n
                            )
    {
      using std::min;
      using std::max;

      typedef typename V::value_traits VT;
      typedef typename V::real_type     T;

      std::vector<V> A(4,V::zero());
      std::vector<V> B(4,V::zero());

      std::vector<V> axes;
      axes.reserve(44u);

      A[0] = tetA.p(0);
      A[1] = tetA.p(1);
      A[2] = tetA.p(2);
      A[3] = tetA.p(3);

      B[0] = tetB.p(0);
      B[1] = tetB.p(1);
      B[2] = tetB.p(2);
      B[3] = tetB.p(3);

      for(unsigned int p = 0u; p < 4u; ++p)
      {
        Triangle<V>               const triangleA = get_opposite_face(p, tetA);
        details::UnscaledPlane<V> const planA     = details::make_unscaled_plane(triangleA);

        if (surface_A[p])
          axes.push_back ( tiny::unit( planA.m_normal ) );

        Triangle<V>               const triangleB = get_opposite_face(p, tetB);
        details::UnscaledPlane<V> const planB     = details::make_unscaled_plane(triangleB);

        if (surface_B[p])
          axes.push_back ( tiny::unit( planB.m_normal ) );
      }

      unsigned int const edge_table[6][2] = {
        { 0, 1}
        , { 0, 2}
        , { 0, 3}
        , { 1, 2}
        , { 1, 3}
        , { 2, 3}
      };

      unsigned int const face_tabel[6][2] = {
        { 2, 3}
        , { 1, 3}
        , { 1, 2}
        , { 0, 3}
        , { 0, 2}
        , { 0, 1}
      };

      for(unsigned int a=0u;a < 6u;++a)
      {
        bool const is_surface_edge_A = surface_A[ face_tabel[a][1] ] || surface_A[ face_tabel[a][0] ];

        if (!is_surface_edge_A )
          continue;

        V const edgeA = A[edge_table[a][1]] - A[edge_table[a][0]];

        for(unsigned int b = 0u; b < 6u; ++b)
        {
          bool const is_surface_edge_B = surface_B[ face_tabel[b][1] ] ||  surface_B[ face_tabel[b][0] ];

          if (!is_surface_edge_B )
            continue;

          V const edgeB = B[edge_table[b][1]] - B[edge_table[b][0]];
          V const AxB   = tiny::cross( edgeA, edgeB );
          T const l     = tiny::norm(AxB);

          if(l > tiny::working_precision<T>() )
          {
            V const axis = AxB / l;

            axes.push_back( axis );
          }
        }
      }

      unsigned int const N = axes.size();

      std::vector<T> a_min( N, VT::highest() );
      std::vector<T> b_min( N, VT::highest() );
      std::vector<T> a_max( N, VT::lowest()  );
      std::vector<T> b_max( N, VT::lowest()  );

      T min_overlap = VT::lowest();

      for(size_t i=0u;i < N; ++i)
      {

        for( typename std::vector<V>::const_iterator p_a = A.begin(); p_a != A.end(); ++p_a)
        {
          T const d = inner_prod( (*p_a), axes[i] );
          a_min[i] = min( a_min[i], d);
          a_max[i] = max( a_max[i], d);
        }
        for( typename std::vector<V>::const_iterator p_b = B.begin(); p_b != B.end(); ++p_b)
        {
          T const d = inner_prod( (*p_b), axes[i] );
          b_min[i] = min( b_min[i], d);
          b_max[i] = max( b_max[i], d);
        }

        if(a_max[i] < b_min[i])
          return false;

        if(b_max[i] < a_min[i])
          return false;

        if(a_min[i] <= b_min[i] &&  b_min[i] <= a_max[i])
        {
          T const overlap = b_min[i] - a_max[i];

          if(overlap > min_overlap)
          {
            min_overlap = overlap;
            n = axes[i];
          }
        }

        if(b_min[i] <= a_min[i] &&  a_min[i] <= b_max[i])
        {
          T const overlap = a_min[i] - b_max[i];
          
          if(overlap > min_overlap)
          {
            min_overlap = overlap;
            n = -axes[i];
          }
        }
      }
      
      return (min_overlap <= VT::zero());
    }

    /**
     * This method determines the contact normal to be the normal-direction 
     * that are defined by the two most opposing surfaces.
     */
    template< typename V>
    inline bool pick_most_opposing_surface_normal(
                            Tetrahedron<V> const & tetA
                            , Tetrahedron<V> const & tetB
                            , std::vector<bool> const & surface_A
                            , std::vector<bool> const & surface_B
                            , V & n
                            )
    {
      using std::min;
      using std::max;

      typedef typename V::value_traits VT;
      typedef typename V::real_type     T;

      bool found_normal = false;

      T best_fit = VT::zero();

      for(unsigned int a = 0u; a < 4u; ++a)
      {
        if (!surface_A[a])
          continue;

        Triangle<V>               const triangleA = get_opposite_face(a, tetA);
        details::UnscaledPlane<V> const planA     = details::make_unscaled_plane(triangleA);

        V const nA = unit(planA.m_normal);

        for(unsigned int b = 0u; b < 4u; ++b)
        {
          if (!surface_B[b])
            continue;

          Triangle<V>               const triangleB = get_opposite_face(b, tetB);
          details::UnscaledPlane<V> const planB     = details::make_unscaled_plane(triangleB);

          V const nB = unit(planB.m_normal);

          T const test = tiny::inner_prod(nA, nB);

          if (test >= VT::zero() ) // surfaces must be opposing each other
            continue;

          //--- Now we know that planes are opposing each other
          //--- Next we will test if A is in front og B face and vice versa

          V const & pB = tetB.p(b);
          V const & pA = tetA.p(a);

          T const testA =  tiny::inner_prod( planA.m_normal, pB - planA.m_point) ;
          T const testB =  tiny::inner_prod( planB.m_normal, pA - planB.m_point) ;

          if (testA < VT::zero())
            continue;

          if (testB < VT::zero())
            continue;

          if( test < best_fit)
          {
            n = nA;
            best_fit = test;
            found_normal = true;
          }

        }
      }
      return found_normal;
    }

    /**
     * This sub-routine just generates all possible intersection points
     * between the two tetrahedra and then projects them onto the contact
     * plane using the contact normal information. Finally, it computes
     * penetration depth based on the given normal direction and filters 
     * away any redundant contact points before reporting them with the
     * callback function.
     */
    template< typename V>
    inline bool generate_contacts_from_intersection(
                                                    Tetrahedron<V> const & A
                                                    , Tetrahedron<V> const & B
                                                    , ContactsCallback<V> & callback
                                                    , V const & n
                                                    )
    {
      using std::min;
      using std::max;

      typedef typename V::value_traits VT;
      typedef typename V::real_type     T;

      std::vector<V> contacts;
      contacts.reserve(16);

      std::vector<Triangle<V> >                trianglesA(4u);
      std::vector<Triangle<V> >                trianglesB(4u);

      std::vector<details::UnscaledPlane<V> >  planesA(4u);
      std::vector<details::UnscaledPlane<V> >  planesB(4u);

      for (unsigned int v =0u; v < 4u; ++v)
      {
        trianglesA[v] = get_opposite_face( v, A );
        trianglesB[v] = get_opposite_face( v, B );
        planesA[v]    = details::make_unscaled_plane(trianglesA[v]);
        planesB[v]    = details::make_unscaled_plane(trianglesB[v]);
      }

      for (unsigned int v =0u; v < 4u; ++v)
      {
        if( details::inside_planes( B.p(v), planesA) )
          contacts.push_back(B.p(v));

        if( details::inside_planes( A.p(v), planesB) )
          contacts.push_back(A.p(v));
      }

      unsigned int edge_table[6][2] = {
        {0, 1}
        , {0, 2}
        , {0, 3}
        , {1, 2}
        , {1, 3}
        , {2, 3}
      };

      for (unsigned int e = 0u; e < 6u; ++e)
      {
        unsigned int const i = edge_table[e][0u];
        unsigned int const j = edge_table[e][1u];

        V const & Ai = A.p(i);
        V const & Aj = A.p(j);
        V const & Bi = B.p(i);
        V const & Bj = B.p(j);

        for (unsigned int p = 0u; p < 4u; ++p)
        {
          details::UnscaledPlane<V> const & planeA = planesA[p];
          details::UnscaledPlane<V> const & planeB = planesB[p];

          if (details::is_crossing_plane(Bi, Bj, planeA) )
          {
            V const qB = details::make_intersection(Bi, Bj, planeA);

            if( details::inside_planes(qB, planesA) )
              contacts.push_back(qB);
          }

          if (details::is_crossing_plane(Ai, Aj, planeB) )
          {
            V const qA = details::make_intersection( Ai, Aj, planeB);

            if( details::inside_planes(qA, planesB) )
              contacts.push_back(qA);
          }
        }
      }

      if(contacts.empty())
      {
        return false;
      }

      // Now we know we have found a bunch of contacts, we will now try to find
      // the best "contact plane" and project all contact points onto that
      // plane and compute penetration measures with respect to that plane too.
      T min_val = VT::highest();
      T max_val = VT::lowest();

      {
        for( typename std::vector<V>::iterator p = contacts.begin(); p!= contacts.end(); ++p)
        {
          T const d = inner_prod( (*p), n );

          min_val = min( min_val, d );
          max_val = max( max_val, d );
        }
      }

      T const depth = min_val - max_val;

      // Project contact points onto cotact plane
      V const mid =  n * (max_val + min_val)*VT::half();

      for( typename std::vector<V>::iterator p = contacts.begin(); p!= contacts.end(); ++p)
      {
        (*p) = (*p) - inner_prod( n, ( (*p) - mid ) ) * n;
      }

      // Some contacts might have been projected to the same point in the
      // contact plane, so we filter away redundant information before
      // using the callback to report the computed contact point.
      typename std::vector<V>::iterator p    = contacts.begin();
      for(; p!= contacts.end(); ++p)
      {
        bool unique = true;
        
        typename std::vector<V>::iterator q = contacts.begin();
        for(; q != p; ++q)
        {
          if( tiny::norm_1( (*q) - (*p) ) < tiny::working_precision<T>())
          {
            unique = false;
            break;
          }
        }
        
        if(unique)
        {
          callback( (*p), n, depth);
        }
      }
      
      return true;
    }

  }// end of namespace details


  /**
   * Algorithm type tags for calling the appropriate contact point generation algorithm that is desired.
   */
  struct TRIANGLE_INTERSECTION {};
  struct VERTEX_ONLY_INTERSECTION {};
  struct CONSISTENT_VERTEX {};
  struct SAT {};
  struct RESTRICTED_SAT {};
  struct MOST_OPPOSING_SURFACES {};

  template< typename V>
  inline bool contacts_tetrahedron_tetrahedron(
                                               Tetrahedron<V> const & A
                                               , Tetrahedron<V> const & B
                                               , ContactsCallback<V> & callback
                                               , std::vector<bool> const & surface_A
                                               , std::vector<bool> const & surface_B
                                               , TRIANGLE_INTERSECTION const & /*algorithm_tag*/
  )
  {

    assert(surface_A.size() == 4u                                          || !"contacts_tetrahedron_tetrahedron(): internal error, must have four surface map values");
    assert( (surface_A[0] || surface_A[1] || surface_A[2] || surface_A[3]) || !"contacts_tetrahedron_tetrahedron(): internal error, tetrahedron A must have at least one surface face");
    assert(surface_B.size() == 4u                                          || !"contacts_tetrahedron_tetrahedron(): internal error, must have four surface map values");
    assert( (surface_B[0] || surface_B[1] || surface_B[2] || surface_B[3]) || !"contacts_tetrahedron_tetrahedron(): internal error, tetrahedron B must have at least one surface face");

    using std::min;
    using std::max;

    typedef typename V::value_traits VT;
    typedef typename V::real_type     T;

    T const small_number = tiny::working_precision<T>();

    unsigned int count = 0u;

    std::vector<Triangle<V> >                trianglesA(4u);
    std::vector<Triangle<V> >                trianglesB(4u);

    std::vector<details::UnscaledPlane<V> >  planesA(4u);
    std::vector<details::UnscaledPlane<V> >  planesB(4u);

    //--- Preprocessing tetrahedrons for fast triangle face lookup info --------
    for (unsigned int v =0u; v < 4u; ++v)
    {
      trianglesA[v] = get_opposite_face( v, A );
      trianglesB[v] = get_opposite_face( v, B );
      planesA[v]    = details::make_unscaled_plane(trianglesA[v]);
      planesB[v]    = details::make_unscaled_plane(trianglesB[v]);
    }

    //--- Search for a pair of triangle surfaces from tetrahedron A and B
    for (unsigned int a =0u; a < 4u; ++a)
    {
      if(!surface_A[a])
        continue;

      for (unsigned int b =0u; b < 4u; ++b)
      {
        if(!surface_B[b])
          continue;

        //--- We now know we have two surface triangles and we wish to generate
        //--- contact between these two surface triangles.

        Triangle<V>               const & triA = trianglesA[a];
        Triangle<V>               const & triB = trianglesB[b];
        details::UnscaledPlane<V> const & plA  = planesA[a];
        details::UnscaledPlane<V> const & plB  = planesB[b];

        //--- Test vertex-face contacts of A vs. B
        for (unsigned int k = 0u; k < 3u; ++k)
        {
          V    const & pk          = triA.p(k);
          T    const   depth       = tiny::inner_prod(plB.m_normal, pk - plB.m_point);

          if (depth > VT::zero())
            continue;

          V    const & n           = plB.m_normal;
          bool         is_inside_B = true;

          for (unsigned int i = 0u; i < 3u; ++i)
          {
            unsigned int j = (i+1) % 3;

            V const & pi = triB.p(i);
            V const & pj = triB.p(j);
            V const e    = pj - pi;
            V const m    = tiny::cross(n,e);
            T const tst  = tiny::inner_prod(m, pk - pi);

            if(tst < VT::zero() )
            {
              is_inside_B = false;
              break;
            }
          }

          if(is_inside_B)
          {
            callback( pk, -n, depth);
            ++count;
          }

        }

        //--- Test vertex-face contacts of B vs. A
        for (unsigned int k = 0u; k < 3u; ++k)
        {
          V    const & pk          = triB.p(k);
          T    const   depth       = tiny::inner_prod(plA.m_normal, pk - plA.m_point);

          if (depth > VT::zero())
            continue;

          V    const & n           = plA.m_normal;
          bool         is_inside_A = true;

          for (unsigned int i = 0u; i < 3u; ++i)
          {
            unsigned int j = (i+1) % 3;

            V const & pi = triA.p(i);
            V const & pj = triA.p(j);
            V const e    = pj - pi;
            V const m    = tiny::cross(n,e);
            T const tst  = tiny::inner_prod(m, pk - pi);

            if(tst < VT::zero() )
            {
              is_inside_A = false;
              break;
            }
          }

          if(is_inside_A)
          {
            callback( pk, n, depth);
            ++count;
          }
        }

        //--- Test all edge-edge contacts of A and B
        for (unsigned int k = 0u; k < 3u; ++k)
        {
          unsigned int         m  = (k+1) % 3;
          V            const & pk = triA.p(k);
          V            const & pm = triA.p(m);
          V            const   eA = unit(pm - pk);

          for (unsigned int i = 0u; i < 3u; ++i)
          {
            unsigned int j = (i+1) % 3;

            V const & pi = triB.p(i);
            V const & pj = triB.p(j);
            V const eB    = unit(pj - pi);

            V const eA_X_eB = cross(eA,eB);

            bool const too_parallel = norm(eA_X_eB) < small_number;

            if( too_parallel )
              continue;

            // Rather that computing closest points first and then find the
            // normal as the unit-separation vector we are going to simply
            // just compute the normal from the edge-cross products.... From
            // geoemtry it should give us the same direction without having
            // to first compute the closest points... secondly getting
            // the normal direction from closest points are ill-conditioned
            // when objects move closer and at exact contact the definition
            // is ill-posed.
            //
            // One more benefit this approach gives is that we can do a
            // cheap SAP test before actual computing the closest points
            // and hence do a quick-rejection test earlier on.
            //
            //
            V const n     =  unit(eA_X_eB); // tiny::unit(pB-pA); should give us same direction

            // Technically, we cheat a little here and use the fact that
            // we have tetrahedra to do the SAT test and not just
            // triangles.... should make the decision a little
            // more robust.... If one triangle became orthogonal to the other
            // then we point see the triangle interval collapse to a
            // single "point".
            //
            // Precision and round-off may in this case lead
            // to b_min = b_max + some_very_small_positive_number rather
            // than b_min = b_max. The tetrahedra will prevent this case
            // from happening.

            T const a0    = tiny::inner_prod(n, A.p(0));
            T const a1    = tiny::inner_prod(n, A.p(1));
            T const a2    = tiny::inner_prod(n, A.p(2));
            T const a3    = tiny::inner_prod(n, A.p(3));
            T const a_min = min(a0, min( a1, min( a2, a3 ) ) );
            T const a_max = max(a0, max( a1, max( a2, a3 ) ) );

            assert( a_min < a_max || !"contacts_tetrahedron_tetrahedron(): Internal error, flat tetrahedron encountered");

            T const b0    = tiny::inner_prod(n, B.p(0));
            T const b1    = tiny::inner_prod(n, B.p(1));
            T const b2    = tiny::inner_prod(n, B.p(2));
            T const b3    = tiny::inner_prod(n, B.p(3));
            T const b_min = min(b0, min( b1, min( b2, b3 ) ) );
            T const b_max = max(b0, max( b1, max( b2, b3 ) ) );

            assert( b_min < b_max || !"contacts_tetrahedron_tetrahedron() Internal error, flat tetrahedron encountered");

            if (a_max <= b_min)   // Separation axis found a-long n-direction no contacts are possible
              return count > 0u;

            if (b_max <= a_min)   // Separation axis found a-long n-direction  no contacts are possible
              return count > 0u;

            // Now we know we have two tetrahedra that overlaps along
            // the n-direction... Careful, this does not necessary imply
            // they should generate any contact points.
            //
            // So next we compute the two closest points between the infinite
            // lines of the two edges we currently examine
            T s = VT::zero();
            T t = VT::zero();

            geometry::closest_points_line_line(pk, eA, pi, eB, s, t);

            V const pA = pk + s*eA;
            V const pB = pi + t*eB;

            // Then we will test if the cloests points are actual on the "edges"
            T const inside_tst1 =  tiny::inner_prod( eA, (pA - pm) );
            T const inside_tst2 =  tiny::inner_prod(-eA, (pA - pk) );
            T const inside_tst3 =  tiny::inner_prod( eB, (pB - pj) );
            T const inside_tst4 =  tiny::inner_prod(-eB, (pB - pi) );

            if(inside_tst1 >= -small_number)
              continue;

            if(inside_tst2 >= -small_number)
              continue;

            if(inside_tst3 >= -small_number)
              continue;

            if(inside_tst4 >= -small_number)
              continue;

            // So we have closest points on the edges, meaning that a sensible
            // predicted contact location would be he mid-point of the two
            // closest points.
            V const p     = (pA + pB)*VT::half();

            // Although we know tetrahedra overlap along the n-direction there
            // might be some other "direction" separating the objects... Hence
            // we will test if the generated contact is truely inside the
            // "triangle" surfaces.
            //
            // We are going to allow for a litle threshold in this test...
            if(  details::get_signed_distance(p, plA) > small_number*norm(plA.m_normal) )
              continue;

            if(  details::get_signed_distance(p, plB) > small_number*norm(plB.m_normal) )
              continue;

            // Now we know we can reply on p-being a true "contact" point so
            // we generate "correct" nornal and depth information.
            if(a_min <= b_min &&  b_min <= a_max)
            {
              T const depth = b_min - a_max;
              callback( p, n, depth);
              ++count;
            }
            
            if(b_min <= a_min &&  a_min <= b_max)
            {
              T const depth = a_min - b_max;
              callback( p, -n, depth);
              ++count;
            }
            
            
          }
        }
      }
    }
    
    return count > 0u;
  }

  template< typename V>
  inline bool contacts_tetrahedron_tetrahedron(
                                               Tetrahedron<V> const & A
                                               , Tetrahedron<V> const & B
                                               , ContactsCallback<V> & callback
                                               , std::vector<bool> const & surface_A
                                               , std::vector<bool> const & surface_B
                                               , VERTEX_ONLY_INTERSECTION const & /*algorithm_tag*/
  )
  {
    assert(surface_A.size() == 4u                                          || !"contacts_tetrahedron_tetrahedron(): internal error, must have four surface map values");
    assert( (surface_A[0] || surface_A[1] || surface_A[2] || surface_A[3]) || !"contacts_tetrahedron_tetrahedron(): internal error, tetrahedron A must have at least one surface face");
    assert(surface_B.size() == 4u                                          || !"contacts_tetrahedron_tetrahedron(): internal error, must have four surface map values");
    assert( (surface_B[0] || surface_B[1] || surface_B[2] || surface_B[3]) || !"contacts_tetrahedron_tetrahedron(): internal error, tetrahedron B must have at least one surface face");

    typedef typename V::value_traits VT;
    typedef typename V::real_type     T;

    unsigned int count = 0u;

    std::vector<Triangle<V> >                trianglesA(4u);
    std::vector<Triangle<V> >                trianglesB(4u);

    std::vector<details::UnscaledPlane<V> >  planesA(4u);
    std::vector<details::UnscaledPlane<V> >  planesB(4u);

    //--- Preprocessing tetrahedrons for fast triangle face lookup info --------
    for (unsigned int v =0u; v < 4u; ++v)
    {
      trianglesA[v] = get_opposite_face( v, A );
      trianglesB[v] = get_opposite_face( v, B );
      planesA[v]    = details::make_unscaled_plane(trianglesA[v]);
      planesB[v]    = details::make_unscaled_plane(trianglesB[v]);
    }

    //--- B's vertices inside A
    for (unsigned int v =0u; v < 4u; ++v)
    {
      V const b = B.p(v);

      T const a0 = get_signed_distance( b, planesA[0] );
      T const a1 = get_signed_distance( b, planesA[1] );
      T const a2 = get_signed_distance( b, planesA[2] );
      T const a3 = get_signed_distance( b, planesA[3] );

      bool b_outside_A =  ( a0 > VT::zero() )
                       || ( a1 > VT::zero() )
                       || ( a2 > VT::zero() )
                       || ( a3 > VT::zero() );

      if( b_outside_A )
        continue;

      T const depth0 = surface_A[0] ? a0 / norm(planesA[0].m_normal) : VT::infinity() ;
      T const depth1 = surface_A[1] ? a1 / norm(planesA[1].m_normal) : VT::infinity() ;
      T const depth2 = surface_A[2] ? a2 / norm(planesA[2].m_normal) : VT::infinity() ;
      T const depth3 = surface_A[3] ? a3 / norm(planesA[3].m_normal) : VT::infinity() ;

      if ( depth0 <= depth1 && depth0 <= depth2 && depth0 <= depth3 )
        callback( b, unit(planesA[0].m_normal), depth0 );
      if ( depth1 <= depth0 && depth1 <= depth2 && depth1 <= depth3 )
        callback( b, unit(planesA[1].m_normal), depth1 );
      if ( depth2 <= depth0 && depth2 <= depth1 && depth2 <= depth3 )
        callback( b, unit(planesA[2].m_normal), depth2 );
      if ( depth3 <= depth0 && depth3 <= depth1 && depth3 <= depth2 )
        callback( b, unit(planesA[3].m_normal), depth3 );

      ++count;
    }

    //--- A's vertices inside B
    for (unsigned int v =0u; v < 4u; ++v)
    {
      V const a = A.p(v);

      T const b0 = get_signed_distance( a, planesB[0] );
      T const b1 = get_signed_distance( a, planesB[1] );
      T const b2 = get_signed_distance( a, planesB[2] );
      T const b3 = get_signed_distance( a, planesB[3] );

      bool a_outside_B =  ( b0 > VT::zero() )
                      || ( b1 > VT::zero() )
                      || ( b2 > VT::zero() )
                      || ( b3 > VT::zero() );

      if( a_outside_B )
        continue;

      T const depth0 = surface_B[0] ? b0 / norm(planesB[0].m_normal) : VT::infinity() ;
      T const depth1 = surface_B[1] ? b1 / norm(planesB[1].m_normal) : VT::infinity() ;
      T const depth2 = surface_B[2] ? b2 / norm(planesB[2].m_normal) : VT::infinity() ;
      T const depth3 = surface_B[3] ? b3 / norm(planesB[3].m_normal) : VT::infinity() ;

      if ( depth0 <= depth1 && depth0 <= depth2 && depth0 <= depth3 )
        callback( a, -unit(planesB[0].m_normal), depth0 );
      if ( depth1 <= depth0 && depth1 <= depth2 && depth1 <= depth3 )
        callback( a, -unit(planesB[1].m_normal), depth1 );
      if ( depth2 <= depth0 && depth2 <= depth1 && depth2 <= depth3 )
        callback( a, -unit(planesB[2].m_normal), depth2 );
      if ( depth3 <= depth0 && depth3 <= depth1 && depth3 <= depth2 )
        callback( a, -unit(planesB[3].m_normal), depth3 );

      ++count;
    }

    return count > 0u;
  }

  template< typename V>
  inline bool contacts_tetrahedron_tetrahedron(
                                               Tetrahedron<V> const & A
                                               , Tetrahedron<V> const & B
                                               , ContactsCallback<V> & callback
                                               , std::vector<bool> const & surface_A
                                               , std::vector<bool> const & surface_B
                                               , CONSISTENT_VERTEX const & /*algorithm_tag*/
  )
  {
    assert(surface_A.size() == 4u                                          || !"contacts_tetrahedron_tetrahedron(): internal error, must have four surface map values");
    assert( (surface_A[0] || surface_A[1] || surface_A[2] || surface_A[3]) || !"contacts_tetrahedron_tetrahedron(): internal error, tetrahedron A must have at least one surface face");
    assert(surface_B.size() == 4u                                          || !"contacts_tetrahedron_tetrahedron(): internal error, must have four surface map values");
    assert( (surface_B[0] || surface_B[1] || surface_B[2] || surface_B[3]) || !"contacts_tetrahedron_tetrahedron(): internal error, tetrahedron B must have at least one surface face");

    typedef typename V::value_traits VT;
    typedef typename V::real_type     T;

    unsigned int count = 0u;

    std::vector<Triangle<V> >                trianglesA(4u);
    std::vector<Triangle<V> >                trianglesB(4u);
    std::vector<bool >                       insideA(4u);
    std::vector<bool >                       insideB(4u);
    std::vector<details::UnscaledPlane<V> >  planesA(4u);
    std::vector<details::UnscaledPlane<V> >  planesB(4u);

    for (unsigned int v =0u; v < 4u; ++v)
    {
      trianglesA[v] = get_opposite_face( v, A );
      trianglesB[v] = get_opposite_face( v, B );
      planesA[v]    = details::make_unscaled_plane(trianglesA[v]);
      planesB[v]    = details::make_unscaled_plane(trianglesB[v]);
    }

    for (unsigned int v =0u; v < 4u; ++v)
    {
      insideA[v] =  details::inside_planes(B.p(v), planesA );
      insideB[v] =  details::inside_planes(A.p(v), planesB );
    }

    unsigned int const edge[4][3] = {
      {1, 2, 3},
      {0, 2, 3},
      {0, 1, 3},
      {0, 1, 2}
    };

    // Find surface plane intersection closest to vertex
    // intersection and use that surface plane as the normal

    for (unsigned int i =0u; i < 4u; ++i)
    {
      if(!insideA[i])
        continue;

      for (unsigned int k =0u; k < 3u; ++k)
      {
        unsigned int const j = edge[i][k];

        if(insideA[j])
          continue;

        // Now we know b(i) is inside and b(j) is outside,
        // now we search for the surface plane intersection
        // closest to b(i) if one exist

        unsigned int best_plane = 4u;
        T            t          = VT::infinity();

        for (unsigned int m =0u; m < 4u; ++m)
        {
          if(! surface_A[m] ) // We will ignore non-surface faces
            continue;

          if( ! details::is_crossing_plane( B.p(i), B.p(j), planesA[m] ) ) // if edge is not crossing this surface then skip it
            continue;

          // Now we know we have found a edge that is crossing a surface face... so
          // we compute the actual edge-length parameter for the intersection.
          T const s = details::make_intersection_parameter( B.p(i), B.p(j), planesA[m] );

          // Check to see if we found a better surface plane
          best_plane = (s  < t) ? m : best_plane;
          t          = (s  < t) ? s : t;
        }

        // If a intersecting surface plane was found then we use its face-plane for
        // generating the contact plane of this vertex-tetrahedron contact
        if(best_plane < 4u)
        {
          V const & n     = planesA[best_plane].m_normal;
          T const   depth = details::get_signed_distance( B.p(i), planesA[best_plane] ) / norm(n);

          callback( B.p(i), unit(n), depth );
          ++count;
        }
      }

    }

    for (unsigned int i =0u; i < 4u; ++i)
    {
      if(!insideB[i])
        continue;

      for (unsigned int k =0u; k < 3u; ++k)
      {
        unsigned int const j = edge[i][k];

        if(insideB[j])
          continue;

        // Now we know a(i) is inside and a(j) is outside,
        // now we search for the surface plane intersection
        // closest to a(i) if one exist

        unsigned int best_plane = 4u;
        T            t          = VT::infinity();

        for (unsigned int m =0u; m < 4u; ++m)
        {
          if(! surface_B[m] ) // We will ignore non-surface faces
            continue;

          if( ! details::is_crossing_plane( A.p(i), A.p(j), planesB[m] ) ) // if edge is not crossing this surface then skip it
            continue;

          // Now we know we have found a edge that is crossing a surface face... so
          // we compute the actual edge-length parameter for the intersection.
          T const s = details::make_intersection_parameter( A.p(i), A.p(j), planesB[m] );

          // Check to see if we found a better surface plane
          best_plane = (s  < t) ? m : best_plane;
          t          = (s  < t) ? s : t;
        }

        // If a intersecting surface plane was found then we use its face-plane for
        // generating the contact plane of this vertex-tetrahedron contact
        if(best_plane < 4u)
        {
          V const & n     = -planesB[best_plane].m_normal;
          T const   depth = details::get_signed_distance( A.p(i), planesB[best_plane] ) / norm(n);

          callback( A.p(i), unit(n), depth );
          ++count;
        }
      }
      
    }

    return count > 0u;
  }

  template< typename V>
  inline bool contacts_tetrahedron_tetrahedron(
                                               Tetrahedron<V> const & A
                                               , Tetrahedron<V> const & B
                                               , ContactsCallback<V> & callback
                                               , std::vector<bool> const & surface_A
                                               , std::vector<bool> const & surface_B
                                               , SAT const & /*algorithm_tag*/
                                               )
  {

    assert(surface_A.size() == 4u                                          || !"contacts_tetrahedron_tetrahedron(): internal error, must have four surface map values");
    assert( (surface_A[0] || surface_A[1] || surface_A[2] || surface_A[3]) || !"contacts_tetrahedron_tetrahedron(): internal error, tetrahedron A must have at least one surface face");
    assert(surface_B.size() == 4u                                          || !"contacts_tetrahedron_tetrahedron(): internal error, must have four surface map values");
    assert( (surface_B[0] || surface_B[1] || surface_B[2] || surface_B[3]) || !"contacts_tetrahedron_tetrahedron(): internal error, tetrahedron B must have at least one surface face");

    using std::min;
    using std::max;

    typedef typename V::value_traits VT;
    typedef typename V::real_type     T;

    bool const overlap = overlap_tetrahedron_tetrahedron(A, B);
    if(!overlap)
      return false;

    V n; // The contact normal to be used

    bool const found_normal = details::pick_sat_normal(A,B,surface_A,surface_B, n);

    if(!found_normal)
      return false;

    return details::generate_contacts_from_intersection(A, B, callback, n);
  }

  template< typename V>
  inline bool contacts_tetrahedron_tetrahedron(
                                               Tetrahedron<V> const & A
                                               , Tetrahedron<V> const & B
                                               , ContactsCallback<V> & callback
                                               , std::vector<bool> const & surface_A
                                               , std::vector<bool> const & surface_B
                                               , RESTRICTED_SAT const & /*algorithm_tag*/
  )
  {
    assert(surface_A.size() == 4u                                          || !"contacts_tetrahedron_tetrahedron(): internal error, must have four surface map values");
    assert( (surface_A[0] || surface_A[1] || surface_A[2] || surface_A[3]) || !"contacts_tetrahedron_tetrahedron(): internal error, tetrahedron A must have at least one surface face");
    assert(surface_B.size() == 4u                                          || !"contacts_tetrahedron_tetrahedron(): internal error, must have four surface map values");
    assert( (surface_B[0] || surface_B[1] || surface_B[2] || surface_B[3]) || !"contacts_tetrahedron_tetrahedron(): internal error, tetrahedron B must have at least one surface face");

    using std::min;
    using std::max;

    typedef typename V::value_traits VT;
    typedef typename V::real_type     T;

    bool const overlap = overlap_tetrahedron_tetrahedron(A, B);
    if(!overlap)
      return false;

    V n; // The contact normal to be used

    bool const found_normal = details::pick_restricted_sat_normal(A,B,surface_A,surface_B, n);

    if(!found_normal)
      return false;

    return details::generate_contacts_from_intersection(A, B, callback, n);
  }


  template< typename V>
  inline bool contacts_tetrahedron_tetrahedron(
                                               Tetrahedron<V> const & A
                                               , Tetrahedron<V> const & B
                                               , ContactsCallback<V> & callback
                                               , std::vector<bool> const & surface_A
                                               , std::vector<bool> const & surface_B
                                               , MOST_OPPOSING_SURFACES const & /*algorithm_tag*/
  )
  {
    assert(surface_A.size() == 4u                                          || !"contacts_tetrahedron_tetrahedron(): internal error, must have four surface map values");
    assert( (surface_A[0] || surface_A[1] || surface_A[2] || surface_A[3]) || !"contacts_tetrahedron_tetrahedron(): internal error, tetrahedron A must have at least one surface face");
    assert(surface_B.size() == 4u                                          || !"contacts_tetrahedron_tetrahedron(): internal error, must have four surface map values");
    assert( (surface_B[0] || surface_B[1] || surface_B[2] || surface_B[3]) || !"contacts_tetrahedron_tetrahedron(): internal error, tetrahedron B must have at least one surface face");

    using std::min;
    using std::max;

    typedef typename V::value_traits VT;
    typedef typename V::real_type     T;

    bool const overlap = overlap_tetrahedron_tetrahedron(A, B);
    if(!overlap)
      return false;

    V n; // The contact normal to be used

    bool const found_normal = details::pick_most_opposing_surface_normal(A,B,surface_A,surface_B, n);

    if(!found_normal)
      return false;

    return details::generate_contacts_from_intersection(A, B, callback, n);
  }

  template< typename V>
  inline bool contacts_tetrahedron_tetrahedron(
                                               Tetrahedron<V> const & A
                                               , Tetrahedron<V> const & B
                                               , ContactsCallback<V> & callback
                                               )
  {
    using std::min;
    using std::max;

    typedef typename V::value_traits VT;
    typedef typename V::real_type     T;

    V n; // The contact normal to be used
    bool const overlap = details::pick_sat_normal(A,B,n);

    if(!overlap)
      return false;

    return details::generate_contacts_from_intersection(A, B, callback, n);
  }


}// end namespace geometry

// GEOMETRY_CONTACTS_TETRAHEDRON_TETRAHEDRON_H
#endif