#ifndef GEOMETRY_CONTACTS_OBB_CYLINDER_H
#define GEOMETRY_CONTACTS_OBB_CYLINDER_H

#include <types/geometry_obb.h>
#include <types/geometry_cylinder.h>
#include <types/geometry_capsule.h>
#include <overlap/geometry_overlap_obb_capsule.h>
#include <contacts/geometry_contacts_callback.h>


#include <tiny_is_finite.h>
#include <tiny_is_number.h>

#include <cmath>              // needed for std::fabs and std::sqrt, std::cos, std::sin
#include <vector>
#include <cassert>

namespace geometry
{

  namespace detail
  {

    /**
     * Extract the "feature" of the OBB that are touching a plane.
     *
     * @param   bitmask      This bitmask contains information about which vertices are touching the plane. From
     *                       this information the method deduce if only a single vertex is touching, and edge
     *                       (two vertices) is touching, or if a side/face (four vertices) are touching.
     *
     * @param   corners      The actual coordinates of the OBB box.
     *
     * @param   feature      Upon return this argument holds the extracted coordiantes of the touching feature.
     *                       If only a single vertex is touching the only one element, if and edge then two
     *                       elements, if a face then four elements (in CCW order to OBB face normal is
     *                       outward pointing). If no vertices are touching the plane then feature is empty.
     *
     * @return               If a feature was extracted then the return value is true otherwise it is false.
     */
    template<typename V>
    inline bool extract_feature_from_bitmask(int const & bitmask, std::vector<V> const & corners, std::vector<V> & feature)
    {
      assert(corners.size() == 8u || !"extract_feature_from_bitmask():  Internal error there must be 8 corners.");

      feature.clear();

      // Vertices of OBB are numbered as follows and mapped to OBB space using
      // the correspodning bit-mask of their vertex number.
      //
      // idx    bitpattern   coordinates (z,y,x)
      //
      //  0       000          -, -, -
      //  1       001          -, -, +
      //  2       010          -, +, -
      //  3       011          -, +, +
      //  4       100          +, -, -
      //  5       101          +, -, +
      //  6       110          +, +, -
      //  7       111          +, +, +
      //
      //
      // This numbering scheme forms the basics for describing the
      // connectivity of the OBB vertices
      //
      //  Let the coordinate direction be given as follows
      //
      //           z
      //           |  / y
      //           | /
      //           |/
      //           *----- x
      //
      //
      //  Then the OBB is defined by
      //
      //       6 *--------------------* 7
      //        /|                   /|
      //       / |                  / |
      //      /  |                 /  |
      //   4 *--------------------* 5 |
      //     |   |                |   |
      //     |   |                |   |
      //     |   |                |   |
      //     | 2 *----------------|---* 3
      //     |  /                 |  /
      //     | /                  | /
      //     |/                   |/
      //     *--------------------*
      //    0                      1
      //
      // We can now describe the touching feature by generating
      // a bitmask whos value indicates the case we are dealing
      // with. Once we know the bitmask-value we can immediate
      // generate the coordinates of the touching feature.
      //
      // Say vertex k is touching then we set bit k to one and otherwise
      // to zero. Hence a byte-bitpattern describes all possible touching
      // patterns.

      switch (bitmask)
      {
          // nothing is touching
        case 0: break;
          // Single vertex on cap bit patterns
          //
          // idx 0 ->    1                (1<<0)
          // idx 1 ->    2                (1<<1)
          // idx 2 ->    4                (1<<2)
          // idx 3 ->    8                (1<<3)
          // idx 4 ->   16                (1<<4)
          // idx 5 ->   32                (1<<5)
          // idx 6 ->   64                (1<<6)
          // idx 7 ->  128                (1<<7)
        case (1<<0): feature.push_back( corners[0] );  break;
        case (1<<1): feature.push_back( corners[1] );  break;
        case (1<<2): feature.push_back( corners[2] );  break;
        case (1<<3): feature.push_back( corners[3] );  break;
        case (1<<4): feature.push_back( corners[4] );  break;
        case (1<<5): feature.push_back( corners[5] );  break;
        case (1<<6): feature.push_back( corners[6] );  break;
        case (1<<7): feature.push_back( corners[7] );  break;
          // Single edge on cap bit patterns
          //
          // idxs (0,1) ->    (1<<0)  |  (1<<1)
          // idxs (0,2) ->    (1<<0)  |  (1<<2)
          // idxs (0,4) ->    (1<<0)  |  (1<<4)
          // idxs (1,3) ->    (1<<1)  |  (1<<3)
          // idxs (1,5) ->    (1<<1)  |  (1<<5)
          // idxs (2,3) ->    (1<<2)  |  (1<<3)
          // idxs (2,6) ->    (1<<2)  |  (1<<6)
          // idxs (3,7) ->    (1<<3)  |  (1<<7)
          // idxs (4,5) ->    (1<<4)  |  (1<<5)
          // idxs (4,6) ->    (1<<4)  |  (1<<6)
          // idxs (5,7) ->    (1<<5)  |  (1<<7)
          // idxs (6,7) ->    (1<<6)  |  (1<<7)
          //
        case (1<<0)|(1<<1): feature.push_back( corners[0] );feature.push_back( corners[1] );  break;
        case (1<<0)|(1<<2): feature.push_back( corners[0] );feature.push_back( corners[2] );  break;
        case (1<<0)|(1<<4): feature.push_back( corners[0] );feature.push_back( corners[4] );  break;
        case (1<<1)|(1<<3): feature.push_back( corners[1] );feature.push_back( corners[3] );  break;
        case (1<<1)|(1<<5): feature.push_back( corners[1] );feature.push_back( corners[5] );  break;
        case (1<<2)|(1<<3): feature.push_back( corners[2] );feature.push_back( corners[3] );  break;
        case (1<<2)|(1<<6): feature.push_back( corners[2] );feature.push_back( corners[6] );  break;
        case (1<<3)|(1<<7): feature.push_back( corners[3] );feature.push_back( corners[7] );  break;
        case (1<<4)|(1<<5): feature.push_back( corners[4] );feature.push_back( corners[5] );  break;
        case (1<<4)|(1<<6): feature.push_back( corners[4] );feature.push_back( corners[6] );  break;
        case (1<<5)|(1<<7): feature.push_back( corners[5] );feature.push_back( corners[7] );  break;
        case (1<<6)|(1<<7): feature.push_back( corners[6] );feature.push_back( corners[7] );  break;
          // Single face on cap bit patterns
          //
          // idxs (0,2,3,1) ->    (1<<0)  |  (1<<1) | (1<<2)  |  (1<<3)   "bottom" face  (-z is constant)
          // idxs (4,5,7,6) ->    (1<<4)  |  (1<<5) | (1<<6)  |  (1<<7)   "top" face     (+z is constant)
          // idxs (0,4,6,2) ->    (1<<0)  |  (1<<2) | (1<<4)  |  (1<<6)   "left" face    (-x is constant)
          // idxs (1,3,7,5) ->    (1<<1)  |  (1<<3) | (1<<5)  |  (1<<7)   "right" face   (+x is constant)
          // idxs (0,1,5,4) ->    (1<<0)  |  (1<<1) | (1<<4)  |  (1<<5)   "front" face   (-y is constant)
          // idxs (2,6,7,3) ->    (1<<2)  |  (1<<3) | (1<<6)  |  (1<<7)   "back" face    (+y is constant)
          //
        case (1<<0)|(1<<1)|(1<<2)|(1<<3):
          feature.push_back( corners[0] );
          feature.push_back( corners[2] );
          feature.push_back( corners[3] );
          feature.push_back( corners[1] );
          break;
        case (1<<4)|(1<<5)|(1<<6)|(1<<7):
          feature.push_back( corners[4] );
          feature.push_back( corners[5] );
          feature.push_back( corners[7] );
          feature.push_back( corners[6] );
          break;
        case (1<<0)|(1<<2)|(1<<4)|(1<<6):
          feature.push_back( corners[0] );
          feature.push_back( corners[4] );
          feature.push_back( corners[6] );
          feature.push_back( corners[2] );
          break;
        case (1<<1)|(1<<3)|(1<<5)|(1<<7):
          feature.push_back( corners[1] );
          feature.push_back( corners[3] );
          feature.push_back( corners[7] );
          feature.push_back( corners[5] );
          break;
        case (1<<0)|(1<<1)|(1<<4)|(1<<5):
          feature.push_back( corners[0] );
          feature.push_back( corners[1] );
          feature.push_back( corners[5] );
          feature.push_back( corners[4] );
          break;
        case (1<<2)|(1<<3)|(1<<6)|(1<<7):
          feature.push_back( corners[2] );
          feature.push_back( corners[6] );
          feature.push_back( corners[7] );
          feature.push_back( corners[3] );
          break;
        default:
          assert(false || !"extract_feature_from_bitmask(): unknown bitmask pattern does not match legal case...");
          break;
      };

      return !feature.empty();
    }

    template<typename V>
    inline unsigned int intersect_line_circle(
                                                V const & p0
                                              , V const & p1
                                              , typename V::real_type const & radius
                                              , V & i0
                                              , V & i1
                                              )
    {
      using std::fabs;
      using std::sqrt;

      typedef typename V::real_type          T;
      typedef typename V::value_traits       VT;

      V const dP = p1-p0;

      T const a = tiny::inner_prod(dP,dP);
      T const b = VT::two()*tiny::inner_prod(dP,p0);
      T const c = tiny::inner_prod(p0,p0) - radius*radius;


      assert(is_finite(a) || !"intersect_line_circle(): inf number");
      assert(is_number(a) || !"intersect_line_circle(): nan number");
      assert(is_finite(b) || !"intersect_line_circle(): inf number");
      assert(is_number(b) || !"intersect_line_circle(): nan number");
      assert(is_finite(c) || !"intersect_line_circle(): inf number");
      assert(is_number(c) || !"intersect_line_circle(): nan number");

      assert(      a > VT::zero() || !"intersect_line_circle(): internal error");
      assert(fabs(b) > VT::zero() || !"intersect_line_circle(): internal error");

      T const d = b*b - VT::four()*a*c;

      assert(is_finite(d) || !"intersect_line_circle(): inf number");
      assert(is_number(d) || !"intersect_line_circle(): nan number");


      if(d>VT::zero())
      {
        T const sqrt_d = sqrt(d);

        assert(is_finite(sqrt_d) || !"intersect_line_circle(): inf number");
        assert(is_number(sqrt_d) || !"intersect_line_circle(): nan number");

        T const t0 = (-b - sqrt_d)/ VT::two()*a;
        T const t1 = (-b + sqrt_d)/ VT::two()*a;

        assert(is_finite(t0) || !"intersect_line_circle(): inf number");
        assert(is_number(t0) || !"intersect_line_circle(): nan number");
        assert(is_finite(t1) || !"intersect_line_circle(): inf number");
        assert(is_number(t1) || !"intersect_line_circle(): nan number");

        i0 = p0 + dP*t0;
        i1 = p0 + dP*t1;

        return 2u;
      }

      if(d==VT::zero())
      {
        T const t = -b / VT::two()*a;

        assert(is_finite(t) || !"intersect_line_circle(): inf number");
        assert(is_number(t) || !"intersect_line_circle(): nan number");

        i1 = i0 = p0 + dP*t;

        return 1u;
      }

      return 0u;
    }

    /**
     * Test point for inclusion in the specified quad. This method assumes that points live in cylinder space.
     */
    template<typename V>
    inline bool inside_quad(V const & p, std::vector<V> & quad)
    {
      typedef typename V::real_type          T;
      typedef typename V::value_traits       VT;

      for(unsigned int k = 0u; k < 4u; ++k)
      {
        V const a = quad[(k+1)%4] - quad[k];
        V const b = p - quad[k];

        T const tst = tiny::cross(a, b)[2];

        if( tst < VT::zero())
          return false;

      }
      
      return true;
    }

    /**
     * This method assumes that all points live in cylinder space.
     */
    template<typename V>
    inline void intersect_polygon_circle( std::vector<V> & feature, typename V::real_type const & radius, std::vector<V> & intersection)
    {
      using std::cos;
      using std::sin;

      typedef typename V::real_type          T;
      typedef typename V::value_traits       VT;

      if (feature.size()==1u)
      {
        T const distance = tiny::norm(V::make(feature[0](0), feature[0](1), VT::zero()) );

        if (distance <=  radius )
        {
          intersection.push_back( feature[0] );
        }

        return;
      }


      if (feature.size()==2u)
      {
        T const distance0 = tiny::norm(V::make(feature[0](0), feature[0](1), VT::zero()) );
        T const distance1 = tiny::norm(V::make(feature[1](0), feature[1](1), VT::zero()) );
        bool const inside0 = (distance0 <=  radius ) ? true : false;
        bool const inside1 = (distance1 <=  radius ) ? true : false;

        if (inside0 && inside1)
        {
          intersection.push_back( feature[0] );
          intersection.push_back( feature[1] );
        }
        if (inside0 && !inside1)
        {

          V p0;
          V p1;

          unsigned int const cnt = intersect_line_circle(feature[0], feature[1], radius, p0, p1);

          assert(cnt==1u || !"intersect_polygon_circle(): internal error, expected exactly one intersection point");

          intersection.push_back( feature[0] );
          intersection.push_back( p0 );
        }
        if (!inside0 && inside1)
        {
          V p0;
          V p1;

          unsigned int const cnt = intersect_line_circle(feature[0], feature[1], radius, p0, p1);

          assert(cnt==1u || !"intersect_polygon_circle(): internal error, expected exactly one intersection point");

          intersection.push_back( p0 );
          intersection.push_back( feature[1] );
        }
        if (!inside0 && !inside1)
        {
          V p0;
          V p1;

          unsigned int const cnt = intersect_line_circle(feature[0], feature[1], radius, p0, p1);

          if (cnt == 2u)
          {
            intersection.push_back( p0 );
            intersection.push_back( p1 );
          }
          if (cnt == 1u)
          {
            intersection.push_back( p0 );
          }
        }

        return;
      }


      if (feature.size()==4u)
      {
        // First we generate the part of the contact area perimeter that are from quad perimeter

        T const distance0 = tiny::norm(V::make(feature[0](0), feature[0](1), VT::zero()) );
        T const distance1 = tiny::norm(V::make(feature[1](0), feature[1](1), VT::zero()) );
        T const distance2 = tiny::norm(V::make(feature[2](0), feature[2](1), VT::zero()) );
        T const distance3 = tiny::norm(V::make(feature[3](0), feature[3](1), VT::zero()) );

        bool const inside0 = (distance0 <=  radius ) ? true : false;
        bool const inside1 = (distance1 <=  radius ) ? true : false;
        bool const inside2 = (distance2 <=  radius ) ? true : false;
        bool const inside3 = (distance3 <=  radius ) ? true : false;

        if(inside0)
          intersection.push_back( feature[0] );
        if(inside1)
          intersection.push_back( feature[1] );
        if(inside2)
          intersection.push_back( feature[2] );
        if(inside3)
          intersection.push_back( feature[3] );

        bool const quad_fully_inside = inside0 && inside1 && inside2 && inside3;

        if (quad_fully_inside)  // no need to do any more....
          return;

        for(unsigned int k = 0u; k < 4u; ++k)
        {
          V p0;
          V p1;

          unsigned int const cnt = intersect_line_circle(
                                                         feature[k]
                                                         , feature[(k+1) % 4]
                                                         , radius, p0, p1
                                                         );

          if (cnt == 2u)
          {
            intersection.push_back( p0 );
            intersection.push_back( p1 );
          }
          if (cnt == 1u)
          {
            intersection.push_back( p0 );
          }
        }

        // Next we generate part from the circle perimeter, we do this by sampling
        // the circle perimeter and testing if sample points are inside the
        // quad. If so we report the sampel points

        unsigned int const max_samples = 12u;  // Magic number....

        T const half_height = (feature[0](2) + feature[1](2) + feature[2](2) + feature[3](2)) / VT::four();
        T const dtheta = VT::two()*VT::pi() / max_samples;
        T       theta = VT::zero();

        for (unsigned int sample = 0u; sample < max_samples; ++sample)
        {
          V const p = V::make( radius*cos(theta), radius*sin(theta), half_height);

          if(inside_quad(p, feature))
            intersection.push_back( p );

          theta += dtheta;
        }
      }

    }

  }// end of namespace detail
  
  /**
   * OBB versus Cylinder Contact Point Generation
   *
   * @param A                 The OBB box.
   * @param B                 The cylinder.
   * @param envelope          The size of the collision envelope.
   * @param callback          A pointer to a callback interface which is used
   *                          to add contact point information
   * @param flip              Set to true when using this to test cylinder versus box
   *                          (ie when objects order are swapped)
   */
  template<typename MT>
  inline bool contacts_obb_cylinder(
                                    OBB<MT> const & A
                                    , Cylinder<typename MT::vector3_type> const & B
                                    , typename MT::real_type const & envelope
                                    , ContactsCallback<typename MT::vector3_type> & callback
                                    , bool const flip = false
                                    )
  {
    using std::min;
    using std::max;

    typedef typename MT::vector3_type       V;
    typedef typename MT::real_type          T;
    typedef typename MT::value_traits       VT;

    //--- First do a quick rejection test by approximating the cylinder with
    //--- a capsule and using a fast overlap test
    Capsule<V> const & cap = convert(B);

    if(! overlap_obb_capsule(A,cap))
      return false;

    //--- Pre-process the OBB corners by transforming them into cylinder
    //--- space and classify the corners position with respect to the cylinder.
    T const half_height = B.half_height();

    std::vector<V> a(8u, V::zero());              // OBB corners in world space
    std::vector<V> b(8u, V::zero());              // OBB corners in cylinder space
    unsigned int cnt_above = 0u;                  // count the number of vertices above top cap
    unsigned int cnt_below = 0u;                  // count the number of vertices below bottom cap
    std::vector<bool> on_top_cap(8u, false);      // flag that is true if vertex is exactl on top cap
    std::vector<bool> on_bottom_cap(8u, false);   // flag that is true if vertex is exactl on bottom cap
    std::vector<bool> inside(8u, false);          // flag that is true if vertex is inside cylinder
    bool inbeween = false;                        // flat that is true if any vertex is found between top and bottom caps

    T const upper_envelope =   half_height - envelope;   // "Fuzzy" zone of upper cap plane to counter finite precision problems and truncation errors
    T const lower_envelope = - half_height + envelope;   // "Fuzzy" zone of lower cap plane to counter finite precision problems and truncation errors

    for (unsigned int i=0u; i<8u; ++i)
    {
      a[i]             = transform_from_obb( get_local_corner(i, A), A );
      b[i]             = transform_to_cylinder(a[i], B );
      inside[i]        = false;
      on_top_cap[i]    = false;
      on_bottom_cap[i] = false;

      if ( b[i](2) > half_height)
      {
        ++cnt_above;
        continue;
      }

      if ( b[i](2) < -half_height )
      {
        ++cnt_below;
        continue;
      }

      T const distance = tiny::norm( V::make(b[i](0), b[i](1), VT::zero()) );

      if( distance > B.radius() )
        continue;

      inside[i]        = true;
      on_top_cap[i]    = ( b[i](2) > upper_envelope );
      on_bottom_cap[i] = ( b[i](2) < lower_envelope );
      inbeween         =   b[i](2) <= upper_envelope  &&  b[i](2) >= lower_envelope ? true : inbeween;

    }

    // Test for trival cases where OBB was clearly above/below caps of the cylinder
    if(cnt_above == 8u)
      return false;

    if(cnt_below == 8u)
      return false;


    //--- Test for special cases of OBB touching the face-caps of the cylinder
    int const top_bit_mask = on_top_cap[0]
                           | on_top_cap[1] << 1
                           | on_top_cap[2] << 2
                           | on_top_cap[3] << 3
                           | on_top_cap[4] << 4
                           | on_top_cap[5] << 5
                           | on_top_cap[6] << 6
                           | on_top_cap[7] << 7;

    int const bottom_bit_mask = on_bottom_cap[0]
                              | on_bottom_cap[1] << 1
                              | on_bottom_cap[2] << 2
                              | on_bottom_cap[3] << 3
                              | on_bottom_cap[4] << 4
                              | on_bottom_cap[5] << 5
                              | on_bottom_cap[6] << 6
                              | on_bottom_cap[7] << 7;

    //---- Test if we only have vertices on top cap and nothing below
    if( top_bit_mask != 0  && !cnt_below && !inbeween )
    {
      std::vector<V> feature;

      if(detail::extract_feature_from_bitmask(top_bit_mask, b, feature))
      {
        std::vector<V> intersection_points;

        detail::intersect_polygon_circle(feature, B.radius(), intersection_points );

        for (typename std::vector<V>::const_iterator p = intersection_points.begin(); p != intersection_points.end(); ++p)
        {
          V const  point      = transform_from_cylinder(*p, B);
          V const  normal     = flip ? B.axis() : -B.axis();
          T const  distance   = VT::zero();
          callback(point, normal, distance);
        }
      }
      return ! feature.empty();
    }


    //---- Test if we only have vertices on bottom cap and nothing above
    if( bottom_bit_mask != 0 && !cnt_above && !inbeween)
    {
      std::vector<V> feature;

      if(detail::extract_feature_from_bitmask(bottom_bit_mask, b, feature))
      {
        std::vector<V> intersection_points;

        detail::intersect_polygon_circle(feature, B.radius(), intersection_points );

        for (typename std::vector<V>::const_iterator p = intersection_points.begin(); p != intersection_points.end(); ++p)
        {
          V const  point      = transform_from_cylinder(*p, B);
          V const  normal     = flip ? -B.axis() : B.axis();
          T const  distance   = VT::zero();
          callback(point, normal, distance);
        }
      }
      return ! feature.empty();
    }


    assert(false || !"contacts_obb_cylinder(): Implementation in-complete, sorry");

    // Now we know that contacts must be on the side of the cylinder sheet.....

    // Test OBB vertices for inclusion in the cylinder and generate a
    // contact point for each vertex inside the cylinder.

    // For each edge of OBB find the closest point between box
    // edges and the cylinder axis if point inside cylinder then
    // add it as contact point


    // For each edge of OBB test if it collides one of the two end-cap
    // planes and if intersection point is inside end-cap circle. If
    // so add the intersection point as a contact point.






    return true;
  }


}//namespace geometry

//GEOMETRY_CONTACTS_OBB_CYLINDER_H
#endif