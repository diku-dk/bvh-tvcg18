#ifndef TINY_VALUE_TRAITS_H
#define TINY_VALUE_TRAITS_H

#include <tiny_constants.h>

namespace tiny
{
  template <typename T>
  class ValueTraits
    {
    public:
      
      static T zero()         { return detail::zero<T>();      }
      static T one()          { return detail::one<T>();       }
      static T two()          { return detail::two<T>();       }
      static T three()        { return detail::three<T>();     }
      static T four()         { return detail::four<T>();      }
      static T eight()        { return detail::eight<T>();     }
      static T infinity()     { return detail::infinity<T>();  }
      static T half()         { return detail::half<T>();      }
      static T pi()           { return detail::pi<T>();        }
      static T pi_2()         { return detail::pi_half<T>();   }
      static T pi_half()      { return detail::pi_half<T>();   }
      static T pi_quarter()   { return detail::pi_quarter<T>();}
      static T pi_4()         { return detail::pi_quarter<T>();}
      static T degree()       { return detail::degree<T>();    }
      static T radian()       { return detail::radian<T>();    }
      static T highest()      { return detail::highest<T>();   }
      static T lowest()       { return detail::lowest<T>();    }
      
      template<typename A>
      static T numeric_cast(A const & value) { return detail::numeric_cast<T>(value);    }

      static T convert_to_degrees(T const & radians){ return radians*degree();  }
      static T convert_to_radians(T const & degrees){ return degrees*radian();  }

    };
  
}  // namespace tiny

//TINY_VALUE_TRAITS_H
#endif
