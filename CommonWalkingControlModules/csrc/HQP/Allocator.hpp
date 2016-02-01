#ifndef __SOTH_ALLOCATOR__
#define __SOTH_ALLOCATOR__


#include <list>
#include <iostream>
#include <soth/api.hpp>

namespace soth
{
  class SOTH_EXPORT AllocatorML
  {
    typedef std::list<unsigned int> resource_t;
    typedef resource_t::iterator resource_iter_t;

    resource_t resource;
    unsigned int max;

  public:
    AllocatorML( unsigned int max ) : resource(),max(max) {}

    void reset();
    void resetTop( unsigned int min );
    unsigned int get();
    void put( const unsigned int & token );
    void disp( std::ostream & os ) const;

    SOTH_EXPORT friend std::ostream& operator<<( std::ostream & os, const AllocatorML & aml );
  };

} // namespace soth


#endif  // #ifndef __SOTH_ALLOCATOR__
