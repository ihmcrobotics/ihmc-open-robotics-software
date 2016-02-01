#include <soth/Allocator.hpp>
#include <assert.h>

namespace soth
{

  /* Replace all the tokens in the resource. */
  void AllocatorML::
  reset()
  {
    resetTop(0);
  }

  /* Replace the tokens [min,max[ in the ressource. */
  void AllocatorML::
  resetTop( unsigned int min )
  {
    assert(min<=max);
    resource.resize( max-min );
    unsigned int inc = min;
    for( resource_iter_t iter=resource.begin();iter!=resource.end();++iter )
      { (*iter) = inc++; }
    assert( resource.size() == 0 || resource.back() == max-1 );
  }

  unsigned int AllocatorML::
  get()
  {
    assert( resource.size()>0 );
    const unsigned int token = resource.front();
    resource.pop_front();
    return token;
  }

  void AllocatorML::
  put( const unsigned int & token )
  {
    resource.push_front(token);
    assert( resource.size()<=max );
  }

  void AllocatorML::
  disp( std::ostream & os ) const
  {
    typedef resource_t::const_iterator resource_citer_t;
    os << "[ ";
    for( resource_citer_t iter=resource.begin();iter!=resource.end();++iter )
      {	  os << *iter << " ";       }
    os << "  ];";
  }

  std::ostream& operator<<( std::ostream & os, const AllocatorML & aml )
  { aml.disp(os); return os; }

} // namespace soth
