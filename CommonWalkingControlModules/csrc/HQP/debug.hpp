/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic, 2005
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotDebug.h
 * Project:   STack of Tasks
 * Author:    Nicolas Mansard
 *
 * Version control
 * ===============
 *
 *  $Id$
 *
 * Description
 * ============
 *
 * Macro de trace et de debugage
 *
 *   - TRACAGE:    TRACE et ERROR_TRACE fonctionnent comme des printf
 * avec retour chariot en fin de fonction.
 *                 CERROR et CTRACE fonctionnent comme les flux de sortie
 * C++ cout et cerr.
 *   - DEBUGAGE:   DEBUG_TRACE(niv,  et DERROR_TRACE(niv, fonctionnent
 * comme des printf, n'imprimant que si le niveau de trace 'niv' est
 * superieur au mode de debugage SOTH_DEBUG_MODE.
 *                 CDEBUG(niv) fonctionne comme le flux de sortie C++ cout.
 *                 DEBUG_ENABLE(niv) vaut 1 ssi le niveau de tracage 'niv'
 * est superieur au  mode de debugage DEBUG_MODE. Il vaut 0 sinon.
 *   - PROG DEFENSIVE: DEFENSIF(a) vaut a ssi le mode defensif est active,
 * et vaut 0 sinon.
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/




#ifndef __SOTH_DEBUG_HH
#define __SOTH_DEBUG_HH

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdarg.h>
#include "soth/api.hpp"

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
#ifdef NDEBUG
#   undef SOTH_DEBUG
#endif

namespace soth
{

#ifndef SOTH_DEBUG_MODE
#define SOTH_DEBUG_MODE 0
#endif
#ifndef SOTH_TEMPLATE_DEBUG_MODE
#define SOTH_TEMPLATE_DEBUG_MODE 0
#endif

#define SOT_COMMON_TRACES do {  \
		    va_list arg; \
		    va_start(arg,format); \
		    vsnprintf( charbuffer,SIZE,format,arg ); \
		    va_end(arg); \
		    outputbuffer << tmpbuffer.str() << charbuffer <<std::endl; \
		} while(0)

class SOTH_EXPORT sotDebugTrace
{
 public:
    static const int SIZE = 512;

    std::stringstream tmpbuffer;
    std::ostream& outputbuffer;
    std::ofstream voidbuffer;
    char charbuffer[SIZE+1];
    int traceLevel;
    int traceLevelTemplate;
    int debugPrior;

    sotDebugTrace( std::ostream& os ): outputbuffer(os),debugPrior(0) {}

    inline void trace( const int level,const char* format,...)
	{ if( level+debugPrior<=traceLevel ) SOT_COMMON_TRACES; tmpbuffer.str(""); }
    inline void trace( const char* format,...){ SOT_COMMON_TRACES;  tmpbuffer.str(""); }
    inline void trace( const int level=-1 )
	{ if( level+debugPrior<=traceLevel ) outputbuffer << tmpbuffer.str(); tmpbuffer.str("");  }

    inline void traceTemplate( const int level,const char* format,...)
	{ if( level+debugPrior<=traceLevelTemplate ) SOT_COMMON_TRACES; tmpbuffer.str(""); }
    inline void traceTemplate( const char* format,...)
	{ SOT_COMMON_TRACES; tmpbuffer.str("");  }

    inline sotDebugTrace& pre( const std::ostream&  ) { return *this; }
    inline sotDebugTrace& pre( const std::ostream& ,int  level)
    { traceLevel = level; return *this; }
/*     inline sotDebugTrace& preTemplate( const std::ostream& dummy,int level )  */
/* 	{ traceLevelTemplate = level; return *this; } */


    static const char * DEBUG_FILENAME_DEFAULT;
    static void openFile( const char * filename = DEBUG_FILENAME_DEFAULT );
    static void closeFile( const char * filename = DEBUG_FILENAME_DEFAULT );

    operator std:: ostream& ()    {  return outputbuffer; }
};


SOTH_EXPORT extern sotDebugTrace sotDEBUGFLOW;
SOTH_EXPORT extern sotDebugTrace sotERRORFLOW;

#ifdef SOTH_DEBUG
#define sotPREDEBUG  "% " << __FILE__ << ": " <<__FUNCTION__  \
                              << "(#" << __LINE__ << ") :\n"
#define sotPREERROR  "\t!! "<<__FILE__ << ": " <<__FUNCTION__  \
                            << "(#" << __LINE__ << ") :"

#  define sotDEBUG(level) if( (level+sotDEBUGFLOW.debugPrior>SOTH_DEBUG_MODE)||(!sotDEBUGFLOW.outputbuffer.good()) ) ;\
    else sotDEBUGFLOW.outputbuffer<<sotPREDEBUG
#  define sotDEBUGMUTE(level) if( (level+sotDEBUGFLOW.debugPrior>SOTH_DEBUG_MODE)||(!sotDEBUGFLOW.outputbuffer.good()) ) ;\
    else sotDEBUGFLOW.outputbuffer
#  define sotERROR  if(!sotDEBUGFLOW.outputbuffer.good()); else sotERRORFLOW.outputbuffer << sotPREERROR
#  define sotDEBUGF if(!sotDEBUGFLOW.outputbuffer.good()); else sotDEBUGFLOW.pre(sotDEBUGFLOW.tmpbuffer<<sotPREDEBUG,SOTH_DEBUG_MODE).trace
#  define sotERRORF if(!sotDEBUGFLOW.outputbuffer.good()); else sotERRORFLOW.pre(sotERRORFLOW.tmpbuffer<<sotPREERROR).trace
// TEMPLATE
#  define sotTDEBUG(level) if( (level+sotDEBUGFLOW.debugPrior>SOTH_TEMPLATE_DEBUG_MODE)||(!sotDEBUGFLOW.outputbuffer.good()) ) ;\
    else sotDEBUGFLOW.outputbuffer << sotPREDEBUG
#  define sotTDEBUGF  if(!sotDEBUGFLOW.outputbuffer.good()); else sotDEBUGFLOW.pre(sotDEBUGFLOW.tmpbuffer<<sotPREDEBUG,SOTH_TEMPLATE_DEBUG_MODE).trace
inline bool sotDEBUG_ENABLE( const int & level ) { return level<=SOTH_DEBUG_MODE; }
inline bool sotTDEBUG_ENABLE( const int & level ) { return level<=SOTH_TEMPLATE_DEBUG_MODE; }

class sotDEBUGPRIORclass
{
 protected:
    int previousLevel;
    sotDEBUGPRIORclass( void ) {};
 public:
 sotDEBUGPRIORclass( int prior )
     : previousLevel(sotDEBUGFLOW.debugPrior)
	{
	    sotDEBUGFLOW.debugPrior+=prior;
	}
    ~sotDEBUGPRIORclass( void )
	{
	    sotDEBUGFLOW.debugPrior=previousLevel;
	}
};

#define sotDEBUGPRIOR(a) sotDEBUGPRIORclass sotDEBUGPRIORclass##__FUNCTION__(a);


/* -------------------------------------------------------------------------- */
#else // #ifdef SOTH_DEBUG
#define sotPREERROR  "\t!! "<<__FILE__ << ": " <<__FUNCTION__  \
                            << "(#" << __LINE__ << ") :"
#  define sotDEBUG(level) if( 1 ) ; else std::cout
#  define sotDEBUGMUTE(level) if( 1 ) ; else std::cout
#  define sotERROR sotERRORFLOW.outputbuffer << sotPREERROR
#define sotDEBUGPRIOR(a) do {} while(0)
inline void sotDEBUGF( const int ,const char* ,...) { return; }
inline void sotDEBUGF( const char* ,...) { return; }
inline void sotERRORF( const int ,const char* ,...) { return; }
inline void sotERRORF( const char* ,...) { return; }
// TEMPLATE
#  define sotTDEBUG(level) if( 1 ) ; else std::cout
inline void sotTDEBUGF( const int ,const char* ,...) { return; }
inline void sotTDEBUGF( const char* ,...) { return; }
#define sotDEBUG_ENABLE(level) false
#define sotTDEBUG_ENABLE(level) false

#endif // #ifdef SOTH_DEBUG
/* -------------------------------------------------------------------------- */

#define sotDEBUGIN(level) sotDEBUG(level) << "# In {" << std::endl
#define sotDEBUGOUT(level) sotDEBUG(level) << "# Out }" << std::endl
#define sotDEBUGINOUT(level) sotDEBUG(level) << "# In/Out { }" << std::endl

#define sotTDEBUGIN(level) sotTDEBUG(level) << "# In {" << std::endl
#define sotTDEBUGOUT(level) sotTDEBUG(level) << "# Out }" << std::endl
#define sotTDEBUGINOUT(level) sotTDEBUG(level) << "# In/Out { }" << std::endl



} //namespace soth;

/*  Prevent unused warning */
#define UNUSED(x) ((void)x)


#endif /* #ifdef __SOTH_DEBUG_HH */


/*
 * Local variables:
 * c-basic-offset: 4
 * End:
 */
