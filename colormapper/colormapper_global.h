#pragma once

#include <QtCore/qglobal.h>

#ifndef BUILD_STATIC
# if defined(COLORMAPPER_LIB)
#  define COLORMAPPER_EXPORT Q_DECL_EXPORT
# else
#  define COLORMAPPER_EXPORT Q_DECL_IMPORT
# endif
#else
# define COLORMAPPER_EXPORT
#endif
