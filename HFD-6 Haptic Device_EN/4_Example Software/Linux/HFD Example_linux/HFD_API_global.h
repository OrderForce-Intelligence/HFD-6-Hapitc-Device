#ifndef HFD_API_GLOBAL_H
#define HFD_API_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(HFD_API_LIBRARY)
#  define HFD_API_EXPORT Q_DECL_EXPORT
#else
#  define HFD_API_EXPORT Q_DECL_IMPORT
#endif

#endif // HFD_API_GLOBAL_H
