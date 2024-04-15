#ifndef PGE_CORE__VISIBILITY_CONTROL_H_
#define PGE_CORE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PGE_CORE_EXPORT __attribute__ ((dllexport))
    #define PGE_CORE_IMPORT __attribute__ ((dllimport))
  #else
    #define PGE_CORE_EXPORT __declspec(dllexport)
    #define PGE_CORE_IMPORT __declspec(dllimport)
  #endif
  #ifdef PGE_CORE_BUILDING_LIBRARY
    #define PGE_CORE_PUBLIC PGE_CORE_EXPORT
  #else
    #define PGE_CORE_PUBLIC PGE_CORE_IMPORT
  #endif
  #define PGE_CORE_PUBLIC_TYPE PGE_CORE_PUBLIC
  #define PGE_CORE_LOCAL
#else
  #define PGE_CORE_EXPORT __attribute__ ((visibility("default")))
  #define PGE_CORE_IMPORT
  #if __GNUC__ >= 4
    #define PGE_CORE_PUBLIC __attribute__ ((visibility("default")))
    #define PGE_CORE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PGE_CORE_PUBLIC
    #define PGE_CORE_LOCAL
  #endif
  #define PGE_CORE_PUBLIC_TYPE
#endif

#endif  // PGE_CORE__VISIBILITY_CONTROL_H_
