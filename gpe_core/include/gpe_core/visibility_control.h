#ifndef GPE_CORE__VISIBILITY_CONTROL_H_
#define GPE_CORE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GPE_CORE_EXPORT __attribute__ ((dllexport))
    #define GPE_CORE_IMPORT __attribute__ ((dllimport))
  #else
    #define GPE_CORE_EXPORT __declspec(dllexport)
    #define GPE_CORE_IMPORT __declspec(dllimport)
  #endif
  #ifdef GPE_CORE_BUILDING_LIBRARY
    #define GPE_CORE_PUBLIC GPE_CORE_EXPORT
  #else
    #define GPE_CORE_PUBLIC GPE_CORE_IMPORT
  #endif
  #define GPE_CORE_PUBLIC_TYPE GPE_CORE_PUBLIC
  #define GPE_CORE_LOCAL
#else
  #define GPE_CORE_EXPORT __attribute__ ((visibility("default")))
  #define GPE_CORE_IMPORT
  #if __GNUC__ >= 4
    #define GPE_CORE_PUBLIC __attribute__ ((visibility("default")))
    #define GPE_CORE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GPE_CORE_PUBLIC
    #define GPE_CORE_LOCAL
  #endif
  #define GPE_CORE_PUBLIC_TYPE
#endif

#endif  // GPE_CORE__VISIBILITY_CONTROL_H_
