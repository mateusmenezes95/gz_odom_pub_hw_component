#ifndef GZ_ODOM_PUB_HW_COMPONENT__VISIBILITY_CONTROL_H_
#define GZ_ODOM_PUB_HW_COMPONENT__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GZ_ODOM_PUB_HW_COMPONENT_EXPORT __attribute__ ((dllexport))
    #define GZ_ODOM_PUB_HW_COMPONENT_IMPORT __attribute__ ((dllimport))
  #else
    #define GZ_ODOM_PUB_HW_COMPONENT_EXPORT __declspec(dllexport)
    #define GZ_ODOM_PUB_HW_COMPONENT_IMPORT __declspec(dllimport)
  #endif
  #ifdef GZ_ODOM_PUB_HW_COMPONENT_BUILDING_LIBRARY
    #define GZ_ODOM_PUB_HW_COMPONENT_PUBLIC GZ_ODOM_PUB_HW_COMPONENT_EXPORT
  #else
    #define GZ_ODOM_PUB_HW_COMPONENT_PUBLIC GZ_ODOM_PUB_HW_COMPONENT_IMPORT
  #endif
  #define GZ_ODOM_PUB_HW_COMPONENT_PUBLIC_TYPE GZ_ODOM_PUB_HW_COMPONENT_PUBLIC
  #define GZ_ODOM_PUB_HW_COMPONENT_LOCAL
#else
  #define GZ_ODOM_PUB_HW_COMPONENT_EXPORT __attribute__ ((visibility("default")))
  #define GZ_ODOM_PUB_HW_COMPONENT_IMPORT
  #if __GNUC__ >= 4
    #define GZ_ODOM_PUB_HW_COMPONENT_PUBLIC __attribute__ ((visibility("default")))
    #define GZ_ODOM_PUB_HW_COMPONENT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GZ_ODOM_PUB_HW_COMPONENT_PUBLIC
    #define GZ_ODOM_PUB_HW_COMPONENT_LOCAL
  #endif
  #define GZ_ODOM_PUB_HW_COMPONENT_PUBLIC_TYPE
#endif

#endif  // GZ_ODOM_PUB_HW_COMPONENT__VISIBILITY_CONTROL_H_
