#ifndef PID_CONTROL_V3__VISIBILITY_CONTROL_H_
#define PID_CONTROL_V3__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PID_CONTROL_V3_EXPORT __attribute__ ((dllexport))
    #define PID_CONTROL_V3_IMPORT __attribute__ ((dllimport))
  #else
    #define PID_CONTROL_V3_EXPORT __declspec(dllexport)
    #define PID_CONTROL_V3_IMPORT __declspec(dllimport)
  #endif
  #ifdef PID_CONTROL_V3_BUILDING_LIBRARY
    #define PID_CONTROL_V3_PUBLIC PID_CONTROL_V3_EXPORT
  #else
    #define PID_CONTROL_V3_PUBLIC PID_CONTROL_V3_IMPORT
  #endif
  #define PID_CONTROL_V3_PUBLIC_TYPE PID_CONTROL_V3_PUBLIC
  #define PID_CONTROL_V3_LOCAL
#else
  #define PID_CONTROL_V3_EXPORT __attribute__ ((visibility("default")))
  #define PID_CONTROL_V3_IMPORT
  #if __GNUC__ >= 4
    #define PID_CONTROL_V3_PUBLIC __attribute__ ((visibility("default")))
    #define PID_CONTROL_V3_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PID_CONTROL_V3_PUBLIC
    #define PID_CONTROL_V3_LOCAL
  #endif
  #define PID_CONTROL_V3_PUBLIC_TYPE
#endif
#endif  // PID_CONTROL_V3__VISIBILITY_CONTROL_H_
// Generated 04-May-2026 23:01:58
 