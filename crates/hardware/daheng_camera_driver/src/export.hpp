#pragma once

// dahengcamera library export macros
#ifdef DAHENGCAMERA_EXPORTS
# define DAHENGCAMERA_API __attribute__((visibility("default")))
#else
# define DAHENGCAMERA_API
#endif
