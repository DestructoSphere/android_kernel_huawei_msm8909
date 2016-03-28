#include <linux/module.h>

#include "hw_camera_log.h"

int g_hw_camera_log_mask = HW_CAMERA_LOG_I;

module_param_named(g_hw_camera_log_mask, g_hw_camera_log_mask, int, 0664);

