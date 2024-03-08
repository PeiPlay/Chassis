#pragma once
#include "main.h"

//* 引入板级支持包
#include "fdcan_bsp.h"
#include "math_bsp.h"
#include "tim_bsp.h"
//* 引入模块组件
#include "DwtClock.h"
#include "TimedInterrupt.h"
#include "Communication.h"
#include "DT35.h"
#include "Gamepad.h"
#include "Route.h"

//* 引入电机驱动
#include "M2006.h"


//*--------------------------------初始化DT35--------------------------------*//
extern DT35_t dt35_left;
extern DT35_t dt35_right;
extern DT35_t dt35_back;
#define DT35_LEFT_INIT (DT35_t) {                                           \
    .hfdcan = &hfdcan1,                                                     \
    .id = 0x35,                                                             \
    .distfilter = {                                                         \
        .map_k = 0.01,                                                      \
        .map_b = 0.00                                                       \
     },                                                                     \
    .speedfilter = {                                                        \
        .map_k = 0.01,                                                      \
        .map_b = 0.00,                                                      \
        .inertia = 0.75,                                                    \
        .deadband = 0.01                                                    \
    }                                                                       \
}
#define DT35_RIGHT_INIT (DT35_t) {                                          \
    .hfdcan = &hfdcan1,                                                     \
    .id = 0x36,                                                             \
    .distfilter = {                                                         \
        .map_k = 0.01,                                                      \
        .map_b = 0.00                                                       \
     },                                                                     \
    .speedfilter = {                                                        \
        .map_k = 0.01,                                                      \
        .map_b = 0.00,                                                      \
        .inertia = 0.75,                                                    \
        .deadband = 0.01                                                    \
    }                                                                       \
}
#define DT35_BACK_INIT (DT35_t) {                                           \
    .hfdcan = &hfdcan1,                                                     \
    .id = 0x37,                                                             \
    .distfilter = {                                                         \
        .map_k = 0.01,                                                      \
        .map_b = 0.00                                                       \
     },                                                                     \
    .speedfilter = {                                                        \
        .map_k = 0.01,                                                      \
        .map_b = 0.00,                                                      \
        .inertia = 0.75,                                                    \
        .deadband = 0.01                                                    \
    }                                                                       \
}
//*--------------------------------初始化码盘--------------------------------*//

//*--------------------------------初始化路径--------------------------------*//


extern Route_Path_t path_seedling;  //取苗路径
extern Route_Path_t path_uphill;    //上坡路径
extern Route_Path_t path_ballserve; //发球路径
extern Route_t route;               //总路径

#define ROUTE_PATH_SEEDLING_INIT (Route_Path_t){                            \
    .point_total = 4,                                                       \
    .point = {                                                              \
        (Route_Point_t){/**Point0**/                                        \
            .approch_check = 5,                                             \
            .x = {                                                          \
                .target = 0,                                                \
                .kp = 0.0,       .ki = 0.0,       .kd = 0.0,                \
                .startzone = 10, .deadband = 0.1, .approach = 0.1,          \
                .integralLimit = 10,    .outputLimit = 20                   \
            },                                                              \
            .y = {                                                          \
                .target = 0,                                                \
                .kp = 0.0,       .ki = 0.0,       .kd = 0.0,                \
                .startzone = 10, .deadband = 0.1, .approach = 0.1,          \
                .integralLimit = 10,    .outputLimit = 20                   \
            },                                                              \
            .z = {                                                          \
                .target = 0,                                                \
                .kp = 0.0,       .ki = 0.0,       .kd = 0.0,                \
                .startzone = 10, .deadband = 0.1, .approach = 0.1,          \
                .integralLimit = 10,    .outputLimit = 20                   \
            }                                                               \
        },                                                                  \
        (Route_Point_t){/**Point1**/                                        \
            .approch_check = 5,                                             \
            .x = {                                                          \
                .target = 0,                                                \
                .kp = 0.0,       .ki = 0.0,       .kd = 0.0,                \
                .startzone = 10, .deadband = 0.1, .approach = 0.1,          \
                .integralLimit = 10,    .outputLimit = 20                   \
            },                                                              \
            .y = {                                                          \
                .target = 0,                                                \
                .kp = 0.0,       .ki = 0.0,       .kd = 0.0,                \
                .startzone = 10, .deadband = 0.1, .approach = 0.1,          \
                .integralLimit = 10,    .outputLimit = 20                   \
            },                                                              \
            .z = {                                                          \
                .target = 0,                                                \
                .kp = 0.0,       .ki = 0.0,       .kd = 0.0,                \
                .startzone = 10, .deadband = 0.1, .approach = 0.1,          \
                .integralLimit = 10,    .outputLimit = 20                   \
            }                                                               \
        },                                                                  \
        (Route_Point_t){/**Point2**/                                        \
            .approch_check = 5,                                             \
            .x = {                                                          \
                .target = 0,                                                \
                .kp = 0.0,       .ki = 0.0,       .kd = 0.0,                \
                .startzone = 10, .deadband = 0.1, .approach = 0.1,          \
                .integralLimit = 10,    .outputLimit = 20                   \
            },                                                              \
            .y = {                                                          \
                .target = 0,                                                \
                .kp = 0.0,       .ki = 0.0,       .kd = 0.0,                \
                .startzone = 10, .deadband = 0.1, .approach = 0.1,          \
                .integralLimit = 10,    .outputLimit = 20                   \
            },                                                              \
            .z = {                                                          \
                .target = 0,                                                \
                .kp = 0.0,       .ki = 0.0,       .kd = 0.0,                \
                .startzone = 10, .deadband = 0.1, .approach = 0.1,          \
                .integralLimit = 10,    .outputLimit = 20                   \
            }                                                               \
        },                                                                  \
            (Route_Point_t){/**Point3**/                                    \
            .approch_check = 5,                                             \
            .x = {                                                          \
                .target = 0,                                                \
                .kp = 0.0,       .ki = 0.0,       .kd = 0.0,                \
                .startzone = 10, .deadband = 0.1, .approach = 0.1,          \
                .integralLimit = 10,    .outputLimit = 20                   \
            },                                                              \
            .y = {                                                          \
                .target = 0,                                                \
                .kp = 0.0,       .ki = 0.0,       .kd = 0.0,                \
                .startzone = 10, .deadband = 0.1, .approach = 0.1,          \
                .integralLimit = 10,    .outputLimit = 20                   \
            },                                                              \
            .z = {                                                          \
                .target = 0,                                                \
                .kp = 0.0,       .ki = 0.0,       .kd = 0.0,                \
                .startzone = 10, .deadband = 0.1, .approach = 0.1,          \
                .integralLimit = 10,    .outputLimit = 20                   \
            }                                                               \
        },                                                                  \
    }                                                                       \
}




