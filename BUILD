cc_library(
    name = "USIWire_slave_impl",
    srcs = [
        "src/USI_TWI_Slave/USI_TWI_Slave.c",
        "src/USI_TWI_Slave/usi_io.h"
    ],
    hdrs = ["src/USI_TWI_Slave/USI_TWI_Slave.h"],
)

cc_inc_library(
    name = "USIWire_slave",
    hdrs = [
        "src/USI_TWI_Slave/USI_TWI_Slave.h",
        "src/USI_TWI_Slave/usi_io.h"
    ],
    prefix = "src/USI_TWI_Slave",
    deps = [":USIWire_slave_impl"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "USIWire_master_impl",
    srcs = [
        "src/USI_TWI_Master/USI_TWI_Master.c",
        "src/USI_TWI_Master/usi_io.h"
    ],
    hdrs = ["src/USI_TWI_Master/USI_TWI_Master.h"],
)

cc_inc_library(
    name = "USIWire_master",
    hdrs = [
        "src/USI_TWI_Master/USI_TWI_Master.h",
        "src/USI_TWI_Master/usi_io.h"
    ],
    prefix = "src/USI_TWI_Master",
    deps = [":USIWire_master_impl"],
    visibility = ["//visibility:public"],
)
