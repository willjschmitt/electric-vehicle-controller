_ELECTRIC_VEHICLE_CONTROLLER_PROCESSOR="32MK1024GPE100"

# The main binary to run on the micro-controller. It is compiled specifically
# for our PIC32MK microcontroller, with memory and peripheral settings compiled
# into the binary.
cc_binary(
    name = "electric_vehicle_controller",
    srcs = [
        "main.cpp",
        "config.h",
    ],
    # TODO(willjschmitt): Instead of using a manual tag, restrict this target by
    #  a platform constraint for the MCP32 CPU.
    tags = [
        "manual",
    ],
    copts = [
        "-mprocessor={}".format(_ELECTRIC_VEHICLE_CONTROLLER_PROCESSOR),
    ],
    linkopts = [
        "-Wl,--defsym=_min_heap_size=0xF00",
        "-mprocessor={}".format(_ELECTRIC_VEHICLE_CONTROLLER_PROCESSOR),
    ],
    deps = [
        "//control:current_regulator",
        "//induction_motor_controls:induction_motor_controller",
        "@pic32mk_gp_dfp//:xc_headers",
    ],
)
