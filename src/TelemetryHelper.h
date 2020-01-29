#ifndef TelemetryHelper_h
#define TelemetryHelper_h

#include <Arduino.h>

/**
 * Possible system states.
 */
enum class SystemState: uint8_t {
    /**
     * The system is currently running the boot sequence.
     */
    BOOTING = 0,

    /**
     * The system is operating normally.
     */
    NORMAL = 1,

    /**
     * The system is currently performing an OTA update.
     */
    UPDATING = 2,

    /**
     * The system is disabled. Either intentionally by the user or
     * by the system due to some internal system condition such as
     * falling into failsafe/configuration mode.
     */
    DISABLED = 3
};

/**
 * Helper class providing static telemetry helper methods.
 */
class TelemetryHelper
{
public:
    /**
     * Gets a string describing the specified MQTT client state.
     * @param state The state to get the description for.
     * @return A string describing the MQTT state.
     */
    static String getMqttStateDesc(int state);
};

#endif