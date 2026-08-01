#ifndef TELEMETRY_PROTOCOL_H
#define TELEMETRY_PROTOCOL_H

#define TELEMETRY_PERIOD_MS              (10UL)
#define TELEMETRY_FRAME_SIZE             (48U)
#define TELEMETRY_PROTOCOL_VERSION       (1U)

#define TELEMETRY_FLAG_IMU_READY         (1U << 0)
#define TELEMETRY_FLAG_IMU_ONLINE        (1U << 1)
#define TELEMETRY_FLAG_RUNNING           (1U << 2)
#define TELEMETRY_FLAG_STRAIGHT          (1U << 3)
#define TELEMETRY_FLAG_CURVE             (1U << 4)
#define TELEMETRY_FLAG_STOPPING          (1U << 5)
#define TELEMETRY_FLAG_ENCODER_VALID     (1U << 6)

#endif /* TELEMETRY_PROTOCOL_H */
