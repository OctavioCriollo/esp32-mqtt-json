# Changelog

All notable user-facing changes to the NT FAN Controller firmware.
Format inspired by [Keep a Changelog](https://keepachangelog.com/);
versions follow the `v*` release tags (each tag publishes a flashable
`.bin` on the Releases page).

## [Unreleased]

### Changed — ⚠️ BREAKING
- **Telemetry topic suffix renamed:** the device now publishes to
  `<operator>/<city>/<site>-<mac>/<subsystem>/telemetry` (was
  `/telemetria`). Broker authorization rules and subscriber wildcards must
  be updated to match. `/control` is unchanged.

### Added
- **Publish interval** (MQTT Setup): dropdown presets from 1 s to 5 min for
  the MQTT publish cadence, NVS-persisted (default 1 s = previous behavior).
- **CA Certificate upload** (MQTT Setup): load your broker's CA from a file
  (Choose File), stored in NVS and applied on the next reconnect; a
  Factory CA button restores the built-in Let's Encrypt root. Brokers with
  a non-Let's Encrypt chain no longer require a reflash.
- **Configurable PWM curve**: power-law `n + (1−n)·xᵖ` edited by dragging
  two points on a live graph (tap a FAN gauge in Control); live FAN1/FAN2
  radial gauges colored by duty.
- **Configurable temperature hysteresis**: draggable alarm reset band
  (Schmitt trigger) beside the High Temp threshold.
- **Configurable relay mapping** (OUT1–4 → temp/door/fan alarms, mutual
  exclusion) and **fan-alarm logic** (OR / AND / single fan).
- **BME280 driver** (I2C temperature/humidity/pressure) — ready, dormant
  until a sensor is connected.
- Tab persistence across reloads; OTA upload progress with inline result.

### Fixed
- Offline detection now shows within ~5–7 s (bounded requests, no more
  minute-long TCP hangs) and no longer flaps on reconnect.
- Intermittent blank dashboard on reload (portal now streamed chunked with
  a size-tracking reserve).
- Touch UX: no stray tap-highlight rectangles, taps no longer land on the
  neighboring element (the 2 s refresh updates data only, never layout),
  no reload flicker (snapshot restore).
- Tachometer pulse counter guarded with a critical section (no lost edges).
- Full English UI (config values such as the subsystem list remain user
  data).

## [6.0.0] — 2026-07-04
End-to-end modernization: pioarduino platform (Arduino core 3.x /
ESP-IDF 5.x), real TLS validation (ISRG Root X1), NVS runtime config with
web portal + browser OTA, dedicated FreeRTOS control task decoupled from
networking, thermal failsafe, AP rescue mode with auto STA recovery,
SNTP timestamps, CI builds + tagged releases.
