# MQTT contract

Everything the device publishes. This is the integration surface — treat changes here as
breaking.

**All publishes are retained.** The device reports only on events (a session ending, a
30-minute heartbeat), so without retain a consumer restarting between events would see nothing
at all until the next irrigation run. Retained values mean Home Assistant has current state
immediately on startup.

Nothing is subscribed. The device is publish-only.

## Status and identity

| Topic | Payload | When |
|---|---|---|
| `irrig_leak/status/LWT` | `Connected` / `Disconnected` | On connect; `Disconnected` is the broker's Last Will |
| `irrig_leak/version` | Version string | On connect |
| `irrig_leak/wifi_ssid` | SSID | On connect, then each heartbeat |
| `irrig_leak/wifi_dbm` | RSSI, integer dBm | On connect, then each heartbeat |

## Idle heartbeat

Published at boot and every `HEARTBEAT_SECS` (30 minutes) while no session is running.

| Topic | Payload | Notes |
|---|---|---|
| `irrig_leak/idle/time_stamp` | RFC3339 timestamp | Proves the device is alive |
| `irrig_leak/idle/water_pressure` | PSI, 2 dp | **Publishes `-99.00` if the sensor is unavailable** |
| `irrig_leak/idle/water_temperature` | °F, 2 dp | Same sentinel behaviour |

The sentinel is intentional here — it lets Home Assistant detect and alert on a failed sensor.

## Session report

Published once when a session ends (`INACTIVITY_TIMEOUT_SECS` with no flow).

| Topic | Payload | Notes |
|---|---|---|
| `irrig_leak/report/time_stamp` | RFC3339 timestamp | Marks the report |
| `irrig_leak/report/tot_gals_all_zones` | Integer gallons | **Includes zone-0 water** |
| `irrig_leak/report/median_gpm_zone_{1..4}` | GPM, 2 dp | Median over the whole zone run |
| `irrig_leak/report/median_gpm_zone_{1..4}/attributes` | JSON | See below |
| `irrig_leak/report/avg_psi_zone_{1..4}` | PSI, 2 dp | **Skipped entirely if unavailable** |
| `irrig_leak/report/avg_psi_zone_{1..4}/attributes` | JSON | Skipped with its state topic |
| `irrig_leak/report/run_dur_zone_{1..4}` | Minutes, 1 dp | |
| `irrig_leak/report/valve_leak` | Integer gallons | Valves-off flow — see below |
| `irrig_leak/report/valve_leak/attributes` | JSON | |

Zones that did not run still publish, reporting 0 gallons and the pressure at report time. A
report is a synchronized snapshot of one session, never a mix of runs.

### Attribute payloads

```jsonc
// median_gpm_zone_N/attributes
{"valveNum": "2", "measuredZoneGallons": "37", "maxGPM": "9.60"}

// avg_psi_zone_N/attributes  — waterTemperature omitted if unavailable
{"valveNum": "2", "maxPSI": "78.25", "minPSI": "71.10", "waterTemperature": "68.40"}

// valve_leak/attributes
{"gallons": "5", "medianGPM": "3.25", "maxGPM": "4.10"}
```

Values are quoted strings, not JSON numbers. Consumers must cast — the Home Assistant templates
use `| int(0)` and `| float(0)`.

## The zone-0 rule

**There is no zone 0 in the per-zone series.** `median_gpm_zone_0` and `avg_psi_zone_0` do not
exist and never will.

Zone 0 means "flow with every valve off," which is a fault condition, not a zone. It is
reported solely through `irrig_leak/report/valve_leak`. Publishing it as a fifth zone invites
consumers to sum or average it alongside real zones, which is meaningless.

### State is gated; the `gallons` attribute is not

| | Below `MIN_LEAK_GALS` | Above |
|---|---|---|
| **State** | `0` | actual gallons |
| **`gallons` attribute** | actual gallons | actual gallons |

The state is gated so ordinary valve-transition noise never raises an alarm. The attribute is
ungated so that noise stays *visible*, which is what lets you tune `MIN_LEAK_GALS` against real
observations rather than guesswork.

### It can arrive mid-session

Unlike every other report topic, `valve_leak` may be published while a session is still
running — once per session, the moment volume crosses the threshold. A stuck-open valve flows
continuously, so the session-ending timeout never fires and an end-of-session-only report would
never be sent. Consumers must not assume `valve_leak` implies the session has ended.

## Tunable parameters

Defined at the top of [src/main.cpp](../src/main.cpp).

| Constant | Default | Effect |
|---|---|---|
| `MIN_LEAK_GALS` | 2 | Zone-0 volume above which flow counts as a leak. Raise if valve-transition noise causes false alarms; the ungated `gallons` attribute shows the real figures to tune against. |
| `INACTIVITY_TIMEOUT_SECS` | 90 | Quiet time that ends a session. Must exceed the longest gap between pulses at your *lowest* flow rate, or one run splits into several reports. |
| `HEARTBEAT_SECS` | 1800 | Idle publish interval. |
| `FLOW_GALS_PER_PULSE` | 1 | Meter calibration. |
| `VALVE_POLL_INTERVAL_MS` | 1000 | How often idle valve changes are logged. Kept off the hot loop because `getActiveValve()` costs a full sample window when no valve is energised. |
| `VALVE_AC_SAMPLE_MS` | 25 | Valve sense window. **Must exceed one mains cycle** — 16.7 ms at 60 Hz, 20 ms at 50 Hz. |
| `GPM_HIST_BIN_WIDTH` | 0.25 | Median resolution. |
| `GPM_HIST_BINS` | 600 | Together with bin width, sets the 0–150 GPM range. **Must exceed any achievable reading** or samples clamp and skew the median. |
| `MQTT_SOCKET_TIMEOUT_SECS` | 3 | Caps a single broker connect attempt. PubSubClient defaults to 15; lowered because this call blocks the main loop, which also polls the flow sensor. |
| `LINK_RETRY_IDLE_MS` | 5000 | Minimum gap between runtime reconnect attempts when idle. |
| `LINK_RETRY_SESSION_MS` | 60000 | Same, but while a session is running. Longer because a blocking broker attempt can cost a counted gallon, and reports are not urgent. |
| `MAX_MQTT_CONNECT_ATTTEMPTS` | 10 | **Boot path only.** How many times `setup()` retries before giving up and entering the main loop anyway. |
| `MAX_PRESSURE` | 100 | Sensor full-scale, PSI. Match your part. |
| `PRESSURE_SENSOR_INSTALLED` | 1 | Set 0 to omit the sensor entirely. |
| `PREFER_FAHRENHEIT` | 1 | Set 0 for Celsius. |
| `PRESSURE_SENSOR_INVALID` | -99.0 | Unavailable sentinel. |
| `LED_BLINK_FAST_MS` | 100 | Yellow while connecting; also the blue report burst rate. |
| `LED_BLINK_SLOW_MS` | 500 | Yellow while MQTT is down. |
| `FLOW_PULSE_LED_MS` | 500 | Blue blip per gallon. |
| `REPORT_BLINK_MS` | 5000 | Blue burst when a report publishes. |
| `WIFI_DIAGNOSTICS` | 0 | Set 1 for a boot-time AP scan and decoded WiFi events. |
