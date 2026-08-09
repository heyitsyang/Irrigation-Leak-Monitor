# Troubleshooting

Symptoms actually encountered on this project, with what turned out to be causing them.

## The device never joins WiFi

**Symptom:** endless `Waiting for WiFi ....`, then a reboot after 90 seconds, forever. The AP
is in range and the credentials are right.

**Cause:** the ESP32 caches WiFi credentials in NVS, and those cached credentials can *shadow*
the ones passed to `WiFi.begin()`. The radio keeps trying to associate with a stale SSID that
may no longer exist.

**Fix:** already in `setup_wifi()` — clear the cache before connecting.

```c
WiFi.mode(WIFI_STA);
WiFi.persistent(false);         // don't let NVS-cached credentials shadow the ones below
WiFi.disconnect(true, true);    // clear stale config left in flash
delay(100);
```

**Do not remove those lines.** They look redundant and are not.

**Diagnostic that identified it:** set `WIFI_DIAGNOSTICS` to `1` and watch the driver events.
The giveaway was the *absence* of `STA_DISCONNECTED` events — status reported `DISCONNECTED`
while the driver logged no disconnect reasons at all, meaning it was never attempting to
associate in the first place. A wrong password produces reason 15; an out-of-range AP produces
reason 201. Silence means it never tried.

The scan output also distinguishes a deaf radio from a refused login: if no APs are listed at
all, check the U.FL antenna before anything else.

## Valve readings oscillate, or gallons land in zone 0

**Symptom:** the log reports the active valve changing repeatedly while the input is held
steady, or normal irrigation produces phantom zone-0 volume and false leak alerts.

**Cause:** 24VAC zero crossings. See
[theory of operation](theory-of-operation.md#the-24vac-problem).

**Fix:** ensure `VALVE_AC_SAMPLE_MS` exceeds one full mains cycle — 16.7 ms at 60 Hz, 20 ms at
50 Hz. The default 25 ms covers both.

The apparent oscillation is a beat frequency between the polling rate and the 120 Hz notches;
if you see a suspiciously regular alternation in a log, suspect aliasing rather than a real
signal.

## Pressure reads exactly -99

That is `PRESSURE_SENSOR_INVALID`, the deliberate "no reading available" sentinel. It is never a
measurement. Two distinct causes:

**The sensor is not responding.** Check wiring and the `0x28` address. The boot log reports
whether the sensor answered.

**The I²C bus is wedged.** If a transaction was interrupted mid-read — an OTA update or reset at
the wrong moment — the M3200 can hold SCL low indefinitely. Nine clock pulses and a STOP
condition will not clear it; neither will `Wire.end()`. This was confirmed by scanning every
address `0x01`–`0x7F` after a full recovery attempt and getting a timeout on all of them.

**Only a power cycle of the sensor clears it.** Use the `i2c-scan` environment to confirm
before pulling things apart:

```bash
pio run -e i2c-scan -t upload
```

If the scan reports SCL stuck LOW before `Wire.begin()`, that is the diagnosis.

## Pressure reads a plausible but wrong number

This one cost real time, so it is worth reading before you start recalibrating anything.

**The symptom was:** the sensor reported 91 PSI where a gauge said 75.

**Neither instrument was at fault in the way it looked.** Two separate bad references:

1. **The analog comparison gauge was faulty.** Replacing it with a known-good gauge produced a
   reasonable match.
2. **The zero-pressure test was performed through a Schrader valve, which self-closes.** The
   core snaps shut and seals the vessel, so the sensor measures a *trapped volume* — not
   atmosphere. That is why the "zero" reading came back as −5 PSI.

**Bench-test rule: never zero through a self-closing valve.** Remove the valve core so the port
is genuinely open to atmosphere.

### The technique that settled it

Convert readings back to raw counts and check whether the *expected* value is even encodable.

Between vented and pressurized states the sensor moved 12,620 counts. If that span represented
75 PSI, full scale would require raw count 17,816 — beyond the 14-bit maximum of 16,383. The
expected value was therefore impossible, which proved the reference wrong **without needing to
trust either instrument**.

A related heuristic: if correcting an offset moves a reading *further* from what you expected,
that is evidence your expectation is what needs correcting.

The transfer functions in `readPressureSensor()` have been validated and are correct. The
temperature conversion is algebraically identical to the datasheet's own formula, which
independently corroborates the interpretation. **Do not "fix" them.**

## A Home Assistant entity shows `unknown` or `unavailable`

**After a topic rename:** the entity is subscribed to a topic nothing publishes any more. Update
`state_topic` in `mqtt.yaml` and restart.

**Attributes missing but state fine:** `json_attributes_topic` was added without a full restart.
A YAML reload does not pick it up. Restart, then wait for the next report — the attributes
appear when the next message is published, not immediately.

**Entity exists but nothing backs it:** stale registry entries survive in `.storage` after the
YAML that created them is deleted. Remove them through the UI; they cannot be deleted by editing
YAML.

## Gallons do not add up

The sharpest test available, and it has caught two real accounting bugs.

Open WebSerial, run a session, and count the physical `flow:` lines in the log. Compare with
`tot_gals_all_zones` and with the sum of the per-zone `measuredZoneGallons` attributes plus the
`valve_leak` `gallons` attribute. They must agree exactly.

Two historical failures this caught:

- **Off-by-one at a boundary** — 34 physical pulses reported as 35.
- **A revisited zone losing its earlier total**, because per-visit pulse count was being assigned
  to the zone accumulator instead of added to it. A zone visited twice reported only its second
  visit.

If the totals disagree, the discrepancy is in attribution or accumulation, not in the meter.

## Median GPM is pinned at one odd value

**Symptom:** every zone reports something like 49.88 GPM regardless of actual flow.

**Cause:** samples exceeding the histogram range clamp into the top bin. `49.88` is the centre
of the top bin of a 0–50 GPM range — the clamp value, not a measurement.

**Fix:** widen the range via `GPM_HIST_BINS`. The default 600 bins × 0.25 GPM covers 0–150 GPM.
The range must exceed any reading physically achievable on your system.

## A session splits into several reports

**Most likely cause:** `INACTIVITY_TIMEOUT_SECS` is shorter than the longest gap between pulses
at your lowest flow rate. At 1 pulse per gallon, a 4 GPM drip zone produces one pulse every 15
seconds — but a 1 GPM zone produces one per minute, which is uncomfortably close to the
90-second default.

**Fix:** raise the timeout, keeping in mind it also sets how long after a run ends before the
report arrives.

**Second cause, if it correlates with WiFi trouble:** blocking in the reconnect path. If
`loop()` stalls for longer than `INACTIVITY_TIMEOUT_SECS` while chasing the broker, the session
times out mid-irrigation and the next pulse opens a new one — splitting the run and dividing the
gallons between reports.

This was a real defect: `connectMQTT()` was called directly from `loop()`, and ten attempts at
PubSubClient's default 15-second socket timeout exceeds two minutes. Fixed by capping a single
attempt at `MQTT_SOCKET_TIMEOUT_SECS`, rate-limiting retries, and never attempting the broker
while WiFi is down. If you see fragmented reports whose timestamps line up with
`MQTT lost, reconnecting...` in the log, check those three are still in place.

## Repeated session reports with nothing running

**Symptom:** reports every ~3 minutes with no irrigation happening, each showing zeros, with a
single blue LED blip at the start of each cycle. Home Assistant's per-zone figures keep
resetting to 0.

**Cause:** the magnet parked on the reed switch, holding the flow input LOW, combined with
level-triggered detection. Fixed by counting falling edges instead — see
[theory of operation](theory-of-operation.md#counting-gallons-falling-edges-not-levels).

**A parked magnet is normal.** It is where the impeller happened to stop, not a fault, and needs
no alert. If you see this symptom on firmware that predates the edge-detection change, that is
the cause.

**To check the input state at any time**, send any character to the WebSerial page. The status
line reports `flow pin LOW (closed)` or `HIGH (open)`. LOW with no water moving means the magnet
is parked — harmless. If it is LOW and *never* changes across several irrigation runs, then
suspect the wiring or a failed-closed reed.

## Sessions are fine but reports never arrive

The device measures with or without a network — accumulators are in RAM and flow detection does
not depend on connectivity. If sessions look right in the WebSerial log but Home Assistant sees
nothing, the publishes are being lost.

Everything is published at QoS 0, so a publish made while the broker is unreachable is simply
dropped; there is no store-and-forward. Check `irrig_leak/status/LWT` — if it reads
`Disconnected`, the broker saw an ungraceful drop and the device has not yet reconnected.

At a site with weak coverage this is the expected failure mode, and it is preferable to the
alternative: the device keeps counting correctly and the *next* successful report is accurate,
rather than rebooting and losing the session outright.
