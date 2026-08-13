# Hardware

## Bill of materials

| Part | Number | Notes |
|---|---|---|
| MCU board | Seeed Studio XIAO ESP32S3 | Needs the external U.FL antenna fitted |
| Flow meter | Hunter HC-100-FLOW | 1" inline, reed switch, 1 pulse per gallon |
| Pressure sensor | TE Connectivity `M32JM-000105-100PG` | 100 PSI **gauge**, I²C, optional |
| Valve sensing | PS2505 optocoupler ×4 | Anti-parallel LEDs — conducts on both AC half-cycles |
| Flow debounce | LS18-P | Conditions the reed switch in hardware; firmware does no software debounce |
| Power | 24VAC from the irrigation controller | Rectified and regulated on the carrier PCB |

The `PG` suffix on the pressure sensor part number means **gauge**, not absolute. It matters:
a gauge sensor reads 0 at atmosphere, so a reading near zero on the bench is correct and does
not indicate a fault. See [troubleshooting](troubleshooting.md) before concluding otherwise.

The sensor is rated to 100 PSI full scale. Compressor tanks sit at 120–175 PSI, so keep a
regulator between tank and sensor when bench testing.

## Pinout

Defined at the top of [src/main.cpp](../src/main.cpp).

| Signal | GPIO | XIAO pad | Direction | Active |
|---|---|---|---|---|
| `VALVE_1_PIN` | 1 | D0 | input | HIGH |
| `VALVE_2_PIN` | 2 | D1 | input | HIGH |
| `VALVE_3_PIN` | 3 | D2 | input | HIGH |
| `VALVE_4_PIN` | 4 | D3 | input | HIGH |
| `FLOW_SENSOR_BLUE_PIN` | 43 | D6 | input | LOW |
| `FLOW_SENSOR_RED_PIN` | 44 | D7 | input | — (wired, not read) |
| `I2C_SDA_PIN` | 5 | D4 | bidirectional | — |
| `I2C_SCL_PIN` | 6 | D5 | bidirectional | — |
| `STATUS_LED_PIN` (blue) | 7 | D8 | output | HIGH |
| `BUILT_IN_LED_PIN` (yellow) | 21 | onboard | output | **LOW** |

The two LEDs have opposite polarity. The onboard yellow LED is active LOW — writing `LOW`
lights it. Getting this backwards is an easy way to produce a permanently-lit "everything is
fine" indicator.

I²C runs at 100 kHz; the M3200 sits at address `0x28`.

## Valve sensing, and the 24VAC problem

Irrigation valves are driven with **24V AC**, not DC. Each valve line feeds a PS2505
optocoupler whose anti-parallel LEDs conduct on both half-cycles, so the phototransistor is on
for most of the waveform.

**But not all of it.** LED current passes through zero twice per mains cycle, and at each
crossing the optocoupler output notches LOW for a short interval — 120 times a second on 60 Hz
mains. The output is emphatically *not* a steady HIGH while a valve is energised.

This is not a defect to design around at the circuit level; it is inherent to sensing AC
through an optocoupler. The firmware handles it by sampling across a window wider than one
full mains cycle. See [theory of operation](theory-of-operation.md#valve-attribution).

An earlier revision used the ILQ620, whose single LED conducts on only one half-cycle and
produces a roughly 50% duty cycle at 60 Hz. Moving to the PS2505 improved matters but did not
eliminate the zero-crossing notches, because nothing can.

## Flow meter

The HC-100 closes a reed switch once per gallon and is rated to **34 GPM**, so pulses arrive no
faster than 1765 ms apart. The input is **active LOW with external pull-ups on the carrier
PCB**, and is debounced in hardware by an **LS18-P** — the firmware does no software debounce
and should not acquire one.

The external pull-ups are a deliberate fail-safe: a severed or disconnected sensor lead floats
HIGH, which reads as *idle*. Without them the input would float and could easily read as a
continuous stream of pulses — a permanent phantom leak. Because the pull-ups exist in hardware,
the firmware configures these pins as plain `INPUT` rather than `INPUT_PULLUP`; the internal
pull-up would be redundant.

The meter has two leads. Only the blue one is read; the red one is wired and pulled up but the
firmware ignores it.

**The magnet can park on the reed switch.** When flow stops the impeller coasts down, and the
magnet may come to rest holding the switch closed — the input then sits LOW with no water
moving. Where it stops is chance, so it happens on some stops and not others. This is a normal
resting state of a healthy sensor, and the firmware counts falling edges specifically so that it
is a non-event. Do not treat a LOW reading at rest as a fault.

## Plumbing topology

**One meter, upstream of the whole manifold.**

```
supply ──▶ flow meter ──▶ manifold ──┬──▶ zone 1 valve ──▶ laterals
                                     ├──▶ zone 2 valve ──▶ laterals
                                     ├──▶ zone 3 valve ──▶ laterals
                                     └──▶ zone 4 valve ──▶ laterals
```

Three consequences run through the entire design:

1. **Zone attribution is inferred, not measured.** The meter cannot tell you which zone the
   water went to. The firmware asks "which valve is energised right now?" and credits the
   gallon there. This is why valve sensing accuracy matters so much — a misread valve does not
   merely mislabel a gallon, it files it under zone 0, the leak bucket.

2. **A closing valve stops flow through the meter immediately.** The lateral downstream of a
   closed valve drains locally; that water never crosses the meter. So there is no legitimate
   "bleed-down" volume to excuse after a zone shuts off — only a gallon or two of transition
   noise while valves overlap.

3. **A stuck-open valve passes full zone flow, continuously.** It is not a subtle signal. This
   is why volume alone is enough to separate a real leak from transition noise, with no need to
   reason about flow rate.

## Known hardware traps

- **Valve zone numbering must match the board.** The valve GPIOs are the mapping from a
  physical zone to the zone number reported over MQTT — nothing else records it. If they are
  assigned in the wrong order the firmware silently reports every zone under another zone's
  name, and the totals still reconcile perfectly, so the error is invisible in the numbers. The
  cheap check is a manual run of each zone in turn against the `[pins: 1=? 2=? 3=? 4=?]` trace
  in the log, which shows raw input levels and does not depend on the mapping being right.
- **GPIO 3 is an ESP32-S3 strapping pin** and carries `VALVE_3_PIN`. It reads correctly in
  service; worth knowing if that channel ever misbehaves at boot.
- **Dead GPIOs on the previous board.** This project ran on a LilyGo T7-S3 until August 2026.
  That specific board developed dead pins on GPIO 15 and GPIO 18 (zones 1 and 4) — wiring and
  optocouplers were both ruled out. If zones drop out on a board swap, suspect the board before
  the field wiring.
- **The XIAO enumerates under two USB identities**, which can renumber the COM port between
  application and bootloader. See [building and flashing](building-and-flashing.md).
- **The M3200 can lock the I²C bus.** If a transaction is interrupted mid-read — an OTA update
  or reset at the wrong moment — the sensor may hold SCL low indefinitely. No software recovery
  clears it; it needs a power cycle. A GPIO-controlled load switch on the sensor's supply would
  let the firmware recover on its own, and is the obvious next hardware revision.
