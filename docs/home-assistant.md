# Home Assistant integration

The firmware measures and reports. **Home Assistant decides.**

For zones 1–4 the firmware has no opinion about what constitutes a leak — it publishes a median
GPM and HA compares that against a per-zone limit. Only the valves-off case is classified on the
device, and even there `MIN_LEAK_GALS` only decides whether a non-zero value is published; the
alerting logic is here.

Notification targets below are placeholders — substitute your own.

## Where things go

| File | Contents | Wiring in `configuration.yaml` |
|---|---|---|
| `mqtt.yaml` | Sensor definitions | `mqtt: !include mqtt.yaml` |
| `templates.yaml` | Per-zone gallon sensors | `template: !include templates.yaml` |
| `hand_coded_automations/automations.yaml` | Alert automations | `automation non-gui: !include_dir_merge_list hand_coded_automations` |

> Entity definitions load at **startup**. Adding a `json_attributes_topic` requires a full
> restart — a YAML reload will not pick it up, and the attributes will silently stay missing.

## `mqtt.yaml` — sensors

```yaml
  # ---- status and idle heartbeat ----
  - unique_id: irrig_last_idle_heartbeat
    name: "Irrig Last Idle Heartbeat"
    state_topic: "irrig_leak/idle/time_stamp"
    qos: 0

  - unique_id: irrig_wifi_ssid
    name: "Irrig WiFi SSID"
    state_topic: "irrig_leak/wifi_ssid"
    qos: 0

  - unique_id: irrig_wifi_dbm
    name: "Irrig WiFi Signal Strength"
    state_topic: "irrig_leak/wifi_dbm"
    unit_of_measurement: "dBm"
    qos: 0

  - unique_id: irrig_idle_water_pressure
    name: "Irrig Idle Water Pressure"
    state_topic: "irrig_leak/idle/water_pressure"
    state_class: measurement
    device_class: pressure
    unit_of_measurement: "psi"
    qos: 0

  - unique_id: irrig_idle_water_temperature
    name: "Irrig Idle Water Temperature"
    state_topic: "irrig_leak/idle/water_temperature"
    state_class: measurement
    device_class: temperature
    unit_of_measurement: "°F"
    qos: 0

  # ---- session report ----
  - unique_id: irrig_last_report_time
    name: "Irrig Last Report Time"
    state_topic: "irrig_leak/report/time_stamp"
    qos: 0

  - unique_id: irrig_report_tot_gallons_all_zones
    name: "Irrig Report Total Gallons All Zones"
    state_topic: "irrig_leak/report/tot_gals_all_zones"
    state_class: total
    device_class: water
    unit_of_measurement: "gal"
    qos: 0

  # Valves-off flow. State is GALLONS; the rate lives in the attributes. Zone 0 is a fault
  # condition, not a zone, so it is reported only here - there is deliberately no
  # median_gpm_zone_0 or avg_psi_zone_0 sensor.
  - unique_id: irrig_report_valve_leak_gallons
    name: "Irrig Report Valve Leak Gallons"
    state_topic: "irrig_leak/report/valve_leak"
    state_class: measurement
    device_class: water
    unit_of_measurement: "gal"
    qos: 0
    json_attributes_topic: "irrig_leak/report/valve_leak/attributes"

  # ---- per zone: repeat each block for zones 1-4, changing only the number ----
  - unique_id: irrig_report_median_gpm_zone_1
    name: "Irrig Report Median GPM Zone 1"
    state_topic: "irrig_leak/report/median_gpm_zone_1"
    state_class: measurement
    qos: 0
    json_attributes_topic: "irrig_leak/report/median_gpm_zone_1/attributes"

  - unique_id: irrig_report_avg_psi_zone_1
    name: "Irrig Report Avg PSI Zone 1"
    state_topic: "irrig_leak/report/avg_psi_zone_1"
    state_class: measurement
    device_class: pressure
    unit_of_measurement: "psi"
    qos: 0
    json_attributes_topic: "irrig_leak/report/avg_psi_zone_1/attributes"

  - unique_id: irrig_leak_report_run_dur_zone_1
    name: "Irrig Report Run Dur Zone 1"
    state_topic: "irrig_leak/report/run_dur_zone_1"
    state_class: measurement
    device_class: duration
    unit_of_measurement: "min"
    qos: 0
```

Twenty sensors in total: five status/idle, three report-level, and three per zone × four zones.

## `templates.yaml` — per-zone gallons

Gallons ride as an attribute of the GPM sensor rather than having their own topic, so a
template sensor surfaces them as first-class entities for dashboards and statistics.

```yaml
- sensor:
    - name: "irrig_zone_0_gals"      # leakage while all valves off = zone zero
      unique_id: irrig_zone_0_gals
      device_class: "volume"
      unit_of_measurement: "gal"
      icon: mdi:waves-arrow-right
      # Read the 'gallons' ATTRIBUTE, not the sensor state: the state is gated to 0 below
      # MIN_LEAK_GALS so sub-threshold noise never alarms, while the attribute always
      # carries true zone-0 volume - which is what this sensor is for.
      state: >-
        {{ state_attr('sensor.irrig_report_valve_leak_gallons', 'gallons') | int(0) }}

- sensor:
    - name: "irrig_zone_1_gals"
      unique_id: irrig_zone_1_gals
      device_class: "volume"
      unit_of_measurement: "gal"
      icon: mdi:waves-arrow-right
      state: >-
        {{ state_attr('sensor.irrig_report_median_gpm_zone_1', 'measuredZoneGallons') | int(0) }}

# ... zones 2, 3, 4 identical but for the number
```

Two things worth copying exactly:

- **Zone 0 reads the attribute, not the state.** The state is gated; the attribute is not. Using
  the state here would hide exactly the sub-threshold data this sensor exists to expose.
- **`| int(0)` with an explicit default.** A bare `| int` errors when the attribute is briefly
  absent — which happens on every restart before the first report arrives.

## Automations

Five: one valve-leak notifier and four per-zone rate alerts.

### Valves-off leak

Triggers on any non-zero value. The firmware has already applied `MIN_LEAK_GALS`, so anything
that arrives here is worth knowing about.

```yaml
- id: aut_irrig_valve_leak_notify
  alias: "aut irrig valve leak notify"
  initial_state: true
  trigger:
    - platform: numeric_state
      entity_id:
        - sensor.irrig_report_valve_leak_gallons
      above: 0
  condition: # this prevents immediate triggering after home assistant startup
    - condition: template
      value_template: >-
        {{ ( ( as_timestamp(now(), '') - (as_timestamp(states('sensor.uptime'), '')
            | float(default=0)) ) | int(default=0) ) > 10 }}
  actions:
    - action: notify.YOUR_MOBILE_APP
      data:
        title: "Leaky irrigation valve detected: {{ states('sensor.irrig_report_valve_leak_gallons') }} gallons"
        message: |
          Leaky irrigation valve detected: {{ states('sensor.irrig_report_valve_leak_gallons') }} gallons
          at {{ state_attr('sensor.irrig_report_valve_leak_gallons', 'medianGPM') }} GPM
          (peak {{ state_attr('sensor.irrig_report_valve_leak_gallons', 'maxGPM') }} GPM)
          were measured between watering cycles or when the valves should have been off.
    - action: notify.persistent_notification
      data:
        title: "Leaky irrigation valve detected: {{ states('sensor.irrig_report_valve_leak_gallons') }} gallons"
        message: "See mobile notification for detail."
```

Quoting both volume *and* rate in one message is deliberate — volume says how much water was
lost, rate says how urgent it is. Five gallons at 40 GPM is a burst; five gallons at 2 GPM is a
seep. Same state value, very different responses.

### Per-zone rate alert

One per zone, comparing the reported median against a user-adjustable limit.

```yaml
- id: irrig_leak_zone_1_front
  alias: "irrig leak zone 1 front"
  initial_state: true
  trigger:
    - platform: template
      value_template: >-
        {{ (states('sensor.irrig_report_median_gpm_zone_1') | float(default=0))
           > states('input_number.irrig_z1_front_yd_gpm_limit') | float(default=7) }}
  condition: # this prevents immediate triggering after home assistant startup
    - condition: template
      value_template: >-
        {{ ( ( as_timestamp(now(), '') - (as_timestamp(states('sensor.uptime'), '')
            | float(default=0)) ) | int(default=0) ) > 10 }}
  actions:
    - action: notify.YOUR_MOBILE_APP
      data:
        title: "Irrig leak detected in zone 1 (front): {{ states('sensor.irrig_report_median_gpm_zone_1') }} GPM"
        message: "Irrig leak detected in zone 1 (front): {{ states('sensor.irrig_report_median_gpm_zone_1') }} GPM"
```

The threshold lives in an `input_number` helper so it can be adjusted from a dashboard without
editing YAML. Create one per zone (`input_number.irrig_z1..z4_*_gpm_limit`) and set each a
little above that zone's observed normal median.

### The startup guard is not optional

Every automation carries the same uptime condition:

```yaml
{{ ( ( as_timestamp(now(), '') - (as_timestamp(states('sensor.uptime'), '')
    | float(default=0)) ) | int(default=0) ) > 10 }}
```

All MQTT values are **retained**. Without this guard, every Home Assistant restart re-delivers
the last leak value, re-fires the trigger, and sends a notification about a leak that was dealt
with weeks ago. It requires the `uptime` integration.

## Traps

- **UI-created helpers live in `.storage`, not YAML.** The `input_number` limits will not appear
  in `input_numbers.yaml` if you made them through the UI. Look in `.storage/input_number`.
- **Changing a `unique_id` creates a new entity** and orphans the old one along with all its
  history. To preserve history, rename the entity ID in the UI *before* reloading.
- **Dashboards are storage-mode.** Cards referencing renamed entities must be repointed through
  the UI; editing `.storage/lovelace.*` while HA is running will be overwritten.
- **`json_attributes_topic` needs a full restart**, not a reload. If attributes are stubbornly
  missing, this is almost always why — and they will not appear until the *next* report is
  published, since a subscription made after a retained message was delivered gets it again
  only on resubscribe.
- **Recorder and InfluxDB include `sensor.irrig_*` by glob**, so entities survive renames as
  long as the `sensor.irrig_` prefix holds.
