---
title: 'MQTT Integration'
description: 'WiCAN Pro to Home Assistant: The Ultimate MQTT Best Practices Guide'
---

# WiCAN Pro to Home Assistant: The Ultimate MQTT Best Practices Guide

Integrating the WiCAN Pro with Home Assistant over MQTT unlocks incredible potential for vehicle monitoring. However, because CAN bus data is highly complex and transmits at lightning speed, a poorly planned setup will quickly result in messy configuration files and broken data.

Follow these best practices to build a bulletproof, highly optimized integration from the ground up.

::alert{type="info"}
**Important Note:** The Expression Hex type is only available on the WiCAN Pro forked releases: [WiCAN Pro Releases](https://github.com/wambs/wican-fw/releases?page=1)
::

## 📡 Why MQTT? The Advantages for Vehicle Telemetry
Before diving into the configuration, it is helpful to understand why MQTT (Message Queuing Telemetry Transport) is the gold standard for bridging your vehicle's CAN bus to your smart home:

* **Extremely Lightweight:** MQTT has incredibly small packet headers. This means it can transmit massive amounts of high-frequency data (like Engine RPM) without bogging down your Wi-Fi network or Home Assistant server.
* **Lightning Fast:** It operates on a publish/subscribe model, allowing for near real-time data streaming from your vehicle straight to your dashboard with virtually zero latency.
* **Low Bandwidth:** By only sending tiny text or hex payloads, it uses a fraction of the bandwidth compared to standard HTTP API requests.
* **Reliability:** It is specifically designed for environments where network connections might drop or fluctuate (like a vehicle pulling into or out of a garage).
* **Rich Entity Customization:** When defining MQTT sensors manually in Home Assistant, you have total control over the metadata. You can explicitly set the `unit_of_measurement` (like PSI or °C), define the `state_class` (like `measurement` for long-term statistics graphing), assign a `unique_id` to allow UI editing, and explicitly define the exact backend name using `default_entity_id`.

---

## 🛠️ The Ultimate Debugging Tool: MQTT Explorer
Before you write a single line of YAML code in Home Assistant, download a free desktop program called **MQTT Explorer**.

When you connect MQTT Explorer to your Home Assistant broker, you get a live, visual tree of every single message the WiCAN broadcasts. It is invaluable for troubleshooting because it allows you to:
* View all WiCAN activity in real-time with precise timestamps.
* Monitor the `online` / `offline` availability topics to confirm connection stability.
* See built-in WiCAN hardware metrics, such as the internal device battery level and signal strength.
* Copy the exact JSON or Hex payloads to reference while building your Home Assistant templates.

::alert{type="success"}
**Tip: Mocking Data:** You can manually inject/publish custom MQTT payloads directly to your broker using MQTT Explorer. This allows you to instantly test and debug your Home Assistant entity decoding logic *without* needing the vehicle running or the WiCAN device connected!
::

---

## Phase 1: Initial MQTT Broker Connection (Settings Page)
Before you can start routing vehicle data, you must connect the WiCAN Pro to your Home Assistant MQTT broker (typically the Mosquitto Broker add-on).

Navigate to the **Settings** tab in the WiCAN Pro web interface and locate the **MQTT** section. Configure the fields as follows:
* **Enable MQTT:** Toggle this to `ON`.
* **Broker Address:** Enter the local IP address of your Home Assistant instance (e.g., `192.168.1.100`). Do not add `http://` or port numbers here.
* **Port:** Enter `1883` (the standard unencrypted MQTT port).
* **Client ID:** Give the WiCAN a unique name so your broker can identify it (e.g., `wican_f350`).
* **Username & Password:** Enter the credentials for the specific Home Assistant user you created for MQTT access.
* **Keep Alive:** `60` seconds is the standard default and works perfectly.

Once entered, save your settings. The WiCAN Pro will now maintain a persistent connection to Home Assistant whenever it is powered on and connected to your network.

---

## Phase 2: MQTT Topic Naming & Payload Format
How you name your MQTT topics dictates how easily you can organize them in Home Assistant. Never dump raw PIDs into the root of your MQTT broker.

### 1. Use a Strict, Hierarchical MQTT Topic Prefix
Use a logical, nested structure that identifies the system, the device, the vehicle, the module, and the specific data point.
* **Format:** `<system>/<device>/<vehicle>/<module>_<data_name>`
* **Good Example:** `homeassistant/wican/f350/pcm_engine_rpm`
* **Good Example:** `homeassistant/wican/f350/bcm_remote_start_failures`

### 2. Choose the Right Payload Format
* **For simple, single-byte values (like RPM or Engine Temp):** Let WiCAN handle the math and send the parsed Decimal/Integer over MQTT.
* **For complex, multi-byte values (like TPMS packets or Bitmaps):** Configure WiCAN to send the RAW HEX string. Sending raw hex bypasses JSON formatting quirks and integer overflow limits, allowing Home Assistant's template engine to perfectly slice and decode the data.

---

## Phase 3: Configuring the WiCAN Automate Rules
To actually tell the WiCAN Pro to start pulling data and pushing it to Home Assistant, you need to set up rules in the WiCAN web interface under **Automate -> Vehicle**. This is where you define exactly how the WiCAN interacts with the vehicle's CAN bus.

There are two primary ways to set these up: **Active Polling** and **Passive Listening**.

### 1. Setting up an Active Request (Polling)
Use this for data the truck doesn't broadcast on its own, requiring the WiCAN to specifically ask for it (e.g., Engine RPM, Transmission Temp, Gear Position).

When creating a new Vehicle Automation, configure the fields as follows:
* **Name:** A clean, recognizable name (e.g., `F350 Gear Position`).
* **Trigger / Interval:** Set how often you want WiCAN to ask for this data (e.g., `1000` ms for fast data like RPM, or `5000` ms for slow data like Coolant Temp).
* **TX ID (Header):** The physical address of the module you are querying (e.g., `7E2` for the TCM).
* **TX Data (Request):** The specific PID request hex (e.g., `22 1E 12`).
* **MQTT Topic:** Paste your structured topic here (e.g., `homeassistant/wican/f350/tcm_gear_raw`).
* **Expression / Output:** Choose how WiCAN formats the reply. Choose standard math expressions if WiCAN is doing the math for you, or select **Raw (Hex)** to pass the raw string directly to HA.

::alert{type="warning"}
**Important: Don't Over-Poll!** Do not poll modules faster than necessary, or you risk flooding the vehicle's CAN bus and causing network collisions.
::

### 2. Setting up a Passive Listener (Sniffing)
Use this for high-frequency data the truck broadcasts automatically, or for event-based data (e.g., TPMS tire pressure bursts, Door Locks, or Remote Start Failures). You do not want to actively poll for these.

* **Trigger / Interval:** Leave the polling interval blank or disabled.
* **RX ID (Listen Address):** Enter the specific CAN ID that broadcasts the data (e.g., `18FEF433` for TPMS events).
* **TX ID / TX Data:** Leave these completely blank! You are only listening, not asking.
* **MQTT Topic:** Enter your topic (e.g., `homeassistant/wican/f350/bcm_slow_evnt1_all`).
* **Expression / Output:** Set this to **Raw (Hex)**. Because broadcast packets are often massive (like a 19-byte TPMS message), forcing it to raw hex prevents WiCAN from trying to format it as JSON and accidentally truncating the data.

::alert{type="success"}
**Tip: Data Starts At:** If you are using Raw (Hex) output, pay attention to where the actual data begins. If WiCAN includes the header in the output (e.g., `07 62 41 B8 08 08 08 08`), you can either tell WiCAN to strip it by adjusting the starting byte, or just pass the whole string to Home Assistant and let Jinja2 slice it up using `regex_findall`. Passing the whole string is usually the safest method!
::

---

## Phase 4: Home Assistant YAML Cleanliness is Critical

### 1. Group Entities Using a Device Anchor
Instead of having 50 loose sensors floating in Home Assistant, assign them all to a single "Vehicle" device page. Define a YAML anchor (`&device_name`) once, and inject it into every sensor.

```yaml
.wican_vehicle_def: &f350_device
  identifiers:
    - "wican_pro_f350"
  name: "Ford F-350"
  model: "Super Duty"
  manufacturer: "Ford"

mqtt:
  sensor:
    - name: "Engine RPM"
      state_topic: "homeassistant/wican/f350/pcm_engine_rpm"
      device: *f350_device # <--- Automatically groups this sensor to the truck.
```
### 2. Decoding Bitmaps (Multiple States in One Message)
Vehicle modules often use Bitmaps to send multiple true/false statuses in a single byte (e.g., Remote Start Failure reasons, where Bit 0 is "Hood Open" and Bit 1 is "Not in Park"). Use Jinja2's bitwise_and filter to extract multiple active states and combine them into a clean, comma-separated list.
```yaml
- name: "Remote Start Failure Reasons"
  state_topic: "homeassistant/wican/f350/bcm_remote_start_failures"
  value_template: >
    {% set raw_val = value | int %}
    {% set result = namespace(reasons=[]) %}
    {% set bitmask = {
        128: 'Alarm Triggered',
        32: 'Hood Open',
        16: 'Ignition Not Off',
        1: 'Vehicle Not In Park'
    } %}
    {% for bit, reason in bitmask.items() %}
      {% if raw_val | bitwise_and(bit) %}
        {% set result.reasons = result.reasons + [reason] %}
      {% endif %}
    {% endfor %}
    {{ result.reasons | join(', ') if result.reasons else 'None' }}
```
### 4. Bulletproof Raw Hex Parsing
If WiCAN sends a long string of Hex data (like 07 62 41 B8 08), spaces and formatting can easily break standard text slicing. Use regex_findall to cleanly extract the hex pairs into a perfect array.
```
value_template: >
        {# Strips all formatting and creates an array of 2-character hex pairs #}
        {% set h = value | regex_findall('[A-Fa-f0-9]{2}') %}
        
        {# Grabs the 5th byte, converts from hex to integer, and does the math #}
        {{ ((h[4] | int(base=16)) * 0.05) | round(1) }}
```
### 5. Use YAML Anchors for Repeating Sensors (Like TPMS)
If you have 4 tires sending identical 19-byte hex strings, do not write the decoding math 4 separate times. Write it once as an anchor (&tpms_pressure), and reuse it to keep your code DRY (Don't Repeat Yourself).
#### Define the template once
```
    - <<: &tpms_pressure
        unit_of_measurement: "PSI"
        value_template: >
          {% set h = value | regex_findall('[A-Fa-f0-9]{38,}') | first %}
          {{ ((h[16:20]|int(base=16)) * 0.05) | round(1) }}
      name: "Tire 1 Pressure"
      state_topic: "homeassistant/wican/f350/evt1_raw"

    # Reuse it infinitely 
    - <<: *tpms_pressure
      name: "Tire 2 Pressure"
      state_topic: "homeassistant/wican/f350/evt2_raw"
