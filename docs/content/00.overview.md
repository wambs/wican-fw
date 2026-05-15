---
id: overview
title: Overview
sidebar_position: 1
---

Here is the updated **Overview** page with those new additions seamlessly integrated into Section 1. This accurately reflects the drag-and-drop capabilities and the true periodic-polling purpose of the groups that you built into the UI.

---

# Overview: WiCAN Pro (Forked Edition)

Welcome to the documentation for the **WiCAN Pro (Forked Edition)**.

If you are looking for a way to pull live vehicle telemetry—like State of Charge, RPM, tire pressures, or raw CAN frames—and route it seamlessly into your smart home or custom servers, you are in the right place.

### What is the WiCAN Pro?

At its core, the WiCAN Pro is a powerful, ESP32-S3-based OBD2 and CAN bus interface. It plugs directly into your vehicle's diagnostic port and acts as a bridge between your car's internal networks and your Wi-Fi or Bluetooth devices. It supports standard OBD2 protocols (via an ELM327-compatible interpreter) as well as raw CAN bus monitoring and injection.

### Why the Forked Edition?

The original WiCAN firmware was great for basic polling, but it relied on a "flat list" architecture. Every PID was polled blindly on a timer, which often resulted in network spam, slow update rates, and unnecessary battery drain when the car was off.

**This Forked Edition completely rewrites the automation and networking engines.** It transforms the WiCAN Pro from a simple OBD scanner into a highly intelligent, edge-computing telemetry node.

---

## What's New? (Key Features)

### 1. Vehicle Groups & Advanced PID Management (The Game Changer)

The old flat list of PIDs is gone, replaced by **Vehicle Groups**. Groups were specifically created to allow you to access different vehicle modules based on dedicated polling periods, optimizing CAN bus traffic and maximizing response efficiency.

* **Advanced Editor & Drag-and-Drop:** The UI now features a robust visual editor. You can easily copy/paste (clone) PIDs, edit parameters on the fly, and seamlessly drag and drop PIDs between different Vehicle Groups to fine-tune your architecture.
* **Smart Polling & Conditional Logic:** Tell a group to only poll when the "Engine is Running" or when the "Battery Voltage" is high, completely halting unnecessary traffic when the car is parked.
* **Zero Network Spam:** Groups package all their PID responses into a single JSON payload and push it to a unified `MQTT_Grp` topic the exact millisecond the batch finishes.
* **Profile Management (Get, Save, Merge):** You can now fetch the latest community-maintained Vehicle Profiles directly from GitHub or save your active custom profile locally. When loading new vehicle data, the UI intelligently prompts you to either safely **merge** the new PIDs into your existing workspace or completely **replace** your active configuration.

### 2. Advanced Destination Routing

You are no longer limited to simple MQTT publishes. The new **Configured Destinations** engine acts as a master router for your data.

* **A Better Routeplanner (ABRP):** Native API integration. Map your live State of Charge, power, and speed directly to ABRP for live EV route planning.
* **Home Assistant Webhooks:** Send your vehicle data directly to Home Assistant via HTTP POST requests, entirely bypassing the need for an MQTT broker. Features "Changed Values Only" mode to drastically reduce overhead.
* **Custom HTTP/HTTPS:** Push your vehicle data to your own servers. Includes advanced authentication support (Bearer Tokens, API Keys, Basic Auth, and custom Query Parameters).

### 3. Enterprise-Grade Networking & Security

Your car moves; your network settings should adapt.

* **WireGuard VPN:** A native WireGuard client is now built into the firmware. You can securely connect your WiCAN back to your home network (like an Unraid or HAOS server) to safely transmit MQTT data over cellular hotspots without exposing ports.
* **SmartConnect & Fallbacks:** Configure up to 5 backup Wi-Fi networks.
* **Home Network Prioritization:** The device scans every 5 minutes and will automatically drop a hotspot connection to switch back to your Home SSID when you pull into the driveway (if the signal is > -75 dBm).
* **Certificate Manager:** Upload your own CA certificates, Client Certificates, and Private Keys for strict mutual TLS (mTLS) authentication on HTTPS endpoints.

### 4. Overhauled Web Interface

The UI has been entirely redesigned for speed and usability.

* **Live Dashboard & Terminal:** Monitor your CAN bus, read live JSON outputs, and send raw ELM327/System commands directly from the built-in web terminal.
* **Real-time Diagnostics:** See live success/fail rates for your Webhooks and Configured Destinations directly in the Automate tab.

---
