---
title: 'Status'
description: 'Overview of the WiCAN Pro device status page'
---

# Device Status

The **Status** page provides real-time telemetry and diagnostic information about your WiCAN Pro device. This page is essential for verifying network connectivity, checking your CAN bus configuration, and monitoring overall system health.

Here is a detailed breakdown of the metrics displayed:

### 🌐 Network Information
* **WiFi Mode:** The current operating mode of the Wi-Fi module (e.g., Station, Access Point).
* **AP Channel:** The specific Wi-Fi frequency channel currently in use.
* **WiFi Station:** Indicates the current connection status to your local network (e.g., Connected, Disconnected).
* **Station IP:** The IPv4 address assigned to the WiCAN Pro device on your local network.
* **DNS Main / Backup:** The primary and secondary Domain Name System servers being used by the device.
* **mDNS:** The local multicast DNS hostname, allowing you to access the web UI without needing the exact IP address (e.g., `wican_fc012ccc4811.local`).
* **VPN Status:** The current connection state of the built-in VPN client (e.g., connecting, connected).

### 🚗 CAN & OBD Interface
* **CAN Bitrate:** The configured operating speed of the CAN bus (e.g., 500K).
* **CAN Mode:** The operational mode of the CAN transceiver (e.g., Normal, Listen Only).
* **Port Type:** The network transport protocol selected for streaming CAN data (TCP or UDP).
* **TCP/UDP Port:** The specific network port configured for data transmission (default is typically 35000).
* **OBD Chip Status:** Indicates whether the onboard OBD processing hardware is initialized and Ready.

### ⚙️ System Diagnostics
* **Time Synced:** Confirms whether the device has successfully synchronized its internal clock via NTP.
* **Uptime:** The total time the device has been running continuously since the last boot (HH:MM:SS format).
* **Last Reset:** The specific trigger that caused the device's previous reboot (e.g., software, hardware watchdog).
* **Planned Restart:** Shows if a system reboot is pending and why (e.g., config apply).
* **Restart Source:** Indicates the interface or process that initiated the restart (e.g., web ui).
* **Last Boot Time:** The exact timestamp of the most recent system startup.
* **Boot Count:** The cumulative number of times the device has been powered on or restarted.

### 🔋 Power Management
* **Battery Voltage:** A live reading of the vehicle's battery voltage (e.g., 12.3V).
* **Time to Sleep:** Displays the remaining countdown timer before the device enters low-power sleep mode. Displays as **N/A** if sleep mode is disabled or currently interrupted.
