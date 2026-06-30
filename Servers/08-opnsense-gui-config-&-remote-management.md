# 08-opnsense-gui-config-&-remote-management

**Author:** Reece Herbert
**Date:** 07/06/2026
**Status:** Ready

---
Following successful installation of OPNsense, the objective was to:

* Complete the setup wizard
* Configure basic settings
* Enable remote management from the existing Router network
* Remove the requirement for a direct laptop connection

---

# Initial Access

The GUI was accessed from a laptop connected directly to:

```text
LAN (igb2)
```

using:

```text
Your IP
```

---

# Setup Wizard Configuration

## WAN Configuration

Type:

```text
DHCP
```

Modified:

```text
Block RFC1918 Private Networks = Disabled
Block Bogon Networks = Enabled
```

Reason:

The WAN interface receives:

```text
Your IP```

from the Router network.

This is an RFC1918 private address and therefore should not be blocked.

---

## LAN Configuration

Configured as:

```text
Your IP```

DHCP Server:

```text
Enabled
```

---

## Deployment Type

Configured as:

```text
Optimize for Multiwan = Disabled
Automatic DHCP/DNS Registration = Enabled
Optimize for IPsec = Disabled
```

Reason:

Only a single WAN connection exists.

---

# GUI Administration Settings

Navigation:

```text
System
→ Settings
→ Administration
```

Verified:

```text
Listen Interfaces = All
```

This allows the GUI to listen on:

```text
Your IP
Your IP
```

---

# DHCP Reservation

A DHCP reservation was created within the Router application.

Reserved address:

```text
Your IP```

Purpose:

Ensures OPNsense always receives the same WAN address after reboot.

Benefits:

* Predictable management address
* Easier documentation
* Easier remote access

---

# WAN Firewall Rule

Navigation:

```text
Firewall
→ Rules
→ WAN
```

Added rule:

```text
Action:
Pass

Interface:
WAN

Protocol:
TCP

Source:
Your IP
Destination:
WAN Address

Port:
HTTPS (443)
```

Description:

```text
Allow GUI from Router Network
```

---

# Why This Rule Was Required

By default:

```text
WAN Access = Blocked
```

This prevented desktop systems from accessing:

```text
Your IP
```

After adding the rule:

```text
Desktop
Phone
Laptop
```

could all access the GUI from the Router network.

---

# Firewall Verification

Firewall logs confirmed traffic was reaching OPNsense.

Example:

```text
Your IP
    │
    ▼
Your IP
```

The firewall was actively processing requests.

This confirmed the rule was functioning correctly.

---

# Final Remote Management Setup

```text
Internet
    │
Router
    │
Your IP 
    │
┌─────────────────┐
│ OPNsense        │
│ Your IP         │
└─────────────────┘
```

Management Access:

```text
Your IP
```

Available from:

* Desktop
* Laptop
* Phone

No direct LAN cable required.

---

# Outcome

Successfully completed setup wizard.

Successfully configured WAN and LAN.

Successfully created DHCP reservation.

Successfully created WAN management rule.

Successfully enabled remote GUI access.

Firewall now manageable from anywhere on the Router network while remaining isolated from the public internet.
