

## Objective

Following successful installation of OPNsense, the objective was to:

* Complete the setup wizard
* Configure basic settings
* Enable remote management from the existing Eero network
* Remove the requirement for a direct laptop connection

---

# Initial Access

The GUI was accessed from a laptop connected directly to:

```text
LAN (igb2)
```

using:

```text
https://192.168.1.1
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
192.168.4.56
```

from the Eero network.

This is an RFC1918 private address and therefore should not be blocked.

---

## LAN Configuration

Configured as:

```text
192.168.1.1/24
```

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
192.168.1.1
192.168.4.56
```

---

# DHCP Reservation

A DHCP reservation was created within the Eero application.

Reserved address:

```text
192.168.4.56
```

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
192.168.4.0/22

Destination:
WAN Address

Port:
HTTPS (443)
```

Description:

```text
Allow GUI from Eero Network
```

---

# Why This Rule Was Required

By default:

```text
WAN Access = Blocked
```

This prevented desktop systems from accessing:

```text
https://192.168.4.56
```

After adding the rule:

```text
Desktop
Phone
Laptop
```

could all access the GUI from the Eero network.

---

# Firewall Verification

Firewall logs confirmed traffic was reaching OPNsense.

Example:

```text
192.168.4.57
    │
    ▼
192.168.4.56
```

The firewall was actively processing requests.

This confirmed the rule was functioning correctly.

---

# Final Remote Management Setup

```text
Internet
    │
Eero
    │
192.168.4.0/22
    │
┌─────────────────┐
│ OPNsense        │
│ WAN 192.168.4.56│
└─────────────────┘
```

Management Access:

```text
https://192.168.4.56
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

Firewall now manageable from anywhere on the Eero network while remaining isolated from the public internet.
