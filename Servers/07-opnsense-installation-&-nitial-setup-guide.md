

# Objective

The objective of this project was to successfully install OPNsense onto a dedicated firewall machine and gain access to the Web GUI for the first time.

The firewall was not intended to replace the existing router at this stage. Instead, it was deployed behind the existing Eero network to allow testing, learning and validation before future deployment into the homelab.

---

# Hardware Used

## Firewall Hardware

* Dedicated firewall PC
* Intel i340-T4 Quad Port NIC
* SSD for OPNsense installation
* Monitor
* Keyboard

## Existing Network

```text
Internet
    │
KCOM / Eero
    │
Existing Home Network
```

---

# Key Discovery

One of the biggest lessons learned during installation was identifying the correct network interfaces.

The motherboard Ethernet port appeared as:

```text
re0
```

The Intel Quad Port NIC appeared as:

```text
igb0
igb1
igb2
igb3
```

The Intel NIC was confirmed working because OPNsense successfully obtained a DHCP address from the Eero network using:

```text
igb3
```

This proved:

* Intel NIC working
* Network cable working
* Eero DHCP working
* OPNsense networking working

As a result, all motherboard networking was ignored for the remainder of the installation.

---

# Physical Cabling

## WAN Connection

The WAN interface was connected directly to the Eero router.

```text
Eero Router
    │
    ▼
Intel NIC Port
```

This was eventually detected as:

```text
igb3
```

---

## LAN Connection

The laptop was connected directly to a second Intel NIC port.

```text
Laptop
    │
    ▼
Intel NIC Port
```

This was eventually detected as:

```text
igb2
```

---

# OPNsense Installation

The OPNsense installer was booted from USB.

The installation process was straightforward:

* Select Install
* Choose keyboard layout
* Select SSD
* Use entire disk
* Confirm disk wipe
* Set root password
* Complete installation
* Reboot
* Remove installation USB

---

# Interface Assignment

## VLAN Configuration

When prompted:

```text
Configure VLANs?
```

Selected:

```text
No
```

No VLANs were required.

---

## LAGG Configuration

When prompted:

```text
Configure LAGGs?
```

Selected:

```text
No
```

No link aggregation was required.

---

# WAN Assignment

Auto-detection was used.

The WAN cable connected to the Eero router was unplugged and reconnected.

OPNsense detected:

```text
igb3
```

Assignment:

```text
WAN = igb3
```

---

# LAN Assignment

Auto-detection was used.

The laptop cable was unplugged and reconnected.

OPNsense detected:

```text
igb2
```

Assignment:

```text
LAN = igb2
```

---

# Initial Network Configuration

After interface assignment:

```text
WAN (igb3) = 192.168.4.56/22
LAN (igb2) = 192.168.1.1/24
```

This confirmed:

* WAN DHCP operational
* LAN configured correctly
* OPNsense running correctly

---

# Accessing The Web GUI

The laptop was connected directly to the LAN interface.

Ubuntu successfully obtained:

```text
192.168.1.193
```

from the OPNsense DHCP server.

Initially the GUI did not respond correctly.

Symptoms included:

* Ping failures
* HTTPS failures
* Network appears operational but GUI inaccessible

A reboot of OPNsense resolved the issue completely.

Following reboot:

```text
ping 192.168.1.1
```

returned successful replies.

The GUI became accessible at:

```text
https://192.168.1.1
```

---

# Final Working Configuration

```text
WAN = igb3
LAN = igb2

WAN Address:
192.168.4.56

LAN Address:
192.168.1.1
```

---

# Lessons Learned

## Use The Intel NIC Only

Avoid using:

```text
re0
```

during initial deployment.

Using only:

```text
igb0
igb1
igb2
igb3
```

greatly simplified troubleshooting.

---

## Auto Detection Works

Using interface auto-detection was significantly easier than attempting to manually map interfaces.

---

## Reboot After Assignment

Although networking appeared configured correctly, a reboot was required before communication became fully operational.

This solved:

* Failed ping
* Failed HTTPS
* GUI access issues

---

# Project Outcome

Successfully installed OPNsense.

Successfully assigned interfaces.

Successfully obtained WAN connectivity.

Successfully obtained LAN connectivity.

Successfully accessed the OPNsense Web GUI.

Firewall ready for GUI configuration and future homelab integration.
