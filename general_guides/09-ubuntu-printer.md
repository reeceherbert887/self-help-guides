# Epson XP-3200 Printer Setup (Ubuntu 24.04)

## Install Epson Driver

```bash
sudo apt update
sudo apt install printer-driver-escpr
```

---

## Restart Printing Service

```bash
sudo systemctl restart cups
```

---

## Reboot System

```bash
sudo reboot
```

---

## Add Printer

Open:

```text
Settings → Printers → Add Printer
```

Select:

```text
XP-3200
Address: 192.168.4.36
```

Do **not** select:

```text
JetDirect-Printer
LPD-Printer
```

---

## Driver Selection

If prompted for a driver, select:

```text
EPSON XP-3200 Series
```

or

```text
ESC/P-R Driver
```

Do not use generic drivers if the Epson driver is available.

---

## Verify Printer Connectivity

```bash
ping 192.168.4.36
```

Expected:

```text
0% packet loss
```

---

## Test Print

Create a test file:

```bash
echo "Hello Epson" > test.txt
```

Print:

```bash
lp test.txt
```

---

## Printer Information

| Item             | Value                |
| ---------------- | -------------------- |
| Printer Model    | Epson XP-3200        |
| Printer IP       | 192.168.4.36         |
| Operating System | Ubuntu 24.04         |
| Driver Package   | printer-driver-escpr |
| Printing Service | CUPS                 |

---

## Future Network Changes

The printer can be powered off and moved without reconfiguration.

Requirements:

1. Reconnect printer to the same WiFi network.
2. Ensure the printer receives an IP address.
3. If configured manually, reserve the printer IP in the router.

Recommended DHCP Reservation:

```text
192.168.4.36
```

This prevents the printer IP from changing after reboots or power cuts.
