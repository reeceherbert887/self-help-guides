# Dual Ubuntu Boot Setup Guide

*(Ubuntu 24.04 Main + Ubuntu 22.04 Secondary)*

This guide documents the steps used to configure a clean **dual-Ubuntu boot system** on both a **laptop** and **desktop**.

The goal was to:

* Make **Ubuntu 24.04 the default OS**
* Keep **Ubuntu 22.04 as a secondary boot option**
* Ensure **GRUB detects both systems**
* Avoid bootloader conflicts
* Keep a clean boot menu

Final boot menu:

```
Ubuntu 24.04 (Main)
Advanced options for Ubuntu 24.04 (Main)
Ubuntu 22.04 (Secondary)
UEFI Firmware Settings
```

---

# 1. Verify Which Ubuntu Is Running

Always run GRUB updates from the **main system (Ubuntu 24.04)**.

```bash
lsb_release -a
```

Expected output:

```
Description: Ubuntu 24.04 LTS
```

---

# 2. Reinstall GRUB From the Main System

This ensures **Ubuntu 24.04 controls the bootloader**.

```bash
sudo grub-install
sudo update-grub
```

During the update, GRUB should detect the second Ubuntu installation:

```
Found Ubuntu 22.04.5 LTS (22.04) on /dev/sdX
```

This confirms the secondary OS is added to the boot menu.

---

# 3. Confirm Boot Mode

Check if the system uses **UEFI or Legacy BIOS**:

```bash
[ -d /sys/firmware/efi ] && echo "UEFI boot" || echo "Legacy BIOS boot"
```

Expected result:

```
UEFI boot
```

UEFI systems use an **EFI partition** for the bootloader.

---

# 4. Verify Firmware Boot Order

Install the EFI management tool:

```bash
sudo apt install efibootmgr
```

Check the firmware boot order:

```bash
sudo efibootmgr
```

Example:

```
BootCurrent: 0000
BootOrder: 0000,...
Boot0000* ubuntu
```

This means the firmware loads **Ubuntu GRUB first**.

---

# 5. Rename the Main Boot Entry

To clearly identify the main system in the GRUB menu:

Edit the GRUB configuration:

```bash
sudo nano /etc/default/grub
```

Modify this line:

```
GRUB_DISTRIBUTOR=`( . /etc/os-release; echo ${NAME:-Ubuntu} )`
```

Replace it with:

```
GRUB_DISTRIBUTOR="Ubuntu 24.04 (Main)"
```

Save the file.

---

# 6. Rebuild GRUB Configuration

Apply the changes:

```bash
sudo update-grub
```

The update should again detect the second OS:

```
Found Ubuntu 22.04.5 LTS (22.04)
```

---

# 7. Configure Boot Menu Visibility

To ensure the menu is visible for a few seconds during startup:

Edit the same GRUB file:

```bash
sudo nano /etc/default/grub
```

Set:

```
GRUB_TIMEOUT_STYLE=menu
GRUB_TIMEOUT=5
```

This displays the GRUB menu for **5 seconds** before booting the default OS.

Then update GRUB:

```bash
sudo update-grub
```

---

# 8. Reboot and Test

Restart the system:

```bash
sudo reboot
```

The GRUB menu should now appear similar to:

```
Ubuntu 24.04 (Main)
Advanced options for Ubuntu 24.04 (Main)
Memory test
Ubuntu 22.04.5 LTS (22.04)
UEFI Firmware Settings
```

The system will automatically boot **Ubuntu 24.04 (Main)** unless another option is selected.

---

# 9. Maintenance Tip

If the secondary Ubuntu installation ever overwrites the bootloader during updates:

Boot into **Ubuntu 24.04** and run:

```bash
sudo grub-install
sudo update-grub
```

This restores the correct boot manager.

---

# Final System Layout

| System       | Role                             |
| ------------ | -------------------------------- |
| Ubuntu 24.04 | Main OS                          |
| Ubuntu 22.04 | Secondary / compatibility system |
| GRUB         | Controlled by Ubuntu 24.04       |

---

# Result

Both laptop and desktop now:

* Boot through **Ubuntu 24.04 GRUB**
* Detect the **secondary Ubuntu installation**
* Present a clean, predictable boot menu
* Default to **Ubuntu 24.04**

---
