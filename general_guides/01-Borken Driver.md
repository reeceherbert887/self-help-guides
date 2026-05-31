# NVIDIA Driver Repair Guide

*(Ubuntu 24.04 + RTX 4060 Multi-Monitor Fix)*

This guide documents the steps used to repair a broken NVIDIA driver installation on **Ubuntu 24.04** after system/package updates caused:

* Only one monitor to work
* Incorrect resolution (640x480)
* NVIDIA Settings to disappear
* `nvidia-smi` to fail
* Additional monitors to not detect

The issue was confirmed to be software-related because the second Ubuntu installation on another drive still functioned correctly.

Final result:

```bash
RTX 4060 detected correctly
NVIDIA driver loaded successfully
All monitors restored
Correct resolutions available
NVIDIA Settings working again
```

---

# 1. Confirm the GPU Is Detected

Check whether Ubuntu can still see the NVIDIA GPU:

```bash
lspci | grep -i nvidia
```

Expected output:

```bash
09:00.0 VGA compatible controller: NVIDIA Corporation AD107 [GeForce RTX 4060]
```

This confirms the hardware itself is functioning correctly.

---

# 2. Check NVIDIA Driver Status

Verify whether the NVIDIA driver is loaded:

```bash
nvidia-smi
```

Broken driver example:

```bash
NVIDIA-SMI has failed because it couldn't communicate with the NVIDIA driver
```

This indicates the NVIDIA kernel module is either:

* Missing
* Corrupted
* Failed after a kernel update
* Or blocked from loading

---

# 3. Find the Recommended NVIDIA Driver

Query Ubuntu for the correct driver version:

```bash
ubuntu-drivers devices
```

Example output:

```bash
driver : nvidia-driver-595-open - distro non-free recommended
```

Important:

Always install the **recommended** driver rather than guessing driver versions manually.

---

# 4. Install the Recommended Driver

Install the recommended package:

```bash
sudo apt install nvidia-driver-595-open
```

During installation Ubuntu will:

* Remove the broken driver
* Install the new NVIDIA packages
* Rebuild DKMS kernel modules
* Regenerate initramfs

Expected successful DKMS output:

```bash
Building initial module for 6.17.0-23-generic
Done.
```

and:

```bash
Installing to /lib/modules/6.17.0-23-generic/updates/dkms/
```

This confirms the NVIDIA kernel module compiled successfully.

---

# 5. Reboot the System

Restart Ubuntu:

```bash
sudo reboot
```

This loads the new NVIDIA kernel modules.

---

# 6. Verify the Driver Loaded Correctly

After reboot:

```bash
nvidia-smi
```

Expected result:

```bash
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 595.xx.xx                                                        |
| GPU  Name                Driver-Version                                     |
| NVIDIA GeForce RTX 4060                                                  |
+-----------------------------------------------------------------------------+
```

At this point:

* NVIDIA Settings should work again
* All monitors should detect correctly
* Proper resolutions should return
* Hardware acceleration should function normally

---

# 7. Optional: Check Connected Displays

Verify detected monitors:

```bash
xrandr
```

This lists all connected displays and active resolutions.

---

# 8. Secure Boot Troubleshooting (If Needed)

If the driver still fails after installation:

Check Secure Boot status:

```bash
mokutil --sb-state
```

Example:

```bash
SecureBoot enabled
```

Secure Boot can prevent NVIDIA kernel modules from loading.

Temporary fix:

1. Reboot into BIOS/UEFI
2. Disable Secure Boot
3. Boot Ubuntu again

Then retest:

```bash
nvidia-smi
```

---

# 9. Repair Broken Packages (If Install Fails)

If package installation errors occur:

```bash
sudo apt --fix-broken install
```

Then retry the driver install:

```bash
sudo apt install nvidia-driver-595-open
```

---

# 10. Notes

The following warning during installation is normal:

```bash
info: The home dir /nonexistent you specified can't be accessed
```

This is related to the `nvidia-persistenced` system user and is harmless.

The following spam is also harmless:

```bash
udevadm hwdb is deprecated
```

---

# Final System State

| Component       | Status             |
| --------------- | ------------------ |
| GPU             | RTX 4060 detected  |
| Driver          | NVIDIA 595 Open    |
| Monitors        | Fully working      |
| NVIDIA Settings | Working            |
| DKMS Modules    | Successfully built |
| Resolution      | Restored correctly |

---

# Result

Ubuntu 24.04 now:

* Loads the NVIDIA driver correctly
* Detects all monitors
* Supports proper resolutions and refresh rates
* Restores full GPU acceleration
* Successfully rebuilds DKMS modules after updates
