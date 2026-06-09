# Home Assistant Recovery After Main Pool Export

## Overview

Following a shutdown of the TrueNAS system and temporary removal of the 8TB HDD containing the `mainpool` ZFS pool, Home Assistant failed to start after the pool was reconnected.

Initial symptoms suggested complete application loss. However, investigation revealed that the Home Assistant data still existed and the issue was caused by missing ZFS mounts and broken application registration.

---

# Symptoms

Observed errors:

```text
Applications are not running
Error in Apps Service
```

Docker status:

```text
[EFAULT] 'mainpool/ix-apps' dataset is not mounted on '/mnt/.ix-apps'
```

Application page showed:

```text
Applications are not running
```

No installed applications appeared.

---

# Root Cause

The 8TB HDD containing:

```text
mainpool
```

had been exported from TrueNAS.

When the pool was reconnected:

```text
Pool imported
```

but several datasets were not automatically mounted.

Critical application dataset:

```text
mainpool/ix-apps
```

was not mounted at:

```text
/mnt/.ix-apps
```

Therefore Docker could not locate:

```text
app_configs
app_mounts
docker
truenas_catalog
```

which caused the Apps service to fail.

---

# Investigation Timeline

## Step 1 - Verify Pool Status

Command:

```bash
sudo zpool import
```

Result:

```text
mainpool ONLINE
```

Pool detected.

---

## Step 2 - Import Pool

Command:

```bash
sudo zpool import mainpool
```

Result:

```text
cannot mount '/mainpool'
failed to create mountpoint
read-only file system
```

Pool imported but datasets were not mounted correctly.

---

## Step 3 - Verify ix-apps

Command:

```bash
sudo zfs list -r mainpool/ix-apps
```

Result:

Datasets existed but were inaccessible to Apps.

---

## Step 4 - Correct Mountpoint

Original:

```text
/.ix-apps
```

Corrected:

```bash
sudo zfs set mountpoint=/mnt/.ix-apps mainpool/ix-apps
```

Mounted:

```bash
sudo zfs mount mainpool/ix-apps
```

---

## Step 5 - Verify App Data

Command:

```bash
sudo zfs list -r mainpool/ix-apps
```

Result:

Existing Home Assistant datasets found:

```text
home-assistant/config
home-assistant/media
home-assistant/postgres_data
```

No data loss occurred.

---

## Step 6 - Recover Application Service

Command:

```bash
sudo midclt call docker.status
```

Eventually returned:

```text
Application(s) are currently running
```

---

# What Worked

✅ Correcting ix-apps mountpoint

✅ Mounting datasets manually

✅ Verifying datasets before reinstalling

✅ Confirming Docker status

---

# What Did NOT Work

❌ Immediate reinstall

❌ Using Host Path mounts

❌ Assuming application data was lost

---

# Final Result

Recovered:

* Home Assistant users
* Devices
* Integrations
* Automations
* Configuration
* Database

No Home Assistant data was lost.

---

# Recovery Flowchart

```text
Pool Exported
      │
      ▼
Apps Missing
      │
      ▼
Verify Pool Exists
      │
      ▼
Import Pool
      │
      ▼
Correct ix-apps Mountpoint
      │
      ▼
Mount Datasets
      │
      ▼
Docker Starts
      │
      ▼
Home Assistant Restored
```

# Lessons Learned

Always verify:

```bash
sudo zpool status
sudo zfs list
```

before reinstalling applications.

In this case the data never disappeared; the datasets simply were not mounted.
