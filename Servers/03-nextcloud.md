# Nextcloud External Storage Integration

## TrueNAS Scale + Nextcloud + SMB Storage

**Author:** Reece Herbert
**Date:** 07/06/2026
**Status:** Ready

---

# Project Goal

The aim of this project was to integrate Nextcloud with my existing TrueNAS storage structure so that:

* PCs and Laptops use SMB shares directly.
* Phones and tablets use Nextcloud.
* All users store files in the same location.
* Snapshots and backups only need configuring once.
* Nextcloud becomes a frontend rather than a separate storage system.

Desired structure:

```text
Laptop/PC
     │
     ▼
 SMB Share
     │
     ▼
TrueNAS Dataset
     ▲
     │
Nextcloud
     ▲
     │
Phone / Tablet
```

---

# Planned Storage Structure

```text
Main Pool
│
├── Reece
├── User
├── User
├── User
│
└── Shared
```

Each user has:

| User  | TrueNAS Dataset | SMB Share | Nextcloud Account |
| ----- | --------------- | --------- | ----------------- |
| Reece | Reece           | Reece     | truenas_admin     |
| User  | User            | User      | User              |
| User  | User            | User      | User              |
| User  | User             | User      | User              |

---

# What Worked Immediately

## SMB Shares

The SMB shares worked perfectly.

Testing:

```text
Windows Explorer
Linux File Manager
Multiple Devices
```

All devices could access storage correctly.

Result:

✅ Working

---

## Nextcloud Installation

After a fresh installation:

```text
PostgreSQL
Redis
Nextcloud
```

all containers started successfully.

Verification:

```bash
sudo docker ps
```

Result:

✅ Working

---

# First Major Mistake

Initially I mapped:

```text
/mnt/mainpool/nextclouddata
```

as the Nextcloud Data Directory.

Unfortunately this already contained:

```text
config
apps
core
3rdparty
data
```

which are Nextcloud application files.

---

## What Happened

Nextcloud expected:

```text
Data Folder
```

but instead received:

```text
Entire Nextcloud Installation
```

This generated:

```text
Your data directory is invalid
```

and:

```text
Ensure there is a file called .ncdata
```

---

## Fix

Completely removed the contents:

```bash
sudo rm -rf /mnt/mainpool/nextclouddata/*
```

and allowed Nextcloud to rebuild itself.

Result:

Fixed

---

# External Storage Configuration

Enabled:

```text
Settings
    └── Apps
           └── External Storage Support
```

Result:

Installed Successfully

---

# Storage Mount Configuration

Created Local Storage Mounts:

```text
/ Reece
/ User
/ User
/ User
```

Mapped to:

```text
/mnt/mainpool/Reece
/mnt/mainpool/User
/mnt/mainpool/User
/mnt/mainpool/User
```

---

# Second Major Problem

Storage displayed:

```text
Pending
```

and:

```text
Storage is temporarily unavailable
```

---

# Investigation Process

## Step 1

Verify container can see storage.

Command:

```bash
sudo docker inspect ix-nextcloud-nextcloud-1
```

Result:

```text
Source:
/mnt/mainpool/Reece

Destination:
/mnt/mainpool/Reece
```

Meaning:

```text
Container sees storage.
```

Result:

Working

---

## Step 2

Verify Nextcloud can see storage.

Command:

```bash
php occ files_external:list
```

Result:

```text
/Reece
/User
/User
/User
```

Result:

Working

---

## Step 3

Verify Storage Health

Command:

```bash
php occ files_external:verify
```

Result:

```text
Status: OK
```

Result:

✅ Working

---

# Real Problem Found

Checking logs:

```text
Permission denied
```

specifically:

```text
opendir(/mnt/mainpool/Reece/)
```

---

# Why This Happened

TrueNAS datasets were owned by:

```text
Owner: Reece
Group: storage_users
```

However Nextcloud runs as:

```text
www-data
```

Inside container:

```bash
id www-data
```

returned:

```text
uid=33(www-data)
gid=33(www-data)
```

Meaning:

```text
www-data
≠
Reece
```

Therefore Nextcloud could not access files.

---

# ACL Investigation

Initial ACL:

```text
user::---
group::rwx
other::---
```

Meaning:

```text
Owner had NO permissions.
```

This was incorrect.

---

# Why chmod and setfacl Failed

The TrueNAS datasets use **ZFS with NFSv4 ACLs**. This means:

```text
chmod  → Operation not permitted
setfacl → Operation not supported
```

Standard Linux permission tools are blocked by ZFS when ACL mode is set to restricted. The only supported method is the TrueNAS ACL editor.

---

# Final Fix — TrueNAS Web UI ACL Editor

## The Problem

All four external storage mounts were failing with:

```text
StorageNotAvailableException: Directory listing failed
```

This affected all users: `User_collinson`, `User_collinson`, `User_herbert`, and `truenas_admin`.

The Nextcloud process runs as `www-data` (UID 33), which had no ACL entry on any of the ZFS datasets.

## The Fix

For each dataset (`Reece`, `User`, `User`, `User`):

1. Open TrueNAS Web UI
2. Navigate to **Storage → Datasets**
3. Select the dataset and click **Edit Permissions**
4. In the ACL Editor, click **Add Item** and configure:

```text
Who:         User
User:        www-data
ACL Type:    Allow
Permissions: Full Control
Flags:       Inherit
```

5. Tick **Apply permissions recursively** ✅
6. Click **Save Access Control List**

## Verification

After applying ACLs, confirmed access from inside the container:

```bash
sudo docker exec -it ix-nextcloud-nextcloud-1 \
su -s /bin/bash www-data -c "ls /mnt/mainpool/Reece"
```

Result:

```text
Documents  Downloads  Music  Photos  Videos  nextcloud-test.txt  test folder  test.txt
```

Then forced a full rescan:

```bash
sudo docker exec -u www-data -it ix-nextcloud-nextcloud-1 php occ files:scan --all
```

Result:

```text
Folders: 40  Files: 229  Updated: 587  Errors: 0
```

Result:

✅ All four mounts working in Nextcloud UI

---

# Architecture Going Forward

```text
                   ┌─────────────┐
                   │  TrueNAS    │
                   └──────┬──────┘
                          │
                ┌─────────┴─────────┐
                │ Main Storage Pool │
                └─────────┬─────────┘
                          │
       ┌──────────────────┼──────────────────┐
       │                  │                  │
       ▼                  ▼                  ▼

    Reece             User              User

       ▲                  ▲                  ▲
       │                  │                  │

   SMB Share         SMB Share         SMB Share

       ▲                  ▲                  ▲
       │                  │                  │

 PCs/Laptops      PCs/Laptops      PCs/Laptops

       ▲
       │

   Nextcloud
       ▲
       │

 Phones/Tablets
```

---

# Lessons Learned

| Lesson                                                               | Importance  |
| -------------------------------------------------------------------- | ----------- |
| Do not point Nextcloud data directory at existing installation files | High        |
| Always verify container mounts first                                 | High        |
| Verify storage using OCC commands before changing configuration      | High        |
| Nextcloud runs as www-data (UID 33), not dataset owners              | Critical    |
| External Storage can pass verification but still fail in UI          | High        |
| SMB and Nextcloud should point to same dataset                       | Recommended |
| Logs are more useful than the GUI status messages                    | Critical    |
| ZFS NFSv4 ACL datasets reject chmod and setfacl entirely             | Critical    |
| Always use the TrueNAS Web UI ACL Editor for ZFS dataset permissions | Critical    |
| Apply permissions recursively or subdirectories remain inaccessible  | Critical    |

---

# Final Status

| Component                     | Status      |
| ----------------------------- | ----------- |
| TrueNAS                       | ✅ Working   |
| SMB Shares                    | ✅ Working   |
| Nextcloud                     | ✅ Working   |
| PostgreSQL                    | ✅ Working   |
| Redis                         | ✅ Working   |
| Dataset Mounts                | ✅ Working   |
| ACL Permissions               | ✅ Working   |
| File Scanning                 | ✅ Working   |
| External Storage Verification | ✅ Working   |
| External Storage UI Access    | ✅ Working   |