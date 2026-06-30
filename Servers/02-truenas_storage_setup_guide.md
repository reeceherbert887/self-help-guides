# TrueNAS Multi-User Storage System - Setup, Configuration and Future Expansion

**Author:** Reece Herbert
**Date:** 07/06/2026
**Status:** Ready

---

The primary objective of this project was to create a centralised storage solution using TrueNAS that could be accessed by multiple users and devices across a home network. The system was designed around an 8TB external hard drive and provides approximately 1.7TiB of allocated storage to four users.

The long-term goal is to expand the system with:

* A dedicated 2TB backup drive.
* Nextcloud integration.
* Mobile device access.
* Automated backups.
* Remote access capabilities.

At the time of writing, the storage system is operational and accessible from Linux devices using SMB shares.

---

# Project Objectives

The key objectives for this project were:

1. Create a centralised storage platform.
2. Support multiple users.
3. Allocate storage limits to each user.
4. Enable network file sharing.
5. Create a backup strategy.
6. Implement snapshot protection.
7. Prepare for future Nextcloud deployment.
8. Create a scalable storage architecture.

---

# Hardware Used

## TrueNAS Server

Current specifications:

```text
CPU: Existing Server Hardware
RAM: Existing Server Hardware
Storage:
    - 8TB External Drive (Primary Storage)
    - 32GB USB Drive (Temporary Backup Testing)
```

---

# Storage Architecture

The storage system was designed around a single primary pool.

```text
mainpool
├── User1
├── User2
├── User3
└── User4
```

Each dataset functions as an individual user storage area.

This approach was chosen because datasets provide:

* Independent quotas.
* Independent permissions.
* Snapshot support.
* Replication support.
* Better management than traditional folders.

---

# Pool Creation

## Primary Storage Pool

Pool Name:

```text
mainpool
```

Configuration:

```text
Layout: Stripe
```

Because only a single drive was available, RAID options were not possible.

Although TrueNAS warns against Stripe configurations, this warning is expected when operating with a single disk.

Benefits:

* Simple setup.
* Full drive capacity available.
* Easy future migration.

Limitations:

* No redundancy.
* Drive failure results in total data loss.

Future upgrades will address this through replication and dedicated backup drives.

---

# User Creation

The following users were created:

```text
User3
User2
User1
User4
```

Each user was configured with:

```text
SMB Access Enabled
Password Authentication Enabled
```

These user accounts allow access to the storage shares from:

* Linux
* Windows
* Android
* iPhone
* Tablets

---

# Group Management

A common storage group was created.

Group Name:

```text
storage_users
```

Purpose:

* Simplify permission management.
* Allow all users to access the shared storage structure.
* Reduce ACL complexity.

---

# Dataset Creation

The following datasets were created:

```text
mainpool/User4
mainpool/User1
mainpool/User2
mainpool/User3
```

Why datasets?

Datasets allow:

* Independent quotas.
* Independent ACLs.
* Snapshots.
* Replication.
* Easier future backup management.

This approach is considerably more flexible than simply creating folders.

---

# Storage Allocation

The 8TB drive provides approximately:

```text
7.14TiB usable space
```

Storage was allocated as:

```text
User1  = 1.7TiB
User2 = 1.7TiB
User3  = 1.7TiB
User4 = 1.7TiB
```

Total allocation:

```text
6.8TiB
```

This leaves remaining free space for:

* ZFS overhead.
* Snapshots.
* Future expansion.

---

# ACL Configuration

Each dataset was configured with:

```text
Owner:
    Corresponding User

Owner Group:
    storage_users
```

Permissions:

```text
owner@                 Full Control

group@ storage_users   Modify

builtin_administrators Full Control
```

This configuration provides:

* User ownership.
* Administrative control.
* Shared family access.

The intention was not to isolate users completely but rather provide organised storage areas whilst maintaining access flexibility.

---

# SMB Configuration

SMB was selected because it offers excellent compatibility across multiple operating systems.

Benefits:

```text
Linux Support
Windows Support
Android Support
iPhone Support
```

Connection example:

```text
smb://IP Address of Server
```

Authentication:

```text
Username: User3
Password: ********
```

Testing confirmed successful access from a Linux laptop.

---

# Linux Client Testing

Testing was performed using Ubuntu Linux.

Connection Method:

```text
Files
→ Other Locations
→ Connect to Server
```

Server:

```text
smb://IP Addres of Server
```

Results:

```text
✓ Authentication Successful
✓ SMB Shares Visible
✓ Read Access
✓ Write Access
✓ Folder Creation
✓ File Upload
✓ File Download
```

This verified the NAS was functioning correctly.

---

# Snapshot Configuration

Snapshots were configured to protect against accidental deletion.

Dataset:

```text
mainpool/User3
```

Configuration:

```text
Recursive: Enabled

Schedule:
Daily

Time:
00:00

Retention:
32 Days
```

Purpose:

* Recover deleted files.
* Recover modified files.
* Create recovery points.
* Provide replication sources.

Important:

Snapshots are NOT backups.

Snapshots remain on the same storage device.

If the primary drive fails, snapshots are also lost.

---

# Temporary Backup Configuration

A temporary backup pool was created using a 32GB USB drive.

Pool:

```text
usbbackup
```

Dataset:

```text
usbbackup/backups
```

Purpose:

* Learn replication.
* Test backup workflows.
* Prepare for future backup hardware.

The USB drive is too small to back up the entire NAS.

Instead it acts as a proof-of-concept backup target.

---

# Replication Configuration

Replication was configured as:

```text
mainpool/User3
        │
        ▼
usbbackup/backups
```

Purpose:

* Create secondary copies.
* Test disaster recovery.
* Learn TrueNAS replication procedures.

Future configuration will replace the USB drive with a dedicated backup drive.

---

# Difference Between Snapshots and Backups

## Snapshot

```text
mainpool/User3
        │
        ▼
Snapshot
```

Stored on:

```text
Same Drive
```

Protects against:

```text
Accidental Deletion
Accidental Modification
```

Does NOT protect against:

```text
Drive Failure
```

---

## Backup

```text
mainpool/User3
        │
        ▼
usbbackup/backups
```

Stored on:

```text
Separate Drive
```

Protects against:

```text
Drive Failure
User Error
Corruption
```

---

# Future Expansion Plan

## Phase 1 (Completed)

```text
✓ Pool Creation
✓ User Creation
✓ Group Creation
✓ Dataset Creation
✓ Quotas
✓ ACLs
✓ SMB Access
✓ Snapshots
✓ Backup Testing
```

---

## Phase 2

Upgrade backup storage.

```text
32GB USB
        ▼
2TB HDD
```

Benefits:

* Larger backup capacity.
* Multiple user backups.
* Long-term retention.

---

## Phase 3

Deploy Nextcloud.

Architecture:

```text
Phones
Laptops
Desktops
      │
      ▼
  Nextcloud
      │
      ▼
   TrueNAS
```

Benefits:

* Remote access.
* Automatic photo uploads.
* Personal cloud storage.
* Device synchronisation.

---

# Lessons Learned

Several key lessons were learned during this setup:

1. Datasets are significantly more flexible than folders.
2. Quotas provide effective storage management.
3. ACLs can initially be confusing but offer powerful control.
4. SMB provides excellent cross-platform compatibility.
5. Snapshots and backups serve different purposes.
6. Replication requires snapshots before it can function correctly.
7. TrueNAS benefits greatly from planning storage architecture before deployment.

---

# What Went Well

The following aspects worked successfully:

```text
✓ User Creation
✓ Dataset Structure
✓ Quotas
✓ ACL Configuration
✓ SMB Connectivity
✓ Linux Client Access
✓ Snapshot Scheduling
✓ Backup Pool Creation
```

Most importantly, the storage system became operational and accessible from network clients.

---

# Known Limitations

Current limitations include:

```text
Single Storage Drive
No Redundancy
32GB Backup Device
No Nextcloud
No Remote Access
```

These limitations will be addressed in future project phases.

---

# Current Project Status

```text
Storage Pool Created        ✓
Users Created               ✓
Datasets Created            ✓
Quotas Configured           ✓
ACLs Configured             ✓
SMB Operational             ✓
Snapshots Operational       ✓
Backup Testing Operational  ✓
Nextcloud                   Planned
2TB Backup Drive            Planned
Remote Access               Planned
```

The system is currently functioning as a centralised multi-user NAS and provides a strong foundation for future expansion.
