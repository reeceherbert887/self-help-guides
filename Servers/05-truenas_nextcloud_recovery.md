# Nextcloud Recovery After Main Pool Export

**Author:** Reece Herbert
**Date:** 23/05/2026
**Status:** Ready

---

After recovery of the mainpool, Nextcloud initially appeared broken.

The application could not be started and later appeared empty.

Investigation confirmed all user data remained intact.

---

# Symptoms

Nextcloud unavailable.

External storage failed:

```text
Failed to mount Windows share
No such file or directory
```

Concern existed that:

```text
Users
Files
Database
```

had been lost.

---

# Investigation

Verified datasets:

```bash
sudo ls -lah /mnt/.ix-apps/app_mounts/nextcloud
```

Found:

```text
data
html
postgres_data
```

---

# User Data Verification

Command:

```bash
sudo ls -1 /mnt/.ix-apps/app_mounts/nextcloud/data
```

Returned:

```text
User_Lastname
User_Lastname
User_Lastname
truenas_admin
```

This confirmed user data still existed.

---

# Database Verification

Dataset:

```text
postgres_data
```

present.

Application database preserved.

---

# Web Application Verification

Container inspection confirmed:

```text
/var/www/html
```

mapped to:

```text
/mnt/.ix-apps/app_mounts/nextcloud/html
```

and

```text
/var/www/html/data
```

mapped to:

```text
/mnt/.ix-apps/app_mounts/nextcloud/data
```

---

# Root Cause

Nextcloud itself was recovered successfully.

The remaining issue was:

```text
External Storage
```

which depended on SMB shares.

Since SMB shares were broken, Nextcloud could not mount them.

---

# What Worked

Verifying datasets before reinstalling

Confirming database presence

Confirming user folders

Confirming container mappings

---

# What Did Not Work

Assuming missing shares meant missing data

Recreating users

Creating new databases

---

# Final Result

Recovered:

* User accounts
* User files
* Database
* Configuration
* Existing uploads

No Nextcloud data was lost.

---

# Recovery Flowchart

```text
Pool Exported
      │
      ▼
Nextcloud Offline
      │
      ▼
Verify Data Dataset
      │
      ▼
Verify Database Dataset
      │
      ▼
Verify Container Mappings
      │
      ▼
Nextcloud Restored
      │
      ▼
External Storage Still Broken
      │
      ▼
Fix SMB Shares
      │
      ▼
External Storage Restored
```

