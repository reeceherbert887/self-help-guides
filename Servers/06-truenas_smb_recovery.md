# SMB Share Recovery After Main Pool Export

**Author:** Reece Herbert
**Date:** 23/05/2026
**Status:** Ready

---

This issue ultimately caused:

* Linux File Explorer failures
* Nextcloud External Storage failures
* Missing SMB shares

even though the actual data still existed.

---

# Symptoms

Linux File Explorer:

```text
Failed to mount Windows share
No such file or directory
```

Nextcloud:

```text
Failed to mount Windows share
```

SMB share discovery:

```text
backups
IPC$
```

Only backup share visible.

---

# Investigation

TrueNAS believed shares existed:

```text
Reece
User
User
User
```

All enabled.

However Samba exported:

```text
[backups]
```

only.

---

# Root Cause

Pool export caused datasets to lose active mountpoints.

Datasets existed but were not mounted.

SMB paths:

```text
/mnt/mainpool/Reece
/mnt/mainpool/User
/mnt/mainpool/User
/mnt/mainpool/User
```

were invalid.

Samba therefore removed them from its active configuration.

---

# Recovery

Corrected dataset mounts.

Verified:

```bash
ls -ld /mnt/mainpool/Reece
```

and equivalent commands.

Datasets became visible again.

---

# SMB Configuration Refresh

Each SMB share was:

```text
Edited
Saved
```

without changing any settings.

This forced TrueNAS to regenerate Samba configuration.

---

# Restart SMB

Command:

```bash
sudo systemctl restart smbd
```

Verification:

```bash
smbclient -L //192.168.4.51 -U Reece
```

Returned:

```text
Reece
User
User
User
backups
```

All shares restored.

---

# What Worked

Verifying dataset existence

Correcting mountpoints

Regenerating SMB shares

Restarting Samba

---

# What Did Not Work

Assuming share deletion

Recreating datasets

Reinstalling SMB

---

# Final Result

Recovered:

* Reece Share
* User Share
* User Share
* User Share
* Nextcloud External Storage Access
* Linux File Explorer Access

No user files were lost.

---

# Recovery Flowchart

```text
Pool Exported
      │
      ▼
Datasets Unmounted
      │
      ▼
SMB Paths Invalid
      │
      ▼
Samba Removes Shares
      │
      ▼
Nextcloud External Storage Fails
      │
      ▼
Mount Datasets
      │
      ▼
Edit & Save SMB Shares
      │
      ▼
Restart Samba
      │
      ▼
Shares Restored
```

# Lessons Learned

If SMB shares disappear after a pool import:

1. Check dataset mountpoints.
2. Verify directories exist.
3. Check Samba export list.
4. Edit and re-save SMB shares.
5. Restart Samba.

Never assume the data itself is gone until ZFS datasets are verified.
