# Git and SSH Setup Guide (Windows & Linux)

This guide covers: - Generating SSH keys - Connecting GitHub securely -
Cloning repositories - Pulling updates - Committing changes - Pushing
changes

Works for both **Windows (Git Bash)** and **Linux (Ubuntu)**.

---


# 1. One-time setup: Git identity

Run anywhere:

``` bash
git config --global user.name "Your Name"
git config --global user.email "your_email@example.com"
```

Check:

``` bash
git config --global --list
```

---


# 2. Generate SSH key

Check if one exists:

``` bash
ls -al ~/.ssh
```

Generate new key:

``` bash
ssh-keygen -t ed25519 -C "your_email@example.com"
```

Press Enter for defaults.

---


# 3. Start SSH agent

``` bash
eval "$(ssh-agent -s)"
```

Add key:

``` bash
ssh-add ~/.ssh/id_ed25519
```

---


# 4. Add SSH key to GitHub

Show public key:

``` bash
cat ~/.ssh/id_ed25519.pub
```

Copy and add to:

GitHub → Settings → SSH and GPG keys → New SSH key

---


# 5. Test SSH connection

``` bash
ssh -T git@github.com
```

Expected output:

``` text
Hi USERNAME! You've successfully authenticated.
```

---



# 6. Clone repository

``` bash
git clone git@github.com:USERNAME/REPO.git
cd REPO
```

---

# 7. Change existing repo from HTTPS to SSH


``` bash
git remote set-url origin git@github.com:USERNAME/REPO.git
```

Verify from within the repo:

``` bash
git remote -v
```

---


# 8. Daily workflow

Enter repo:

``` bash
cd path/to/repo
```

Pull latest changes:

``` bash
git pull
```

---


# 9. Create or edit files

Example using nano:

``` bash
nano test.py
```

Save:

CTRL+O → Enter → CTRL+X

---


# 10. Commit ONE file

``` bash
git add test.py
git commit -m "Add test.py"
git push
```

---

# 11. Commit ALL changes

``` bash
git add .
git commit -m "Update project"
git push
```

---


# 12. Check repo status

``` bash
git status
```


# 13. View commit history

``` bash
git log --oneline
```

---


# 14. Summary workflow

Most common usage:

``` bash
git pull
git add .
git commit -m "message"
git push
```

---


# 15. File locations

Linux:

    ~/.ssh/

Windows:

    C:\Users\YourName\.ssh\

---


# 16. SSH advantages

-   No password required
-   Works with all repos
-   More secure than HTTPS
-   Works on Windows and Linux

