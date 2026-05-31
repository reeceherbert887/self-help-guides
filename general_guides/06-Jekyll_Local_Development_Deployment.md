# Jekyll Local Development and Deployment Guide

**Hull Robotics Society Website**

This guide explains how to run the website locally, preview changes, and deploy updates to GitHub Pages. These steps work on both Windows and Linux using Bash.

---

# Overview

The Hull Robotics Society website uses:

* Jekyll (static site generator)
* GitHub Pages (hosting)
* Minimal Mistakes theme
* Markdown (`.md`) files for content
* `/media/` folder for images and videos

---


# 1. Prerequisites

Install the following:

## Windows

Install:

* Ruby + DevKit: https://rubyinstaller.org/
* Git: https://git-scm.com/

During Ruby install, select:

```
Add Ruby to PATH
Install MSYS2
```

## Linux (Ubuntu/Debian)

```bash
sudo apt update
sudo apt install ruby-full build-essential zlib1g-dev git
```

---


# 2. Clone the Website Repository

```bash
git clone https://github.com/Hull-Robotics-Society/Hull-Robotics-Society.github.io.git
cd Hull-Robotics-Society.github.io
```

---


# 3. Install Jekyll Dependencies

Ensure the `Gemfile` contains:

```ruby
source "https://rubygems.org"

gem "github-pages", group: :jekyll_plugins
gem "webrick", "~> 1.8"
gem "tzinfo-data", platforms: [:mingw, :mswin, :x64_mingw, :jruby]
gem "wdm", ">= 0.1.0", platforms: [:mingw, :mswin, :x64_mingw]
```

Install dependencies:

```bash
bundle install
```

---


# 4. Run the Website Locally

Start the Jekyll server:

```bash
bundle exec jekyll serve --livereload
```

You should see:

```
Server address: http://127.0.0.1:4000
Server running... press ctrl-c to stop.
```

Open browser:

```
http://localhost:4000
```

---


# 5. Editing the Website

Edit Markdown files:

```
_pages/
_baxter/
_dalek/
_tutorials/
```

Add media files:

```
media/baxter/
media/dalek/
```

Example image in Markdown:

```md
![Baxter Arm](/media/baxter/example.png)
```

Example video:

```html
<video controls width="100%">
  <source src="/media/baxter/example.mp4" type="video/mp4">
</video>
```

Changes appear instantly after refreshing the browser.

---


# 6. Stopping the Server

Press:

```
CTRL + C
```

---


# 7. Deploy Changes to GitHub Pages

Open a new terminal in the repo folder.

Check status:

```bash
git status
```

Add files:

```bash
git add .
```

Commit changes:

```bash
git commit -m "Updated Baxter documentation and media"
```

Push to GitHub:

```bash
git push
```

GitHub Pages will automatically rebuild the website.

Live site:

```
https://hull-robotics-society.github.io/
```

---

# 8. Recommended .gitignore

Create `.gitignore` file:

```
.vs/
_site/
vendor/
.bundle/
```

This prevents local build files from being uploaded.

---

# 9. Common Commands Reference

Start server:

```bash
bundle exec jekyll serve --livereload
```

Install dependencies:

```bash
bundle install
```

Stop server:

```
CTRL + C
```

Check git status:

```bash
git status
```

Commit changes:

```bash
git commit -m "message"
```

Push changes:

```bash
git push
```

Pull latest changes:

```bash
git pull
```

---

# 10. Folder Structure Overview

```
Hull-Robotics-Society.github.io/
│
├── _pages/
├── _baxter/
├── _dalek/
├── media/
│   └── baxter/
│   └── dalek/
├── assets/
├── Gemfile
├── _config.yml
```

---


# 11. Development Workflow Summary

```
Start server:
bundle exec jekyll serve --livereload

Edit files

Preview changes in browser

Commit and push:
git add .
git commit -m "message"
git push
```

---

# 12. Gemfile for each OS


