# GitHub Large Video File Push Error and Git LFS Fix

## Problem Summary

When pushing the repository to GitHub, the push failed because one of the video files was larger than GitHub's normal file size limit.

The key error was:

```bash
remote: error: File 551530_ACW_202323817_202421412/ros2_ws/src/task1/task1/demo/Robotics Simulation Task1.mp4 is 139.59 MB; this exceeds GitHub's file size limit of 100.00 MB
remote: error: GH001: Large files detected. You may want to try Git Large File Storage
```

GitHub allows normal Git files up to **100 MB**. Anything larger than this cannot be pushed normally. In this case, the file:

```bash
551530_ACW_202323817_202421412/ros2_ws/src/task1/task1/demo/Robotics Simulation Task1.mp4
```

was **139.59 MB**, so GitHub rejected the push.

There were also warnings for other videos:

```bash
remote: warning: File ... task1 simulation pt1.mp4 is 71.80 MB; this is larger than GitHub's recommended maximum file size of 50.00 MB
remote: warning: File ... task1 node list.mp4 is 60.28 MB; this is larger than GitHub's recommended maximum file size of 50.00 MB
remote: warning: File ... task1 movement.mp4 is 61.77 MB; this is larger than GitHub's recommended maximum file size of 50.00 MB
```

These files were not blocked because they were below 100 MB, but GitHub still warned that they were larger than the recommended 50 MB limit.

---

## Why This Happened

Git stores file history permanently. This means that even if a large file is removed or re-added later, the large version may still exist in the commit history.

In this case, the video file was first committed as a normal Git file. After that, GitHub rejected it because it was over 100 MB.

Simply removing and re-adding the file with Git LFS was not enough at first, because the old 139 MB version still existed inside the previous Git commit history.

---

## What Git LFS Does

Git LFS, which stands for **Git Large File Storage**, stores large files separately from the normal Git history.

Instead of storing the full video directly inside Git, the repository stores a small pointer file. The actual video is uploaded to GitHub's LFS storage.

This means:

- The video still appears in the GitHub repository.
- The file path can remain the same.
- The normal Git push does not contain the full 139 MB video file.
- Other users can still download the video if they have Git LFS installed.

---

## Fix Option 1: Use Git LFS for the Large Video

This is the best fix when the video needs to stay in the repository.

### Step 1: Go to the repository

```bash
cd ~/repos/ACW_Group_Project
```

### Step 2: Install Git LFS

```bash
sudo apt update
sudo apt install git-lfs -y
```

### Step 3: Initialise Git LFS

```bash
git lfs install
```

### Step 4: Track MP4 files with Git LFS

```bash
git lfs track "*.mp4"
git add .gitattributes
```

This creates or updates the `.gitattributes` file so that MP4 files are handled by Git LFS.

### Step 5: Remove the large video from normal Git tracking

```bash
git rm --cached "551530_ACW_202323817_202421412/ros2_ws/src/task1/task1/demo/Robotics Simulation Task1.mp4"
```

This removes the file from normal Git tracking but keeps the actual file on the computer.

### Step 6: Add the video back as an LFS file

```bash
git add "551530_ACW_202323817_202421412/ros2_ws/src/task1/task1/demo/Robotics Simulation Task1.mp4"
```

### Step 7: Commit the change

```bash
git commit -m "Track task demo video with Git LFS"
```

### Step 8: Push again

```bash
git push
```

---

## Problem With Fix Option 1

In this case, the push still failed because the large video already existed in an earlier commit.

The output showed that the LFS upload worked:

```bash
Uploading LFS objects: 100% (1/1), 146 MB | 21 MB/s, done.
```

However, GitHub still rejected the push:

```bash
remote: error: File 551530_ACW_202323817_202421412/ros2_ws/src/task1/task1/demo/Robotics Simulation Task1.mp4 is 139.59 MB; this exceeds GitHub's file size limit of 100.00 MB
```

This meant the old version of the file was still inside Git history.

---

## Fix Option 2: Rewrite Git History With Git LFS Migrate

This is the fix that solved the issue.

Because the large MP4 was already committed normally, the Git history needed to be rewritten so that all MP4 files were converted to Git LFS objects.

### Step 1: Check the repository status

```bash
git status
```

The output showed several MP4 files as modified:

```bash
Changes not staged for commit:
  modified: Markdown/Images + Videos/Task 1 Video/Code expanation-Reece/task1 movement.mp4
  modified: Markdown/Images + Videos/Task 1 Video/Code expanation-Reece/task1 node list.mp4
  modified: Markdown/Images + Videos/Task 1 Video/Code expanation-Reece/task1 simulation pt1.mp4
```

This happened because Git LFS had converted the videos locally.

### Step 2: Add the converted MP4 files

```bash
git add .gitattributes
git add "Markdown/Images + Videos/Task 1 Video/Code expanation-Reece/"*.mp4
```

### Step 3: Commit the LFS conversion

```bash
git commit -m "Track task 1 videos with Git LFS"
```

Example output:

```bash
[main 48b4f16] Track task 1 videos with Git LFS
 12 files changed, 0 insertions(+), 0 deletions(-)
 rewrite Markdown/Images + Videos/Task 1 Video/Code expanation-Reece/task1 movement.mp4 (99%)
 rewrite Markdown/Images + Videos/Task 1 Video/Code expanation-Reece/task1 node list.mp4 (99%)
 rewrite Markdown/Images + Videos/Task 1 Video/Code expanation-Reece/task1 simulation pt1.mp4 (99%)
```

The `rewrite` messages show that the large video files were replaced by Git LFS pointer files.

### Step 4: Rewrite the history for all MP4 files

```bash
git lfs migrate import --include="*.mp4"
```

Example output:

```bash
migrate: Fetching remote refs: ..., done.
migrate: Sorting commits: ..., done.
migrate: Rewriting commits: 100% (3/3), done.
main 48b4f167388747ba1b5cf421b7ada736921ac8e0 -> 17c1041bbb7ddfbacdc1031152e901ffe36d7fc0
migrate: Updating refs: ..., done.
migrate: checkout: ..., done.
```

This command rewrote the commit history and converted all MP4 files into Git LFS objects.

### Step 5: Force push safely

```bash
git push --force-with-lease
```

Example successful output:

```bash
Uploading LFS objects: 100% (13/13), 716 MB | 28 MB/s, done.
Writing objects: 100% (28/28), 3.26 KiB | 303.00 KiB/s, done.
To github.com:ACW-Team/ACW_Group_Project.git
   d438387..17c1041  main -> main
```

This showed that the push worked.

---

## Why `--force-with-lease` Was Needed

The command:

```bash
git lfs migrate import --include="*.mp4"
```

rewrites Git history. This changes the commit IDs.

Because the local history no longer matched the remote history, a normal push would not work. A force push was required.

However, `--force-with-lease` is safer than `--force` because it checks that the remote branch has not changed unexpectedly before overwriting it.

---

## Final Working Command Sequence

The final successful fix was:

```bash
cd ~/repos/ACW_Group_Project

git add .gitattributes
git add "Markdown/Images + Videos/Task 1 Video/Code expanation-Reece/"*.mp4

git commit -m "Track task 1 videos with Git LFS"

git lfs migrate import --include="*.mp4"

git push --force-with-lease
```

---

## How to Know It Worked

The successful push showed:

```bash
Uploading LFS objects: 100% (13/13), 716 MB | 28 MB/s, done.
Writing objects: 100% (28/28), 3.26 KiB | 303.00 KiB/s, done.
main -> main
```

This means:

- The videos were uploaded through Git LFS.
- Git only pushed small pointer files.
- GitHub accepted the push.
- The repository was successfully updated.

---

## Fix Option 3: Remove Videos From GitHub Completely

Another possible fix is to remove the videos from the repository and upload them somewhere else, such as:

- OneDrive
- Google Drive
- YouTube as an unlisted video
- The university submission portal

Then, a link can be added to the README file or report.

Example:

```md
## Demonstration Video

The demonstration video is available here:

[Task 1 Demonstration Video](PASTE_VIDEO_LINK_HERE)
```

This approach is simpler if the coursework does not require videos to be stored directly inside the GitHub repository.

---

## Fix Option 4: Compress the Video Below 100 MB

Another possible solution is to compress the video so it is smaller than 100 MB.

Example command using `ffmpeg`:

```bash
ffmpeg -i "Robotics Simulation Task1.mp4" -vcodec libx264 -crf 28 -preset medium -acodec aac "Robotics Simulation Task1_compressed.mp4"
```

However, this is not always ideal because:

- The video quality may be reduced.
- The file may still be above 100 MB.
- GitHub still recommends keeping files below 50 MB.
- Large videos make the repository slower to clone.

For this project, Git LFS was the better fix because the videos needed to stay in the repository.

---

## Recommended Approach for This Project

For this project, the best solution was to use Git LFS because the video needed to remain in the repository path:

```bash
551530_ACW_202323817_202421412/ros2_ws/src/task1/task1/demo/Robotics Simulation Task1.mp4
```

The final result was successful because all MP4 files were migrated into Git LFS and the repository was pushed successfully.

---

## Key Lesson

If a file is larger than 100 MB, GitHub will reject it if it is committed as a normal Git file.

If the file has already been committed, simply removing and re-adding it may not be enough because the large file can still exist in Git history.

In that case, the correct fix is:

```bash
git lfs migrate import --include="*.mp4"
```

followed by:

```bash
git push --force-with-lease
```
