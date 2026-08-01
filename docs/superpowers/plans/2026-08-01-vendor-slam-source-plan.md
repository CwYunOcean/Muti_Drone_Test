# Vendor SLAM Source Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Version-control the exact shared SLAM source baseline while keeping aircraft-specific calibration outputs, logs, point clouds, and generated workspace products out of Git.

**Architecture:** Vendor the source directories below `slam_ws/src` into the root repository. Root ignore rules distinguish reusable code, default configuration, licenses, and documentation from runtime outputs under `Log`, `PCD`, `calib_data`, and `output`. A pytest contract verifies a representative FAST-LIO manifest is tracked while representative local data remains ignored.

**Tech Stack:** Git, pytest, ROS 2 workspace layout, Markdown documentation.

---

### Task 1: Add A Failing SLAM Source-Boundary Contract

**Files:**
- Create: `tests/test_slam_source_tracking.py`

- [ ] **Step 1: Write the failing test**

```python
def test_slam_source_is_tracked_and_local_outputs_are_ignored():
    assert git("check-ignore", "-q", "slam_ws/src/FAST_LIO/package.xml").returncode == 1
    assert git("ls-files", "--error-unmatch", "slam_ws/src/FAST_LIO/package.xml").returncode == 0
    assert git("check-ignore", "-q", "slam_ws/src/FAST_LIO/Log/imu.txt").returncode == 0
```

- [ ] **Step 2: Run the test before changing ignore rules**

Run: `pytest -q tests/test_slam_source_tracking.py`

Expected: failure because the representative FAST-LIO package is ignored and absent from the index.

### Task 2: Vendor Source And Exclude Local Data

**Files:**
- Modify: `.gitignore`
- Track: `slam_ws/src/**` except local-data paths
- Create: `slam_ws/UPSTREAM.md`

- [ ] **Step 1: Permit source and add explicit local-data exclusions**

Remove the broad `slam_ws/src/*` ignore rule. Keep `slam_ws/local`, workspace `build`, `install`, and `log` ignored. Add explicit ignores for `FAST-Calib-ROS2/calib_data`, `FAST-Calib-ROS2/output`, `FAST-LIVO2/Log`, `FAST_LIO/Log`, `FAST_LIO/PCD`, and the accidental `rpg_vikit/ername` file.

- [ ] **Step 2: Record baseline provenance and GPL obligations**

Create `slam_ws/UPSTREAM.md` listing the vendored component directories. State that the original source tree had no nested Git metadata, so this commit is the authoritative baseline and historical manifests cannot prove an exact base SHA. Require retention of each component's license, shared review for source changes, and no airframe-specific settings in tracked SLAM source.

- [ ] **Step 3: Stage only source baseline files**

Stage `slam_ws/src` after the exclusions are active. Confirm all listed local-data paths remain untracked and ignored.

### Task 3: Align Documentation With Clone-Complete SLAM Source

**Files:**
- Modify: `AGENTS.md`
- Modify: `CLAUDE.md`
- Modify: `README.md`
- Modify: `manifests/README.md`
- Modify: `docs/runbooks/github-multi-drone-repository.md`
- Delete: `manifests/slam_ws.repos`

- [ ] **Step 1: Remove SLAM import instructions**

Update active repository and bootstrap documentation so a clean clone already includes `slam_ws/src`; only external workspaces still use `vcs import`.

- [ ] **Step 2: Add existing-aircraft migration guidance**

Require an aircraft with an ignored pre-vendoring `slam_ws/src` to back it up before the first pull. Explain that shared SLAM changes must be committed and reviewed, while LiDAR IPs, calibration output, logs, and maps remain local.

### Task 4: Verify And Commit The SLAM Baseline

**Files:**
- Verify: `.gitignore`, `tests/test_slam_source_tracking.py`, `slam_ws/UPSTREAM.md`, `slam_ws/src/**`, active bootstrap documentation

- [ ] **Step 1: Verify the boundary**

Run:

```bash
git check-ignore -q slam_ws/build
git check-ignore -q slam_ws/install
git check-ignore -q slam_ws/log
git check-ignore -q slam_ws/src/FAST_LIO/Log/imu.txt
git check-ignore -q slam_ws/src/FAST_LIO/PCD/1
git ls-files --error-unmatch slam_ws/src/FAST_LIO/package.xml
```

- [ ] **Step 2: Run focused tests**

Run: `pytest -q tests/test_robot_config.py tests/test_nav_source_tracking.py tests/test_slam_source_tracking.py`

Expected: all tests pass.

- [ ] **Step 3: Commit and push**

```bash
git add AGENTS.md CLAUDE.md README.md .gitignore manifests docs/runbooks \
  docs/superpowers/plans/2026-08-01-vendor-slam-source-plan.md \
  slam_ws tests/test_slam_source_tracking.py
git commit -m "chore: vendor slam source baseline"
git push origin main
```

- [ ] **Step 4: Verify final repository state**

Run: `git status --short && git log -1 --oneline && git fsck --no-reflogs`

Expected: clean worktree, the new commit at `HEAD`, and no Git object errors.
