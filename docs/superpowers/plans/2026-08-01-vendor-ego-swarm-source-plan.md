# Vendor EGO-Swarm Source Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the exact EGO-Swarm source currently used by the flight stack part of the `Drone_SLAM` Git history so a new aircraft clone contains it without a separate upstream import.

**Architecture:** Treat `nav_ws/src/ego-swarm-ros2` as a vendored source baseline in the root repository. Continue to ignore ROS build products and preserve the original license. A repository-boundary test verifies that a representative planner file is tracked and not ignored; documentation identifies this baseline as authoritative because the original directory had no nested Git metadata.

**Tech Stack:** Git, pytest, ROS 2 workspace layout, Markdown documentation.

---

### Task 1: Enforce The Navigation Source Boundary

**Files:**
- Create: `tests/test_nav_source_tracking.py`
- Modify: `.gitignore`
- Track: `nav_ws/src/ego-swarm-ros2/**`

- [ ] **Step 1: Write the failing repository-boundary test**

```python
def test_ego_swarm_source_is_tracked_and_not_ignored():
    source_file = "nav_ws/src/ego-swarm-ros2/planner/ego_planner/package.xml"
    assert git("check-ignore", "-q", source_file).returncode == 1
    assert git("ls-files", "--error-unmatch", source_file).returncode == 0
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `pytest -q tests/test_nav_source_tracking.py`

Expected: failure because `.gitignore` currently ignores `nav_ws/src/*` and the source is absent from the Git index.

- [ ] **Step 3: Permit tracked navigation source and stage the exact baseline**

Remove only the two `nav_ws/src` ignore rules from `.gitignore`, retain the global ROS `build/`, `install/`, and `log/` rules, then stage every file below `nav_ws/src/ego-swarm-ros2`.

- [ ] **Step 4: Run the repository-boundary test to verify it passes**

Run: `pytest -q tests/test_nav_source_tracking.py`

Expected: `1 passed` after the representative planner manifest is both unignored and staged.

### Task 2: Make Bootstrap Documentation Match The Vendored Source

**Files:**
- Create: `nav_ws/UPSTREAM.md`
- Modify: `README.md`
- Modify: `CLAUDE.md`
- Modify: `manifests/README.md`
- Modify: `docs/runbooks/github-multi-drone-repository.md`
- Delete: `manifests/nav_ws.repos`

- [ ] **Step 1: Record authoritative-source provenance**

Create `nav_ws/UPSTREAM.md` stating that `nav_ws/src/ego-swarm-ros2` is vendored in this repository, retains its own `LICENSE`, and that this commit is the authoritative baseline because the original directory had no nested Git history. Record the upstream URL `https://github.com/legubiao/ego-swarm-ros2.git` as a reference only, without claiming an unverified base SHA.

- [ ] **Step 2: Remove duplicate-import instructions**

Delete `manifests/nav_ws.repos`. Update the root README, CLAUDE workspace table, manifest README, and multi-drone runbook so that only Livox, SLAM, and formation dependencies use `vcs import`; a fresh clone already includes navigation source.

- [ ] **Step 3: Document existing-aircraft migration safety**

In the multi-drone runbook, require an existing aircraft to back up or compare its ignored `nav_ws/src/ego-swarm-ros2` directory before the first pull that introduces tracked vendor files. State that later navigation changes use the same `main` branch and must not be made as per-aircraft edits.

### Task 3: Verify And Commit The Reproducible Navigation Baseline

**Files:**
- Verify: `.gitignore`, `tests/test_nav_source_tracking.py`, `nav_ws/UPSTREAM.md`, `nav_ws/src/ego-swarm-ros2/**`, documentation files

- [ ] **Step 1: Verify repository content and exclusion boundaries**

Run:

```bash
git check-ignore -q nav_ws/build
git check-ignore -q nav_ws/install
git check-ignore -q nav_ws/log
git check-ignore nav_ws/src/ego-swarm-ros2/planner/ego_planner/package.xml
git ls-files --error-unmatch nav_ws/src/ego-swarm-ros2/planner/ego_planner/package.xml
git diff --cached --check -- .gitignore README.md CLAUDE.md manifests \
  docs/runbooks/github-multi-drone-repository.md nav_ws/UPSTREAM.md \
  tests/test_nav_source_tracking.py
```

Expected: build, install, and log paths are ignored; the representative source file is not ignored and is tracked; no whitespace errors are introduced by this change. The vendored upstream baseline is committed verbatim and is not reformatted during this migration.

- [ ] **Step 2: Run focused tests**

Run: `pytest -q tests/test_robot_config.py tests/test_nav_source_tracking.py`

Expected: all tests pass.

- [ ] **Step 3: Commit the vendor baseline**

```bash
git add .gitignore README.md CLAUDE.md manifests docs/runbooks/github-multi-drone-repository.md \
  nav_ws/UPSTREAM.md nav_ws/src/ego-swarm-ros2 tests/test_nav_source_tracking.py
git commit -m "chore: vendor ego swarm source baseline"
```

- [ ] **Step 4: Verify the committed worktree**

Run: `git status --short && git log -1 --oneline && git fsck --no-reflogs`

Expected: clean worktree, the new commit at `HEAD`, and no Git object errors.
