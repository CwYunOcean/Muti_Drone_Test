# EGO-Swarm Vendored Source Baseline

`nav_ws/src/ego-swarm-ros2` is version-controlled directly by the root
`Drone_SLAM` repository. A clean clone therefore contains the exact navigation
source used by this flight stack and must not run `vcs import` for `nav_ws`.

The reference upstream is:

```text
https://github.com/legubiao/ego-swarm-ros2.git
```

The directory that was first added to this repository had no nested Git
metadata. Its exact upstream base revision cannot be established from this
working copy, so the vendoring commit is the authoritative reproducible
baseline. Do not claim that it matches an upstream SHA without a separate
source comparison.

The vendored source retains its upstream `LICENSE`, which is GNU GPLv3. Keep
that license with every distribution and mark material local source changes in
their commits. Changes under this directory are shared flight-source changes:
make them on a feature branch, test the navigation-dependent stack, and never
use them for airframe-specific IP addresses, identities, or calibrations.
