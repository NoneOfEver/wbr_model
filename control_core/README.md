# control_core

This directory is the platform-independent destination of the WBR controller.
Public fixed-size contracts live under `include/wbr/control`. During migration,
only modules that have no MuJoCo dependency are moved here. See
`docs/CONTROL_ARCHITECTURE.md` for the dependency and ownership rules.

