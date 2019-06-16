
# DDR2 Controller

The current implementation relies on the MIG core generator. This core requires a couple tweaks after generation
as the clocking scheme isn't quite what it expects. The patches in the coregen folder must be applied after
generation is complete. The planAhead project Makefile does this automatically when creating the project ("make project").

The two memory controllers are mapped at 0x40000000 and 0x44000000 respectively. These are currently mapped on a 32-bit
wide bus running at 25 MHz, so the full memory controller bandwidth is not really available to the CPU.
