# RLG ABB IRB-140 Driver

This repo provides driver code for interfacing our software stack
(primarily (Spartan)[https://github.com/RobotLocomotion/spartan] and
its predecessors) to an ABB IRB-140 industrial robot arm.

In particular, it provides:
- `irb_converter.py`, which converts from Drake+Spartan
state and plan messages, into equivalent formats available in
this repo.
- 'irb_client.py', which unrolls robot plans and feeds them to
a server running on the robot, and relays robot state from that server.

And it submodules
(a fork of open_abb)[https://github.com/gizatt/open_abb/tree/gizatt-add-ft-query],
which provides the actual interop and server that runs on the arm controller.