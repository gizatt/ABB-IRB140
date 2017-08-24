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
- 'abb.py', which provides a backend for `irb_client.py`.
- 'RAPID/SERVER.mod', which is ABB RAPID code for the arm-server-side
of this system.
- 'RAPID/LOGGER.mod', which is ABB RAPID code for an asynchronous
server-side process to stream position, and force-torque, information.

The RAPID code and `abb.py` originate in the `open_abb` project, which is
forked (here)[https://github.com/gizatt/open_abb/tree/gizatt-add-ft-query]
and originates upstream of there.