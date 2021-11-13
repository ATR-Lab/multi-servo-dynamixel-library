## Explanation of each file that is important:
- initialize_ports.py: Contains coordiate codes for moving a puppet arm around
- initialize_ports_copy.py: Copies the present positions of the motors from the puppet to the master; puppet uses pseudo-PID interfacing for movement

Note: Any additional files aren't needed (aside from the dynomix_driver and dynomix_tools directories), as their use was for a python driven ZeroRPC implementation.