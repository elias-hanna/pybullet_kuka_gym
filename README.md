# Gym environments for mimicking deformable object manipulation using a Kuka 7-dof robotic arm

This package contains 2 Gym environments mimicking the interaction of a Kuka 7-dof robotic arm with a deformable object, either with a cube constrained to a single axis or with a deformable object modelized using a neo-Hookean solid hyperelastic material model.
This package was designed to work with evolutionary algorithms, more especially diversity algorithms from the python package [diversity_algorithms](https://github.com/robotsthatdream/diversity_algorithms).
__Warning__: using diversity algorithms with the Novelty Search algorithm may result in simulation failure as the algorithm exploits the simulator to produce non-physical behaviors that generate a lot of novelty compared to physical behaviors.
