# Interactive Robot Problems

For now, this repo serves as my scratchpad for mocking up controllers, robot simulations, and experiments with potentially useful libraries, packages, and languages.

Keep in mind that code posted here is not production-ready or even working in most cases. Files should be read mostly as my train-of-thought when trying new things or practicing writing robot software.

## robottools.py

This is an experiment to develop generalized functions useful for multiple robot architectures. The goals are:

1. Make functions general enough to be used across multiple codebases
2. Make functions general enough to be used with various types of robots
3. Prefer pure functions, as they retain the most extensibility and reusability

The tools enclosed are intended mostly for robot control.

## realtime_plotting_optimization1.py

This script successfully (but slowly) implements naive minimization of a distance-to-goal cost function for the differential drive mobile robot model.

Clicking on the screen sets a new goal and runs an optimization to that x and y position. The program then moves the robot forward one index of the predicted solution.

----------

## TODO

- [ ] Make smarter MPC algorithm for differential drive model
