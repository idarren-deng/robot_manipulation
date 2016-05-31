Change history
==============

1.4.1 (2016-01-16)
------------------

 * Fixed weird parsing issue with certain URDFs.


1.4.0 (2016-01-16)
------------------

 * Added example program for using TRAC-IK.
 * Added ability to get KDL chain structures from TRAC-IK class.

1.3.9 (2016-01-16)
------------------

 * Added new TRAC-IK constructor that takes URDF location, base link, and tip link, instead of KDL structures.
 * Some README fixes based on user comments

1.3.8 (2016-01-04)
------------------

 * Fixed how continuous joints are handled in random restarts

1.3.7 (2016-01-04)
------------------

 * Fixed bug introduced in 1.3.6 where tip frame was being used for Inverse Jacobian, which was never converging.

1.3.6 (2016-01-04)
------------------

 * Fixed tolerances to be in tip frame, not base frame.
 * Fixed Hydro compile issue
 * Changed to std::numeric_limits for +/-FLT_MAX and epsilons
 * Fixed potential error with URDF soft limits

1.3.5 (2015-12-08)
------------------

 * Overhaul of how multiple solutions are managed in TRAC-IK threads.
 * Improved speed and accurate returning of unique results found.

1.3.4 (2015-12-10)
------------------

 * Fixed a bug where we were looping over solutions inside of looping over solutions. 

1.3.3 (2015-12-10)
------------------

 * Change final normalization for manipulability metrics with continuous joints. Make continuous joints explicitly handled (if lower_limit >= upper_limit, joint is continuous).

1.3.2 (2015-12-10)
------------------

 * Fixed bug where TRAC-IK multisolution modes weren't using the computed random seeds, but were using the same seed over and over.
 * Fixed potential issues with large bound joints seraching too large of a space

1.3.1 (2015-12-08)
------------------

 * Slight tweak to Manipulation metrics function to ensure it always works, even with weird Jacobians.

1.3.0 (2015-12-07)
------------------

 * Added a mode to TRAC-IK constructor that determines how the IK solver runs: 1) Speed return immediately whenever any solution is found (all other modes run for the requested timeout to try to find multiple solutions); 2) Distance runs for the timeout and returns the solution found that minimizes the Sum-of-Squares error from the seed; 3) Manip1 returns the solution that maximizes the manipulation metric sqrt(det(J*J^T)); 4)  Manip2 returns the metric that minimizes the condition number |J||J^-1|.
 * Added support for these modes in the MoveIt! plugin via the kinematic.yaml parameter solve_type, which can be one of the strings "Speed", "Distance", "Manipulation1", "Manipulation2".


1.2.1 (2015-12-07)
------------------

 * Fixed a bug in the MoveIt! plugin FK call that assumed all joint poses were desired.


1.2.0 (2015-12-04)
------------------

 * Extended TRAC-IK to both run in two ways: 1) the old mode of first IK solution found causes TRAC-IK to return immediately, versus 2) the new mode where TRAC-IK runs for the full requested timeout duration, then sorts all solutions according to distance from the seed and returns the minimum.
 * Made MoveIt! support this new IK run mode if the user desires.
 * Improved timing info to use a higher solution clock.
 * Fixed TRAC-IK's abort/reset of KDL-RR and NLOpt-IK to catch race conditions.


1.1.2 (2015-12-3)
------------------

 * Fixed issue where clamping a seed to be within the joint limits might still have values outside the limits.
 * Fixed issue where MoveIt! plugin was not thread safe.
 * Fixed an issue in MoveIt! plugin where error_code passed in uninitialized to SUCCESS could cause IK to say it failed when it did not.


1.1.1 (2015-11-19)
------------------

 * Prepared code to have auto test suite run to generate data in main README.md.


1.1.0 (2015-11-12)
------------------

 * Improvements to KDL-RR that better handle joint limits on rotational joints that can turn +- PI.
 * Fixed bug where continuous joints could cause problems.
 * Made NLOpt modes enums instead of integer parameters.


1.0.0 (2015-11-10)
------------------

 * Initial checkin of TRAC-IK as of Humanoids 2015 submission.  Pulled from private repo.
 * Made trac_ik packages conform to rosdep standards.
