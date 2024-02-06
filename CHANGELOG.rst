^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sick_visionary_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
1.1.2 (2024-02-06)
------------------
* update sick_visionary_cpp_shread to fix CoLa2 session timeout
* bump package.xml version
* fix CMakeLists.txt used wrong PCL LIBRARY variable, fix project version

1.1.1 (2023-12-14)
------------------
* fix: bump package.xml version
* add tags to package.xml for ros-index

1.1.0 (2023-12-08)
------------------
* add ROS diagnostics feature to check topic publishing frequencies (Contribution by 4am-robotics)
* update README with new parameters and fix relative links
* add new parameters to launch files

1.0.0 (2023-11-16)
------------------
* First public release
* Enforce style guide using pre-commit
* simplify project structure
* remove support for discontinued devices

0.2.3 (2022-06-08)
------------------
* Auto connect to device if connection is lost

0.2.2 (2020-12-02)
------------------
* support for Visionary-T Mini CX (V3S105-1AAAAAA)
* Fix bug for Visionary-S CX (V3S102-1AAAAAA and V3S102-1AABAAB)  pointcloud data

0.2.1 (2019-12-23)
------------------
* adapted VisionayControl module to control also through CoLa 2 besides CoLa B
* support for control of Visionary-T CX VGA

0.2.0 (2019-11-03)
------------------
* support for Visionary-T CX VGA

0.1.1 (2017-12-06)
------------------
* increased performance
* support cartesian data e.g. for Visionary-T DT or AG
* support polar data

0.1.0 (2017-09-01)
------------------
* rename to visionary
* usage of Visionary_Common library
* support for Visionary-S CX (V3S102-1AAAAAA and V3S102-1AABAAB)
