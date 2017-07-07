^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package spqrel_navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.1 (2017-07-07)
------------------
* added dynreconf
* added move_base_msgs
* added tf dep
* added roscpp
* Merge branch 'master' of github.com:LCAS/spqrel_navigation
* ROS planner draft
* Handling temporal obstacles
* Adding management of blind zones in laser scan
* README updated
* Merge branch 'unified_cmake'
* revision and test OK
* added missing build_depend in messgae_generation
* curated package.xml
* all in one CMakeLists.txt for ROS and qibuild
* Adding visualization of variables in naoqi_planner_gui. Naoqi_planner_gui sends goals in meters.
* Aligning with srrg_types
* update and clean ros/naoqi build
* refactoring ros/naoqi folders
* added PDF document
* map compatible with ROS format, target goal now in world coords [meters] - candidate release 1.1
* clean
* added write_pose
* (commented) 1 Hz when no target goal
* (commented) 1 Hz when no target goal
* catch error and don't compute dynamic map when no laser data available
* Printing some info in GUI (pepper_planner for now)
* changed name of input parameter to initial_pose_theta
* collision protection management
* renamed to spqrel
* Merge remote-tracking branch 'origin/navigation'
* Initial commit
* cleaning cmakelists
* Adding DIAG maps
* Added planner remote gui
* setting max range when retrieving laser
* changes in naoqi_planner: 1) enable/disable move, 2) disabling self collision avoidance, 3) setting usable_range for computing map obstacles
* controlling fixed cycle time
* Adding management of dynamic obstacles in planner
* rotation-only behaviour
* First steps in planner
* Contributors: Jaime Pulido Fentanes, Luca Iocchi, Marc Hanheide, Mayte, mtlazaro
