^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package spqrel_navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2018-01-23)
------------------
* removed debug prints; fixed parameter names
* naoqi planner refactored first working tests
* changed function name run()
* Partial commit on new refactored naoqi_planner
* Adding min angular vel as ros param
* Added min rotation velocity in rotation-only behaviour
* Fully replaced new version of ros planner after refactoring
* adding recovery procedure to approach goal if there are obstacles in the way
* fixed goal moving while updating the map
* New planner after refactoring compiling
* goal is not cancel upon arrival of a new map
* Publising planner result
* Gui option as ros param
* Added parameter passing to ros node
* restarting distance and cost maps after reset
* setters and getters of max vels and accs
* Added setters, getters in planner, publishing path on ROS
* Going on with refactoring, ros node working
* Going on with refactoring, starting ros planner
* refactoring: moved naoqi and ros to common dynamic_map source
* starting refactoring
* Merge branch 'master' of https://github.com/LCAS/spqrel_navigation
* Added map dis corridor including room b101
* update dynamic map management
* failure feedback, path finding improvements, CTRL+click for GUI target
* ROS planner mutex for map and goal messages
* debug prints - BUG imgshow
* removed debug print
* ROSplanner: removed debug print
* ROS planner changes for dynamic map update
* Merge branch 'master' of https://github.com/LCAS/spqrel_navigation
* Printing cycle times in localizer
* Merge branch 'master' of github.com:LCAS/spqrel_navigation
* ROSplanner changes to make it work in slam+exploration mode
* Reduced laser queue size in ROS planner
* Merge pull request `#5 <https://github.com/LCAS/spqrel_navigation/issues/5>`_ from warp1337/master
  Added example launchfile
* Added example launchfile
* stop when close to target
* Merge branch 'master' of github.com:LCAS/spqrel_navigation
* bugfix
* added lastNode
* Merge branch 'master' of github.com:LCAS/spqrel_navigation
* WIP
* WIP
* WIP
* WIP
* display scans on navigation gui, subscribe to event goal changes
* Merge pull request `#4 <https://github.com/LCAS/spqrel_navigation/issues/4>`_ from LCAS/event-driven
  Event driven toponav
* Merge branch 'event-driven' of github.com:LCAS/spqrel_navigation into event-driven
* computing nominal paths
* bug fixes
* Merge branch 'event-driven' of github.com:LCAS/spqrel_navigation into event-driven
* local changes
* max tries fixed
* Merge branch 'master' of github.com:LCAS/spqrel_navigation into event-driven
* tidy
* first try with new toponav
* Merge branch 'master' of https://github.com/LCAS/spqrel_navigation
* nagoya gimp mapping (adding non visible obstacle by laser)
* fake nav goes through route
* merged version
* fixed bugs
* fixed path to msg
* load the map image from the right place
* added missing math import
* Merge pull request `#3 <https://github.com/LCAS/spqrel_navigation/issues/3>`_ from LCAS/marcs-fixes
  hopefully fixes the mess with outdated topological_map.py in src/
* hopefully fixes the mess with outdated topological_map.py in src/
* Merge pull request `#2 <https://github.com/LCAS/spqrel_navigation/issues/2>`_ from Jailander/lincoln-camo
  Topological Navigation
* inserts list of nodes in memory
* revised covery, needs more testing
* reduced covariance when setpose callback
* fixed previous commit not compiling
* planner computes path when a dynamic obstacle fully blocks the path and tries to reach the closest reachable point in this path
* minor changes in visualization of texts in the images
* localizer force_update
* publishing topological localisation and printing help menu
* adding arguments to clients
* Merge branch 'master' of https://github.com/LCAS/spqrel_navigation into lincoln-camo
* deleting unnecesary file
* Merge pull request `#1 <https://github.com/LCAS/spqrel_navigation/issues/1>`_ from pet1330/lincoln-camo
  Node Deletion
* allow deletion of nodes
  + clean up and pep8 formatting
* pep8 formatting
* seperated msg
* script to get plan
* adding get plan interface
* Fixed linking issue in navigation_gui for Pepper
* get_plan function
* drawing wp name
* No with Status signal and error codes
* now script names make sense
* View map and improvements to top nav
* edited map
* New map from Lincoln
* Added common naoqi GUI for localizer and planner
* Adding SetPose event in localizer
* Publish planner status as events. Fixed executionStatus publisher
* Catching exception when trying to change the self collision protection without being enabled in the robot's settings
* Publishing status
* added entrance of roblab
* Lincoln's lab map
* Merge branch 'topological_navigation' of https://github.com/Jailander/spqrel_navigation
* first commit for topological navigation
* Merge branch 'master' of github.com:LCAS/spqrel_navigation
* ROS planner v1
* Contributors: Florian Lier, Jaime Pulido Fentanes, Luca Iocchi, Marc Hanheide, Mayte, Peter Lightbody, SPQReL, jailander, mtlazaro

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
