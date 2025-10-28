^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vision_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.0.0 (2022-03-19)
------------------
* Merge pull request `#67 <https://github.com/ros-perception/vision_msgs/issues/67>`_ from ijnek/ijnek-point-2d
  Add Point2d message, and use it in Pose2D
* Update msg/Point2D.msg
  Co-authored-by: Adam Allevato <Kukanani@users.noreply.github.com>
* add Point2D message
* replace deprecated pose2d with pose (`#64 <https://github.com/ros-perception/vision_msgs/issues/64>`_)
  * replace deprecated geometry_msgs/Pose2D with geometry_msgs/Pose
  * replace Pose2D with Pose.
  * add Pose2D.msg
  * undo some changes made
  * Update msg/Pose2D.msg
  Co-authored-by: Adam Allevato <Kukanani@users.noreply.github.com>
  Co-authored-by: Adam Allevato <Kukanani@users.noreply.github.com>
* Clarify array messages in readme
* Trailing newline in bbox2Darray
* Add bbox array msgs to CMakeLists
* Added BoundingBox2DArray message
* Contributors: Adam Allevato, Fruchtzwerg94, Kenji Brameld

3.0.1 (2021-07-20)
------------------
* Patch for how C++14 is set for ROS2 (`#58 <https://github.com/ros-perception/vision_msgs/issues/58>`_)
* Contributors: Dustin Franklin

3.0.0 (2021-04-13)
------------------
* Add license snippet in CONTRIBUTING.md
* Decouple source data from the detection/classification messages. (`#53 <https://github.com/ros-perception/vision_msgs/issues/53>`_)
  * Decouple source data from the detection/classification messages.
  This commit drops dependency on sensor_msgs
  * Improved documentation.
* Merge pull request `#52 <https://github.com/ros-perception/vision_msgs/issues/52>`_ from mintar/clarify-class-object-id
  Rename tracking_id -> id, id -> class_id
* Rename DetectionXD.tracking_id -> id
* Rename ObjectHypothesis.id -> class_id
* Merge pull request `#51 <https://github.com/ros-perception/vision_msgs/issues/51>`_ from ros-perception/clarify-bbox-size
  Clarify comment for size fields in bounding box messages
* Revert confusing comment about bbox orientation
* Merge pull request `#50 <https://github.com/ros-perception/vision_msgs/issues/50>`_ from ros-perception/remove-is-tracking-field
  Remove is_tracking field
* Remove other mentions to is_tracking field
* Clarify bbox size comment
* Remove tracking_id from Detection3D as well
* Remove is_tracking field
  This field does not seem useful, and we are not aware of anyone using it at this time. `VisionInfo` is probably a better place for this information anyway, if it were needed.
  See `#47 <https://github.com/ros-perception/vision_msgs/issues/47>`_ for earlier discussions.
* Clarify: ObjectHypothesis[] ~= Classification (`#49 <https://github.com/ros-perception/vision_msgs/issues/49>`_)
  * Clarify: ObjectHypothesis[] ~= Classification
  https://github.com/ros-perception/vision_msgs/issues/46 requested Array message types for ObjectHypothesis and/or ObjectHypothesisWithPose. As pointed out in the issue, these already exist in the form of the `ClassificationXD` and `DetectionXD` message types.
  * Clarify ObjectHypothesisWithPose[] ~= Detection
* Use composition in ObjectHypothesisWithPose (`#48 <https://github.com/ros-perception/vision_msgs/issues/48>`_)
* Contributors: Adam Allevato, Martin Günther, Martin Pecka, root

2.0.0 (2020-08-11)
------------------
* Fix lint error for draconian header guard rule
* Rename create_aabb to use C++ extension
  This fixes linting errors which assume that .h means that a file
  is C (rather than C++).
* Add CONTRIBUTING.md
* Fix various linting issues
* Add gitignore
  Sync ros2 with master
* Update test for ros2
* add BoundingBox3DArray message (`#30 <https://github.com/Kukanani/vision_msgs/issues/30>`_)
  * add BoundingBoxArray message
* Make msg gen package deps more specific (`#24 <https://github.com/Kukanani/vision_msgs/issues/24>`_)
  Make message_generation and message_runtime use more specific depend tags
* Merge branch 'kinetic-devel'
* Removed "proposal" from readme (`#23 <https://github.com/Kukanani/vision_msgs/issues/23>`_)
* add tracking ID to the Detection Message (`#19 <https://github.com/Kukanani/vision_msgs/issues/19>`_)
  * add tracking ID to the Detection
  * modify comments
  * Change UUID messages to strings
  * Improve comment for tracking_id and fix whitespace
* Convert id to string (`#22 <https://github.com/Kukanani/vision_msgs/issues/22>`_)
* Specify that id is explicitly for object class
* Fix dependency of unit test. (`#14 <https://github.com/Kukanani/vision_msgs/issues/14>`_)
* 0.0.1
* Pre-release commit - setting up versioning and changelog
* Rolled BoundingRect into BoundingBox2D
  Added helper functions to make it easier to go from corner-size representation to
  center-size representation, plus associated tests.
* Added license
* Small fixes in message comments (`#10 <https://github.com/Kukanani/vision_msgs/issues/10>`_)
* Contributors: Adam Allevato, Leroy Rügemer, Martin Günther, Masaya Kataoka, Ronald Ensing, Shane Loretz, mistermult
* Switched to ROS2 for package definition files, create_aabb, etc.
* [ros2] use package format 3 (`#12 <https://github.com/Kukanani/vision_msgs/issues/12>`_)
* Contributors: Adam Allevato, Martin Günther, Mikael Arguedas, procopiostein
