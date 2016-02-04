package us.ihmc.aware.controller.position;

public enum QuadrupedPositionControllerEvent
{
   /**
    * Fired when the low-level actuator has been brought online and a safe desired angle is set.
    */
   JOINTS_INITIALIZED,

   /**
    * Fired when the robot has been brought to its starting pose.
    */
   STARTING_POSE_REACHED,

   REQUEST_STAND_PREP,
   REQUEST_POSITION_BASED_CRAWL,
}
