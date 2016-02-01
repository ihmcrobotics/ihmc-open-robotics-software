package us.ihmc.aware.controller;

public enum QuadrupedControllerEvent
{
   /**
    * Fired when the low-level actuator has been brought online and a safe desired angle is set.
    */
   JOINTS_INITIALIZED,

   /**
    * Fired when the robot has been brought to its starting pose.
    */
   STARTING_POSE_REACHED,

   POSITION_BASED_CRAWL,
   VIRTUAL_MODEL_BASED_STEP,
}
