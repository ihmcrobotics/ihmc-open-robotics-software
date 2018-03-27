package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.states;

/**
 * Defines states for the {@code JumpingHighLevelHumanoidController} 
 * @author Apoorv
 * Extend to allow for jumping on a single foot / alternating feet (isn't that running !?) 
 */
public enum JumpStateEnum
{
   STANDING,
   /**
    * Generates the required momentum for the robot to take flight
    */
   TAKE_OFF,
   /**
    * Aerial phase of the jump
    */
   FLIGHT,
   /**
    * Momentum rejection state at the end of jump
    */
   LANDING;
}
