package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

/**
 * Defines states for the {@code JumpingHighLevelHumanoidController} 
 * @author apoorv
 * Extend to allow for jumping on a single foot / alternating feet (isn't that running !?) 
 */
public enum JumpStateEnum
{
   /**
    * Generates the required momentum for the robot to take flight
    */
   TAKE_OFF,
   /**
    * Transition state that allows for the ground reactions to be ramped to prevent 
    * the leg from snapping after entering flight phase 
    */
   UNLOADING,
   /**
    * Aerial phase of the jump
    */
   FLIGHT,
   /**
    * Transition state to ramp up the ground reaction forces to prevent leg collapse on touchdown
    */
   LOADING,
   /**
    * Momentum rejection state at the end of jump
    */
   LANDING;
}
