package us.ihmc.humanoidRobotics.communication.packets.dataobjects;

import us.ihmc.communication.ros.generators.RosEnumValueDocumentation;

public enum NewHighLevelControllerStates
{
   @RosEnumValueDocumentation(documentation = "do nothing state. the robot will start in this state, and report this state when falling and ramping down the controller. This state is intended for feedback only. Requesting this state is not supported and can cause the robot to shut down.")
   DO_NOTHING_STATE,
   @RosEnumValueDocumentation(documentation = "Stand prep state.")
   STAND_PREP_STATE,
   @RosEnumValueDocumentation(documentation = "Freeze state.")
   FREEZE_STATE,
   @RosEnumValueDocumentation(documentation = "Stand transition state.")
   STAND_TRANSITION_STATE,
   @RosEnumValueDocumentation(documentation = "whole body force control employing IHMC walking, balance, and manipulation algorithms")
   WALKING_STATE,
   @RosEnumValueDocumentation(documentation = "The robot is peforming an automated diagnostic routine")
   DIAGNOSTICS,
   @RosEnumValueDocumentation(documentation = "Automated calibration routine depending on the robot. For Valkyrie: estimation of the joint torque offsets.")
   CALIBRATION;

   public static final NewHighLevelControllerStates[] values = values();
}
