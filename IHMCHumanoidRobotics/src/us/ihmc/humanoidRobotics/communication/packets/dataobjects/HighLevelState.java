package us.ihmc.humanoidRobotics.communication.packets.dataobjects;

import us.ihmc.communication.ros.generators.RosEnumValueDocumentation;

/**
 * @author twan
 *         Date: 5/6/13
 */
public enum HighLevelState
{
   @RosEnumValueDocumentation(documentation = "whole body force control employing IHMC walking, balance, and manipulation algorithms")
   WALKING,
   @RosEnumValueDocumentation(documentation = "do nothing behavior. the robot will start in this behavior, and report this behavior when falling and ramping down the controller. This behavior is intended for feedback only. Requesting this behavior is not supported and can cause the robot to shut down.")
   DO_NOTHING_BEHAVIOR,
   @RosEnumValueDocumentation(documentation = "The robot is peforming an automated diagnostic routine")
   DIAGNOSTICS,
   @RosEnumValueDocumentation(documentation = "Automated calibration routine depending on the robot. For Valkyrie: estimation of the joint torque offsets.")
   CALIBRATION;

   public static final HighLevelState[] values = values();
}
