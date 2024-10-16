package us.ihmc.robotics.partNames;

import us.ihmc.commons.robotics.partNames.ArmJointName;
import us.ihmc.commons.robotics.partNames.LegJointName;
import us.ihmc.commons.robotics.partNames.NeckJointName;
import us.ihmc.commons.robotics.partNames.SpineJointName;

/**
 * Classes implementing the {@code RobotSpecificJointNames} provide the enum values of the legs ({@code LegJointName}), arms ({@code ArmJointName}), spine ({@code SpineJointName}), and neck ({@code NeckJointName}) available for a specific robot.
 *
 */
public interface RobotSpecificJointNames
{
   public abstract LegJointName[] getLegJointNames();
   public abstract ArmJointName[] getArmJointNames();
   public abstract SpineJointName[] getSpineJointNames();
   public abstract NeckJointName[] getNeckJointNames();
}

