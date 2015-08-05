package us.ihmc.commonWalkingControlModules.outputs;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LowerBodyTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.UpperBodyTorques;
import us.ihmc.humanoidRobotics.partNames.ArmJointName;
import us.ihmc.humanoidRobotics.partNames.LegJointName;
import us.ihmc.humanoidRobotics.partNames.NeckJointName;
import us.ihmc.humanoidRobotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;

public interface ProcessedOutputsInterface
{
   public abstract void setLowerBodyTorques(LowerBodyTorques lowerBodyTorques);
   public abstract void setUpperBodyTorques(UpperBodyTorques upperBodyTorques);
   
   public abstract double getDesiredLegJointTorque(RobotSide robotSide, LegJointName jointName);

//   public abstract void resetAllDesiredJointVelocities();
   public abstract void setAllDesiredTorquesToZero();

//   public abstract void setDampingParameter(RobotSide robotSide, LegJointName jointName, double damping);
//   public abstract void setDesiredJointVelocity(RobotSide robotSide, LegJointName jointName, double desiredJointVelocity);
   public abstract void incrementProcessedOutputsWhiteBoardIndex();

   public abstract void setLegJointTau(RobotSide robotSide, LegJointName jointName, double tau);
   public abstract void setArmJointTau(RobotSide robotSide, ArmJointName jointName, double tau);
   public abstract void setSpineJointTau(SpineJointName jointName, double tau);
   public abstract void setNeckJointTau(NeckJointName jointName, double tau);

}
