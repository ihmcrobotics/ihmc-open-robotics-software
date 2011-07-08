package us.ihmc.commonWalkingControlModules.outputs;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LowerBodyTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.UpperBodyTorques;
import us.ihmc.robotSide.RobotSide;

public interface ProcessedOutputsInterface
{
   public abstract void setLowerBodyTorques(LowerBodyTorques lowerBodyTorques);
   public abstract void setUpperBodyTorques(UpperBodyTorques upperBodyTorques);
   
   public abstract void setDesiredLegJointTorque(RobotSide robotSide, LegJointName jointName, double desiredTorque);
   public abstract double getDesiredLegJointTorque(RobotSide robotSide, LegJointName jointName);

   public abstract void resetAllDesiredJointVelocities();
   public abstract void setAllDesiredTorquesToZero();

   public abstract void setDamping(RobotSide robotSide, LegJointName jointName, double damping);
   public abstract void setDesiredJointVelocity(RobotSide robotSide, LegJointName jointName, double desiredJointVelocity);

}
