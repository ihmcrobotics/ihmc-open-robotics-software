package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointAccelerations;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointPositions;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointVelocities;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.robotics.robotSide.RobotSide;

public interface SwingLegTorqueControlOnlyModule
{
   public abstract void compute(LegTorques legTorquesToPackForSwingLeg, LegJointPositions jointPositions, LegJointVelocities jointVelocities, LegJointAccelerations jointAccelerations);
   
   public abstract void computePreSwing(LegTorques legTorquesToPackForSwingLeg);
   
   public abstract void setAnkleGainsSoft(RobotSide swingSide);

   public abstract void setAnkleGainsDefault(RobotSide swingSide);
   
//   public abstract void setDampingToZero(RobotSide swingSide);

}
