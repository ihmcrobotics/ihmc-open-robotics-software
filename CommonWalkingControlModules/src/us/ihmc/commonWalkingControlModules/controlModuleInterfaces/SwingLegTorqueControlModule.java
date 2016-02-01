package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.robotSide.RobotSide;

public interface SwingLegTorqueControlModule
{
   public abstract void compute(LegTorques legTorquesToPackForSwingLeg, FramePoint desiredFootPosition, FrameOrientation desiredFootOrientation,
         FrameVector desiredFootVelocity, FrameVector desiredFootAngularVelocity, FrameVector desiredFootAcceleration,
         FrameVector desiredFootAngularAcceleration);
   
   public abstract void computePreSwing(RobotSide swingSide);
   
   public abstract void setAnkleGainsSoft(RobotSide swingSide);

   public abstract void setAnkleGainsDefault(RobotSide swingSide);
}
