package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.robotSide.RobotSide;

public interface SwingLegTorqueControlModule
{
   public abstract void compute(LegTorques legTorquesToPackForSwingLeg, FramePoint desiredFootPosition, FrameOrientation desiredFootOrientation,
         FrameVector desiredFootVelocity, FrameVector desiredFootAngularVelocity, FrameVector desiredFootAcceleration,
         FrameVector desiredFootAngularAcceleration);
   
   public abstract void computePreSwing(RobotSide swingSide);
   
   public abstract void setAnkleGainsSoft(RobotSide swingSide);

   public abstract void setAnkleGainsDefault(RobotSide swingSide);
}
