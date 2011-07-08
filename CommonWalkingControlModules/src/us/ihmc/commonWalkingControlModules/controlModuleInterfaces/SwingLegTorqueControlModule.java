package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.Orientation;

public interface SwingLegTorqueControlModule
{
   public abstract void compute(LegTorques legTorquesToPackForSwingLeg, FramePoint desiredFootPosition, Orientation desiredFootOrientation,
         FrameVector desiredFootVelocity, FrameVector desiredFootAngularVelocity, FrameVector desiredFootAcceleration,
         FrameVector desiredFootAngularAcceleration);
   
   public abstract void computePreSwing(RobotSide swingSide);
   
   public abstract void setAnkleGainsSoft(RobotSide swingSide);

   public abstract void setAnkleGainsDefault(RobotSide swingSide);
}
