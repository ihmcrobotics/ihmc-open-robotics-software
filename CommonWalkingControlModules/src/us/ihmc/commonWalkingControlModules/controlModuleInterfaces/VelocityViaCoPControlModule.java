package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;

public interface VelocityViaCoPControlModule
{
   /**
    * Computes the desired CoP when the robot is in single support.
    * @param supportLeg
    * @param desiredVelocity the desired center of mass velocity
    * @return the desired Center of Pressure. Should be inside the base of support.
    */
   public abstract FramePoint2d computeDesiredCoPSingleSupport(RobotSide supportLeg, FrameVector2d desiredVelocity);
   
   /**
    * Computes the desired CoP when the robot is in double support.
    * @param desiredVelocity the desired center of mass velocity
    * @return the desired Center of Pressure. Should be inside the base of support.
    */
   public abstract FramePoint2d computeDesiredCoPDoubleSupport(RobotSide loadingLeg, FrameVector2d desiredVelocity);
}
