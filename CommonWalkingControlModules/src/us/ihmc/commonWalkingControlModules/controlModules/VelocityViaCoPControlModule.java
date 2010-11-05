package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.RobotSide;
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

   public abstract FramePoint2d computeCapturePoint();

   /**
    * Tells this module to put the weight on the toes since whatever is calling it wants the robot to start toeing off.
    * @param robotSide
    */
   public abstract void setPutWeightOnToes(RobotSide robotSide);

   /**
    * Tells this module to stop putting the weight on the toes since whatever is calling it wants the robot to start toeing off.
    * @param robotSide
    */
   public abstract void unSetPutWeightOnToes(RobotSide robotSide);

   public abstract void setDesiredCoPOffset(FramePoint2d framePoint);

   public abstract FramePoint2d getDesiredCoPOffset();

   public abstract FramePoint2d computeDesiredCoPSingleSupport(RobotSide supportLeg);   
   
}
