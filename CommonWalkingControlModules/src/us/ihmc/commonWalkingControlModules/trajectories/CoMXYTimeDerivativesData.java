package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.robotics.geometry.FramePoint2D;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class CoMXYTimeDerivativesData
{
   private FramePoint2D comXYPosition = new FramePoint2D(ReferenceFrame.getWorldFrame());
   private FrameVector2d comXYVelocity = new FrameVector2d(ReferenceFrame.getWorldFrame());
   private FrameVector2d comXYAcceleration = new FrameVector2d(ReferenceFrame.getWorldFrame());

   public void set(CoMXYTimeDerivativesData comXYData)
   {
      this.comXYPosition.set(comXYData.comXYPosition);
      this.comXYVelocity.set(comXYData.comXYVelocity);
      this.comXYAcceleration.set(comXYData.comXYAcceleration);
   }

   public void getCoMXYPosition(FramePoint2D comXYPositionToPack)
   {
      comXYPositionToPack.set(this.comXYPosition);
   }

   public void getCoMXYVelocity(FrameVector2d comXYVelocityToPack)
   {
      comXYVelocityToPack.set(this.comXYVelocity);
   }

   public void getCoMXYAcceleration(FrameVector2d comXYAccelerationToPack)
   {
      comXYAccelerationToPack.set(this.comXYAcceleration);
   }

   public void setCoMXYPosition(FramePoint2D comXYPosition)
   {
      this.comXYPosition.set(comXYPosition);
   }

   public void setCoMXYVelocity(FrameVector2d comXYVelocity)
   {
      this.comXYVelocity.set(comXYVelocity);
   }

   public void setCoMXYAcceleration(FrameVector2d comXYAcceleration)
   {
      this.comXYAcceleration.set(comXYAcceleration);
   }
}
