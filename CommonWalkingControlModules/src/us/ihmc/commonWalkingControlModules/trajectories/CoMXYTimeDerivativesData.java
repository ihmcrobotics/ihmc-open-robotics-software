package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class CoMXYTimeDerivativesData
{
   private FramePoint2D comXYPosition = new FramePoint2D(ReferenceFrame.getWorldFrame());
   private FrameVector2D comXYVelocity = new FrameVector2D(ReferenceFrame.getWorldFrame());
   private FrameVector2D comXYAcceleration = new FrameVector2D(ReferenceFrame.getWorldFrame());

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

   public void getCoMXYVelocity(FrameVector2D comXYVelocityToPack)
   {
      comXYVelocityToPack.set(this.comXYVelocity);
   }

   public void getCoMXYAcceleration(FrameVector2D comXYAccelerationToPack)
   {
      comXYAccelerationToPack.set(this.comXYAcceleration);
   }

   public void setCoMXYPosition(FramePoint2D comXYPosition)
   {
      this.comXYPosition.set(comXYPosition);
   }

   public void setCoMXYVelocity(FrameVector2D comXYVelocity)
   {
      this.comXYVelocity.set(comXYVelocity);
   }

   public void setCoMXYAcceleration(FrameVector2D comXYAcceleration)
   {
      this.comXYAcceleration.set(comXYAcceleration);
   }
}
