package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

public class CoMXYTimeDerivativesData
{
   private FramePoint2D comXYPosition = new FramePoint2D(ReferenceFrame.getWorldFrame());
   private FrameVector3D comVelocity = new FrameVector3D();
   private FrameVector3D comAcceleration = new FrameVector3D();

   public void set(CoMXYTimeDerivativesData comXYData)
   {
      this.comXYPosition.set(comXYData.comXYPosition);
      this.comVelocity.set(comXYData.comVelocity);
      this.comAcceleration.set(comXYData.comAcceleration);
   }

   public void getCoMXYPosition(FramePoint2D comXYPositionToPack)
   {
      comXYPositionToPack.set(this.comXYPosition);
   }


   public FrameVector3DReadOnly getCoMVelocity()
   {
      return comVelocity;
   }

   public FrameVector3DReadOnly getCoMAcceleration()
   {
      return comAcceleration;
   }

   public void setCoMXYPosition(FramePoint3DReadOnly comXYPosition)
   {
      this.comXYPosition.set(comXYPosition);
   }

   public void setCoMVelocity(FrameVector3DReadOnly comVelocity)
   {
      this.comVelocity.set(comVelocity);
   }

   public void setCoMAcceleration(FrameVector3DReadOnly comAcceleration)
   {
      this.comAcceleration.set(comAcceleration);
   }
}
