package us.ihmc.commonWalkingControlModules.heightPlanning;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

public class CoMXYTimeDerivativesData
{
   private FramePoint2D comPosition = new FramePoint2D(ReferenceFrame.getWorldFrame());
   private FrameVector2D comVelocity = new FrameVector2D();
   private FrameVector2D comAcceleration = new FrameVector2D();

   public void set(CoMXYTimeDerivativesData comXYData)
   {
      setCoMPosition(comXYData.getCoMPosition());
      setCoMVelocity(comXYData.getCoMVelocity());
      setCoMAcceleration(comXYData.getCoMAcceleration());
   }

   public FramePoint2DReadOnly getCoMPosition()
   {
      return comPosition;
   }


   public FrameVector2DReadOnly getCoMVelocity()
   {
      return comVelocity;
   }

   public FrameVector2DReadOnly getCoMAcceleration()
   {
      return comAcceleration;
   }

   public void setCoMPosition(FramePoint2DReadOnly comXYPosition)
   {
      this.comPosition.set(comXYPosition);
   }

   public void setCoMVelocity(FrameVector2DReadOnly comVelocity)
   {
      this.comVelocity.set(comVelocity);
   }

   public void setCoMAcceleration(FrameVector2DReadOnly comAcceleration)
   {
      this.comAcceleration.set(comAcceleration);
   }
}
