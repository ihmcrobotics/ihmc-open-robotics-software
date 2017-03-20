package us.ihmc.robotics.geometry;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public interface FrameObject<T extends FrameObject<T>> extends ReferenceFrameHolder, GeometryObject<T>
{
   public abstract void changeFrame(ReferenceFrame desiredFrame);

   public abstract void changeFrameUsingTransform(ReferenceFrame desiredFrame, Transform transformToNewFrame);

   public abstract void setToZero(ReferenceFrame referenceFrame);

   public abstract void setToNaN(ReferenceFrame referenceFrame);
   
   /**
    * Sets this frame object to zero at the origin of the given reference frame,
    * then changes back to this objects current frame.
    * 
    * @param referenceFrame reference frame to set to
    */
   public default void setFromReferenceFrame(ReferenceFrame referenceFrame)
   {
      ReferenceFrame thisReferenceFrame = getReferenceFrame();
      setToZero(referenceFrame);
      changeFrame(thisReferenceFrame);
   }
}