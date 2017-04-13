package us.ihmc.robotics.geometry;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * @author Twan koolen
 */
public abstract class AbstractReferenceFrameHolder implements ReferenceFrameHolder
{
   /**
    * Check if the frames hold by {@code this} and referenceFrameHolder match.
    *
    * @param referenceFrameHolder ReferenceFrameHolder
    * @throws ReferenceFrameMismatchException
    */
   @Override
   public final void checkReferenceFrameMatch(ReferenceFrameHolder referenceFrameHolder) throws ReferenceFrameMismatchException
   {
      checkReferenceFrameMatch(referenceFrameHolder.getReferenceFrame());
   }

   /**
    * Check if frame matches the ReferenceFrame of this referenceFrameHolder.
    *
    * @param frame ReferenceFrame
    * @throws ReferenceFrameMismatchException
    */
   @Override
   public final void checkReferenceFrameMatch(ReferenceFrame frame) throws ReferenceFrameMismatchException
   {
      getReferenceFrame().checkReferenceFrameMatch(frame);
   }

   public abstract ReferenceFrame getReferenceFrame();
}
