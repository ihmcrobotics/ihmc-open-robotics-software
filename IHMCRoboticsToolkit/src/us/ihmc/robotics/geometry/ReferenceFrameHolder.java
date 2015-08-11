package us.ihmc.robotics.geometry;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2008</p>
 *
 * <p>Company: </p>
 *
 * @author Twan koolen
 * @version 1.0
 */
public abstract class ReferenceFrameHolder
{
   /**
    * Check if the frames hold by {@code this} and referenceFrameHolder match.
    *
    * @param referenceFrameHolder ReferenceFrameHolder
    * @throws ReferenceFrameMismatchException
    */
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
   public final void checkReferenceFrameMatch(ReferenceFrame frame) throws ReferenceFrameMismatchException
   {
      getReferenceFrame().checkReferenceFrameMatch(frame);
   }

   public abstract ReferenceFrame getReferenceFrame();
}
