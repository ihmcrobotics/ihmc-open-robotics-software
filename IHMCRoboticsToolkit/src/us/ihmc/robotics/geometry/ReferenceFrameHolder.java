package us.ihmc.robotics.geometry;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public interface ReferenceFrameHolder
{
   public abstract void checkReferenceFrameMatch(ReferenceFrameHolder referenceFrameHolder) throws ReferenceFrameMismatchException;

   public abstract void checkReferenceFrameMatch(ReferenceFrame frame) throws ReferenceFrameMismatchException;

   public abstract ReferenceFrame getReferenceFrame();
}
