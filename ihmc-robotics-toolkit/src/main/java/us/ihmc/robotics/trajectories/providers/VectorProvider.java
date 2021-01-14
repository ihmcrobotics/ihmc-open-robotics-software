package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;

public interface VectorProvider extends ReferenceFrameHolder
{
   default void get(FrameVector3DBasics frameVectorToPack)
   {
      frameVectorToPack.setReferenceFrame(this.getReferenceFrame());
      get((FixedFrameVector3DBasics) frameVectorToPack);
   }

   default void get(FixedFrameVector3DBasics frameVectorToPack)
   {
      frameVectorToPack.setMatchingFrame(get());
   }

   FrameVector3DReadOnly get();
}
