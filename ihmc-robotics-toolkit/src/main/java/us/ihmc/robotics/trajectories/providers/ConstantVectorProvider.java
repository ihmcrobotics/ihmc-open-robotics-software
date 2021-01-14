package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

@Deprecated
public class ConstantVectorProvider implements VectorProvider
{
   private final FrameVector3D frameVector;

   public ConstantVectorProvider(FrameVector3D frameVector)
   {
      this.frameVector = new FrameVector3D(frameVector);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return frameVector.getReferenceFrame();
   }

   @Override
   public FrameVector3DReadOnly get()
   {
      return frameVector;
   }
}
