package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.FrameVector3D;

public class ConstantVectorProvider implements VectorProvider
{
   private final FrameVector3D frameVector;

   public ConstantVectorProvider(FrameVector3D frameVector)
   {
      this.frameVector = frameVector;
   }

   public void get(FrameVector3D frameVectorToPack)
   {
      frameVectorToPack.setIncludingFrame(frameVector);
   }
}
