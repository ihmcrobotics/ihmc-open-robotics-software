package us.ihmc.robotics.trajectories.providers;

import us.ihmc.robotics.geometry.FrameVector;

public class ConstantVectorProvider implements VectorProvider
{
   private final FrameVector frameVector;

   public ConstantVectorProvider(FrameVector frameVector)
   {
      this.frameVector = frameVector;
   }

   public void get(FrameVector frameVectorToPack)
   {
      frameVectorToPack.setIncludingFrame(frameVector);
   }
}
