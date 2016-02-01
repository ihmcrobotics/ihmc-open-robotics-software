package us.ihmc.robotics.math.trajectories.providers;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.VectorProvider;


public class YoVelocityProvider implements VectorProvider
{
   private final YoFrameVector frameVector;

   public YoVelocityProvider(YoFrameVector frameVector)
   {
      this.frameVector = frameVector;
   }
   
   public YoVelocityProvider(String name, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      this.frameVector = new YoFrameVector(name, referenceFrame, registry);
   }

   public void get(FrameVector velocityToPack)
   {
      velocityToPack.setIncludingFrame(frameVector.getFrameVectorCopy());
   }

   public void set(FrameVector frameVector)
   {
      this.frameVector.set(frameVector);
   }
   
   public void set(Vector3d vector)
   {
      this.frameVector.set(vector);
   }
}
