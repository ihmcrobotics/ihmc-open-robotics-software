package us.ihmc.robotics.math.trajectories.providers;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFrameVector;
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

   public void get(FrameVector3D velocityToPack)
   {
      frameVector.getFrameTupleIncludingFrame(velocityToPack);
   }

   public void set(FrameVector3D frameVector)
   {
      this.frameVector.set(frameVector);
   }
   
   public void set(Vector3D vector)
   {
      this.frameVector.set(vector);
   }
}
