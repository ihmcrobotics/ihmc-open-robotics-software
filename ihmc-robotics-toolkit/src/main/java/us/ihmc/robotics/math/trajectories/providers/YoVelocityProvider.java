package us.ihmc.robotics.math.trajectories.providers;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.trajectories.providers.VectorProvider;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;


public class YoVelocityProvider implements VectorProvider
{
   private final YoFrameVector3D frameVector;

   public YoVelocityProvider(YoFrameVector3D frameVector)
   {
      this.frameVector = frameVector;
   }
   
   public YoVelocityProvider(String name, ReferenceFrame referenceFrame, YoRegistry registry)
   {
      this.frameVector = new YoFrameVector3D(name, referenceFrame, registry);
   }

   public void get(FrameVector3D velocityToPack)
   {
      velocityToPack.setIncludingFrame(frameVector);
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
