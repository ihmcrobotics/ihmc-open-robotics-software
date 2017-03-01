package us.ihmc.jMonkeyEngineToolkit;

import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;

public interface GPULidar
{
   public void setTransformFromWorld(AffineTransform transformToWorld, double time);

   public void setTransformFromWorld(RigidBodyTransform transformToWorld, double time);

   public void addGPULidarListener(GPULidarListener listener);
}
