package us.ihmc.graphics3DAdapter;

import us.ihmc.utilities.math.geometry.RigidBodyTransform;

public interface GPULidar
{
   public void setTransformFromWorld(RigidBodyTransform transformToWorld, double time);
   
   public void addGPULidarListener(GPULidarListener listener);
}
