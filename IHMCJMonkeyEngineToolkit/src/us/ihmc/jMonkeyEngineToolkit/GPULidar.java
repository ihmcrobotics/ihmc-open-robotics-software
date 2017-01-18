package us.ihmc.jMonkeyEngineToolkit;

import us.ihmc.robotics.geometry.RigidBodyTransform;

public interface GPULidar
{
   public void setTransformFromWorld(RigidBodyTransform transformToWorld, double time);
   
   public void addGPULidarListener(GPULidarListener listener);
}
