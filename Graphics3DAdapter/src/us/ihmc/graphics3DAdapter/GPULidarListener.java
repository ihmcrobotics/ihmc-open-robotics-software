package us.ihmc.graphics3DAdapter;

import us.ihmc.robotics.geometry.RigidBodyTransform;

public interface GPULidarListener
{
   public void scan(float[] scan, RigidBodyTransform currentTransform, double time);
}
