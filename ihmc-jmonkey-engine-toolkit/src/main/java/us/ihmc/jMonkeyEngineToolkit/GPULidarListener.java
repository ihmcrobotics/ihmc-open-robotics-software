package us.ihmc.jMonkeyEngineToolkit;

import us.ihmc.euclid.transform.RigidBodyTransform;

public interface GPULidarListener
{
   public void scan(float[] scan, RigidBodyTransform currentTransform, double time);
}
