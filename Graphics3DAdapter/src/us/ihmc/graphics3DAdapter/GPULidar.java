package us.ihmc.graphics3DAdapter;

import us.ihmc.utilities.math.geometry.Transform3d;

public interface GPULidar
{
   public void setTransformFromWorld(Transform3d transformToWorld, double time);
   
   public void addGPULidarListener(GPULidarListener listener);
}
