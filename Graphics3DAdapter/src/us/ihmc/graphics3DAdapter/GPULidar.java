package us.ihmc.graphics3DAdapter;

import javax.media.j3d.Transform3D;

public interface GPULidar
{
   public void setTransformFromWorld(Transform3D transformToWorld, double time);
   
   public void addGPULidarListener(GPULidarListener listener);
}
