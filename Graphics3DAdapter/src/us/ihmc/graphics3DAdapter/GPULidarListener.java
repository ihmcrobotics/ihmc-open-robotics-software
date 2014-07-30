package us.ihmc.graphics3DAdapter;

import javax.media.j3d.Transform3D;

public interface GPULidarListener
{
   public void scan(float[] scan, Transform3D currentTransform, double time);
}
