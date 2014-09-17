package us.ihmc.graphics3DAdapter;

import us.ihmc.utilities.math.geometry.Transform3d;

public interface GPULidarListener
{
   public void scan(float[] scan, Transform3d currentTransform, double time);
}
