package us.ihmc.graphics3DAdapter.camera;

import us.ihmc.utilities.math.geometry.Transform3d;

public interface CameraMountInterface
{
   public abstract void getTransformToCamera(Transform3d transformToPack);
   public abstract double getFieldOfView();
   public abstract double getClipDistanceNear();
   public abstract double getClipDistanceFar();
   public abstract String getName();
   
   public void zoom(double amount);
}
