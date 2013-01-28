package us.ihmc.graphics3DAdapter.camera;

import javax.media.j3d.Transform3D;

public interface CameraMountInterface
{
   public abstract void getTransformToCamera(Transform3D transformToPack);
   public abstract double getFieldOfView();
   public abstract double getClipDistanceNear();
   public abstract double getClipDistanceFar();
   public abstract String getName();
   
   public void zoom(double amount);
}
