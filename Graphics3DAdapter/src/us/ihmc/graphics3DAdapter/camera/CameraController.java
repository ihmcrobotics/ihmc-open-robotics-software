package us.ihmc.graphics3DAdapter.camera;


import us.ihmc.utilities.math.geometry.Transform3d;


public interface CameraController
{
   public void computeTransform(Transform3d cameraTransform);
   public double getHorizontalFieldOfViewInRadians();
   public double getClipNear();
   public double getClipFar();
}
