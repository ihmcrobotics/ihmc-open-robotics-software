package us.ihmc.graphics3DAdapter.camera;


import javax.media.j3d.Transform3D;


public interface CameraController
{
   public void computeTransform(Transform3D cameraTransform);
   public double getFieldOfViewInRadians();
   public double getClipNear();
   public double getClipFar();
}
