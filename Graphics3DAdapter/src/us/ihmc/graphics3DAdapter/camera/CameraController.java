package us.ihmc.graphics3DAdapter.camera;

import us.ihmc.utilities.math.geometry.RigidBodyTransform;

public interface CameraController
{
   public void computeTransform(RigidBodyTransform cameraTransform);
   
   public double getHorizontalFieldOfViewInRadians();
   
   public double getClipNear();
   
   public double getClipFar();
   
   public void closeAndDispose();
}
