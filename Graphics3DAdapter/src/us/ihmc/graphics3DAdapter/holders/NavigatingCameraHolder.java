package us.ihmc.graphics3DAdapter.holders;

import us.ihmc.graphics3DAdapter.camera.CameraController;



public class NavigatingCameraHolder
{
   private CameraController navigatingCamera;

   public NavigatingCameraHolder()
   {
   }

   public void setNavigatingCamera(CameraController j3dCameraController)
   {
      this.navigatingCamera = j3dCameraController;
   }

   public CameraController getNavigatingCamera()
   {
      return this.navigatingCamera;
   }
   
   public boolean isThisTheNavigatingCamera(CameraController cameraToCheck)
   {
      return (this.navigatingCamera == cameraToCheck);
   }
   
}
