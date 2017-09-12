package us.ihmc.jMonkeyEngineToolkit.holders;

import us.ihmc.jMonkeyEngineToolkit.camera.CameraController;



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
