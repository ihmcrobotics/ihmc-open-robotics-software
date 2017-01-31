package us.ihmc.jMonkeyEngineToolkit.camera;

public interface TrackingDollyCameraController extends CameraController, CameraPropertiesHolder
{
   public void reset();

   public void setTrackingOffsets(double dx, double dy, double dz);

   public void setDollyOffsets(double dx, double dy, double dz);

   public void setFixPosition(double fixX, double fixY, double fixZ);

   public void setCameraPosition(double posX, double posY, double posZ);

   public void setTracking(boolean track, boolean trackX, boolean trackY, boolean trackZ);

   public void setDolly(boolean dolly, boolean dollyX, boolean dollyY, boolean dollyZ);

   public CameraTrackingAndDollyPositionHolder getCameraTrackAndDollyVariablesHolder();

   public void setConfiguration(CameraConfiguration config, CameraMountList mountList);

   public boolean setCameraKeyPoint(int index);

   public void removeCameraKeyPoint(int index);

   public void nextCameraKeyPoint(int index);

   public void previousCameraKeyPoint(int index);

   public void toggleCameraKeyMode();

   public boolean useKeyCameraPoints();

   public void setKeyFrameTime(int index);

   public void setUseCameraKeyPoints(boolean b);
   
   public void copyPositionTrackingDollyConfiguration(TrackingDollyCameraController otherCamera);
}
