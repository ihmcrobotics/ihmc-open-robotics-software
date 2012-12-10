package us.ihmc.graphics3DAdapter.camera;

import javax.media.j3d.Transform3D;

import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;

public interface CameraController extends CameraPropertiesHolder
{

   public void useFreeFlyingCamera();
   public void trackNode(Graphics3DNode node);
   public void attachCameraToNode(Graphics3DNode node, Transform3D offset);
   
   public void reset();

   public void setTrackingOffsets(double dx, double dy, double dz);

   public void setDollyOffsets(double dx, double dy, double dz);

   public void setFixPosition(double fixX, double fixY, double fixZ);

   public void setCameraPosition(double posX, double posY, double posZ);

   public void setTracking(boolean track, boolean trackX, boolean trackY, boolean trackZ);

   public void setDolly(boolean dolly, boolean dollyX, boolean dollyY, boolean dollyZ);

   public CameraTrackAndDollyVariablesHolder getCameraTrackAndDollyVariablesHolder();

  
   public void setConfiguration(CameraConfiguration config, CameraMountList mountList);

   public boolean setCameraKeyPoint(int index);

   public void removeCameraKeyPoint(int index);

   public void nextCameraKeyPoint(int index);

   public void previousCameraKeyPoint(int index);

   public void toggleCameraKeyMode();

   public boolean useKeyCameraPoints();

   public void setKeyFrameTime(int index);

   public void setUseCameraKeyPoints(boolean b);
}
