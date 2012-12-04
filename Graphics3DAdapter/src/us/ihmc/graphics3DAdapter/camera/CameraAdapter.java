package us.ihmc.graphics3DAdapter.camera;

import java.awt.Canvas;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;

public interface CameraAdapter
{
   public Canvas getCanvas();
   public void setTransform(Point3d cameraFocalPoint, Quat4d cameraAngleQuaternion);
   public void setFieldOfView(double horizontalFieldOfView);
   public void setAspectRatio(double verticalToHorizontalAspectRatio);
   public void setNearClipPlaneDistance(double nearClipPlaneDistance);
   public void setFarClipPlaneDistance(double farClipPlaneDistance);
   public void destroy();
}
