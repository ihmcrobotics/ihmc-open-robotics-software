package us.ihmc.jMonkeyEngineToolkit.camera;

import java.awt.image.BufferedImage;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface CameraStreamer
{
   public long getTimeStamp();

   public Point3DReadOnly getCameraPosition();

   public QuaternionReadOnly getCameraOrientation();

   public double getFieldOfView();

   public void updateImage(BufferedImage bufferedImage, long timeStamp, Point3DReadOnly cameraPosition, QuaternionReadOnly cameraOrientation, double fov);

   public boolean isReadyForNewData();

}
