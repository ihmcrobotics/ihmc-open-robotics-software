package us.ihmc.jMonkeyEngineToolkit.camera;

import java.awt.image.BufferedImage;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

public interface CameraStreamer
{
   public long getTimeStamp();
   public Point3d getCameraPosition();
   public Quat4d getCameraOrientation();
   public double getFieldOfView();
   public void updateImage(BufferedImage bufferedImage, long timeStamp, Point3d cameraPosition, Quat4d cameraOrientation, double fov);
   public boolean isReadyForNewData();

}
