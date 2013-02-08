package us.ihmc.graphics3DAdapter.camera;

import java.awt.image.BufferedImage;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

public interface CameraStreamer
{
   
   public Point3d getCameraPosition();
   public Quat4d getCameraOrientation();
   public double getFieldOfView();
   public void updateImage(BufferedImage bufferedImage, Point3d cameraPosition, Quat4d cameraOrientation, double fov);
   public boolean isReadyForNewData();

}
