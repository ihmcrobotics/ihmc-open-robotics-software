package us.ihmc.graphics3DAdapter.camera;

import java.awt.image.BufferedImage;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

public interface CameraStreamer
{

   public void updateImage(BufferedImage bufferedImage, Point3d cameraLocation, Quat4d cameraOrientation, float fov);

}
