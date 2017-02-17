package us.ihmc.jMonkeyEngineToolkit.camera;

import java.awt.image.BufferedImage;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public interface CameraImageListener
{
   public void updateCameraImage(BufferedImage bufferedImage, Point3D cameraPosition, Quaternion cameraOrientation, IntrinsicParameters intrinsicParameters);
}
