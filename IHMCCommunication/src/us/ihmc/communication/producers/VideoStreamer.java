package us.ihmc.communication.producers;

import java.awt.image.BufferedImage;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public interface VideoStreamer
{
   public void updateImage(BufferedImage bufferedImage, Point3D cameraPosition, Quaternion cameraOrientation, IntrinsicParameters intrinsicParamaters);
}
