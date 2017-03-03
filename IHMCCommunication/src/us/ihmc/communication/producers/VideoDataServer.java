package us.ihmc.communication.producers;

import java.awt.image.BufferedImage;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public interface VideoDataServer
{
   public abstract void updateImage(VideoSource videoSource, BufferedImage bufferedImage, long timeStamp, Point3D cameraPosition, Quaternion cameraOrientation, IntrinsicParameters intrinsicParameters);

   public abstract void close();

   public abstract boolean isConnected();
}
