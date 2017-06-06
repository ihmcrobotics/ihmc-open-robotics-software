package us.ihmc.communication.producers;

import java.awt.image.BufferedImage;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface VideoDataServer
{
   public abstract void updateImage(VideoSource videoSource, BufferedImage bufferedImage, long timeStamp, Point3DReadOnly cameraPosition, QuaternionReadOnly cameraOrientation, IntrinsicParameters intrinsicParameters);

   public abstract void close();

   public abstract boolean isConnected();
}
