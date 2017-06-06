package us.ihmc.communication.producers;

import java.awt.image.BufferedImage;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.communication.interfaces.Connectable;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface VideoDataServer extends Connectable
{
   public void updateImage(VideoSource videoSource, BufferedImage bufferedImage, long timeStamp, Point3DReadOnly cameraPosition, QuaternionReadOnly cameraOrientation, IntrinsicParameters intrinsicParameters);
}
