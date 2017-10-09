package us.ihmc.communication.video;

import java.awt.image.BufferedImage;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface VideoCallback
{
   public void onFrame(VideoSource videoSource, BufferedImage bufferedImage, long timestamp, Point3DReadOnly cameraPosition,
                       QuaternionReadOnly cameraOrientation, IntrinsicParameters intrinsicParamaters);
}
