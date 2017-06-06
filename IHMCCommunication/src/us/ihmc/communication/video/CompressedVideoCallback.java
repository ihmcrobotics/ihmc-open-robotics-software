package us.ihmc.communication.video;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface CompressedVideoCallback
{
   public void onFrame(VideoSource videoSource, byte[] compressedImageData, long timestamp, Point3DReadOnly cameraPosition,
                       QuaternionReadOnly cameraOrientation, IntrinsicParameters intrinsicParamaters);
}
