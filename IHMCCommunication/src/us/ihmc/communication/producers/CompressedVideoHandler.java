package us.ihmc.communication.producers;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.communication.interfaces.Connectable;
import us.ihmc.communication.net.ConnectionStateListener;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface CompressedVideoHandler extends Connectable
{
   public void newVideoPacketAvailable(VideoSource videoSource, long timeStamp, byte[] data, Point3DReadOnly position, QuaternionReadOnly orientation, IntrinsicParameters intrinsicParameters);

   public void addNetStateListener(ConnectionStateListener compressedVideoDataServer);
}
