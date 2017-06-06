package us.ihmc.communication.producers;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.communication.net.ConnectionStateListener;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface CompressedVideoHandler
{
   public abstract void newVideoPacketAvailable(VideoSource videoSource, long timeStamp, byte[] data, Point3DReadOnly position, QuaternionReadOnly orientation, IntrinsicParameters intrinsicParameters);

   public abstract void addNetStateListener(ConnectionStateListener compressedVideoDataServer);
   
   public abstract boolean isConnected();
}
