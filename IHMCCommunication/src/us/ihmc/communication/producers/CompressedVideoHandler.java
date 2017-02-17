package us.ihmc.communication.producers;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.communication.net.NetStateListener;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public interface CompressedVideoHandler
{
   public abstract void newVideoPacketAvailable(VideoSource videoSource, long timeStamp, byte[] data, Point3D position, Quaternion orientation, IntrinsicParameters intrinsicParameters);

   public abstract void addNetStateListener(NetStateListener compressedVideoDataServer);
   
   public abstract boolean isConnected();
}
