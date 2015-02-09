package us.ihmc.communication.producers;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.net.NetStateListener;

public interface CompressedVideoHandler
{
   public abstract void newVideoPacketAvailable(long timeStamp, byte[] data, Point3d position, Quat4d orientation, double fieldOfView);

   public abstract void addNetStateListener(NetStateListener compressedVideoDataServer);
   
   public abstract boolean isConnected();
}
