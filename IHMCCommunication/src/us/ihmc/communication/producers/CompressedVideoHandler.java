package us.ihmc.communication.producers;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

public interface CompressedVideoHandler
{
   public abstract void newVideoPacketAvailable(long timeStamp, byte[] data, Point3d position, Quat4d orientation, double fieldOfView);

   public abstract void addNetStateListener(CompressedVideoDataServer compressedVideoDataServer);
   
   public abstract boolean isConnected();
}
