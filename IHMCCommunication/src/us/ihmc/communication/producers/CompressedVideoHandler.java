package us.ihmc.communication.producers;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.communication.net.NetStateListener;
import us.ihmc.robotics.robotSide.RobotSide;

public interface CompressedVideoHandler
{
   public abstract void newVideoPacketAvailable(RobotSide robotSide, VideoSource videoSource, long timeStamp, byte[] data, Point3d position, Quat4d orientation, IntrinsicParameters intrinsicParameters);

   public abstract void addNetStateListener(NetStateListener compressedVideoDataServer);
   
   public abstract boolean isConnected();
}
