package us.ihmc.humanoidRobotics.communication.packets.sensing;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.communication.packets.HighBandwidthPacket;
import us.ihmc.communication.producers.VideoSource;

@HighBandwidthPacket
public class FisheyePacket extends VideoPacket
{
   public FisheyePacket()
   {
      super();
   }

   public FisheyePacket(VideoSource videoSource, long timeStamp, byte[] data, Point3d position, Quat4d orientation, IntrinsicParameters intrinsicParameters)
   {
      super(videoSource, timeStamp, data, position, orientation, intrinsicParameters);
   }
}
