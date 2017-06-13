package us.ihmc.humanoidRobotics.communication.packets.sensing;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.communication.packets.HighBandwidthPacket;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

@HighBandwidthPacket
public class FisheyePacket extends VideoPacket
{
   public FisheyePacket()
   {
      super();
   }

   public FisheyePacket(VideoSource videoSource, long timeStamp, byte[] data, Point3DReadOnly position, QuaternionReadOnly orientation, IntrinsicParameters intrinsicParameters)
   {
      super(videoSource, timeStamp, data, position, orientation, intrinsicParameters);
   }
}
