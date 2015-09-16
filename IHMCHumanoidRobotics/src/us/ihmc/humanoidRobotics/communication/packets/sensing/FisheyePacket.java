package us.ihmc.humanoidRobotics.communication.packets.sensing;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.communication.packets.HighBandwidthPacket;
import us.ihmc.robotics.robotSide.RobotSide;

@HighBandwidthPacket
public class FisheyePacket extends VideoPacket
{
   public FisheyePacket()
   {
      super();
   }

   public FisheyePacket(RobotSide robotSide, long timeStamp, byte[] data, Point3d position, Quat4d orientation, IntrinsicParameters intrinsicParameters)
   {
      super(robotSide, timeStamp, data, position, orientation, intrinsicParameters);
   }
}
