package us.ihmc.communication.packets.manipulation;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Random;

@ClassDocumentation("Request taring of the wrist force/torque sensors.")
public class AtlasWristSensorCalibrationRequestPacket extends IHMCRosApiPacket<AtlasWristSensorCalibrationRequestPacket>
{
   public RobotSide robotSide;

   public AtlasWristSensorCalibrationRequestPacket()
   {
      setDestination(PacketDestination.CONTROLLER.ordinal());
      // Empty constructor for deserialization
   }

   public AtlasWristSensorCalibrationRequestPacket(RobotSide robotSide)
   {
      setDestination(PacketDestination.CONTROLLER.ordinal());
      this.robotSide = robotSide;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public boolean equals(Object obj)
   {
      return ((obj instanceof AtlasWristSensorCalibrationRequestPacket) && this.epsilonEquals((AtlasWristSensorCalibrationRequestPacket) obj, 0));
   }

   @Override
   public boolean epsilonEquals(AtlasWristSensorCalibrationRequestPacket other, double epsilon)
   {
      boolean ret = this.getRobotSide().equals(other.getRobotSide());

      return ret;
   }

   public AtlasWristSensorCalibrationRequestPacket(Random random)
   {
      this(random.nextBoolean() ? RobotSide.LEFT : RobotSide.RIGHT);

   }
}
