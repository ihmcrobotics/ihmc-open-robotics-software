package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.robotics.robotSide.RobotSide;

@RosMessagePacket(documentation = "Request taring of the wrist force/torque sensors.",
      rosPackage = "ihmc_atlas_ros",
      topic = "/control/request_wrist_sensor_calibration")
public class AtlasWristSensorCalibrationRequestPacket extends Packet<AtlasWristSensorCalibrationRequestPacket>
{
   @RosExportedField(documentation = "The robot side (left or right) for the wrist sensor you would like to request calibration for.")
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
