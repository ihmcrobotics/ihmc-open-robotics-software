package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;

@RosMessagePacket(documentation = "Request taring of the wrist force/torque sensors.",
      rosPackage = "ihmc_atlas_ros",
      topic = "/control/request_wrist_sensor_calibration")
public class AtlasWristSensorCalibrationRequestPacket extends Packet<AtlasWristSensorCalibrationRequestPacket>
{
   @RosExportedField(documentation = "The robot side (left or right) for the wrist sensor you would like to request calibration for.")
   public byte robotSide;

   public AtlasWristSensorCalibrationRequestPacket()
   {
      setDestination(PacketDestination.CONTROLLER.ordinal());
      // Empty constructor for deserialization
   }

   @Override
   public void set(AtlasWristSensorCalibrationRequestPacket other)
   {
      setPacketInformation(other);
      robotSide = other.robotSide;
   }

   public byte getRobotSide()
   {
      return robotSide;
   }

   @Override
   public boolean equals(Object obj)
   {
      return obj instanceof AtlasWristSensorCalibrationRequestPacket && epsilonEquals((AtlasWristSensorCalibrationRequestPacket) obj, 0);
   }

   @Override
   public boolean epsilonEquals(AtlasWristSensorCalibrationRequestPacket other, double epsilon)
   {
      return robotSide == other.robotSide;
   }
}
