package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.DesiredAccelerationsMessage;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;

@RosMessagePacket(documentation = "This message gives the user the option to bypass IHMC feedback controllers for the arm joints by sending desired arm joint accelerations."
      + " One needs experience in control when activating the bypass as it can result in unexpected behaviors for unreasonable accelerations."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/control/arm_desired_joint_accelerations")
public class ArmDesiredAccelerationsMessage extends Packet<ArmDesiredAccelerationsMessage>
{
   @RosExportedField(documentation = "Specifies the side of the robot that will execute the trajectory.")
   public byte robotSide;
   @RosExportedField(documentation = "The desired joint acceleration information.")
   public DesiredAccelerationsMessage desiredAccelerations;

   public ArmDesiredAccelerationsMessage()
   {
      desiredAccelerations = new DesiredAccelerationsMessage();
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public byte getRobotSide()
   {
      return robotSide;
   }

   public void setDesiredAccelerations(DesiredAccelerationsMessage desiredAccelerations)
   {
      this.desiredAccelerations = desiredAccelerations;
   }

   public DesiredAccelerationsMessage getDesiredAccelerations()
   {
      return desiredAccelerations;
   }

   @Override
   public void set(ArmDesiredAccelerationsMessage other)
   {
      desiredAccelerations = new DesiredAccelerationsMessage();
      desiredAccelerations.set(other.desiredAccelerations);
      robotSide = other.robotSide;
      setPacketInformation(other);
   }

   @Override
   public void setUniqueId(long uniqueId)
   {
      super.setUniqueId(uniqueId);
      if (desiredAccelerations != null)
         desiredAccelerations.setUniqueId(uniqueId);
   }

   @Override
   public boolean epsilonEquals(ArmDesiredAccelerationsMessage other, double epsilon)
   {
      if (robotSide != other.robotSide)
         return false;
      return desiredAccelerations.epsilonEquals(other.desiredAccelerations, epsilon);
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateArmDesiredAccelerationsMessage(this);
   }
}
