package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.AbstractDesiredAccelerationsMessage;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.robotics.robotSide.RobotSide;

@RosMessagePacket(documentation =
      "This message gives the user the option to bypass IHMC feedback controllers for the arm joints by sending desired arm joint accelerations."
            + " One needs experience in control when activating the bypass as it can result in unexpected behaviors for unreasonable accelerations."
            + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.",
      rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
      topic = "/control/arm_desired_joint_accelerations")
public class ArmDesiredAccelerationsMessage extends Packet<ArmDesiredAccelerationsMessage>
{
   @RosExportedField(documentation = "Specifies the side of the robot that will execute the trajectory.")
   public RobotSide robotSide;
   @RosExportedField(documentation = "The desired joint acceleration information.")
   public AbstractDesiredAccelerationsMessage desiredAccelerations; 

   public ArmDesiredAccelerationsMessage()
   {
      desiredAccelerations = new AbstractDesiredAccelerationsMessage();
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public ArmDesiredAccelerationsMessage(Random random)
   {
      desiredAccelerations = new AbstractDesiredAccelerationsMessage(random);
      robotSide = RandomNumbers.nextEnum(random, RobotSide.class);
   }

   public ArmDesiredAccelerationsMessage(RobotSide robotSide, double[] armDesiredJointAccelerations)
   {
      desiredAccelerations = new AbstractDesiredAccelerationsMessage(armDesiredJointAccelerations);
      this.robotSide = robotSide;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }
   
   public AbstractDesiredAccelerationsMessage getDesiredAccelerations()
   {
      return desiredAccelerations;
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
