package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
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
public class ArmDesiredAccelerationsMessage extends AbstractDesiredAccelerationsMessage<ArmDesiredAccelerationsMessage>
{
   @RosExportedField(documentation = "Specifies the side of the robot that will execute the trajectory.")
   public RobotSide robotSide;

   public ArmDesiredAccelerationsMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public ArmDesiredAccelerationsMessage(Random random)
   {
      super(random);
      robotSide = RandomNumbers.nextEnum(random, RobotSide.class);
   }

   public ArmDesiredAccelerationsMessage(RobotSide robotSide, double[] armDesiredJointAccelerations)
   {
      super(armDesiredJointAccelerations);
      this.robotSide = robotSide;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   @Override
   public boolean epsilonEquals(ArmDesiredAccelerationsMessage other, double epsilon)
   {
      if (robotSide != other.robotSide)
         return false;
      return super.epsilonEquals(other, epsilon);
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateArmDesiredAccelerationsMessage(this);
   }
}
