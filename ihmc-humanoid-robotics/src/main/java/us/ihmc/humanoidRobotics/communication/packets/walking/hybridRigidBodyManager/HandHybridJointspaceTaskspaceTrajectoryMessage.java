package us.ihmc.humanoidRobotics.communication.packets.walking.hybridRigidBodyManager;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.JointspaceTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.SE3TrajectoryMessage;
import us.ihmc.robotics.robotSide.RobotSide;

@RosMessagePacket(documentation =
      "This message commands the controller to move the chest in both taskspace amd jointspace to the desired orientation and joint angles while going through the specified trajectory points.",
                  rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
                  topic = "/control/hybrid_hand_trajectory")
public class HandHybridJointspaceTaskspaceTrajectoryMessage extends Packet<HandHybridJointspaceTaskspaceTrajectoryMessage>
{
   @RosExportedField(documentation = "Specifies the side of the robot that will execute the trajectory.")
   public RobotSide robotSide;
   @RosExportedField(documentation = "The taskspace trajectory information.")
   public SE3TrajectoryMessage taskspaceTrajectoryMessage;
   @RosExportedField(documentation = "The jointspace trajectory information.")
   public JointspaceTrajectoryMessage jointspaceTrajectoryMessage;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public HandHybridJointspaceTaskspaceTrajectoryMessage()
   {
      super();
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Clone constructor.
    * @param message to clone.
    */
   public HandHybridJointspaceTaskspaceTrajectoryMessage(HandHybridJointspaceTaskspaceTrajectoryMessage hybridJointspaceTaskspaceMessage)
   {
      robotSide = hybridJointspaceTaskspaceMessage.robotSide;
      taskspaceTrajectoryMessage = new SE3TrajectoryMessage(hybridJointspaceTaskspaceMessage.getTaskspaceTrajectoryMessage());
      jointspaceTrajectoryMessage = new JointspaceTrajectoryMessage(hybridJointspaceTaskspaceMessage.jointspaceTrajectoryMessage);
      setUniqueId(hybridJointspaceTaskspaceMessage.getUniqueId());
   }

   /**
    * Typical constructor to use, pack the two taskspace and joint space commands.
    * If these messages conflict, the qp weights and gains will dictate the desireds
    * @param taskspaceTrajectoryMessage
    * @param jointspaceTrajectoryMessage
    */
   public HandHybridJointspaceTaskspaceTrajectoryMessage(RobotSide robotSide, SE3TrajectoryMessage taskspaceTrajectoryMessage, JointspaceTrajectoryMessage jointspaceTrajectoryMessage)
   {
      if (!taskspaceTrajectoryMessage.getQueueingProperties().epsilonEquals(jointspaceTrajectoryMessage.getQueueingProperties(), 0.0))
         throw new IllegalArgumentException("The trajectory messages should have the same queueing properties.");

      this.robotSide = robotSide;
      this.taskspaceTrajectoryMessage = new SE3TrajectoryMessage(taskspaceTrajectoryMessage);
      this.jointspaceTrajectoryMessage = new JointspaceTrajectoryMessage(jointspaceTrajectoryMessage);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public SE3TrajectoryMessage getTaskspaceTrajectoryMessage()
   {
      return taskspaceTrajectoryMessage;
   }

   public void setTaskspaceTrajectoryMessage(SE3TrajectoryMessage taskspaceTrajectoryMessage)
   {
      this.taskspaceTrajectoryMessage = taskspaceTrajectoryMessage;
   }

   public JointspaceTrajectoryMessage getJointspaceTrajectoryMessage()
   {
      return jointspaceTrajectoryMessage;
   }

   public void setJointspaceTrajectoryMessage(JointspaceTrajectoryMessage jointspaceTrajectoryMessage)
   {
      this.jointspaceTrajectoryMessage = jointspaceTrajectoryMessage;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   @Override
   public boolean epsilonEquals(HandHybridJointspaceTaskspaceTrajectoryMessage other, double epsilon)
   {
      if (!taskspaceTrajectoryMessage.epsilonEquals(other.taskspaceTrajectoryMessage, epsilon))
         return false;
      if (!jointspaceTrajectoryMessage.epsilonEquals(other.jointspaceTrajectoryMessage, epsilon))
         return false;
      return true;
   }
}
