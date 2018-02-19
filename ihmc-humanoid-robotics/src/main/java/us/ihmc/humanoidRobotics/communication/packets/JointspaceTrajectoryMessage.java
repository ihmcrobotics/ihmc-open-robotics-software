package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.QueueableMessage;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.OneDoFJointTrajectoryMessage;
import us.ihmc.idl.PreallocatedList;

@RosMessagePacket(documentation = "", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/control/jointspace_trajectory")
public final class JointspaceTrajectoryMessage extends Packet<JointspaceTrajectoryMessage>
{
   @RosExportedField(documentation = "List of points in the trajectory.")
   public PreallocatedList<OneDoFJointTrajectoryMessage> jointTrajectoryMessages = new PreallocatedList<>(OneDoFJointTrajectoryMessage.class, OneDoFJointTrajectoryMessage::new, 10);
   @RosExportedField(documentation = "Properties for queueing trajectories.")
   public QueueableMessage queueingProperties = new QueueableMessage();

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public JointspaceTrajectoryMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Clone constructor.
    * @param other message to clone.
    */
   public JointspaceTrajectoryMessage(JointspaceTrajectoryMessage other)
   {
      setUniqueId(other.getUniqueId());
      setDestination(other.getDestination());
      queueingProperties.set(other.queueingProperties);
      MessageTools.copyData(other.jointTrajectoryMessages, jointTrajectoryMessages);
   }

   @Override
   public void set(JointspaceTrajectoryMessage other)
   {
      queueingProperties.set(other.queueingProperties);
      MessageTools.copyData(other.jointTrajectoryMessages, jointTrajectoryMessages);
      setPacketInformation(other);
   }

   /**
    * Set the trajectory points to be executed by this joint.
    * @param jointIndex index of the joint that will go through the trajectory points.
    * @param oneDoFJointTrajectoryMessage joint trajectory points to be executed.
    */
   public void setTrajectory1DMessage(int jointIndex, OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage)
   {
      rangeCheck(jointIndex);
      jointTrajectoryMessages.get(jointIndex).set(oneDoFJointTrajectoryMessage);
   }

   /**
    * Create a trajectory point.
    * @param jointIndex index of the joint that will go through the trajectory point.
    * @param trajectoryPointIndex index of the trajectory point to create.
    * @param time time at which the trajectory point has to be reached. The time is relative to when the trajectory starts.
    * @param position define the desired 1D position to be reached at this trajectory point.
    * @param velocity define the desired 1D velocity to be reached at this trajectory point.
    */
   public void setTrajectoryPoint(int jointIndex, int trajectoryPointIndex, double time, double position, double velocity)
   {
      rangeCheck(jointIndex);
      jointTrajectoryMessages.get(jointIndex).setTrajectoryPoint(trajectoryPointIndex, time, position, velocity);
   }
   
   public void setQPWeight(int jointIndex, double weight)
   {
      rangeCheck(jointIndex);
      jointTrajectoryMessages.get(jointIndex).setWeight(weight);
   }

   public int getNumberOfJoints()
   {
      return jointTrajectoryMessages.size();
   }

   public OneDoFJointTrajectoryMessage getJointTrajectoryPointList(int jointIndex)
   {
      rangeCheck(jointIndex);
      return jointTrajectoryMessages.get(jointIndex);
   }

   public int getNumberOfJointTrajectoryPoints(int jointIndex)
   {
      rangeCheck(jointIndex);
      return jointTrajectoryMessages.get(jointIndex).getNumberOfTrajectoryPoints();
   }

   public PreallocatedList<OneDoFJointTrajectoryMessage> getTrajectoryPointLists()
   {
      return jointTrajectoryMessages;
   }

   public TrajectoryPoint1DMessage getJointTrajectoryPoint(int jointIndex, int trajectoryPointIndex)
   {
      rangeCheck(jointIndex);
      return jointTrajectoryMessages.get(jointIndex).getTrajectoryPoint(trajectoryPointIndex);
   }

   public void getFinalJointAngles(double[] finalJointAnglesToPack)
   {
      for (int i = 0; i < getNumberOfJoints(); i++)
      {
         finalJointAnglesToPack[i] = jointTrajectoryMessages.get(i).getLastTrajectoryPoint().position;
      }
   }

   public double getTrajectoryTime()
   {
      double trajectoryTime = 0.0;
      for (int i = 0; i < getNumberOfJoints(); i++)
      {
         OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage = jointTrajectoryMessages.get(i);
         if(oneDoFJointTrajectoryMessage != null)
         {
            trajectoryTime = Math.max(trajectoryTime, oneDoFJointTrajectoryMessage.getLastTrajectoryPoint().time);
         }
      }
      return trajectoryTime;
   }

   public void setJointTrajectoryMessages(OneDoFJointTrajectoryMessage[] jointTrajectoryMessages)
   {
      MessageTools.copyData(jointTrajectoryMessages, this.jointTrajectoryMessages);
   }

   public PreallocatedList<OneDoFJointTrajectoryMessage> getJointTrajectoryMessages()
   {
      return jointTrajectoryMessages;
   }

   public void setQueueingProperties(QueueableMessage queueingProperties)
   {
      this.queueingProperties = queueingProperties;
   }

   public QueueableMessage getQueueingProperties()
   {
      return queueingProperties;
   }

   private void rangeCheck(int jointIndex)
   {
      if (jointIndex >= getNumberOfJoints() || jointIndex < 0)
         throw new IndexOutOfBoundsException("Joint index: " + jointIndex + ", number of joints: " + getNumberOfJoints());
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateJointspaceTrajectoryMessage(this);
   }

   @Override
   public boolean epsilonEquals(JointspaceTrajectoryMessage other, double epsilon)
   {
      if (!queueingProperties.epsilonEquals(other.queueingProperties, epsilon))
         return false;

      if (jointTrajectoryMessages.size() != other.jointTrajectoryMessages.size())
      {
         return false;
      }

      for (int i = 0; i < jointTrajectoryMessages.size(); i++)
      {
         if (!jointTrajectoryMessages.get(i).epsilonEquals(other.jointTrajectoryMessages.get(i), epsilon))
         {
            return false;
         }
      }

      return true;
   }
}
