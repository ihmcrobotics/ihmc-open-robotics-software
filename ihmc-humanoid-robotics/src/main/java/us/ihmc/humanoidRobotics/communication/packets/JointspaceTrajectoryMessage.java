package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.QueueableMessage;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.OneDoFJointTrajectoryMessage;

@RosMessagePacket(documentation = "", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/control/jointspace_trajectory")
public final class JointspaceTrajectoryMessage extends Packet<JointspaceTrajectoryMessage>
{
   @RosExportedField(documentation = "List of points in the trajectory.")
   public OneDoFJointTrajectoryMessage[] jointTrajectoryMessages;
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
    * @param trajectoryMessage message to clone.
    */
   public JointspaceTrajectoryMessage(JointspaceTrajectoryMessage trajectoryMessage)
   {
      setUniqueId(trajectoryMessage.getUniqueId());
      setDestination(trajectoryMessage.getDestination());
      queueingProperties.set(trajectoryMessage.queueingProperties);
      jointTrajectoryMessages = new OneDoFJointTrajectoryMessage[trajectoryMessage.getNumberOfJoints()];

      for (int i = 0; i < getNumberOfJoints(); i++)
      {
         if(trajectoryMessage.jointTrajectoryMessages[i] != null)
         {
            jointTrajectoryMessages[i] = new OneDoFJointTrajectoryMessage(trajectoryMessage.jointTrajectoryMessages[i]);
         }
      }
   }

   @Override
   public void set(JointspaceTrajectoryMessage other)
   {
      queueingProperties.set(other.queueingProperties);
      jointTrajectoryMessages = new OneDoFJointTrajectoryMessage[other.jointTrajectoryMessages.length];
      for (int i = 0; i < jointTrajectoryMessages.length; i++)
      {
         jointTrajectoryMessages[i] = new OneDoFJointTrajectoryMessage();
         jointTrajectoryMessages[i].set(other.jointTrajectoryMessages[i]);
      }
      setPacketInformation(other);
   }

   /**
    * Set the trajectory points to be executed by this joint.
    * @param jointIndex index of the joint that will go through the trajectory points.
    * @param trajectory1DMessage joint trajectory points to be executed.
    */
   public void setTrajectory1DMessage(int jointIndex, OneDoFJointTrajectoryMessage trajectory1DMessage)
   {
      rangeCheck(jointIndex);
      jointTrajectoryMessages[jointIndex] = trajectory1DMessage;
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
      jointTrajectoryMessages[jointIndex].setTrajectoryPoint(trajectoryPointIndex, time, position, velocity);
   }
   
   public void setQPWeight(int jointIndex, double weight)
   {
      rangeCheck(jointIndex);
      jointTrajectoryMessages[jointIndex].setWeight(weight);
   }

   public int getNumberOfJoints()
   {
      return jointTrajectoryMessages.length;
   }

   public OneDoFJointTrajectoryMessage getJointTrajectoryPointList(int jointIndex)
   {
      rangeCheck(jointIndex);
      return jointTrajectoryMessages[jointIndex];
   }

   public int getNumberOfJointTrajectoryPoints(int jointIndex)
   {
      rangeCheck(jointIndex);
      return jointTrajectoryMessages[jointIndex].getNumberOfTrajectoryPoints();
   }

   public OneDoFJointTrajectoryMessage[] getTrajectoryPointLists()
   {
      return jointTrajectoryMessages;
   }

   public TrajectoryPoint1DMessage getJointTrajectoryPoint(int jointIndex, int trajectoryPointIndex)
   {
      rangeCheck(jointIndex);
      return jointTrajectoryMessages[jointIndex].getTrajectoryPoint(trajectoryPointIndex);
   }

   public void getFinalJointAngles(double[] finalJointAnglesToPack)
   {
      for (int i = 0; i < getNumberOfJoints(); i++)
      {
         finalJointAnglesToPack[i] = jointTrajectoryMessages[i].getLastTrajectoryPoint().position;
      }
   }

   public double getTrajectoryTime()
   {
      double trajectoryTime = 0.0;
      for (int i = 0; i < getNumberOfJoints(); i++)
      {
         OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage = jointTrajectoryMessages[i];
         if(oneDoFJointTrajectoryMessage != null)
         {
            trajectoryTime = Math.max(trajectoryTime, oneDoFJointTrajectoryMessage.getLastTrajectoryPoint().time);
         }
      }
      return trajectoryTime;
   }

   public void setJointTrajectoryMessages(OneDoFJointTrajectoryMessage[] jointTrajectoryMessages)
   {
      this.jointTrajectoryMessages = jointTrajectoryMessages;
   }

   public OneDoFJointTrajectoryMessage[] getJointTrajectoryMessages()
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

      if (jointTrajectoryMessages.length != other.jointTrajectoryMessages.length)
      {
         return false;
      }

      for (int i = 0; i < jointTrajectoryMessages.length; i++)
      {
         if (!jointTrajectoryMessages[i].epsilonEquals(other.jointTrajectoryMessages[i], epsilon))
         {
            return false;
         }
      }

      return true;
   }
}
