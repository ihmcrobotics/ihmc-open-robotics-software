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

   /**
    * Use this constructor to go straight to the given end points.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredJointPositions desired joint positions. The array length should be equal to the number of controlled joints.
    */
   public JointspaceTrajectoryMessage(double trajectoryTime, double[] desiredJointPositions)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      jointTrajectoryMessages = new OneDoFJointTrajectoryMessage[desiredJointPositions.length];
      for (int jointIndex = 0; jointIndex < getNumberOfJoints(); jointIndex++)
         jointTrajectoryMessages[jointIndex] = new OneDoFJointTrajectoryMessage(trajectoryTime, desiredJointPositions[jointIndex]);
   }
   
   /**
    * Use this constructor to go straight to the given end points using the specified qp weights.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredJointPositions desired joint positions. The array length should be equal to the number of controlled joints.
    * @param weights the qp weights for the joint accelerations
    */
   public JointspaceTrajectoryMessage(double trajectoryTime, double[] desiredJointPositions,  double[] weights)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      jointTrajectoryMessages = new OneDoFJointTrajectoryMessage[desiredJointPositions.length];
      for (int jointIndex = 0; jointIndex < getNumberOfJoints(); jointIndex++)
      {
         OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage = new OneDoFJointTrajectoryMessage(trajectoryTime, desiredJointPositions[jointIndex]);
         oneDoFJointTrajectoryMessage.setWeight(weights[jointIndex]);
         jointTrajectoryMessages[jointIndex] = oneDoFJointTrajectoryMessage;
      }
         
   }

   /**
    * Use this constructor to build a message with more than one trajectory point.
    * This constructor only allocates memory for the trajectories, you need to call {@link #setTrajectory1DMessage(int, OneDoFJointTrajectoryMessage)} for each joint afterwards.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param numberOfJoints number of joints that will be executing the message.
    */
   public JointspaceTrajectoryMessage(int numberOfJoints)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      jointTrajectoryMessages = new OneDoFJointTrajectoryMessage[numberOfJoints];
   }

   /**
    * Use this constructor to build a message with more than one trajectory points.
    * This constructor only allocates memory for the trajectories, you need to call {@link #setTrajectoryPoint(int, int, double, double, double)} for each joint and trajectory point afterwards.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param numberOfJoints number of joints that will be executing the message.
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the controller.
    */
   public JointspaceTrajectoryMessage(int numberOfJoints, int numberOfTrajectoryPoints)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      jointTrajectoryMessages = new OneDoFJointTrajectoryMessage[numberOfJoints];
      for (int i = 0; i < numberOfJoints; i++)
         jointTrajectoryMessages[i] = new OneDoFJointTrajectoryMessage(numberOfTrajectoryPoints);
   }

   /**
    * Create a message using the given joint trajectory points.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param jointTrajectory1DListMessages joint trajectory points to be executed.
    */
   public JointspaceTrajectoryMessage(OneDoFJointTrajectoryMessage[] jointTrajectory1DListMessages)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.jointTrajectoryMessages = jointTrajectory1DListMessages;
   }

   public void set(JointspaceTrajectoryMessage other)
   {
      queueingProperties.set(other.queueingProperties);
      jointTrajectoryMessages = new OneDoFJointTrajectoryMessage[other.jointTrajectoryMessages.length];
      for (int i = 0; i < jointTrajectoryMessages.length; i++)
      {
         jointTrajectoryMessages[i] = new OneDoFJointTrajectoryMessage();
         jointTrajectoryMessages[i].set(other.jointTrajectoryMessages[i]);
      }
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
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

      if (this.jointTrajectoryMessages.length != other.jointTrajectoryMessages.length)
      {
         return false;
      }

      for (int i = 0; i < this.jointTrajectoryMessages.length; i++)
      {
         if (!this.jointTrajectoryMessages[i].epsilonEquals(other.jointTrajectoryMessages[i], epsilon))
         {
            return false;
         }
      }

      return true;
   }
}
