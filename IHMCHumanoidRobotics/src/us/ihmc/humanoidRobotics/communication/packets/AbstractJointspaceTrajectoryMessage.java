package us.ihmc.humanoidRobotics.communication.packets;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.QueueableMessage;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.OneDoFJointTrajectoryMessage;

public abstract class AbstractJointspaceTrajectoryMessage<T extends AbstractJointspaceTrajectoryMessage<T>> extends QueueableMessage<T>
{
   @RosExportedField(documentation = "List of points in the trajectory.")
   public OneDoFJointTrajectoryMessage[] jointTrajectoryMessages;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public AbstractJointspaceTrajectoryMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Clone constructor.
    * @param neckTrajectoryMessage message to clone.
    */
   public AbstractJointspaceTrajectoryMessage(T trajectoryMessage)
   {
      setUniqueId(trajectoryMessage.getUniqueId());
      setDestination(trajectoryMessage.getDestination());
      jointTrajectoryMessages = new OneDoFJointTrajectoryMessage[trajectoryMessage.getNumberOfJoints()];

      for (int i = 0; i < getNumberOfJoints(); i++)
         jointTrajectoryMessages[i] = new OneDoFJointTrajectoryMessage(trajectoryMessage.jointTrajectoryMessages[i]);

      setExecutionMode(trajectoryMessage.getExecutionMode(), trajectoryMessage.getPreviousMessageId());
   }

   /**
    * Use this constructor to go straight to the given end points.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredJointPositions desired joint positions. The array length should be equal to the number of controlled joints.
    */
   public AbstractJointspaceTrajectoryMessage(double trajectoryTime, double[] desiredJointPositions)
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
   public AbstractJointspaceTrajectoryMessage(double trajectoryTime, double[] desiredJointPositions,  double[] weights)
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
   public AbstractJointspaceTrajectoryMessage(int numberOfJoints)
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
   public AbstractJointspaceTrajectoryMessage(int numberOfJoints, int numberOfTrajectoryPoints)
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
   public AbstractJointspaceTrajectoryMessage(OneDoFJointTrajectoryMessage[] jointTrajectory1DListMessages)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.jointTrajectoryMessages = jointTrajectory1DListMessages;
   }

   public AbstractJointspaceTrajectoryMessage(Random random)
   {
      super(random);
      jointTrajectoryMessages = new OneDoFJointTrajectoryMessage[random.nextInt(10) + 1];
      for (int i = 0; i < getNumberOfJoints(); i++)
         setTrajectory1DMessage(i, new OneDoFJointTrajectoryMessage(random));
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
         trajectoryTime = Math.max(trajectoryTime, jointTrajectoryMessages[i].getLastTrajectoryPoint().time);
      return trajectoryTime;
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
   public boolean epsilonEquals(T other, double epsilon)
   {
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

      return super.epsilonEquals(other, epsilon);
   }
}
