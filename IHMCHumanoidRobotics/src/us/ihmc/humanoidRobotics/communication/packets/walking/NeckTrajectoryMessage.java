package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.AbstractJointspaceTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.OneDoFJointTrajectoryMessage;

@RosMessagePacket(documentation =
      "This message commands the controller to move the neck in jointspace to the desired joint angles while going through the specified trajectory points."
      + " A third order polynomial function is used to interpolate between trajectory points."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.",
                  rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
                  topic = "/control/neck_trajectory")
public class NeckTrajectoryMessage extends AbstractJointspaceTrajectoryMessage<NeckTrajectoryMessage> implements VisualizablePacket
{
   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public NeckTrajectoryMessage()
   {
      super();
   }

   /**
    * Random constructor for unit testing this packet
    * @param random seed
    */
   public NeckTrajectoryMessage(Random random)
   {
      super(random);
   }

   /**
    * Clone constructor.
    * @param neckTrajectoryMessage message to clone.
    */
   public NeckTrajectoryMessage(NeckTrajectoryMessage neckTrajectoryMessage)
   {
      super(neckTrajectoryMessage);
   }

   /**
    * Use this constructor to go straight to the given end points.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredJointPositions desired joint positions. The array length should be equal to the number of joints.
    */
   public NeckTrajectoryMessage(double trajectoryTime, double[] desiredJointPositions)
   {
      super(trajectoryTime, desiredJointPositions);
   }

   /**
    * Create a message using the given joint trajectory points.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param jointTrajectory1DListMessages joint trajectory points to be executed.
    */
   public NeckTrajectoryMessage(OneDoFJointTrajectoryMessage[] jointTrajectory1DListMessages)
   {
      super(jointTrajectory1DListMessages);
   }

   /**
    * Use this constructor to build a message with more than one trajectory point.
    * This constructor only allocates memory for the trajectories, you need to call {@link #setTrajectory1DMessage(int, OneDoFJointTrajectoryMessage)} for each joint afterwards.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param numberOfJoints number of joints that will be executing the message.
    */
   public NeckTrajectoryMessage(int numberOfJoints)
   {
      super(numberOfJoints);
   }

   /**
    * Use this constructor to build a message with more than one trajectory points.
    * This constructor only allocates memory for the trajectories, you need to call {@link #setTrajectoryPoint(int, int, double, double, double)} for each joint and trajectory point afterwards.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param numberOfJoints number of joints that will be executing the message.
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the controller.
    */
   public NeckTrajectoryMessage(int numberOfJoints, int numberOfTrajectoryPoints)
   {
      super(numberOfJoints, numberOfTrajectoryPoints);
   }

}
