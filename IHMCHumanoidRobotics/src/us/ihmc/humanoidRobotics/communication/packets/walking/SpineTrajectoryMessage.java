package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.AbstractJointspaceTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.OneDoFJointTrajectoryMessage;

public class SpineTrajectoryMessage extends AbstractJointspaceTrajectoryMessage<SpineTrajectoryMessage>
{
   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public SpineTrajectoryMessage()
   {
      super();
   }

   /**
    * Random constructor for unit testing this packet
    * @param random seed
    */
   public SpineTrajectoryMessage(Random random)
   {
      super(random);
   }

   /**
    * Use this constructor to build a message with more than one trajectory points.
    * This constructor only allocates memory for the trajectories, you need to call {@link #setTrajectoryPoint(int, int, double, double, double)} for each joint and trajectory point afterwards.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param numberOfJoints number of joints that will be executing the message.
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the controller.
    */
   public SpineTrajectoryMessage(int numberOfJoints, int numberOfTrajectoryPoints)
   {
      super(numberOfJoints, numberOfTrajectoryPoints);
   }

   /**
    * Use this constructor to build a message with more than one trajectory point.
    * This constructor only allocates memory for the trajectories, you need to call {@link #setTrajectory1DMessage(int, OneDoFJointTrajectoryMessage)} for each joint afterwards.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param numberOfJoints number of joints that will be executing the message.
    */
   public SpineTrajectoryMessage(int numberOfJoints)
   {
      super(numberOfJoints);
   }

   /**
    * Use this constructor to go straight to the given end points.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param jointDesireds desired joint positions. The array length should be equal to the number of joints.
    */
   public SpineTrajectoryMessage(double trajectoryTime, double[] jointDesireds)
   {
      super(trajectoryTime, jointDesireds);
   }
}
