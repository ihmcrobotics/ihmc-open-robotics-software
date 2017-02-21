package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.AbstractSO3TrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;

@RosMessagePacket(documentation =
      "This message commands the controller to move in taskspace the head to the desired orientation while going through the specified trajectory points."
      + " A hermite based curve (third order) is used to interpolate the orientations."
      + " To excute a simple trajectory to reach a desired head orientation, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.",
                  rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
                  topic = "/control/head_trajectory")
public class HeadTrajectoryMessage extends AbstractSO3TrajectoryMessage<HeadTrajectoryMessage> implements VisualizablePacket
{
   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public HeadTrajectoryMessage()
   {
      super();
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public HeadTrajectoryMessage(Random random)
   {
      super(random);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Clone constructor.
    * @param headTrajectoryMessage message to clone.
    */
   public HeadTrajectoryMessage(HeadTrajectoryMessage headTrajectoryMessage)
   {
      super(headTrajectoryMessage);
      setUniqueId(headTrajectoryMessage.getUniqueId());
      setDestination(headTrajectoryMessage.getDestination());
   }

   /**
    * Use this constructor to execute a simple interpolation in taskspace to the desired orientation.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param trajectoryTime how long it takes to reach the desired orientation.
    * @param desiredOrientation desired head orientation expressed in world frame.
    */
   public HeadTrajectoryMessage(double trajectoryTime, Quaternion desiredOrientation)
   {
      super(trajectoryTime, desiredOrientation);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Use this constructor to build a message with more than one trajectory point.
    * This constructor only allocates memory for the trajectory points, you need to call {@link #setTrajectoryPoint(int, double, Quaternion, Vector3D)} for each trajectory point afterwards.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the controller.
    */
   public HeadTrajectoryMessage(int numberOfTrajectoryPoints)
   {
      super(numberOfTrajectoryPoints);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public boolean epsilonEquals(HeadTrajectoryMessage other, double epsilon)
   {
      return super.epsilonEquals(other, epsilon);
   }

   @Override
   public HeadTrajectoryMessage transform(RigidBodyTransform transform)
   {
      HeadTrajectoryMessage transformedHeadTrajectoryMessage = new HeadTrajectoryMessage(this);
      transformedHeadTrajectoryMessage.applyTransform(transform);
      return transformedHeadTrajectoryMessage;
   }

   @Override
   public String toString()
   {
      if (taskspaceTrajectoryPoints != null)
         return "Head SO3 trajectory: number of SO3 trajectory points = " + getNumberOfTrajectoryPoints();
      else
         return "Head SO3 trajectory: no SO3 trajectory points";
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateHeadTrajectoryMessage(this);
   }
}
