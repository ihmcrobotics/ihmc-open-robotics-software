package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.AbstractSE3TrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

@RosMessagePacket(documentation =
      "This message commands the controller first to unload if necessary and then to move in taskspace a foot to the desired pose (position & orientation) while going through the specified trajectory points."
            + " A third order polynomial function is used to interpolate positions and a hermite based curve (third order) is used to interpolate the orientations."
            + " To excute a single straight line trajectory to reach a desired foot pose, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time."
            + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.",
      rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
      topic = "/control/foot_trajectory")
public class FootTrajectoryMessage extends AbstractSE3TrajectoryMessage<FootTrajectoryMessage> implements VisualizablePacket
{
   @RosExportedField(documentation = "Specifies the which foot will execute the trajectory.")
   public RobotSide robotSide;
   private static final long WORLD_FRAME_HASH_CODE = ReferenceFrame.getWorldFrame().getNameBasedHashCode();

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public FootTrajectoryMessage()
   {
      super();
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      setExpressedInReferenceFrameId(WORLD_FRAME_HASH_CODE);
      setTrajectoryReferenceFrameId(WORLD_FRAME_HASH_CODE);
   }

   public FootTrajectoryMessage(Random random)
   {
      super(random);
      robotSide = RandomNumbers.nextEnum(random, RobotSide.class);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      setExpressedInReferenceFrameId(WORLD_FRAME_HASH_CODE);
      setTrajectoryReferenceFrameId(WORLD_FRAME_HASH_CODE);
   }

   /**
    * Clone constructor.
    * @param footTrajectoryMessage message to clone.
    */
   public FootTrajectoryMessage(FootTrajectoryMessage footTrajectoryMessage)
   {
      super(footTrajectoryMessage);
      setUniqueId(footTrajectoryMessage.getUniqueId());
      setDestination(footTrajectoryMessage.getDestination());
      robotSide = footTrajectoryMessage.robotSide;
      setExpressedInReferenceFrameId(footTrajectoryMessage.getExpressedInReferenceFrameId());
      setTrajectoryReferenceFrameId(footTrajectoryMessage.getTrajectoryReferenceFrameId());
   }

   /**
    * Use this constructor to execute a straight line trajectory in taskspace. The chest is used as the base for the control.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide is used to define which foot is performing the trajectory.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredPosition desired foot position expressed in world frame.
    * @param desiredOrientation desired foot orientation expressed in world frame.
    */
   public FootTrajectoryMessage(RobotSide robotSide, double trajectoryTime, Point3D desiredPosition, Quaternion desiredOrientation)
   {
      super(trajectoryTime, desiredPosition, desiredOrientation, WORLD_FRAME_HASH_CODE, WORLD_FRAME_HASH_CODE);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.robotSide = robotSide;
      setExpressedInReferenceFrameId(WORLD_FRAME_HASH_CODE);
      setTrajectoryReferenceFrameId(WORLD_FRAME_HASH_CODE);
   }
   
   /**
    * Use this constructor to build a message with more than one trajectory point.
    * This constructor only allocates memory for the trajectory points, you need to call {@link #setTrajectoryPoint(int, double, Point3D, Quaternion, Vector3D, Vector3D)} for each trajectory point afterwards.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide is used to define which foot is performing the trajectory.
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the controller.
    */
   public FootTrajectoryMessage(RobotSide robotSide, int numberOfTrajectoryPoints)
   {
      super(numberOfTrajectoryPoints);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.robotSide = robotSide;
      setExpressedInReferenceFrameId(WORLD_FRAME_HASH_CODE);
      setTrajectoryReferenceFrameId(WORLD_FRAME_HASH_CODE);
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   @Override
   public boolean epsilonEquals(FootTrajectoryMessage other, double epsilon)
   {
      if (robotSide != other.robotSide)
         return false;

      return super.epsilonEquals(other, epsilon);
   }

   @Override
   public String toString()
   {
      String ret = "";
      if (taskspaceTrajectoryPoints != null)
         ret = "Foot SE3 trajectory: number of SE3 trajectory points = " + getNumberOfTrajectoryPoints();
      else
         ret = "Foot SE3 trajectory: no SE3 trajectory points";

      return ret + ", robotSide = " + robotSide + ".";
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateFootTrajectoryMessage(this);
   }
   
   public final void setTrajectoryPoint(int trajectoryPointIndex, double time, Point3D position, Quaternion orientation, Vector3D linearVelocity, Vector3D angularVelocity)
   {
      super.setTrajectoryPoint(trajectoryPointIndex, time, position, orientation, linearVelocity, angularVelocity, WORLD_FRAME_HASH_CODE);
   }
}
