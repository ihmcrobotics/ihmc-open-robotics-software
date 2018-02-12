package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.humanoidRobotics.communication.packets.SE3TrajectoryMessage;
import us.ihmc.robotics.robotSide.RobotSide;

@RosMessagePacket(documentation =
"This message commands the controller first to unload if necessary and then to move in taskspace a foot to the desired pose (position & orientation) while going through the specified trajectory points."
      + " A third order polynomial function is used to interpolate positions and a hermite based curve (third order) is used to interpolate the orientations."
      + " To excute a single straight line trajectory to reach a desired foot pose, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.",
      rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
      topic = "/control/foot_trajectory")
public class FootTrajectoryMessage extends Packet<FootTrajectoryMessage>
{
   @RosExportedField(documentation = "Specifies which foot will execute the trajectory.")
   public RobotSide robotSide;
   @RosExportedField(documentation = "The position/orientation trajectory information.")
   public SE3TrajectoryMessage se3Trajectory;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public FootTrajectoryMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public FootTrajectoryMessage(Random random)
   {
      se3Trajectory = new SE3TrajectoryMessage(random);
      robotSide = RandomNumbers.nextEnum(random, RobotSide.class);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Clone constructor.
    * @param footTrajectoryMessage message to clone.
    */
   public FootTrajectoryMessage(FootTrajectoryMessage footTrajectoryMessage)
   {
      se3Trajectory = new SE3TrajectoryMessage(footTrajectoryMessage.se3Trajectory);
      robotSide = footTrajectoryMessage.robotSide;
      setUniqueId(footTrajectoryMessage.getUniqueId());
      setDestination(footTrajectoryMessage.getDestination());
   }

   public FootTrajectoryMessage(RobotSide robotSide, SE3TrajectoryMessage trajectoryMessage)
   {
      se3Trajectory = new SE3TrajectoryMessage(trajectoryMessage);
      this.robotSide = robotSide;
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Use this constructor to execute a straight line trajectory in taskspace. The chest is used as the base for the control.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide is used to define which foot is performing the trajectory.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredPosition desired foot position expressed in world frame.
    * @param desiredOrientation desired foot orientation expressed in world frame.
    */
   public FootTrajectoryMessage(RobotSide robotSide, double trajectoryTime, Point3DReadOnly desiredPosition, QuaternionReadOnly desiredOrientation)
   {
      se3Trajectory = new SE3TrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation, ReferenceFrame.getWorldFrame());
      this.robotSide = robotSide;
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
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
      se3Trajectory = new SE3TrajectoryMessage(numberOfTrajectoryPoints);
      this.robotSide = robotSide;
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public FootTrajectoryMessage(RobotSide robotSide, double trajectoryTime, FramePose3D desiredPose)
   {
      this(robotSide, trajectoryTime, desiredPose.getPosition(), desiredPose.getOrientation());
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public void set(FootTrajectoryMessage other)
   {
      se3Trajectory = new SE3TrajectoryMessage(other.se3Trajectory);
      robotSide = other.robotSide;
      setUniqueId(other.getUniqueId());
      setDestination(other.getDestination());
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public void setSe3Trajectory(SE3TrajectoryMessage se3Trajectory)
   {
      this.se3Trajectory = se3Trajectory;
   }

   public SE3TrajectoryMessage getSe3Trajectory()
   {
      return se3Trajectory;
   }

   @Override
   public void setUniqueId(long uniqueId)
   {
      super.setUniqueId(uniqueId);
      if (se3Trajectory != null)
         se3Trajectory.setUniqueId(uniqueId);
   }

   @Override
   public boolean epsilonEquals(FootTrajectoryMessage other, double epsilon)
   {
      if (robotSide != other.robotSide)
         return false;

      return se3Trajectory.epsilonEquals(other.se3Trajectory, epsilon);
   }

   @Override
   public String toString()
   {
      String ret = "";
      if (se3Trajectory.taskspaceTrajectoryPoints != null)
         ret = "Foot SE3 trajectory: number of SE3 trajectory points = " + se3Trajectory.getNumberOfTrajectoryPoints();
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
}
