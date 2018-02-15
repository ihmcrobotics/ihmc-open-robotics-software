package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.humanoidRobotics.communication.packets.SE3TrajectoryMessage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.frames.CommonReferenceFrameIds;

@RosMessagePacket(documentation =
      "This message commands the controller to move in taskspace a hand to the desired pose (position & orientation) while going through the specified trajectory points."
      + " A third order polynomial function is used to interpolate positions and a hermite based curve (third order) is used to interpolate the orientations."
      + " To excute a single straight line trajectory to reach a desired hand pose, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.",
      rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
      topic = "/control/hand_trajectory")
public class HandTrajectoryMessage extends Packet<HandTrajectoryMessage>
{
   @RosExportedField(documentation = "Specifies which hand will execute the trajectory.")
   public RobotSide robotSide;
   @RosExportedField(documentation = "The position/orientation trajectory information.")
   public SE3TrajectoryMessage se3Trajectory;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public HandTrajectoryMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Clone constructor.
    * @param handTrajectoryMessage message to clone.
    */
   public HandTrajectoryMessage(HandTrajectoryMessage handTrajectoryMessage)
   {
      se3Trajectory = new SE3TrajectoryMessage(handTrajectoryMessage.se3Trajectory);
      robotSide = handTrajectoryMessage.robotSide;
      setUniqueId(handTrajectoryMessage.getUniqueId());
      setDestination(handTrajectoryMessage.getDestination());
   }

   public HandTrajectoryMessage(RobotSide robotSide, SE3TrajectoryMessage trajectoryMessage)
   {
      se3Trajectory = new SE3TrajectoryMessage(trajectoryMessage);
      this.robotSide = robotSide;
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Use this constructor to execute a straight line trajectory in taskspace. The chest is used as the base for the control.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide is used to define which hand is performing the trajectory.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredPosition desired hand position expressed in world frame.
    * @param desiredOrientation desired hand orientation expressed in world frame.
    */
   public HandTrajectoryMessage(RobotSide robotSide, double trajectoryTime, Point3DReadOnly desiredPosition, QuaternionReadOnly desiredOrientation, long trajectoryReferenceFrameId)
   {
      se3Trajectory = new SE3TrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation, trajectoryReferenceFrameId);
      this.robotSide = robotSide;
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Use this constructor to execute a straight line trajectory in taskspace. The chest is used as the base for the control.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide is used to define which hand is performing the trajectory.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredPosition desired hand position expressed in world frame.
    * @param desiredOrientation desired hand orientation expressed in world frame.
    */
   public HandTrajectoryMessage(RobotSide robotSide, double trajectoryTime, Point3D desiredPosition, QuaternionReadOnly desiredOrientation, ReferenceFrame trajectoryReferenceFrame)
   {
      se3Trajectory = new SE3TrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation, trajectoryReferenceFrame);
      this.robotSide = robotSide;
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Use this constructor to build a message with more than one trajectory point.
    * By default this constructor sets the trajectory frame to {@link CommonReferenceFrameIds#CHEST_FRAME} and the data frame to World
    * This constructor only allocates memory for the trajectory points, you need to call {@link #setTrajectoryPoint(int, double, Point3D, QuaternionReadOnly, Vector3D, Vector3D)} for each trajectory point afterwards.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide is used to define which hand is performing the trajectory.
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the controller.
    */
   public HandTrajectoryMessage(RobotSide robotSide, int numberOfTrajectoryPoints)
   {
      se3Trajectory = new SE3TrajectoryMessage(numberOfTrajectoryPoints);
      this.robotSide = robotSide;
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(HandTrajectoryMessage other)
   {
      se3Trajectory = new SE3TrajectoryMessage(other.se3Trajectory);
      robotSide = other.robotSide;
      setPacketInformation(other);
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
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
   public boolean epsilonEquals(HandTrajectoryMessage other, double epsilon)
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
         ret = "Hand SE3 trajectory: number of SE3 trajectory points = " + se3Trajectory.getNumberOfTrajectoryPoints();
      else
         ret = "Hand SE3 trajectory: no SE3 trajectory points";

      return ret + ", robotSide = " + robotSide;
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateHandTrajectoryMessage(this);
   }
}
