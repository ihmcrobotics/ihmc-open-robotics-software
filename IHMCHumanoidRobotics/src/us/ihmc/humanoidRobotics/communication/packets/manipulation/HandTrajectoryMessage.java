package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.communication.ros.generators.RosEnumValueDocumentation;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.AbstractSE3TrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.robotics.robotSide.RobotSide;

@RosMessagePacket(documentation =
      "This message commands the controller to move in taskspace a hand to the desired pose (position & orientation) while going through the specified trajectory points."
      + " A third order polynomial function is used to interpolate positions and a hermite based curve (third order) is used to interpolate the orientations."
      + " To excute a single straight line trajectory to reach a desired hand pose, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.",
      rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
      topic = "/control/hand_trajectory")
public class HandTrajectoryMessage extends AbstractSE3TrajectoryMessage<HandTrajectoryMessage> implements VisualizablePacket
{
   public enum BaseForControl
   {
      @RosEnumValueDocumentation(documentation = "The hand is controlled with respect to the chest. In other words, the controlled hand moves along with the chest.")
      CHEST,
      @RosEnumValueDocumentation(documentation = "The hand is controlled with respect to the estimated world. In other words, the controlled hand will remain fixed in world even if the robot starts moving.")
      WORLD,
      @RosEnumValueDocumentation(documentation = "The hand is controlled with respect to the middle of the feet. In other words, the controlled hand moves along with the robot when walking but is not affected by swaying.")
      WALKING_PATH
   }

   @RosExportedField(documentation = "Specifies which hand will execute the trajectory.")
   public RobotSide robotSide;
   @RosExportedField(documentation = "Specifies whether the pose should be held with respect to the world or the chest. Note that in any case the desired hand pose must be expressed in world frame.")
   public BaseForControl baseForControl;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public HandTrajectoryMessage()
   {
      super();
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public HandTrajectoryMessage(Random random)
   {
      super(random);
      robotSide = RandomNumbers.nextEnum(random, RobotSide.class);
      baseForControl = RandomNumbers.nextEnum(random, BaseForControl.class);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Clone constructor.
    * @param handTrajectoryMessage message to clone.
    */
   public HandTrajectoryMessage(HandTrajectoryMessage handTrajectoryMessage)
   {
      super(handTrajectoryMessage);
      setUniqueId(handTrajectoryMessage.getUniqueId());
      setDestination(handTrajectoryMessage.getDestination());
      robotSide = handTrajectoryMessage.robotSide;
      baseForControl = handTrajectoryMessage.baseForControl;
   }

   /**
    * Use this constructor to execute a straight line trajectory in taskspace. The chest is used as the base for the control.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide is used to define which hand is performing the trajectory.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredPosition desired hand position expressed in world frame.
    * @param desiredOrientation desired hand orientation expressed in world frame.
    */
   public HandTrajectoryMessage(RobotSide robotSide, double trajectoryTime, Point3D desiredPosition, Quaternion desiredOrientation)
   {
      this(robotSide, BaseForControl.CHEST, trajectoryTime, desiredPosition, desiredOrientation);
   }

   /**
    * Use this constructor to execute a straight line trajectory in taskspace.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide is used to define which hand is performing the trajectory.
    * @param base define with respect to what base the hand is controlled.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredPosition desired hand position expressed in world frame.
    * @param desiredOrientation desired hand orientation expressed in world frame.
    */
   public HandTrajectoryMessage(RobotSide robotSide, BaseForControl base, double trajectoryTime, Point3D desiredPosition, Quaternion desiredOrientation)
   {
      super(trajectoryTime, desiredPosition, desiredOrientation);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.robotSide = robotSide;
      this.baseForControl = base;
   }

   /**
    * Use this constructor to build a message with more than one trajectory point.
    * This constructor only allocates memory for the trajectory points, you need to call {@link #setTrajectoryPoint(int, double, Point3D, Quaternion, Vector3D, Vector3D)} for each trajectory point afterwards.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide is used to define which hand is performing the trajectory.
    * @param base define with respect to what base the hand is controlled.
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the controller.
    */
   public HandTrajectoryMessage(RobotSide robotSide, BaseForControl base, int numberOfTrajectoryPoints)
   {
      super(numberOfTrajectoryPoints);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.robotSide = robotSide;
      this.baseForControl = base;
   }

   @Override
   public void set(HandTrajectoryMessage other)
   {
      super.set(other);
      robotSide = other.robotSide;
      baseForControl = other.baseForControl;
      setExecutionMode(other.getExecutionMode(), other.getPreviousMessageId());
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public BaseForControl getBase()
   {
      return baseForControl;
   }

   @Override
   public boolean epsilonEquals(HandTrajectoryMessage other, double epsilon)
   {
      if (robotSide != other.robotSide)
         return false;
      if (baseForControl != other.baseForControl)
         return false;

      return super.epsilonEquals(other, epsilon);
   }

   @Override
   public HandTrajectoryMessage transform(RigidBodyTransform transform)
   {
      HandTrajectoryMessage transformedHandTrajectoryMessage = new HandTrajectoryMessage(this);
      transformedHandTrajectoryMessage.applyTransform(transform);
      return transformedHandTrajectoryMessage;
   }

   @Override
   public String toString()
   {
      String ret = "";
      if (taskspaceTrajectoryPoints != null)
         ret = "Hand SE3 trajectory: number of SE3 trajectory points = " + getNumberOfTrajectoryPoints();
      else
         ret = "Hand SE3 trajectory: no SE3 trajectory points";

      return ret + ", robotSide = " + robotSide + ", base for control = " + baseForControl;
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateHandTrajectoryMessage(this);
   }
}
