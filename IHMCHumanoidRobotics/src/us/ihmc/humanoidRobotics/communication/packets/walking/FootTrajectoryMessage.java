package us.ihmc.humanoidRobotics.communication.packets.walking;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.humanoidRobotics.communication.packets.AbstractSE3TrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;

@ClassDocumentation("This message commands the controller first to unload if necessary and then to move in taskspace a foot to the desired pose (position & orientation) while going through the specified trajectory points."
      + " A third order polynomial function is used to interpolate positions and a hermite based curve (third order) is used to interpolate the orientations."
      + " To excute a single straight line trajectory to reach a desired foot pose, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.")
public class FootTrajectoryMessage extends AbstractSE3TrajectoryMessage<FootTrajectoryMessage> implements VisualizablePacket
{
   @FieldDocumentation("Specifies the which foot will execute the trajectory.")
   public RobotSide robotSide;
   @FieldDocumentation("When OVERRIDE is chosen:"
         + "\n - The time of the first trajectory point can be zero, in which case the controller will start directly at the first trajectory point."
         + " Otherwise the controller will prepend a first trajectory point at the current desired position."
         + "\n When QUEUE is chosen:"
         + "\n - The message must carry the ID of the message it should be queued to."
         + "\n - The very first message of a list of queued messages has to be an OVERRIDE message."
         + "\n - The trajectory point times are relative to the the last trajectory point time of the previous message."
         + "\n - The controller will queue the joint trajectory messages as a per joint basis."
         + " The first trajectory point has to be greater than zero.")
   public ExecutionMode executionMode;
   @FieldDocumentation("Only needed when using QUEUE mode, it refers to the message Id to which this message should be queued to."
         + " It is used by the controller to ensure that no message has been lost on the way."
         + " If a message appears to be missing (previousMessageId different from the last message ID received by the controller), the motion is aborted."
         + " If previousMessageId == 0, the controller will not check for the ID of the last received message.")
   public long previousMessageId = INVALID_MESSAGE_ID;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public FootTrajectoryMessage()
   {
      super();
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      executionMode = ExecutionMode.OVERRIDE;
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
      executionMode = footTrajectoryMessage.executionMode;
   }

   /**
    * Use this constructor to execute a straight line trajectory in taskspace. The chest is used as the base for the control.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide is used to define which foot is performing the trajectory.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredPosition desired foot position expressed in world frame.
    * @param desiredOrientation desired foot orientation expressed in world frame.
    */
   public FootTrajectoryMessage(RobotSide robotSide, double trajectoryTime, Point3d desiredPosition, Quat4d desiredOrientation)
   {
      super(trajectoryTime, desiredPosition, desiredOrientation);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.robotSide = robotSide;
      executionMode = ExecutionMode.OVERRIDE;
   }

   /**
    * Use this constructor to build a message with more than one trajectory point.
    * This constructor only allocates memory for the trajectory points, you need to call {@link #setTrajectoryPoint(int, double, Point3d, Quat4d, Vector3d, Vector3d)} for each trajectory point afterwards.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide is used to define which foot is performing the trajectory.
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the controller.
    */
   public FootTrajectoryMessage(RobotSide robotSide, int numberOfTrajectoryPoints)
   {
      super(numberOfTrajectoryPoints);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.robotSide = robotSide;
      executionMode = ExecutionMode.OVERRIDE;
   }

   /**
    * Set how the controller should consume this message:
    * <li> {@link ExecutionMode#OVERRIDE}: this message will override any previous message, including canceling any active execution of a message.
    * <li> {@link ExecutionMode#QUEUE}: this message is queued and will be executed once all the previous messages are done.
    * @param executionMode
    * @param previousMessageId when queuing, one needs to provide the ID of the message this message should be queued to.
    */
   public void setExecutionMode(ExecutionMode executionMode, long previousMessageId)
   {
      this.executionMode = executionMode;
      this.previousMessageId = previousMessageId;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public ExecutionMode getExecutionMode()
   {
      return executionMode;
   }

   public long getPreviousMessageId()
   {
      return previousMessageId;
   }

   @Override
   public boolean epsilonEquals(FootTrajectoryMessage other, double epsilon)
   {
      if (robotSide != other.robotSide)
         return false;
      if (executionMode != other.executionMode)
         return false;

      if (executionMode == ExecutionMode.OVERRIDE && previousMessageId != other.previousMessageId)
         return false;

      return super.epsilonEquals(other, epsilon);
   }

   @Override
   public FootTrajectoryMessage transform(RigidBodyTransform transform)
   {
      FootTrajectoryMessage transformedFootTrajectoryMessage = new FootTrajectoryMessage(this);
      transformedFootTrajectoryMessage.applyTransform(transform);
      return transformedFootTrajectoryMessage;
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
}
