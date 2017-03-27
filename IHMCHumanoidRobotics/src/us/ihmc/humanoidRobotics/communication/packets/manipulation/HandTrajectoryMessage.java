package us.ihmc.humanoidRobotics.communication.packets.manipulation;

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
import us.ihmc.sensorProcessing.frames.CommonReferenceFrameIds;

@RosMessagePacket(documentation =
      "This message commands the controller to move in taskspace a hand to the desired pose (position & orientation) while going through the specified trajectory points."
      + " A third order polynomial function is used to interpolate positions and a hermite based curve (third order) is used to interpolate the orientations."
      + " To excute a single straight line trajectory to reach a desired hand pose, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.",
      rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
      topic = "/control/hand_trajectory")
public class HandTrajectoryMessage extends AbstractSE3TrajectoryMessage<HandTrajectoryMessage> implements VisualizablePacket
{
   @RosExportedField(documentation = "Specifies which hand will execute the trajectory.")
   public RobotSide robotSide;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public HandTrajectoryMessage()
   {
      super();
   }

   public HandTrajectoryMessage(Random random)
   {
      super(random);
      robotSide = RandomNumbers.nextEnum(random, RobotSide.class);
   }

   /**
    * Clone constructor.
    * @param handTrajectoryMessage message to clone.
    */
   public HandTrajectoryMessage(HandTrajectoryMessage handTrajectoryMessage)
   {
      super(handTrajectoryMessage);
      robotSide = handTrajectoryMessage.robotSide;
   }

   /**
    * Use this constructor to execute a straight line trajectory in taskspace. The chest is used as the base for the control.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide is used to define which hand is performing the trajectory.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredPosition desired hand position expressed in world frame.
    * @param desiredOrientation desired hand orientation expressed in world frame.
    */
   public HandTrajectoryMessage(RobotSide robotSide, double trajectoryTime, Point3D desiredPosition, Quaternion desiredOrientation, long expressedInReferenceFrameId, long trajectoryReferenceFrameId)
   {
      super(trajectoryTime, desiredPosition, desiredOrientation, expressedInReferenceFrameId, trajectoryReferenceFrameId);
      this.robotSide = robotSide;
   }
   
   /**
    * Use this constructor to execute a straight line trajectory in taskspace. The chest is used as the base for the control.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide is used to define which hand is performing the trajectory.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredPosition desired hand position expressed in world frame.
    * @param desiredOrientation desired hand orientation expressed in world frame.
    */
   public HandTrajectoryMessage(RobotSide robotSide, double trajectoryTime, Point3D desiredPosition, Quaternion desiredOrientation, ReferenceFrame expressedInReferenceFrame, ReferenceFrame trajectoryReferenceFrame)
   {
      super(trajectoryTime, desiredPosition, desiredOrientation, expressedInReferenceFrame, trajectoryReferenceFrame);
      this.robotSide = robotSide;
   }

   /**
    * Use this constructor to build a message with more than one trajectory point.
    * By default this constructor sets the trajectory frame to {@link CommonReferenceFrameIds#CHEST_FRAME} and the data frame to World
    * This constructor only allocates memory for the trajectory points, you need to call {@link #setTrajectoryPoint(int, double, Point3D, Quaternion, Vector3D, Vector3D)} for each trajectory point afterwards.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide is used to define which hand is performing the trajectory.
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the controller.
    */
   public HandTrajectoryMessage(RobotSide robotSide, int numberOfTrajectoryPoints)
   {
      super(numberOfTrajectoryPoints);
      this.robotSide = robotSide;
      super.setTrajectoryReferenceFrameId(CommonReferenceFrameIds.CHEST_FRAME.getHashId());
      super.setDataReferenceFrameId(ReferenceFrame.getWorldFrame());
      
   }

   @Override
   public void set(HandTrajectoryMessage other)
   {
      super.set(other);
      robotSide = other.robotSide;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   @Override
   public boolean epsilonEquals(HandTrajectoryMessage other, double epsilon)
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
         ret = "Hand SE3 trajectory: number of SE3 trajectory points = " + getNumberOfTrajectoryPoints();
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
