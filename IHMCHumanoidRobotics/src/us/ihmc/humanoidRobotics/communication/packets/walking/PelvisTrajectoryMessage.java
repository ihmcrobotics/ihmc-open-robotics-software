package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.AbstractSE3TrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

@RosMessagePacket(documentation =
      "This message commands the controller to move in taskspace the pelvis to the desired pose (position & orientation) while going through the specified trajectory points."
      + " A third order polynomial function is used to interpolate positions and a hermite based curve (third order) is used to interpolate the orientations."
      + " To excute a single straight line trajectory to reach a desired pelvis pose, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time."
      + " Note that the pelvis position is limited keep the robot's balance (center of mass has to remain inside the support polygon)."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.",
                  rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
                  topic = "/control/pelvis_trajectory")
public class PelvisTrajectoryMessage extends AbstractSE3TrajectoryMessage<PelvisTrajectoryMessage> implements VisualizablePacket
{
   private static final long WORLD_FRAME_HASH_CODE = ReferenceFrame.getWorldFrame().getNameBasedHashCode();

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public PelvisTrajectoryMessage()
   {
      super();
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      setDataReferenceFrameId(WORLD_FRAME_HASH_CODE);
      setTrajectoryReferenceFrameId(WORLD_FRAME_HASH_CODE);
   }

   public PelvisTrajectoryMessage(Random random)
   {
      super(random);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      setDataReferenceFrameId(WORLD_FRAME_HASH_CODE);
      setTrajectoryReferenceFrameId(WORLD_FRAME_HASH_CODE);
   }

   /**
    * Clone constructor.
    * @param pelvisTrajectoryMessage message to clone.
    */
   public PelvisTrajectoryMessage(PelvisTrajectoryMessage pelvisTrajectoryMessage)
   {
      super(pelvisTrajectoryMessage);
      setUniqueId(pelvisTrajectoryMessage.getUniqueId());
      setDestination(pelvisTrajectoryMessage.getDestination());
      setDataReferenceFrameId(pelvisTrajectoryMessage.getDataReferenceFrameId());
      setTrajectoryReferenceFrameId(pelvisTrajectoryMessage.getTrajectoryReferenceFrameId());
   }

   /**
    * Use this constructor to execute a straight line trajectory in taskspace.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredPosition desired pelvis position expressed in world frame.
    * @param desiredOrientation desired pelvis orientation expressed in world frame.
    */
   public PelvisTrajectoryMessage(double trajectoryTime, Point3D desiredPosition, Quaternion desiredOrientation)
   {
      super(trajectoryTime, desiredPosition, desiredOrientation, WORLD_FRAME_HASH_CODE, WORLD_FRAME_HASH_CODE);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      setDataReferenceFrameId(WORLD_FRAME_HASH_CODE);
      setTrajectoryReferenceFrameId(WORLD_FRAME_HASH_CODE);
   }

   /**
    * Use this constructor to build a message with more than one trajectory point.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * This constructor only allocates memory for the trajectory points, you need to call {@link #setTrajectoryPoint(int, double, Point3D, Quaternion, Vector3D, Vector3D)} for each trajectory point afterwards.
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the controller.
    */
   public PelvisTrajectoryMessage(int numberOfTrajectoryPoints)
   {
      super(numberOfTrajectoryPoints);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      setDataReferenceFrameId(WORLD_FRAME_HASH_CODE);
      setTrajectoryReferenceFrameId(WORLD_FRAME_HASH_CODE);
   }

   @Override
   public boolean epsilonEquals(PelvisTrajectoryMessage other, double epsilon)
   {
      return super.epsilonEquals(other, epsilon);
   }

   @Override
   public String toString()
   {
      if (taskspaceTrajectoryPoints != null)
         return "Pelvis SE3 trajectory: number of SE3 trajectory points = " + getNumberOfTrajectoryPoints();
      else
         return "Pelvis SE3 trajectory: no SE3 trajectory points";
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validatePelvisTrajectoryMessage(this);
   }
   
   public final void setTrajectoryPoint(int trajectoryPointIndex, double time, Point3D position, Quaternion orientation, Vector3D linearVelocity, Vector3D angularVelocity)
   {
      super.setTrajectoryPoint(trajectoryPointIndex, time, position, orientation, linearVelocity, angularVelocity, WORLD_FRAME_HASH_CODE);
   }
}
