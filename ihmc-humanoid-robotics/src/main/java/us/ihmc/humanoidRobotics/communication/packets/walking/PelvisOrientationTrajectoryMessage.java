package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.humanoidRobotics.communication.packets.SO3TrajectoryMessage;

@RosMessagePacket(documentation = "This message commands the controller to move in taskspace the pelvis to the desired orientation while going through the specified trajectory points."
      + " A hermite based curve (third order) is used to interpolate the orientations."
      + " This message allows controlling the pelvis orientation without interferring with position that will still be controlled to maintain the current desired capture poit position."
      + " To excute a normal trajectory to reach a desired pelvis orientation, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/control/pelvis_orientation_trajectory")
public class PelvisOrientationTrajectoryMessage extends Packet<PelvisOrientationTrajectoryMessage>
{
   public boolean enableUserPelvisControlDuringWalking = false;
   @RosExportedField(documentation = "The orientation trajectory information.")
   public SO3TrajectoryMessage so3Trajectory;

   /**
    * Empty constructor for serialization. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public PelvisOrientationTrajectoryMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public PelvisOrientationTrajectoryMessage(Random random)
   {
      so3Trajectory = new SO3TrajectoryMessage(random);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Clone constructor.
    * 
    * @param pelvisOrientationTrajectoryMessage message to clone.
    */
   public PelvisOrientationTrajectoryMessage(PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage)
   {
      so3Trajectory = new SO3TrajectoryMessage(pelvisOrientationTrajectoryMessage.so3Trajectory);
      setUniqueId(pelvisOrientationTrajectoryMessage.getUniqueId());
      setDestination(pelvisOrientationTrajectoryMessage.getDestination());
   }

   /**
    * Use this constructor to execute a simple interpolation towards the given endpoint. Set the id
    * of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * 
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredOrientation desired pelvis orientation expressed in world frame.
    */
   public PelvisOrientationTrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation)
   {
      so3Trajectory = new SO3TrajectoryMessage(trajectoryTime, desiredOrientation, ReferenceFrame.getWorldFrame());
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Use this constructor to build a message with more than one trajectory point. Set the id of the
    * message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}. This constructor only allocates memory for
    * the trajectory points, you need to call
    * {@link #setTrajectoryPoint(int, double, Quaternion, Vector3D)} for each trajectory point
    * afterwards.
    * 
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the
    *           controller.
    */
   public PelvisOrientationTrajectoryMessage(int numberOfTrajectoryPoints)
   {
      so3Trajectory = new SO3TrajectoryMessage(numberOfTrajectoryPoints);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public PelvisOrientationTrajectoryMessage(double trajectoryTime, QuaternionReadOnly desiredOrientation, ReferenceFrame trajectoryFrame)
   {
      so3Trajectory = new SO3TrajectoryMessage(trajectoryTime, desiredOrientation, trajectoryFrame);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public boolean isEnableUserPelvisControlDuringWalking()
   {
      return enableUserPelvisControlDuringWalking;
   }

   public void setEnableUserPelvisControlDuringWalking(boolean enableUserPelvisControlDuringWalking)
   {
      this.enableUserPelvisControlDuringWalking = enableUserPelvisControlDuringWalking;
   }

   public void set(PelvisOrientationTrajectoryMessage other)
   {
      so3Trajectory = new SO3TrajectoryMessage(other.so3Trajectory);
      setUniqueId(other.getUniqueId());
      setDestination(other.getDestination());
   }

   @Override
   public void setUniqueId(long uniqueId)
   {
      super.setUniqueId(uniqueId);
      if (so3Trajectory != null)
         so3Trajectory.setUniqueId(uniqueId);
   }

   public SO3TrajectoryMessage getSO3Trajectory()
   {
      return so3Trajectory;
   }

   @Override
   public boolean epsilonEquals(PelvisOrientationTrajectoryMessage other, double epsilon)
   {
      if (!so3Trajectory.epsilonEquals(other.so3Trajectory, epsilon))
         return false;
      if (enableUserPelvisControlDuringWalking != other.enableUserPelvisControlDuringWalking)
         return false;
      return true;
   }

   @Override
   public String toString()
   {
      if (so3Trajectory.taskspaceTrajectoryPoints != null)
         return "Pelvis SO3 trajectory: number of SO3 trajectory points = " + so3Trajectory.getNumberOfTrajectoryPoints();
      else
         return "Pelvis SO3 trajectory: no SO3 trajectory points";
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validatePelvisOrientationTrajectoryMessage(this);
   }
}
