package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.humanoidRobotics.communication.packets.SE3TrajectoryMessage;

@RosMessagePacket(documentation =
      "This message commands the controller to move in taskspace the pelvis to the desired pose (position & orientation) while going through the specified trajectory points."
      + " A third order polynomial function is used to interpolate positions and a hermite based curve (third order) is used to interpolate the orientations."
      + " To excute a single straight line trajectory to reach a desired pelvis pose, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time."
      + " Note that the pelvis position is limited keep the robot's balance (center of mass has to remain inside the support polygon)."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.",
                  rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
                  topic = "/control/pelvis_trajectory")
public class PelvisTrajectoryMessage extends Packet<PelvisTrajectoryMessage>
{
   private static final long WORLD_FRAME_HASH_CODE = ReferenceFrame.getWorldFrame().getNameBasedHashCode();

   public boolean enableUserPelvisControl = false;
   public boolean enableUserPelvisControlDuringWalking = false;
   @RosExportedField(documentation = "The position/orientation trajectory information.")
   public SE3TrajectoryMessage se3Trajectory;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public PelvisTrajectoryMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Clone constructor.
    * @param other message to clone.
    */
   public PelvisTrajectoryMessage(PelvisTrajectoryMessage other)
   {
      set(other);
   }

   @Override
   public void set(PelvisTrajectoryMessage other)
   {
      enableUserPelvisControl = other.enableUserPelvisControl;
      enableUserPelvisControlDuringWalking = other.enableUserPelvisControlDuringWalking;
      se3Trajectory = new SE3TrajectoryMessage();
      se3Trajectory.set(other.se3Trajectory);
      setPacketInformation(other);
   }

   public boolean isEnableUserPelvisControlDuringWalking()
   {
      return enableUserPelvisControlDuringWalking;
   }

   public void setEnableUserPelvisControlDuringWalking(boolean enableUserPelvisControlDuringWalking)
   {
      this.enableUserPelvisControlDuringWalking = enableUserPelvisControlDuringWalking;
   }

   public boolean isEnableUserPelvisControl()
   {
      return enableUserPelvisControl;
   }

   public void setEnableUserPelvisControl(boolean enableUserPelvisControl)
   {
      this.enableUserPelvisControl = enableUserPelvisControl;
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
   public boolean epsilonEquals(PelvisTrajectoryMessage other, double epsilon)
   {
      return se3Trajectory.epsilonEquals(other.se3Trajectory, epsilon);
   }

   @Override
   public String toString()
   {
      if (se3Trajectory.taskspaceTrajectoryPoints != null)
         return "Pelvis SE3 trajectory: number of SE3 trajectory points = " + se3Trajectory.getNumberOfTrajectoryPoints();
      else
         return "Pelvis SE3 trajectory: no SE3 trajectory points";
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validatePelvisTrajectoryMessage(this);
   }

   public final void setTrajectoryPoint(int trajectoryPointIndex, double time, Point3DReadOnly position, QuaternionReadOnly orientation, Vector3DReadOnly linearVelocity, Vector3DReadOnly angularVelocity)
   {
      se3Trajectory.setTrajectoryPoint(trajectoryPointIndex, time, position, orientation, linearVelocity, angularVelocity, WORLD_FRAME_HASH_CODE);
   }
}
