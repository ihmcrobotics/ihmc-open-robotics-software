package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.humanoidRobotics.communication.packets.SO3TrajectoryMessage;

@RosMessagePacket(documentation = "This message commands the controller to move in taskspace the pelvis to the desired orientation while going through the specified trajectory points."
      + " A hermite based curve (third order) is used to interpolate the orientations."
      + " This message allows controlling the pelvis orientation without interferring with position that will still be controlled to maintain the current desired capture poit position."
      + " To excute a normal trajectory to reach a desired pelvis orientation, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/control/pelvis_orientation_trajectory")
public class PelvisOrientationTrajectoryMessage extends Packet<PelvisOrientationTrajectoryMessage>
{
   @RosExportedField(documentation = "Whether the pelvis orientation is allowed to controlled by the user when the robot is walking.")
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

   public boolean getEnableUserPelvisControlDuringWalking()
   {
      return enableUserPelvisControlDuringWalking;
   }

   public void setEnableUserPelvisControlDuringWalking(boolean enableUserPelvisControlDuringWalking)
   {
      this.enableUserPelvisControlDuringWalking = enableUserPelvisControlDuringWalking;
   }

   @Override
   public void set(PelvisOrientationTrajectoryMessage other)
   {
      so3Trajectory = new SO3TrajectoryMessage(other.so3Trajectory);
      enableUserPelvisControlDuringWalking = other.enableUserPelvisControlDuringWalking;
      setPacketInformation(other);
   }

   @Override
   public void setUniqueId(long uniqueId)
   {
      super.setUniqueId(uniqueId);
      if (so3Trajectory != null)
         so3Trajectory.setUniqueId(uniqueId);
   }

   public SO3TrajectoryMessage getSo3Trajectory()
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
         return "Pelvis SO3 trajectory: number of SO3 trajectory points = " + so3Trajectory.taskspaceTrajectoryPoints.size();
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
