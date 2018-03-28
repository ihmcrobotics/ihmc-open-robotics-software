package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.humanoidRobotics.communication.packets.SO3TrajectoryMessage;

@RosMessagePacket(documentation =
      "This message commands the controller to move in taskspace the head to the desired orientation while going through the specified trajectory points."
      + " A hermite based curve (third order) is used to interpolate the orientations."
      + " To excute a simple trajectory to reach a desired head orientation, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.",
                  rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
                  topic = "/control/head_trajectory")
public class HeadTrajectoryMessage extends Packet<HeadTrajectoryMessage>
{
   @RosExportedField(documentation = "The orientation trajectory information.")
   public SO3TrajectoryMessage so3Trajectory = new SO3TrajectoryMessage();

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public HeadTrajectoryMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Clone constructor.
    * @param message to clone.
    */
   public HeadTrajectoryMessage(HeadTrajectoryMessage headTrajectoryMessage)
   {
      so3Trajectory = new SO3TrajectoryMessage(headTrajectoryMessage.so3Trajectory);
      setUniqueId(headTrajectoryMessage.getUniqueId());
      setDestination(headTrajectoryMessage.getDestination());
   }

   @Override
   public void set(HeadTrajectoryMessage other)
   {
      so3Trajectory = new SO3TrajectoryMessage();
      so3Trajectory.set(other.so3Trajectory);
      setPacketInformation(other);
   }

   @Override
   public void setUniqueId(long uniqueId)
   {
      super.setUniqueId(uniqueId);
      if (so3Trajectory != null)
         so3Trajectory.setUniqueId(uniqueId);
   }

   public void setSo3Trajectory(SO3TrajectoryMessage so3Trajectory)
   {
      this.so3Trajectory = so3Trajectory;
   }

   public SO3TrajectoryMessage getSo3Trajectory()
   {
      return so3Trajectory;
   }

   @Override
   public boolean epsilonEquals(HeadTrajectoryMessage other, double epsilon)
   {
      if (!so3Trajectory.epsilonEquals(other.so3Trajectory, epsilon))
         return false;
      return true;
   }

   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateHeadTrajectoryMessage(this);
   }
}
