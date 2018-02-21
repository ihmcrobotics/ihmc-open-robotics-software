package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.humanoidRobotics.communication.packets.SO3TrajectoryMessage;

@RosMessagePacket(documentation = "This message commands the controller to move in taskspace the chest to the desired orientation while going through the specified trajectory points."
      + " A hermite based curve (third order) is used to interpolate the orientations."
      + " To excute a simple trajectory to reach a desired chest orientation, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/control/chest_trajectory")
public class ChestTrajectoryMessage extends Packet<ChestTrajectoryMessage>
{
   @RosExportedField(documentation = "The orientation trajectory information.")
   public SO3TrajectoryMessage so3Trajectory = new SO3TrajectoryMessage();

   /**
    * Empty constructor for serialization. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public ChestTrajectoryMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Clone constructor.
    * 
    * @param message to clone.
    */
   public ChestTrajectoryMessage(ChestTrajectoryMessage chestTrajectoryMessage)
   {
      so3Trajectory = new SO3TrajectoryMessage(chestTrajectoryMessage.so3Trajectory);
      setUniqueId(chestTrajectoryMessage.getUniqueId());
      setDestination(chestTrajectoryMessage.getDestination());
   }

   @Override
   public void set(ChestTrajectoryMessage other)
   {
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

   public SO3TrajectoryMessage getSo3Trajectory()
   {
      return so3Trajectory;
   }

   @Override
   public boolean epsilonEquals(ChestTrajectoryMessage other, double epsilon)
   {
      if (!so3Trajectory.epsilonEquals(other.so3Trajectory, epsilon))
         return false;
      return true;
   }

   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateChestTrajectoryMessage(this);
   }
}
