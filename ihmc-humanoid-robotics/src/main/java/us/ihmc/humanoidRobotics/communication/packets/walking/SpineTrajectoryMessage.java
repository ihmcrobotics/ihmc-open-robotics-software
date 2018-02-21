package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.humanoidRobotics.communication.packets.JointspaceTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;

public class SpineTrajectoryMessage extends Packet<SpineTrajectoryMessage>
{
   @RosExportedField(documentation = "Trajectories for each joint.")
   public JointspaceTrajectoryMessage jointspaceTrajectory;

   /**
    * Empty constructor for serialization. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public SpineTrajectoryMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Clone constructor.
    * 
    * @param spineTrajectoryMessage message to clone.
    */
   public SpineTrajectoryMessage(SpineTrajectoryMessage spineTrajectoryMessage)
   {
      jointspaceTrajectory = new JointspaceTrajectoryMessage(spineTrajectoryMessage.jointspaceTrajectory);
      setUniqueId(spineTrajectoryMessage.getUniqueId());
   }

   @Override
   public void set(SpineTrajectoryMessage other)
   {
      jointspaceTrajectory = new JointspaceTrajectoryMessage();
      jointspaceTrajectory.set(other.jointspaceTrajectory);
      setPacketInformation(other);
   }

   @Override
   public void setUniqueId(long uniqueId)
   {
      super.setUniqueId(uniqueId);
      if (jointspaceTrajectory != null)
         jointspaceTrajectory.setUniqueId(uniqueId);
   }

   public void setJointspaceTrajectory(JointspaceTrajectoryMessage jointspaceTrajectory)
   {
      this.jointspaceTrajectory = jointspaceTrajectory;
   }

   public JointspaceTrajectoryMessage getJointspaceTrajectory()
   {
      return jointspaceTrajectory;
   }

   @Override
   public boolean epsilonEquals(SpineTrajectoryMessage other, double epsilon)
   {
      if (!jointspaceTrajectory.epsilonEquals(other.jointspaceTrajectory, epsilon))
         return false;
      return true;
   }

   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateSpineTrajectoryMessage(this);
   }
}
