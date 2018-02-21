package us.ihmc.humanoidRobotics.communication.packets;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.QueueableMessage;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;

@RosMessagePacket(documentation = "", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/control/desired_joint_accelerations")
public final class DesiredAccelerationsMessage extends Packet<DesiredAccelerationsMessage>
{
   @RosExportedField(documentation = "Specifies the desired joint accelerations.")
   public TDoubleArrayList desiredJointAccelerations = new TDoubleArrayList();
   @RosExportedField(documentation = "Properties for queueing trajectories.")
   public QueueableMessage queueingProperties = new QueueableMessage();

   public DesiredAccelerationsMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(DesiredAccelerationsMessage other)
   {
      MessageTools.copyData(other.desiredJointAccelerations, desiredJointAccelerations);
      queueingProperties.set(other.queueingProperties);
      setPacketInformation(other);
   }

   public TDoubleArrayList getDesiredJointAccelerations()
   {
      return desiredJointAccelerations;
   }

   public QueueableMessage getQueueingProperties()
   {
      return queueingProperties;
   }

   @Override
   public boolean epsilonEquals(DesiredAccelerationsMessage other, double epsilon)
   {
      if (!MessageTools.epsilonEquals(desiredJointAccelerations, other.desiredJointAccelerations, epsilon))
         return false;
      return true;
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateDesiredAccelerationsMessage(this, true);
   }

   @Override
   public String toString()
   {
      String ret = "desired accelerations = [";
      NumberFormat doubleFormat = new DecimalFormat(" 0.00;-0.00");
      for (int i = 0; i < desiredJointAccelerations.size(); i++)
      {
         double jointDesiredAcceleration = desiredJointAccelerations.get(i);
         ret += doubleFormat.format(jointDesiredAcceleration);
         if (i < desiredJointAccelerations.size() - 1)
            ret += ", ";
      }
      return ret + "].";
   }
}
