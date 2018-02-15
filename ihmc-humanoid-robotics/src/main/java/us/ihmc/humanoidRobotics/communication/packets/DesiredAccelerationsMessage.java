package us.ihmc.humanoidRobotics.communication.packets;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.QueueableMessage;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.tools.ArrayTools;

@RosMessagePacket(documentation = "", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/control/desired_joint_accelerations")
public final class DesiredAccelerationsMessage extends Packet<DesiredAccelerationsMessage>
{
   @RosExportedField(documentation = "Specifies the desired joint accelerations.")
   public double[] desiredJointAccelerations;
   @RosExportedField(documentation = "Properties for queueing trajectories.")
   public QueueableMessage queueingProperties = new QueueableMessage();

   public DesiredAccelerationsMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public DesiredAccelerationsMessage(Random random)
   {
      int randomNumberOfAccels = random.nextInt(16) + 1;
      desiredJointAccelerations = new double[randomNumberOfAccels];

      for(int i = 0; i < randomNumberOfAccels; i++)
      {
         desiredJointAccelerations[i] = RandomNumbers.nextDoubleWithEdgeCases(random, 0.01);
      }
   }

   public DesiredAccelerationsMessage(double[] desiredJointAccelerations)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.desiredJointAccelerations = desiredJointAccelerations;
   }

   public int getNumberOfJoints()
   {
      if (desiredJointAccelerations == null)
         return 0;
      else
         return desiredJointAccelerations.length;
   }

   public double[] getDesiredJointAccelerations()
   {
      return desiredJointAccelerations;
   }

   public double getDesiredJointAcceleration(int jointIndex)
   {
      return desiredJointAccelerations[jointIndex];
   }

   public QueueableMessage getQueueingProperties()
   {
      return queueingProperties;
   }

   @Override
   public boolean epsilonEquals(DesiredAccelerationsMessage other, double epsilon)
   {
      if (!ArrayTools.deltaEquals(getDesiredJointAccelerations(), other.getDesiredJointAccelerations(), epsilon))
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
         for (int i = 0; i < getNumberOfJoints(); i++)
         {
            double jointDesiredAcceleration = desiredJointAccelerations[i];
            ret += doubleFormat.format(jointDesiredAcceleration);
            if (i < getNumberOfJoints() - 1)
               ret += ", ";
         }
         return ret + "].";
   }
}
