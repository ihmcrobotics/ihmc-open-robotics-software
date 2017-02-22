package us.ihmc.humanoidRobotics.communication.packets;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.Random;

import us.ihmc.communication.packets.TrackablePacket;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.robotics.random.RandomTools;

public abstract class AbstractDesiredAccelerationsMessage<T extends AbstractDesiredAccelerationsMessage<T>> extends TrackablePacket<T>
{
   @RosExportedField(documentation = "Specifies the desired joint accelerations.")
   public double[] desiredJointAccelerations;

   public AbstractDesiredAccelerationsMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public AbstractDesiredAccelerationsMessage(Random random)
   {
      int randomNumberOfAccels = random.nextInt(16) + 1;
      desiredJointAccelerations = new double[randomNumberOfAccels];

      for(int i = 0; i < randomNumberOfAccels; i++)
      {
         desiredJointAccelerations[i] = RandomTools.generateRandomDoubleWithEdgeCases(random, 0.01);
      }
   }

   public AbstractDesiredAccelerationsMessage(double[] desiredJointAccelerations)
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
