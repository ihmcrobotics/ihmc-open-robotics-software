package us.ihmc.humanoidRobotics.communication.packets.wholebody;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.robotics.random.RandomTools;

public class MultiJointAnglePacket extends Packet<MultiJointAnglePacket> implements VisualizablePacket
{
   public SingleJointAnglePacket[] singleJointAnglePackets;

   public MultiJointAnglePacket()
   {
   }

   public MultiJointAnglePacket(Random random)
   {
      int randomLength = random.nextInt(30) + 1;
      singleJointAnglePackets = new SingleJointAnglePacket[randomLength];

      for (int i = 0; i < randomLength; i++)
      {
         singleJointAnglePackets[i] = new SingleJointAnglePacket("testJoint" + i, RandomTools.generateRandomDouble(random, -5.0, 5.0),
               RandomTools.generateRandomDouble(random, 0.0, 10.0), Double.NaN);
      }
   }

   public MultiJointAnglePacket(SingleJointAnglePacket[] singleJointAnglePackets)
   {
      this.singleJointAnglePackets = singleJointAnglePackets;
   }

   @Override
   public boolean epsilonEquals(MultiJointAnglePacket other, double epsilon)
   {
      if (singleJointAnglePackets.length != other.singleJointAnglePackets.length)
      {
         return false;
      }

      for (int i = 0; i < singleJointAnglePackets.length; i++)
      {
         if (!singleJointAnglePackets[i].epsilonEquals(other.singleJointAnglePackets[i], epsilon))
         {
            return false;
         }
      }

      return true;
   }

}
