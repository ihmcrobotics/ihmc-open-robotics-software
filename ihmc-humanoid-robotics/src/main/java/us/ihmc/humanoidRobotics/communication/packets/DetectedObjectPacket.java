package us.ihmc.humanoidRobotics.communication.packets;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class DetectedObjectPacket extends Packet<DetectedObjectPacket>
{
   public RigidBodyTransform pose;
   public int id;
   
   public DetectedObjectPacket()
   {
   }
   
   public DetectedObjectPacket(RigidBodyTransform pose, int id)
   {
      this.pose = pose;
      this.id = id;
   }

   public DetectedObjectPacket(Random random)
   {
      pose = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
      id = random.nextInt(255);
   }
   
   @Override
   public boolean epsilonEquals(DetectedObjectPacket other, double epsilon)
   {
      return pose.epsilonEquals(other.pose, epsilon) && id == other.id;
   }

}
