package us.ihmc.communication.packets.wholebody;

import us.ihmc.communication.packets.Packet;


/**
 * 
 * @author WillRifenburgh
 *
 */

public class WholeBodyTrajectoryDevelopmentPacket extends Packet<WholeBodyTrajectoryDevelopmentPacket>
{

   @Override
   public boolean epsilonEquals(WholeBodyTrajectoryDevelopmentPacket other, double epsilon)
   {
      return true;
   }


}
