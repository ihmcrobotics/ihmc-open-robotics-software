package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.VisualizablePacket;

public class SpineTrajectoryMessage extends Packet<SpineTrajectoryMessage> implements VisualizablePacket
{

   @Override
   public boolean epsilonEquals(SpineTrajectoryMessage other, double epsilon)
   {
      // TODO Auto-generated method stub
      return false;
   }

}
