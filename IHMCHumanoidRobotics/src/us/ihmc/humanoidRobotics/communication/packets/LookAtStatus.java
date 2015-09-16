package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;

public class LookAtStatus extends Packet<LookAtStatus>
{

   @Override
   public boolean epsilonEquals(LookAtStatus other, double epsilon)
   {
      // TODO Auto-generated method stub
      return false;
   }

   public boolean isFinished()
   {
      // TODO Auto-generated method stub
      return false;
   }

   public int getIndex()
   {
      // TODO Auto-generated method stub
      return 0;
   }

}
