package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.Packet;

public class PoseValidityRequestPacket extends Packet<PoseValidityRequestPacket>
{
   public boolean validityResult = false;
   
   public PoseValidityRequestPacket()
   {
      PrintTools.info("PoseValidityRequestPacket");
   }
   
   public void set(boolean setResult)
   {
      this.validityResult = setResult;
   }
   
   @Override
   public boolean epsilonEquals(PoseValidityRequestPacket other, double epsilon)
   {
      if (validityResult != other.validityResult)
         return false;
      return false;
   }

}
