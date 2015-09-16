package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;

@ClassDocumentation("Stop the execution of a Whole Body Trajectory")
public class StopMotionPacket extends IHMCRosApiPacket<StopMotionPacket>
{

   public StopMotionPacket()
   {
   }
   
   public StopMotionPacket(Random random)
   {
      
   }

   @Override
   public boolean epsilonEquals(StopMotionPacket other, double epsilon)
   {
      return true;
   }
}
