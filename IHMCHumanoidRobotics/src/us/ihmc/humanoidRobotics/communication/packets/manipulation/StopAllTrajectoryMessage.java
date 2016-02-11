package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;

@ClassDocumentation("Stop the execution of any trajectory being executed.")
public class StopAllTrajectoryMessage extends IHMCRosApiPacket<StopAllTrajectoryMessage>
{

   public StopAllTrajectoryMessage()
   {
   }
   
   public StopAllTrajectoryMessage(Random random)
   {
      
   }

   @Override
   public boolean epsilonEquals(StopAllTrajectoryMessage other, double epsilon)
   {
      return true;
   }
}
