package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import us.ihmc.communication.packets.StatusPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningToolboxOutputStatus;

public class RRTPlanningToolboxOutputStatus extends StatusPacket<FootstepPlanningToolboxOutputStatus>
{

   @Override
   public boolean epsilonEquals(FootstepPlanningToolboxOutputStatus other, double epsilon)
   {
      // TODO Auto-generated method stub
      return false;
   }

   @Override
   public void set(FootstepPlanningToolboxOutputStatus other)
   {
      // TODO Auto-generated method stub
      
   }

}
