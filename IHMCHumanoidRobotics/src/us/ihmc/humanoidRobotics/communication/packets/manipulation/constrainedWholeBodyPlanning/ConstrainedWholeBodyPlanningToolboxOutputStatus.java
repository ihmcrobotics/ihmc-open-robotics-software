package us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning;

import us.ihmc.communication.packets.StatusPacket;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;

public class ConstrainedWholeBodyPlanningToolboxOutputStatus extends StatusPacket<ConstrainedWholeBodyPlanningToolboxOutputStatus>
{
   /**
    * 0: not completed.
    * 1: fail to find initial guess.
    * 2: fail to complete expanding tree.
    * 3: fail to optimize path.
    * 4: solution is available.
    */
   public int planningResult;

   public WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage;

   public ConstrainedWholeBodyPlanningToolboxOutputStatus()
   {
      
   }
   
   @Override
   public boolean epsilonEquals(ConstrainedWholeBodyPlanningToolboxOutputStatus other, double epsilon)
   {
      if (planningResult != other.planningResult)
         return false;
      return (wholeBodyTrajectoryMessage.epsilonEquals(other.wholeBodyTrajectoryMessage, epsilon));
   }

   @Override
   public void set(ConstrainedWholeBodyPlanningToolboxOutputStatus other)
   {
      planningResult = other.planningResult;
      wholeBodyTrajectoryMessage = other.wholeBodyTrajectoryMessage;
   }

}
