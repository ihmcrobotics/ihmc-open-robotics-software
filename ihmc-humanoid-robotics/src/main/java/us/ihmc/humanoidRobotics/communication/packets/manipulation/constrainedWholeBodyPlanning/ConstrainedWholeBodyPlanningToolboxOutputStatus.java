package us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
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
   public int planningResult = 0;

   public WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();

   public KinematicsToolboxOutputStatus[] robotConfigurations;
   public double[] trajectoryTimes;
   
   public ConstrainedWholeBodyPlanningToolboxOutputStatus()
   {
      
   }
   
   public ConstrainedWholeBodyPlanningToolboxOutputStatus(ConstrainedWholeBodyPlanningToolboxOutputStatus other)
   {
      set(other);
   }
      
//   public void set(KinematicsToolboxOutputStatus[] robotConfigurations, double[] trajectoryTimes)
//   {
//      if(robotConfigurations.length != trajectoryTimes.length)
//         throw new RuntimeException("length is different");
//      
//      this.robotConfigurations = robotConfigurations;
//      this.trajectoryTimes = trajectoryTimes;
//   }

   @Override
   public boolean epsilonEquals(ConstrainedWholeBodyPlanningToolboxOutputStatus other, double epsilon)
   {
      if (getPlanningResult() != other.getPlanningResult())
         return false;     
      
      int numberOfConfigurations = getRobotConfigurations().length;
      if(numberOfConfigurations == other.robotConfigurations.length)
      {
         for(int i=0;i<numberOfConfigurations;i++)
         {
            if (!getRobotConfigurations()[i].epsilonEquals(other.robotConfigurations[i], 1e-3f));
               return false;
         }     
      }
      else
      {
         return false;
      }
      
      if(!getWholeBodyTrajectoryMessage().epsilonEquals(other.getWholeBodyTrajectoryMessage(), 1e-3f))
         return false;
      
      return true;
   }

   @Override
   public void set(ConstrainedWholeBodyPlanningToolboxOutputStatus other)
   {
      setPlanningResult(other.planningResult);
      
      int numberOfConfigurations = other.robotConfigurations.length;
      
      robotConfigurations = new KinematicsToolboxOutputStatus[numberOfConfigurations];
            
      for(int i=0;i<numberOfConfigurations;i++)
      {
         if(other.robotConfigurations[i] == null)
            PrintTools.info("something wrong");
         robotConfigurations[i] = new KinematicsToolboxOutputStatus(other.robotConfigurations[i]);
      }
      
      trajectoryTimes = new double[numberOfConfigurations];
      for(int i=0;i<numberOfConfigurations;i++)
         trajectoryTimes[i] = other.trajectoryTimes[i];
      
      wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
      
      wholeBodyTrajectoryMessage = other.wholeBodyTrajectoryMessage;
   }

   public int getPlanningResult()
   {
      return planningResult;
   }

   public void setPlanningResult(int planningResult)
   {
      this.planningResult = planningResult;
   }

   public KinematicsToolboxOutputStatus[] getRobotConfigurations()
   {
      return robotConfigurations;
   }

   public void setRobotConfigurations(KinematicsToolboxOutputStatus[] robotConfigurations)
   {
      int numberOfConfigurations = robotConfigurations.length;
      
      this.robotConfigurations = new KinematicsToolboxOutputStatus[numberOfConfigurations];
      for(int i=0;i<numberOfConfigurations;i++)
         this.robotConfigurations[i] = new KinematicsToolboxOutputStatus(robotConfigurations[i]);
                  
   }

   public double[] getTrajectoryTimes()
   {
      return trajectoryTimes;
   }

   public void setTrajectoryTimes(double[] trajectoryTimes)
   {
      this.trajectoryTimes = trajectoryTimes;
   }
   
   public void setWholeBodyTrajectoryMessage(WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage)
   {
      this.wholeBodyTrajectoryMessage = wholeBodyTrajectoryMessage;
   }
   
   public WholeBodyTrajectoryMessage getWholeBodyTrajectoryMessage()
   {
      return wholeBodyTrajectoryMessage;
   }
}