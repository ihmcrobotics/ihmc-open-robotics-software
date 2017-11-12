package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.communication.packets.SettablePacket;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;

public class WholeBodyTrajectoryToolboxOutputStatus extends SettablePacket<WholeBodyTrajectoryToolboxOutputStatus>
{
   /**
    * <ul>
    * <li>0: not completed.
    * <li>1: fail to find initial guess.
    * <li>2: fail to complete expanding tree.
    * <li>3: fail to optimize path.
    * <li>4: solution is available.
    * </ul>
    */
   public int planningResult = 0;

   public double[] trajectoryTimes;
   public KinematicsToolboxOutputStatus[] robotConfigurations;

   public WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();

   public WholeBodyTrajectoryToolboxOutputStatus()
   {

   }

   public WholeBodyTrajectoryToolboxOutputStatus(WholeBodyTrajectoryToolboxOutputStatus other)
   {
      set(other);
   }

   @Override
   public void set(WholeBodyTrajectoryToolboxOutputStatus other)
   {
      setPlanningResult(other.planningResult);

      int numberOfConfigurations = other.robotConfigurations.length;

      trajectoryTimes = new double[numberOfConfigurations];
      robotConfigurations = new KinematicsToolboxOutputStatus[numberOfConfigurations];

      for (int i = 0; i < numberOfConfigurations; i++)
      {
         trajectoryTimes[i] = other.trajectoryTimes[i];
         robotConfigurations[i] = new KinematicsToolboxOutputStatus(other.robotConfigurations[i]);
      }

      wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage(other.wholeBodyTrajectoryMessage);
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
      for (int i = 0; i < numberOfConfigurations; i++)
      {
         this.robotConfigurations[i] = new KinematicsToolboxOutputStatus(robotConfigurations[i]);
      }
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

   public double getTrajectoryTime()
   {
      return trajectoryTimes[trajectoryTimes.length - 1];
   }

   public KinematicsToolboxOutputStatus getLastRobotConfiguration()
   {
      return robotConfigurations[robotConfigurations.length - 1];
   }

   @Override
   public boolean epsilonEquals(WholeBodyTrajectoryToolboxOutputStatus other, double epsilon)
   {
      if (planningResult != other.planningResult)
      {
         return false;
      }

      int numberOfConfigurations = robotConfigurations.length;

      if (numberOfConfigurations != other.robotConfigurations.length)
      {
         return false;
      }

      for (int i = 0; i < numberOfConfigurations; i++)
      {
         if (!robotConfigurations[i].epsilonEquals(other.robotConfigurations[i], epsilon))
         {
            return false;
         }
      }

      if (!wholeBodyTrajectoryMessage.epsilonEquals(other.wholeBodyTrajectoryMessage, epsilon))
      {
         return false;
      }

      return true;
   }
}