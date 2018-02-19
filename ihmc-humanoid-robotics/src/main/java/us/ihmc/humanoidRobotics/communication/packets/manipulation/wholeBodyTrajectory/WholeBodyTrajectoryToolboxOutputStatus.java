package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.idl.PreallocatedList;

public class WholeBodyTrajectoryToolboxOutputStatus extends Packet<WholeBodyTrajectoryToolboxOutputStatus>
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

   public TDoubleArrayList trajectoryTimes = new TDoubleArrayList();
   public PreallocatedList<KinematicsToolboxOutputStatus> robotConfigurations = new PreallocatedList<>(KinematicsToolboxOutputStatus.class,
                                                                                                       KinematicsToolboxOutputStatus::new, 50);

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

      MessageTools.copyData(other.trajectoryTimes, trajectoryTimes);
      MessageTools.copyData(other.robotConfigurations, robotConfigurations);
      setPacketInformation(other);
   }

   public int getPlanningResult()
   {
      return planningResult;
   }

   public void setPlanningResult(int planningResult)
   {
      this.planningResult = planningResult;
   }

   public PreallocatedList<KinematicsToolboxOutputStatus> getRobotConfigurations()
   {
      return robotConfigurations;
   }

   public void setRobotConfigurations(KinematicsToolboxOutputStatus[] robotConfigurations)
   {
      MessageTools.copyData(robotConfigurations, this.robotConfigurations);
   }

   public TDoubleArrayList getTrajectoryTimes()
   {
      return trajectoryTimes;
   }

   public void setTrajectoryTimes(double[] trajectoryTimes)
   {
      this.trajectoryTimes.reset();
      this.trajectoryTimes.add(trajectoryTimes);
   }

   public double getTrajectoryTime()
   {
      return trajectoryTimes.get(trajectoryTimes.size() - 1);
   }

   public KinematicsToolboxOutputStatus getLastRobotConfiguration()
   {
      return robotConfigurations.get(robotConfigurations.size() - 1);
   }

   @Override
   public boolean epsilonEquals(WholeBodyTrajectoryToolboxOutputStatus other, double epsilon)
   {
      if (planningResult != other.planningResult)
      {
         return false;
      }

      if (MessageTools.epsilonEquals(trajectoryTimes, other.trajectoryTimes, epsilon))
         return false;
      if (MessageTools.epsilonEquals(robotConfigurations, other.robotConfigurations, epsilon))
         return false;
      return true;
   }
}