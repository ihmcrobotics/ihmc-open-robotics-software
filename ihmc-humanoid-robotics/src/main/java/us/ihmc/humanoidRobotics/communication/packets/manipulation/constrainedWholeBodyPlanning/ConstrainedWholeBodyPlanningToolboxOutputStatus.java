package us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning;

import java.util.ArrayList;

import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.communication.packets.StatusPacket;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

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

   // public WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage;

   //TODO : will be get rid of.
   public SideDependentList<ArrayList<Pose3D>> handTrajectories = new SideDependentList<>();

   public KinematicsToolboxOutputStatus[] robotConfigurations;

   public double[] trajectoryTimes;

   public ConstrainedWholeBodyPlanningToolboxOutputStatus()
   {
      handTrajectories.put(RobotSide.LEFT, new ArrayList<Pose3D>());
      handTrajectories.put(RobotSide.RIGHT, new ArrayList<Pose3D>());

      
   }
   
   public void set(KinematicsToolboxOutputStatus[] robotConfigurations, double[] trajectoryTimes)
   {
      if(robotConfigurations.length != trajectoryTimes.length)
         throw new RuntimeException("length is different");
      
      this.robotConfigurations = robotConfigurations;
      this.trajectoryTimes = trajectoryTimes;
   }

   @Override
   public boolean epsilonEquals(ConstrainedWholeBodyPlanningToolboxOutputStatus other, double epsilon)
   {
      if (getPlanningResult() != other.getPlanningResult())
         return false;      
      // TODO : more equals
      return true;
   }

   @Override
   public void set(ConstrainedWholeBodyPlanningToolboxOutputStatus other)
   {
      setPlanningResult(other.getPlanningResult());
      handTrajectories = other.handTrajectories;
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
      this.robotConfigurations = robotConfigurations;
   }

   public double[] getTrajectoryTimes()
   {
      return trajectoryTimes;
   }

   public void setTrajectoryTimes(double[] trajectoryTimes)
   {
      this.trajectoryTimes = trajectoryTimes;
   }
   
   
}