package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConfigurationBuildOrder;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConfigurationSpace;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConstrainedEndEffectorTrajectory;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.TaskRegion;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public class CuttingWallTrajectory extends ConstrainedEndEffectorTrajectory
{

   public CuttingWallTrajectory(double trajectoryTime)
   {
      super(trajectoryTime);
      // TODO Auto-generated constructor stub
   }

   @Override
   public SideDependentList<SelectionMatrix6D> defineControllableSelectionMatrices()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public SideDependentList<ConfigurationBuildOrder> defineConfigurationBuildOrders()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public TaskRegion defineTaskRegion()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   protected SideDependentList<ConfigurationSpace> getConfigurationSpace(double time)
   {
      // TODO Auto-generated method stub
      return null;
   }

}
