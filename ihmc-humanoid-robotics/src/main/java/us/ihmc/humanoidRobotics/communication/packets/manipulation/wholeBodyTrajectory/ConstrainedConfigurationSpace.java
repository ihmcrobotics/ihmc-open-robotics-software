package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public interface ConstrainedConfigurationSpace
{
   abstract SideDependentList<SelectionMatrix6D> defineControllableSelectionMatrices();

   abstract SideDependentList<ConfigurationBuildOrder> defineConfigurationBuildOrders();
   
   abstract TaskRegion defineTaskRegion();
}