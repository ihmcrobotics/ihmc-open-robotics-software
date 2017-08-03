package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public interface ConstrainedConfigurationSpace
{
   abstract SelectionMatrix6D defineControllableSelectionMatrix();

   abstract ConfigurationBuildOrder defineConfigurationBuildOrder();
   
   abstract CTTaskNodeRegion defineTaskNodeRegion();
}
