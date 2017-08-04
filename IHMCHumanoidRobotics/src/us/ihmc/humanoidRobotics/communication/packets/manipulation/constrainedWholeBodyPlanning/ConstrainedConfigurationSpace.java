package us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning;

import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public interface ConstrainedConfigurationSpace
{
   abstract SelectionMatrix6D defineControllableSelectionMatrix();

   abstract ConfigurationBuildOrder defineConfigurationBuildOrder();
   
   abstract TaskRegion defineTaskRegion();
}
