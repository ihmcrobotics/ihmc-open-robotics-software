package us.ihmc.humanoidRobotics.manipulation;

import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public interface ConstrainedConfigurationSpace
{
   abstract SelectionMatrix6D defineControllableSelectionMatrix();

   abstract ConfigurationBuildOrder defineConfigurationBuildOrder();
}
