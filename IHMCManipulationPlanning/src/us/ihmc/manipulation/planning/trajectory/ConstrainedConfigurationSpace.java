package us.ihmc.manipulation.planning.trajectory;

import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public interface ConstrainedConfigurationSpace
{
   abstract SelectionMatrix6D defineControllableSelectionMatrix();

   abstract ConfigurationBuildOrder defineConfigurationBuildOrder();
}
