package us.ihmc.avatar.multiContact.pushRecovery;

import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandContactCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.SideDependentList;

public interface StandingReactiveBracingPlanner
{
   /**
    * Generates plan for optimal hand-wall contact placement
    */
   void plan(ReducedOrderRobotModel reducedOrderRobotModel, SideDependentList<HandContactCommand> contactCommandsToPack);

   /**
    * Sets the detected planar regions list
    */
   void setPlanarRegions(PlanarRegionsListCommand planarRegionsListCommand);
}
