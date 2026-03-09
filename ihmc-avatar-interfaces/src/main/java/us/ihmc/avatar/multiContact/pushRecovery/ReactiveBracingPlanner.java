package us.ihmc.avatar.multiContact.pushRecovery;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.BipedTimedStep;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandContactCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;

public interface ReactiveBracingPlanner
{
   /**
    * Generates plan for optimal hand-wall contact placement, given the robot's current state and upcoming footsteps
    */
   void plan(ReducedOrderRobotModel reducedOrderRobotModel, List<BipedTimedStep> plannedSteps, SideDependentList<HandContactCommand> contactCommandsToPack);

   /**
    * Sets the detected list of planar regions
    */
   void setPlanarRegions(PlanarRegionsListCommand planarRegionsListCommand);

   /**
    * YoRegistry for graphics, etc.
    */
   YoRegistry getRegistry();
}
