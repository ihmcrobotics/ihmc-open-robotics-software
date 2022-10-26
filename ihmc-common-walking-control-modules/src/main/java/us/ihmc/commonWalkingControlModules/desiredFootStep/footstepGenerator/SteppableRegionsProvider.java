package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.List;

public interface SteppableRegionsProvider
{
   void consume(PlanarRegionsListCommand command);

   List<PlanarRegion> getSteppableRegions();
}
