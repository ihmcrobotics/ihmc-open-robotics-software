package us.ihmc.commonWalkingControlModules.messageHandlers;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionCommand;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class PlanarRegionHandler
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoBoolean hasNewPlanarRegion = new YoBoolean("hasNewPlanarRegion", registry);
   private final PlanarRegion planarRegion = new PlanarRegion();

   private final YoBoolean waitingOnNewPlanarRegion = new YoBoolean("waitingOnNewPlanarRegion", registry);

   public PlanarRegionHandler(YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
   }

   public void handlePlanarRegionsListCommand(PlanarRegionCommand planarRegionsListCommand)
   {
      planarRegionsListCommand.getPlanarRegion(planarRegion);

      hasNewPlanarRegion.set(true);
      waitingOnNewPlanarRegion.set(false);
   }

   public void requestPlanarRegion()
   {
      waitingOnNewPlanarRegion.set(true);
   }

   public boolean hasNewPlanarRegion()
   {
      return hasNewPlanarRegion.getBooleanValue();
   }

   public PlanarRegion pollHasNewPlanarRegion()
   {
      if (!hasNewPlanarRegion.getBooleanValue())
         return null;

      hasNewPlanarRegion.set(false);

      return planarRegion;
   }
}
