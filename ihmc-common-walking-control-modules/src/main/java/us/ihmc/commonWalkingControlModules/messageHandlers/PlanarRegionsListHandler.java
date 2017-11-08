package us.ihmc.commonWalkingControlModules.messageHandlers;

import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;

public class PlanarRegionsListHandler
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private static final int maxNumberOfPlanarRegions = 100;
   private final YoBoolean hasNewPlanarRegionsList = new YoBoolean("hasNewPlanarRegionsList", registry);
   private final YoInteger currentNumberOfPlanarRegions = new YoInteger("currentNumberOfPlanarRegions", registry);
   private final RecyclingArrayList<PlanarRegion> planarRegions = new RecyclingArrayList<>(maxNumberOfPlanarRegions, PlanarRegion.class);

   public PlanarRegionsListHandler(YoVariableRegistry parentRegistry)
   {
      planarRegions.clear();

      parentRegistry.addChild(registry);
   }

   public void handlePlanarRegionsListCommand(PlanarRegionsListCommand planarRegionsListCommand)
   {
      for (int i = 0; i < planarRegionsListCommand.getNumberOfPlanarRegions(); i++)
      {
         planarRegionsListCommand.getPlanarRegionCommand(i).getPlanarRegion(planarRegions.add());
         currentNumberOfPlanarRegions.increment();
      }

      hasNewPlanarRegionsList.set(true);
   }
}
