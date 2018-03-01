package us.ihmc.commonWalkingControlModules.messageHandlers;

import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage;
import us.ihmc.communication.packets.PlanarRegionsRequestType;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
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

   private final YoBoolean waitingOnNewPlanarRegions = new YoBoolean("waitingOnNewPlanarRegions", registry);

   private final StatusMessageOutputManager statusOutputManager;
   private final RequestPlanarRegionsListMessage planarRegionsRequestMessage = MessageTools.createRequestPlanarRegionsListMessage(PlanarRegionsRequestType.SINGLE_UPDATE);

   public PlanarRegionsListHandler(StatusMessageOutputManager requestOutputManager, YoVariableRegistry parentRegistry)
   {
      this.statusOutputManager = requestOutputManager;

      planarRegions.clear();
      planarRegionsRequestMessage.setDestination(PacketDestination.CONTROLLER);

      parentRegistry.addChild(registry);
   }

   public void handlePlanarRegionsListCommand(PlanarRegionsListCommand planarRegionsListCommand)
   {
      planarRegions.clear();

      for (int i = 0; i < planarRegionsListCommand.getNumberOfPlanarRegions(); i++)
      {
         planarRegionsListCommand.getPlanarRegionCommand(i).getPlanarRegion(planarRegions.add());
         currentNumberOfPlanarRegions.increment();
      }

      hasNewPlanarRegionsList.set(true);
      waitingOnNewPlanarRegions.set(false);
   }

   public void requestPlanarRegions()
   {
      statusOutputManager.reportStatusMessage(planarRegionsRequestMessage);
      waitingOnNewPlanarRegions.set(true);
   }

   public boolean hasNewPlanarRegions()
   {
      return hasNewPlanarRegionsList.getBooleanValue();
   }

   public boolean pollHasNewPlanarRegionsList(PlanarRegionsList planarRegionsListToPack)
   {
      if (!hasNewPlanarRegionsList.getBooleanValue())
         return false;

      planarRegionsListToPack.clear();
      for (int i = 0; i <planarRegions.size(); i++)
         planarRegionsListToPack.addPlanarRegion(planarRegions.get(i));

      hasNewPlanarRegionsList.set(false);
      planarRegions.clear();

      return true;
   }

   public RecyclingArrayList<PlanarRegion> pollHasNewPlanarRegionsList()
   {
      hasNewPlanarRegionsList.set(false);
      return planarRegions;
   }
}
