package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.PlanarRegionMessage;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.robotics.lists.RecyclingArrayList;

import java.util.List;

public class PlanarRegionsListCommand implements Command<PlanarRegionsListCommand, PlanarRegionsListMessage>
{
   private final RecyclingArrayList<PlanarRegionCommand> planarRegions = new RecyclingArrayList<>(30, PlanarRegionCommand.class);

   public PlanarRegionsListCommand()
   {
      clear();
   }

   @Override
   public void clear()
   {
      planarRegions.clear();
   }

   @Override
   public void set(PlanarRegionsListMessage message)
   {
      clear();

      List<PlanarRegionMessage> dataList = message.getPlanarRegions();
      if (dataList != null)
      {
         for (int i = 0; i < dataList.size(); i++)
            planarRegions.add().set(dataList.get(i));
      }
   }

   @Override
   public void set(PlanarRegionsListCommand other)
   {
      clear();

      RecyclingArrayList<PlanarRegionCommand> dataList = other.getPlanarRegions();
      if (dataList != null)
      {
         for (int i = 0; i < dataList.size(); i++)
            planarRegions.add().set(dataList.get(i));
      }
   }

   public int getNumberOfPlanarRegions()
   {
      return planarRegions.size();
   }

   public RecyclingArrayList<PlanarRegionCommand> getPlanarRegions()
   {
      return planarRegions;
   }

   public PlanarRegionCommand getPlanarRegionCommand(int i)
   {
      return planarRegions.get(i);
   }

   @Override
   public Class<PlanarRegionsListMessage> getMessageClass()
   {
      return PlanarRegionsListMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return getNumberOfPlanarRegions() > 0;
   }
}
