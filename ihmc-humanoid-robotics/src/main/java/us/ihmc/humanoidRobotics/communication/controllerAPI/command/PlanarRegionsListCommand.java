package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.PlanarRegionMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.lists.RecyclingArrayList;

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

      RecyclingArrayList<PlanarRegionMessage> dataList = message.getPlanarRegions();
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
