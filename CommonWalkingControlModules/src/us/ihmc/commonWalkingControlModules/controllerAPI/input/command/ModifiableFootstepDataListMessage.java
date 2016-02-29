package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import java.util.ArrayList;

import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class ModifiableFootstepDataListMessage
{
   private double swingTime;
   private double transferTime = 0.0;
   private final RecyclingArrayList<ModifiableFootstepDataMessage> footsteps = new RecyclingArrayList<>(30, ModifiableFootstepDataMessage.class);

   public ModifiableFootstepDataListMessage()
   {
      footsteps.clear();
   }

   public void set(FootstepDataListMessage footstepDataListMessage)
   {
      swingTime = footstepDataListMessage.swingTime;
      transferTime = footstepDataListMessage.transferTime;
      ArrayList<FootstepDataMessage> dataList = footstepDataListMessage.getDataList();
      footsteps.clear();
      if (dataList != null)
      {
         for (int i = 0; i < dataList.size(); i++)
            footsteps.add().set(dataList.get(i));
      }
   }

   public double getSwingTime()
   {
      return swingTime;
   }

   public double getTransferTime()
   {
      return transferTime;
   }

   public RecyclingArrayList<ModifiableFootstepDataMessage> getFootsteps()
   {
      return footsteps;
   }
}
