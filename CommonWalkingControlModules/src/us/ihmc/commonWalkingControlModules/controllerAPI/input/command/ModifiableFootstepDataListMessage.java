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

   public void set(ModifiableFootstepDataListMessage other)
   {
      swingTime = other.swingTime;
      transferTime = other.transferTime;
      RecyclingArrayList<ModifiableFootstepDataMessage> otherFootsteps = other.getFootsteps();
      footsteps.clear();
      if (otherFootsteps != null)
      {
         for (int i = 0; i < otherFootsteps.size(); i++)
            footsteps.add().set(otherFootsteps.get(i));
      }
   }

   public void clearFoosteps()
   {
      footsteps.clear();
   }

   public void addFootstep(ModifiableFootstepDataMessage footstep)
   {
      footsteps.add().set(footstep);
   }

   public void setSwingTime(double swingTime)
   {
      this.swingTime = swingTime;
   }

   public void setTransferTime(double transferTime)
   {
      this.transferTime = transferTime;
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
