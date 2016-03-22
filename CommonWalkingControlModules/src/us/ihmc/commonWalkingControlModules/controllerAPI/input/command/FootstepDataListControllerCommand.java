package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import java.util.ArrayList;

import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class FootstepDataListControllerCommand implements ControllerCommand<FootstepDataListControllerCommand, FootstepDataListMessage>
{
   private double swingTime;
   private double transferTime = 0.0;
   private final RecyclingArrayList<FootstepDataControllerCommand> footsteps = new RecyclingArrayList<>(30, FootstepDataControllerCommand.class);

   public FootstepDataListControllerCommand()
   {
      clear();
   }

   @Override
   public void clear()
   {
      swingTime = 0.0;
      transferTime = 0.0;
      footsteps.clear();
   }

   @Override
   public void set(FootstepDataListMessage message)
   {
      clear();

      swingTime = message.swingTime;
      transferTime = message.transferTime;
      ArrayList<FootstepDataMessage> dataList = message.getDataList();
      if (dataList != null)
      {
         for (int i = 0; i < dataList.size(); i++)
            footsteps.add().set(dataList.get(i));
      }
   }

   @Override
   public void set(FootstepDataListControllerCommand other)
   {
      clear();

      swingTime = other.swingTime;
      transferTime = other.transferTime;
      RecyclingArrayList<FootstepDataControllerCommand> otherFootsteps = other.getFootsteps();
      if (otherFootsteps != null)
      {
         for (int i = 0; i < otherFootsteps.size(); i++)
            footsteps.add().set(otherFootsteps.get(i));
      }
   }

   public void clearFoosteps()
   {
      clear();
   }

   public void addFootstep(FootstepDataControllerCommand footstep)
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

   public RecyclingArrayList<FootstepDataControllerCommand> getFootsteps()
   {
      return footsteps;
   }

   public FootstepDataControllerCommand getFootstep(int footstepIndex)
   {
      return footsteps.get(footstepIndex);
   }

   public int getNumberOfFootsteps()
   {
      return footsteps.size();
   }

   @Override
   public Class<FootstepDataListMessage> getMessageClass()
   {
      return FootstepDataListMessage.class;
   }
}
