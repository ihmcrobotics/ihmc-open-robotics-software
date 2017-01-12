package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.ArrayList;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class FootstepDataListCommand implements Command<FootstepDataListCommand, FootstepDataListMessage>
{
   private double swingTime;
   private double transferTime = 0.0;
   private ExecutionMode executionMode = ExecutionMode.OVERRIDE;
   private final RecyclingArrayList<FootstepDataCommand> footsteps = new RecyclingArrayList<>(30, FootstepDataCommand.class);

   public FootstepDataListCommand()
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

      swingTime = message.swingDefaultTime;
      transferTime = message.defaultTransferTime;
      executionMode = message.executionMode;
      ArrayList<FootstepDataMessage> dataList = message.getDataList();
      if (dataList != null)
      {
         for (int i = 0; i < dataList.size(); i++)
            footsteps.add().set(dataList.get(i));
      }
   }

   @Override
   public void set(FootstepDataListCommand other)
   {
      clear();

      swingTime = other.swingTime;
      transferTime = other.transferTime;
      executionMode = other.executionMode;
      RecyclingArrayList<FootstepDataCommand> otherFootsteps = other.getFootsteps();
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

   public void addFootstep(FootstepDataCommand footstep)
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

   public void setExecutionMode(ExecutionMode executionMode)
   {
      this.executionMode = executionMode;
   }

   public double getSwingTime()
   {
      return swingTime;
   }

   public double getTransferTime()
   {
      return transferTime;
   }

   public ExecutionMode getExecutionMode()
   {
      return executionMode;
   }

   public RecyclingArrayList<FootstepDataCommand> getFootsteps()
   {
      return footsteps;
   }

   public void removeFootstep(int footstepIndex)
   {
      footsteps.remove(footstepIndex);
   }

   public FootstepDataCommand getFootstep(int footstepIndex)
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

   @Override
   public boolean isCommandValid()
   {
      return getNumberOfFootsteps() > 0;
   }
}
