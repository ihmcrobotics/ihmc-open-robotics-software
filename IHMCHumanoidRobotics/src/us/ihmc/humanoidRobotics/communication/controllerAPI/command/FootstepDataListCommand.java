package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.ArrayList;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionTiming;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class FootstepDataListCommand implements Command<FootstepDataListCommand, FootstepDataListMessage>
{
   private double defaultSwingDuration;
   private double defaultTransferDuration;
   private double finalTransferDuration;
   private ExecutionMode executionMode = ExecutionMode.OVERRIDE;
   private ExecutionTiming executionTiming = ExecutionTiming.CONTROL_DURATIONS;
   private final RecyclingArrayList<FootstepDataCommand> footsteps = new RecyclingArrayList<>(30, FootstepDataCommand.class);

   public FootstepDataListCommand()
   {
      clear();
   }

   @Override
   public void clear()
   {
      defaultSwingDuration = 0.0;
      defaultTransferDuration = 0.0;
      finalTransferDuration = 0.0;
      footsteps.clear();
   }

   @Override
   public void set(FootstepDataListMessage message)
   {
      clear();

      defaultSwingDuration = message.defaultSwingDuration;
      defaultTransferDuration = message.defaultTransferDuration;
      finalTransferDuration = message.finalTransferDuration;
      executionMode = message.executionMode;
      executionTiming = message.executionTiming;
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

      defaultSwingDuration = other.defaultSwingDuration;
      defaultTransferDuration = other.defaultTransferDuration;
      finalTransferDuration = other.finalTransferDuration;
      executionMode = other.executionMode;
      executionTiming = other.executionTiming;
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

   public void setDefaultSwingDuration(double defaultSwingDuration)
   {
      this.defaultSwingDuration = defaultSwingDuration;
   }

   public void setDefaultTransferDuration(double defaultTransferDuration)
   {
      this.defaultTransferDuration = defaultTransferDuration;
   }

   public void setExecutionMode(ExecutionMode executionMode)
   {
      this.executionMode = executionMode;
   }

   public double getDefaultSwingDuration()
   {
      return defaultSwingDuration;
   }

   public double getDefaultTransferDuration()
   {
      return defaultTransferDuration;
   }

   public double getFinalTransferDuration()
   {
      return finalTransferDuration;
   }

   public ExecutionTiming getExecutionTiming()
   {
      return executionTiming;
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
