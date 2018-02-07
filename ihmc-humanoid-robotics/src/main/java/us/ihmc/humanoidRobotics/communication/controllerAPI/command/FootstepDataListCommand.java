package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.ArrayList;

import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.communication.packets.ExecutionTiming;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class FootstepDataListCommand extends QueueableCommand<FootstepDataListCommand, FootstepDataListMessage>
{
   private double defaultSwingDuration;
   private double defaultTransferDuration;
   private double finalTransferDuration;
   private ExecutionTiming executionTiming = ExecutionTiming.CONTROL_DURATIONS;
   private final RecyclingArrayList<FootstepDataCommand> footsteps = new RecyclingArrayList<>(30, FootstepDataCommand.class);

   /** If {@code false} the controller adjust each footstep height to be at the support sole height. */
   private boolean trustHeightOfFootsteps = true;
   /** If {@code false} the controller can adjust the footsteps. */
   private boolean areFootstepsAdjustable = false;
   /** If {@code true} the controller will adjust upcoming footsteps with the location error of previous steps. */
   private boolean offsetFootstepsWithExecutionError = false;

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
      clearQueuableCommandVariables();
   }

   @Override
   public void set(FootstepDataListMessage message)
   {
      clear();

      defaultSwingDuration = message.defaultSwingDuration;
      defaultTransferDuration = message.defaultTransferDuration;
      finalTransferDuration = message.finalTransferDuration;
      executionTiming = message.executionTiming;
      trustHeightOfFootsteps = message.trustHeightOfFootsteps;
      areFootstepsAdjustable = message.areFootstepsAdjustable;
      offsetFootstepsWithExecutionError = message.isOffsetFootstepsWithExecutionError();
      ArrayList<FootstepDataMessage> dataList = message.getDataList();
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      if (dataList != null)
      {
         for (int i = 0; i < dataList.size(); i++)
            footsteps.add().set(worldFrame, dataList.get(i));
      }
      setQueueableCommandVariables(message.getUniqueId(), message.getQueueingProperties());
   }

   @Override
   public void set(FootstepDataListCommand other)
   {
      clear();

      defaultSwingDuration = other.defaultSwingDuration;
      defaultTransferDuration = other.defaultTransferDuration;
      finalTransferDuration = other.finalTransferDuration;
      executionTiming = other.executionTiming;
      adjustedExecutionTime = other.adjustedExecutionTime;
      trustHeightOfFootsteps = other.trustHeightOfFootsteps;
      areFootstepsAdjustable = other.areFootstepsAdjustable;
      offsetFootstepsWithExecutionError = other.offsetFootstepsWithExecutionError;
      RecyclingArrayList<FootstepDataCommand> otherFootsteps = other.getFootsteps();
      if (otherFootsteps != null)
      {
         for (int i = 0; i < otherFootsteps.size(); i++)
            footsteps.add().set(otherFootsteps.get(i));
      }
      setQueueableCommandVariables(other);
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
      return getNumberOfFootsteps() > 0 && executionModeValid();
   }

   public boolean isTrustHeightOfFootsteps()
   {
      return trustHeightOfFootsteps;
   }

   public boolean areFootstepsAdjustable()
   {
      return areFootstepsAdjustable;
   }

   public boolean isOffsetFootstepsWithExecutionError()
   {
      return offsetFootstepsWithExecutionError;
   }

   @Override
   public void addTimeOffset(double timeOffset)
   {
      // Not needed for footsteps since timing is defined in durations inside the command rather then
      // absolute trajectory point times.
      throw new RuntimeException("This method should not be used with footstep lists.");
   }
}
