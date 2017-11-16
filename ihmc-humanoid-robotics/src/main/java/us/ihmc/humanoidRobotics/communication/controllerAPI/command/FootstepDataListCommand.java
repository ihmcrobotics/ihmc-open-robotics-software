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

   /** the time to delay this command on the controller side before being executed **/
   private double executionDelayTime;
   /** the execution time. This number is set if the execution delay is non zero**/
   private double adjustedExecutionTime;
   /** If {@code false} the controller adjust each footstep height to be at the support sole height. */
   private boolean trustHeightOfFootsteps = true;
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
      executionDelayTime = message.executionDelayTime;
      trustHeightOfFootsteps = message.trustHeightOfFootsteps;
      offsetFootstepsWithExecutionError = message.isOffsetFootstepsWithExecutionError();
      ArrayList<FootstepDataMessage> dataList = message.getDataList();
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      if (dataList != null)
      {
         for (int i = 0; i < dataList.size(); i++)
            footsteps.add().set(worldFrame, dataList.get(i));
      }
      setQueueableCommandVariables(message);
   }

   @Override
   public void set(FootstepDataListCommand other)
   {
      clear();

      defaultSwingDuration = other.defaultSwingDuration;
      defaultTransferDuration = other.defaultTransferDuration;
      finalTransferDuration = other.finalTransferDuration;
      executionTiming = other.executionTiming;
      executionDelayTime = other.executionDelayTime;
      adjustedExecutionTime = other.adjustedExecutionTime;
      trustHeightOfFootsteps = other.trustHeightOfFootsteps;
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

   /**
    * returns the amount of time this command is delayed on the controller side before executing
    * @return the time to delay this command in seconds
    */
   @Override
   public double getExecutionDelayTime()
   {
      return executionDelayTime;
   }

   /**
    * sets the amount of time this command is delayed on the controller side before executing
    * @param delayTime the time in seconds to delay after receiving the command before executing
    */
   @Override
   public void setExecutionDelayTime(double delayTime)
   {
      this.executionDelayTime = delayTime;
   }
   /**
    * returns the expected execution time of this command. The execution time will be computed when the controller
    * receives the command using the controllers time plus the execution delay time.
    * This is used when {@code getExecutionDelayTime} is non-zero
    */
   @Override
   public double getExecutionTime()
   {
      return adjustedExecutionTime;
   }

   /**
    * sets the execution time for this command. This is called by the controller when the command is received.
    */
   @Override
   public void setExecutionTime(double adjustedExecutionTime)
   {
      this.adjustedExecutionTime = adjustedExecutionTime;
   }

   /**
    * tells the controller if this command supports delayed execution
    * (Spoiler alert: It does)
    * @return
    */
   @Override
   public boolean isDelayedExecutionSupported()
   {
      return true;
   }

   public boolean isTrustHeightOfFootsteps()
   {
      return trustHeightOfFootsteps;
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
