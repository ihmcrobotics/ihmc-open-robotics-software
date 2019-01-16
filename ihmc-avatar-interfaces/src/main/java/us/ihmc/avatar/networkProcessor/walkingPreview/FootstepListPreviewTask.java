package us.ihmc.avatar.networkProcessor.walkingPreview;

import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager.StatusMessageListener;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.tools.taskExecutor.Task;

public class FootstepListPreviewTask implements Task
{
   private final FootstepDataListCommand footstepList;
   private final CommandInputManager walkingInputManager;
   private final StatusMessageOutputManager walkingOutputManager;

   private int numberOfFootstepsRemaining;
   private final AtomicReference<WalkingStatus> latestWalkingStatus = new AtomicReference<>(null);
   private StatusMessageListener<FootstepStatusMessage> footstepStatusMessageListener = this::processFootstepStatus;
   private StatusMessageListener<WalkingStatusMessage> walkingStatusMessageListener = this::processWalkingStatus;

   public FootstepListPreviewTask(FootstepDataListCommand footstepList, CommandInputManager walkingInputManager,
                                  StatusMessageOutputManager walkingOutputManager)
   {
      this.footstepList = footstepList;
      this.walkingInputManager = walkingInputManager;
      this.walkingOutputManager = walkingOutputManager;
   }

   @Override
   public void doTransitionIntoAction()
   {
      walkingInputManager.submitCommand(footstepList);
      numberOfFootstepsRemaining = footstepList.getNumberOfFootsteps();
      walkingOutputManager.attachStatusMessageListener(FootstepStatusMessage.class, footstepStatusMessageListener);
      walkingOutputManager.attachStatusMessageListener(WalkingStatusMessage.class, walkingStatusMessageListener);
   }

   private void processFootstepStatus(FootstepStatusMessage status)
   {
      if (FootstepStatus.fromByte(status.getFootstepStatus()) == FootstepStatus.COMPLETED)
         numberOfFootstepsRemaining--;
   }

   private void processWalkingStatus(WalkingStatusMessage status)
   {
      latestWalkingStatus.set(WalkingStatus.fromByte(status.getWalkingStatus()));
   }

   @Override
   public void doAction()
   {
   }

   @Override
   public void doTransitionOutOfAction()
   {
      destroyListeners();
   }

   @Override
   public boolean isDone()
   {
      if (numberOfFootstepsRemaining > 0)
         return false;
      if (latestWalkingStatus.get() == null)
         return false;
      return latestWalkingStatus.get() == WalkingStatus.COMPLETED;
   }

   @Override
   protected void finalize() throws Throwable
   {
      super.finalize();

      destroyListeners(); // In case the doTransitionOutOfAction() was not called somehow.
   }

   private void destroyListeners()
   {
      if (footstepStatusMessageListener != null)
      {
         walkingOutputManager.detachStatusMessageListener(FootstepStatusMessage.class, footstepStatusMessageListener);
         footstepStatusMessageListener = null;
      }
      if (walkingStatusMessageListener != null)
      {
         walkingOutputManager.detachStatusMessageListener(WalkingStatusMessage.class, walkingStatusMessageListener);
         walkingStatusMessageListener = null;
      }
   }
}
