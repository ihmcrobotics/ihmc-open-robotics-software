package us.ihmc.avatar.networkProcessor.walkingPreview;

import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager.StatusMessageListener;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class FootstepListPreviewTask implements WalkingPreviewTask
{
   private final FootstepDataListCommand footstepList;
   private final CommandInputManager walkingInputManager;
   private final StatusMessageOutputManager walkingOutputManager;

   private int numberOfFootstepsRemaining;
   private final AtomicReference<WalkingStatus> latestWalkingStatus = new AtomicReference<>(null);
   private StatusMessageListener<FootstepStatusMessage> footstepStatusMessageListener = this::processFootstepStatus;
   private StatusMessageListener<WalkingStatusMessage> walkingStatusMessageListener = this::processWalkingStatus;

   private final SideDependentList<YoPlaneContactState> footContactStates;
   private final SideDependentList<WalkingPreviewContactPointHolder> contactStateHolders = new SideDependentList<>();
   private final InverseDynamicsCommandList commandList = new InverseDynamicsCommandList();

   public FootstepListPreviewTask(FootstepDataListCommand footstepList, CommandInputManager walkingInputManager,
                                  StatusMessageOutputManager walkingOutputManager, SideDependentList<YoPlaneContactState> footContactStates)
   {
      this.footstepList = footstepList;
      this.walkingInputManager = walkingInputManager;
      this.walkingOutputManager = walkingOutputManager;
      this.footContactStates = footContactStates;
   }

   @Override
   public void doTransitionIntoAction()
   {
      for (RobotSide robotSide : RobotSide.values)
         contactStateHolders.put(robotSide, WalkingPreviewContactPointHolder.holdAtCurrent(footContactStates.get(robotSide)));

      walkingInputManager.submitCommand(footstepList);
      numberOfFootstepsRemaining = footstepList.getNumberOfFootsteps();
      walkingOutputManager.attachStatusMessageListener(FootstepStatusMessage.class, footstepStatusMessageListener);
      walkingOutputManager.attachStatusMessageListener(WalkingStatusMessage.class, walkingStatusMessageListener);
   }

   private void processFootstepStatus(FootstepStatusMessage statusMessage)
   {
      RobotSide side = RobotSide.fromByte(statusMessage.getRobotSide());
      FootstepStatus status = FootstepStatus.fromByte(statusMessage.getFootstepStatus());
      FramePose3D desiredFootstep = new FramePose3D(ReferenceFrame.getWorldFrame(), statusMessage.getDesiredFootPositionInWorld(),
                                                    statusMessage.getDesiredFootOrientationInWorld());

      switch (status)
      {
      case STARTED:
         contactStateHolders.put(side, null);
         break;
      case COMPLETED:
         numberOfFootstepsRemaining--;
         contactStateHolders.put(side, new WalkingPreviewContactPointHolder(footContactStates.get(side), desiredFootstep));
         break;
      default:
         throw new RuntimeException("Unexpected status: " + status);
      }
   }

   private void processWalkingStatus(WalkingStatusMessage status)
   {
      latestWalkingStatus.set(WalkingStatus.fromByte(status.getWalkingStatus()));
   }

   @Override
   public void doAction()
   {
      commandList.clear();

      for (RobotSide robotSide : RobotSide.values)
      {
         WalkingPreviewContactPointHolder contactStateHolder = contactStateHolders.get(robotSide);
         if (contactStateHolder == null)
            continue;
         contactStateHolder.doControl();
         commandList.addCommand(contactStateHolder.getOutput());
      }
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
   public InverseDynamicsCommand<?> getOutput()
   {
      return commandList;
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
