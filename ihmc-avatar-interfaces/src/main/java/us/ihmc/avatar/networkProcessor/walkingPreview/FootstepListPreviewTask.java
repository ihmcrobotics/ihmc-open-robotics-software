package us.ihmc.avatar.networkProcessor.walkingPreview;

import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.capturePoint.BalanceManager;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.SettableFootSwitch;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager.StatusMessageListener;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class FootstepListPreviewTask implements WalkingPreviewTask
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FloatingJointReadOnly rootJoint;

   private final FootstepDataListCommand footstepList;
   private final CommandInputManager walkingInputManager;
   private final StatusMessageOutputManager walkingOutputManager;

   private int numberOfFootstepsRemaining;
   private final AtomicReference<WalkingStatus> latestWalkingStatus = new AtomicReference<>(null);
   private StatusMessageListener<FootstepStatusMessage> footstepStatusMessageListener = this::processFootstepStatus;
   private StatusMessageListener<WalkingStatusMessage> walkingStatusMessageListener = this::processWalkingStatus;

   private final SideDependentList<YoPlaneContactState> footContactStates;
   private final SideDependentList<WalkingPreviewContactStateHolder> contactStateHolders = new SideDependentList<>();
   private final InverseDynamicsCommandList commandList = new InverseDynamicsCommandList();

   private final SideDependentList<SettableFootSwitch> footSwitches;
   private RobotSide currentSwingSide = null;

   private final BalanceManager balanceManager;

   /**
    * Creates a new task for previewing a series of footsteps.
    * 
    * @param rootJoint used to determine when the robot is standing still once done with the footstep
    *           sequence.
    * @param footstepList the list of footsteps to preview.
    * @param walkingInputManager the input to the walking controller for submitting the footsteps.
    * @param walkingOutputManager the output of the walking controller for listening to footstep
    *           status.
    * @param footContactStates they are used get the active set of contact points which are then held
    *           in place by creating an additional QP objective.
    * @param balanceManager it is used to query the swing time remaining which is used to trigger the
    *           end-of-swing/beginning-of-support.
    * @param footSwitchesToUpdate this tasks updates the foot switch used in the walking controller.
    */
   public FootstepListPreviewTask(FloatingJointReadOnly rootJoint, FootstepDataListCommand footstepList, CommandInputManager walkingInputManager,
                                  StatusMessageOutputManager walkingOutputManager, SideDependentList<YoPlaneContactState> footContactStates,
                                  BalanceManager balanceManager, SideDependentList<SettableFootSwitch> footSwitchesToUpdate)
   {
      this.rootJoint = rootJoint;
      this.footstepList = footstepList;
      this.walkingInputManager = walkingInputManager;
      this.walkingOutputManager = walkingOutputManager;
      this.footContactStates = footContactStates;
      this.balanceManager = balanceManager;
      this.footSwitches = footSwitchesToUpdate;
   }

   @Override
   public void onEntry()
   {
      for (RobotSide robotSide : RobotSide.values)
         contactStateHolders.put(robotSide, WalkingPreviewContactStateHolder.holdAtCurrent(footContactStates.get(robotSide)));

      walkingInputManager.submitCommand(footstepList);
      numberOfFootstepsRemaining = footstepList.getNumberOfFootsteps();
      walkingOutputManager.attachStatusMessageListener(FootstepStatusMessage.class, footstepStatusMessageListener);
      walkingOutputManager.attachStatusMessageListener(WalkingStatusMessage.class, walkingStatusMessageListener);
   }

   private void processFootstepStatus(FootstepStatusMessage statusMessage)
   {
      RobotSide side = RobotSide.fromByte(statusMessage.getRobotSide());
      FootstepStatus status = FootstepStatus.fromByte(statusMessage.getFootstepStatus());
      FramePose3D desiredFootstep = new FramePose3D(worldFrame, statusMessage.getDesiredFootPositionInWorld(),
                                                    statusMessage.getDesiredFootOrientationInWorld());

      switch (status)
      {
      case STARTED:
         contactStateHolders.put(side, null);
         footSwitches.get(side).setFootContactState(false);
         currentSwingSide = side;
         break;
      case COMPLETED:
         numberOfFootstepsRemaining--;
         contactStateHolders.put(side, new WalkingPreviewContactStateHolder(footContactStates.get(side), desiredFootstep));
         currentSwingSide = null;
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
   public void doAction(double timeInState)
   {
      commandList.clear();

      for (RobotSide robotSide : RobotSide.values)
      {
         WalkingPreviewContactStateHolder contactStateHolder = contactStateHolders.get(robotSide);
         if (contactStateHolder == null)
            continue;
         contactStateHolder.doControl();
         commandList.addCommand(contactStateHolder.getOutput());
      }

      if (currentSwingSide != null)
      { // Testing for the end of swing purely relying on the swing time:
         if (balanceManager.isICPPlanDone())
            footSwitches.get(currentSwingSide).setFootContactState(true);
      }
   }

   @Override
   public void onExit()
   {
      destroyListeners();
   }

   @Override
   public boolean isDone(double timeInState)
   {
      if (numberOfFootstepsRemaining > 0)
         return false;
      if (latestWalkingStatus.get() == null)
         return false;
      return latestWalkingStatus.get() == WalkingStatus.COMPLETED && rootJoint.getJointTwist().getLinearPart().length() < 0.005;
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
