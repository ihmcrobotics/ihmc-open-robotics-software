package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.configurations.LeapOfFaithParameters;
import us.ihmc.commonWalkingControlModules.configurations.PelvisOffsetWhileWalkingParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisOrientationTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class PelvisOrientationManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final StateMachine<PelvisOrientationControlMode, PelvisOrientationControlState> stateMachine;
   private final YoEnum<PelvisOrientationControlMode> requestedState;
   private final YoBoolean enableUserPelvisControlDuringWalking = new YoBoolean("EnableUserPelvisControlDuringWalking", registry);
   private final YoBoolean doPrepareForLocomotion = new YoBoolean("doPreparePelvisForLocomotion", registry);

   private final ControllerPelvisOrientationManager walkingManager;
   private final UserPelvisOrientationManager userManager;

   public PelvisOrientationManager(PID3DGainsReadOnly gains, PelvisOffsetWhileWalkingParameters pelvisOffsetWhileWalkingParameters,
                                   LeapOfFaithParameters leapOfFaithParameters, HighLevelHumanoidControllerToolbox controllerToolbox,
                                   YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      YoDouble yoTime = controllerToolbox.getYoTime();
      String namePrefix = getClass().getSimpleName();
      requestedState = new YoEnum<>(namePrefix + "RequestedControlMode", registry, PelvisOrientationControlMode.class, true);

      walkingManager = new ControllerPelvisOrientationManager(gains, pelvisOffsetWhileWalkingParameters, leapOfFaithParameters, controllerToolbox, registry);
      userManager = new UserPelvisOrientationManager(gains, controllerToolbox, registry);

      stateMachine = setupStateMachine(namePrefix, yoTime);

      enableUserPelvisControlDuringWalking.set(false);
   }

   private StateMachine<PelvisOrientationControlMode, PelvisOrientationControlState> setupStateMachine(String namePrefix, DoubleProvider timeProvider)
   {
      StateMachineFactory<PelvisOrientationControlMode, PelvisOrientationControlState> factory = new StateMachineFactory<>(PelvisOrientationControlMode.class);
      factory.setNamePrefix(namePrefix).setRegistry(registry).buildYoClock(timeProvider);

      factory.addState(PelvisOrientationControlMode.WALKING_CONTROLLER, walkingManager);
      factory.addState(PelvisOrientationControlMode.USER, userManager);

      for (PelvisOrientationControlMode from : PelvisOrientationControlMode.values())
      {
         factory.addRequestedTransition(from, requestedState);
         factory.addRequestedTransition(from, from, requestedState);
      }

      return factory.build(PelvisOrientationControlMode.WALKING_CONTROLLER);
   }

   public void setWeights(Vector3DReadOnly weight)
   {
      walkingManager.setWeights(weight);
      userManager.setWeights(weight);
   }

   public void setPrepareForLocomotion(boolean value)
   {
      doPrepareForLocomotion.set(value);
   }

   public void compute()
   {
      stateMachine.doActionAndTransition();
   }

   public void initialize()
   {
      walkingManager.resetOrientationOffset();
      requestState(PelvisOrientationControlMode.WALKING_CONTROLLER);
      walkingManager.setToZeroInMidFeetZUpFrame();
   }

   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      if (stateMachine.getCurrentStateKey() == PelvisOrientationControlMode.USER)
         walkingManager.setOffset(userManager.getDesiredOrientation());
      requestState(PelvisOrientationControlMode.WALKING_CONTROLLER);
   }

   public void prepareForLocomotion(double trajectoryTime)
   {
      if (enableUserPelvisControlDuringWalking.getBooleanValue())
         return;

      if (doPrepareForLocomotion.getValue())
      {
         if (stateMachine.getCurrentStateKey() == PelvisOrientationControlMode.WALKING_CONTROLLER)
            walkingManager.goToHomeFromCurrentDesired(trajectoryTime);
         else
            walkingManager.goToHomeFromOffset(userManager.getDesiredOrientation(), trajectoryTime);
      }
      else
      {
         if (stateMachine.getCurrentStateKey() == PelvisOrientationControlMode.USER)
            walkingManager.setOffset(userManager.getDesiredOrientation());
      }
      requestState(PelvisOrientationControlMode.WALKING_CONTROLLER);
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return stateMachine.getCurrentState().getFeedbackControlCommand();
   }

   public void goToHomeFromCurrentDesired(double trajectoryTime)
   {
      if (stateMachine.getCurrentStateKey() == PelvisOrientationControlMode.USER)
         walkingManager.setOffset(userManager.getDesiredOrientation());
      walkingManager.goToHomeFromCurrentDesired(trajectoryTime);
      requestState(PelvisOrientationControlMode.WALKING_CONTROLLER);
   }

   public void setTrajectoryTime(double trajectoryTime)
   {
      walkingManager.setTrajectoryTime(trajectoryTime);
   }

   public void moveToAverageInSupportFoot(RobotSide supportSide)
   {
      walkingManager.moveToAverageInSupportFoot(supportSide);
   }

   public void resetOrientationOffset()
   {
      walkingManager.resetOrientationOffset();
   }

   public void setToHoldCurrentDesiredInMidFeetZUpFrame()
   {
      walkingManager.setToHoldCurrentDesiredInMidFeetZUpFrame();
   }

   public void centerInMidFeetZUpFrame(double trajectoryTime)
   {
      walkingManager.centerInMidFeetZUpFrame(trajectoryTime);
   }

   public void setToHoldCurrentDesiredInSupportFoot(RobotSide supportSide)
   {
      walkingManager.setToHoldCurrentDesiredInSupportFoot(supportSide);
   }

   public void setToHoldCurrentInWorldFrame()
   {
      walkingManager.setToHoldCurrentInWorldFrame();
   }

   public void setToZeroInMidFeetZUpFrame()
   {
      walkingManager.setToZeroInMidFeetZUpFrame();
   }

   public void initializeStanding()
   {
      walkingManager.initializeStanding();
   }

   public void initializeSwing(RobotSide supportSide, double swingDuration, double nextTransferDuration, double nextSwingDuration)
   {
      walkingManager.initializeSwing(supportSide, swingDuration, nextTransferDuration, nextSwingDuration);
   }

   public void setUpcomingFootstep(Footstep upcomingFootstep)
   {
      walkingManager.setUpcomingFootstep(upcomingFootstep);
   }

   public void initializeTransfer(RobotSide transferToSide, double transferDuration, double swingDuration)
   {
      walkingManager.initializeTransfer(transferToSide, transferDuration, swingDuration);
   }

   public void initializeTrajectory()
   {
      walkingManager.updateTrajectoryFromFootstep();
   }

   public void updateTrajectoryFromFootstep()
   {
      walkingManager.updateTrajectoryFromFootstep();
   }

   public void setTrajectoryFromFootstep()
   {
      walkingManager.setTrajectoryFromFootstep();
   }

   public boolean handlePelvisOrientationTrajectoryCommands(PelvisOrientationTrajectoryCommand command)
   {
      if (command.getSO3Trajectory().useCustomControlFrame())
      {
         LogTools.warn("Can not use custom control frame with pelvis orientation.");
         return false;
      }
      enableUserPelvisControlDuringWalking.set(command.isEnableUserPelvisControlDuringWalking());
      if (userManager.handlePelvisOrientationTrajectoryCommands(command))
      {
         requestState(PelvisOrientationControlMode.USER);
         return true;
      }
      else
      {
         walkingManager.setToHoldCurrentInWorldFrame();
         requestState(PelvisOrientationControlMode.WALKING_CONTROLLER);
         return false;
      }
   }

   private final PelvisOrientationTrajectoryCommand tempPelvisOrientationTrajectoryCommand = new PelvisOrientationTrajectoryCommand();

   public boolean handlePelvisTrajectoryCommand(PelvisTrajectoryCommand command)
   {
      SelectionMatrix3D angularSelectionMatrix = command.getSE3Trajectory().getSelectionMatrix().getAngularPart();

      if (angularSelectionMatrix.isXSelected() || angularSelectionMatrix.isYSelected() || angularSelectionMatrix.isZSelected())
      { // At least one axis is to be controlled, process the command.
         tempPelvisOrientationTrajectoryCommand.set(command);
         return handlePelvisOrientationTrajectoryCommands(tempPelvisOrientationTrajectoryCommand);
      }
      else
      { // The user does not want to control the pelvis orientation, do nothing.
        // TODO Has to return true otherwise the command won't get to the height and XY managers.
         return true;
      }
   }

   private void requestState(PelvisOrientationControlMode state)
   {
      if (stateMachine.getCurrentStateKey() != state)
      {
         requestedState.set(state);
      }
   }

   public void setSelectionMatrix(SelectionMatrix3D selectionMatrix)
   {
      walkingManager.setSelectionMatrix(selectionMatrix);
   }

   public TaskspaceTrajectoryStatusMessage pollStatusToReport()
   {
      return userManager.pollStatusToReport();
   }

   public FeedbackControlCommandList createFeedbackControlTemplate()
   {
      FeedbackControlCommandList ret = new FeedbackControlCommandList();
      for (PelvisOrientationControlMode mode : PelvisOrientationControlMode.values())
      {
         PelvisOrientationControlState state = stateMachine.getState(mode);
         if (state != null && state.getFeedbackControlCommand() != null)
            ret.addCommand(state.getFeedbackControlCommand());
      }
      return ret;
   }
}
