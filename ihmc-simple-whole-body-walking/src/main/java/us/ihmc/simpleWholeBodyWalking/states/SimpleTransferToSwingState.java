package us.ihmc.simpleWholeBodyWalking.states;

import org.apache.commons.math3.util.Precision;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.TouchdownErrorCompensator;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepShiftFractions;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.simpleWholeBodyWalking.SimpleControlManagerFactory;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimpleTransferToSwingState extends SimpleTransferState
{
   private static final int numberOfFootstepsToConsider = 3;
   private final Footstep[] footsteps = Footstep.createFootsteps(numberOfFootstepsToConsider);
   private final FootstepTiming[] footstepTimings = FootstepTiming.createTimings(numberOfFootstepsToConsider);
   private final FootstepShiftFractions[] footstepShiftFractions = FootstepShiftFractions.createShiftFractions(numberOfFootstepsToConsider);

   private final YoDouble currentTransferDuration = new YoDouble("CurrentTransferDuration", registry);

   public SimpleTransferToSwingState(SimpleWalkingStateEnum stateEnum,
                                     WalkingMessageHandler walkingMessageHandler,
                                     HighLevelHumanoidControllerToolbox controllerToolbox,
                                     SimpleControlManagerFactory managerFactory,
                                     WalkingFailureDetectionControlModule failureDetectionControlModule,
                                     YoVariableRegistry parentRegistry)
   {
      super(stateEnum, walkingMessageHandler, controllerToolbox, managerFactory, failureDetectionControlModule, parentRegistry);
   }

   @Override
   protected void updateICPPlan()
   {
      super.updateICPPlan();

      // This needs to check `TO_STANDING` as well as messages could be received on the very first controller tick at which point
      // the robot is not in the standing state but not yet walking either.
      if (getPreviousWalkingStateEnum() == SimpleWalkingStateEnum.STANDING || getPreviousWalkingStateEnum() == SimpleWalkingStateEnum.TO_STANDING)
      {
         walkingMessageHandler.reportWalkingStarted();
      }

      if (isInitialTransfer())
      {
         pelvisOrientationManager.moveToAverageInSupportFoot(transferToSide);
      }
      else
      {
         // In middle of walking or leaving foot pose, pelvis is good leave it like that.
         pelvisOrientationManager.setToHoldCurrentDesiredInSupportFoot(transferToSide);
      }

      double finalTransferTime = walkingMessageHandler.getFinalTransferTime();
      double finalTransferSplitFraction = walkingMessageHandler.getFinalTransferSplitFraction();
      double finalTransferWeightDistribution = walkingMessageHandler.getFinalTransferWeightDistribution();
      walkingMessageHandler.requestPlanarRegions();
      balanceManager.setFinalTransferTime(finalTransferTime);
      balanceManager.setFinalTransferSplitFraction(finalTransferSplitFraction);
      balanceManager.setFinalTransferWeightDistribution(finalTransferWeightDistribution);

      int stepsToAdd = Math.min(numberOfFootstepsToConsider, walkingMessageHandler.getCurrentNumberOfFootsteps());
      if (stepsToAdd < 1)
      {
         throw new RuntimeException("Can not go to walking single support if there are no upcoming footsteps.");
      }
      for (int i = 0; i < stepsToAdd; i++)
      {
         Footstep footstep = footsteps[i];
         FootstepTiming timing = footstepTimings[i];
         FootstepShiftFractions shiftFractions = footstepShiftFractions[i];
         walkingMessageHandler.peekFootstep(i, footstep);
         walkingMessageHandler.peekTiming(i, timing);
         walkingMessageHandler.peekShiftFraction(i, shiftFractions);

         balanceManager.addFootstepToPlan(footstep, timing, shiftFractions);
      }

      balanceManager.setICPPlanTransferToSide(transferToSide);
      FootstepTiming firstTiming = footstepTimings[0];
      currentTransferDuration.set(firstTiming.getTransferTime());
      balanceManager.initializeICPPlanForTransfer(firstTiming.getSwingTime(), firstTiming.getTransferTime(), finalTransferTime);

      if (balanceManager.wasTimingAdjustedForReachability())
      {
         double currentTransferDuration = balanceManager.getCurrentTransferDurationAdjustedForReachability();
         double currentSwingDuration = balanceManager.getCurrentSwingDurationAdjustedForReachability();
         firstTiming.setTimings(currentSwingDuration, currentTransferDuration);
      }

      pelvisOrientationManager.setUpcomingFootstep(footsteps[0]);
      pelvisOrientationManager.initializeTransfer(transferToSide, firstTiming.getTransferTime(), firstTiming.getSwingTime());
   }

   @Override
   public void doAction(double timeInState)
   {
      super.doAction(timeInState);
   }

   @Override
   public void onExit()
   {
      super.onExit();

      balanceManager.minimizeAngularMomentumRateZ(false);
   }
}