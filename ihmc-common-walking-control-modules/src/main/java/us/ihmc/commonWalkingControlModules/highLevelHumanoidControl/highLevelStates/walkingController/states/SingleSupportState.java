package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.referenceFrames.WalkingTrajectoryPath;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class SingleSupportState extends WalkingState
{
   protected final RobotSide swingSide;
   protected final RobotSide supportSide;

   private final YoBoolean hasMinimumTimePassed = new YoBoolean("hasMinimumTimePassed", registry);
   protected final YoDouble minimumSwingFraction = new YoDouble("minimumSwingFraction", registry);

   protected final WalkingMessageHandler walkingMessageHandler;
   protected final WalkingTrajectoryPath walkingTrajectoryPath;
   protected final SideDependentList<FootSwitchInterface> footSwitches;
   protected final FullHumanoidRobotModel fullRobotModel;

   private final CenterOfMassHeightManager comHeightManager;

   public SingleSupportState(WalkingStateEnum singleSupportStateEnum,
                             WalkingMessageHandler walkingMessageHandler,
                             HighLevelHumanoidControllerToolbox controllerToolbox,
                             HighLevelControlManagerFactory managerFactory,
                             YoRegistry parentRegistry)
   {
      super(singleSupportStateEnum, managerFactory, controllerToolbox, parentRegistry);

      this.supportSide = singleSupportStateEnum.getSupportSide();
      swingSide = supportSide.getOppositeSide();

      minimumSwingFraction.set(0.5);

      this.walkingMessageHandler = walkingMessageHandler;
      walkingTrajectoryPath = controllerToolbox.getWalkingTrajectoryPath();
      footSwitches = controllerToolbox.getFootSwitches();
      fullRobotModel = controllerToolbox.getFullRobotModel();

      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();
   }

   public RobotSide getSwingSide()
   {
      return swingSide;
   }

   @Override
   public RobotSide getSupportSide()
   {
      return supportSide;
   }

   @Override
   public void doAction(double timeInState)
   {
   }

   @Override
   public boolean isDone(double timeInState)
   {
      hasMinimumTimePassed.set(hasMinimumTimePassed(timeInState));

      if (!hasMinimumTimePassed.getBooleanValue())
         return false;

      return hasMinimumTimePassed.getBooleanValue() && footSwitches.get(swingSide).hasFootHitGroundFiltered();
   }

   protected abstract boolean hasMinimumTimePassed(double timeInState);

   @Override
   public void onEntry()
   {
      balanceManager.clearICPPlan();
      footSwitches.get(swingSide).reset();
      balanceManager.setHoldSplitFractions(false);

      comHeightManager.setSupportLeg(swingSide.getOppositeSide());
      initializeWalkingTrajectoryPath();
   }

   @Override
   public void onExit(double timeInState)
   {
   }

   public void initializeWalkingTrajectoryPath()
   {
      walkingTrajectoryPath.clearFootsteps();
      walkingTrajectoryPath.addFootsteps(walkingMessageHandler);
      walkingTrajectoryPath.initializeSingleSupport(supportSide);
   }
}