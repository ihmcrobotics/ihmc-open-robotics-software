package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.commonWalkingControlModules.capturePoint.BalanceManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class SingleSupportState extends WalkingState
{
   protected final RobotSide swingSide;
   protected final RobotSide supportSide;

   private final YoBoolean hasMinimumTimePassed = new YoBoolean("hasMinimumTimePassed", registry);
   protected final YoDouble minimumSwingFraction = new YoDouble("minimumSwingFraction", registry);

   protected final WalkingMessageHandler walkingMessageHandler;
   protected final SideDependentList<FootSwitchInterface> footSwitches;
   protected final FullHumanoidRobotModel fullRobotModel;

   protected final BalanceManager balanceManager;

   public SingleSupportState(WalkingStateEnum singleSupportStateEnum, WalkingMessageHandler walkingMessageHandler,
                             HighLevelHumanoidControllerToolbox controllerToolbox, HighLevelControlManagerFactory managerFactory,
                             YoVariableRegistry parentRegistry)
   {
      super(singleSupportStateEnum, parentRegistry);

      this.supportSide = singleSupportStateEnum.getSupportSide();
      swingSide = supportSide.getOppositeSide();

      minimumSwingFraction.set(0.5);

      this.walkingMessageHandler = walkingMessageHandler;
      footSwitches = controllerToolbox.getFootSwitches();
      fullRobotModel = controllerToolbox.getFullRobotModel();

      balanceManager = managerFactory.getOrCreateBalanceManager();
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

      return hasMinimumTimePassed.getBooleanValue() && footSwitches.get(swingSide).hasFootHitGround();
   }

   protected abstract boolean hasMinimumTimePassed(double timeInState);

   @Override
   public void onEntry()
   {
      balanceManager.clearICPPlan();
      footSwitches.get(swingSide).reset();
      integrateAnkleAccelerationsOnSwingLeg(swingSide);
   }

   @Override
   public void onExit()
   {
      balanceManager.resetPushRecovery();
      resetLoadedLegIntegrators(swingSide);
   }

   private void integrateAnkleAccelerationsOnSwingLeg(RobotSide swingSide)
   {
      fullRobotModel.getLegJoint(swingSide, LegJointName.ANKLE_PITCH).setIntegrateDesiredAccelerations(true);
      fullRobotModel.getLegJoint(swingSide, LegJointName.ANKLE_ROLL).setIntegrateDesiredAccelerations(true);
      fullRobotModel.getLegJoint(swingSide.getOppositeSide(), LegJointName.ANKLE_PITCH).setIntegrateDesiredAccelerations(false);
      fullRobotModel.getLegJoint(swingSide.getOppositeSide(), LegJointName.ANKLE_ROLL).setIntegrateDesiredAccelerations(false);
   }

   private void resetLoadedLegIntegrators(RobotSide robotSide)
   {
      for (LegJointName jointName : fullRobotModel.getRobotSpecificJointNames().getLegJointNames())
         fullRobotModel.getLegJoint(robotSide, jointName).resetDesiredAccelerationIntegrator();
   }
}