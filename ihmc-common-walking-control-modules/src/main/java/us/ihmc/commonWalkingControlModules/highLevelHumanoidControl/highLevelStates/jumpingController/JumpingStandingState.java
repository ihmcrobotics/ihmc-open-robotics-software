package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import us.ihmc.commonWalkingControlModules.capturePoint.JumpingBalanceManager;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class JumpingStandingState extends JumpingState
{
   private final JumpingControllerToolbox controllerToolbox;
   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final JumpingBalanceManager balanceManager;
   private final JumpingPelvisOrientationManager pelvisOrientationManager;
   private final RigidBodyControlManager chestManager;
   private final YoDouble standingHeight = new YoDouble("StandingHeight", registry);
   private final YoDouble squattingHeight = new YoDouble("SquattingHeight", registry);

   private final YoBoolean squat = new YoBoolean("ShouldBeSquatting", registry);

   public JumpingStandingState(JumpingControllerToolbox controllerToolbox,
                               JumpingControlManagerFactory managerFactory,
                               WalkingFailureDetectionControlModule failureDetectionControlModule,
                               YoRegistry parentRegistry)
   {
      super(JumpingStateEnum.STANDING, parentRegistry);

      this.controllerToolbox = controllerToolbox;
      this.failureDetectionControlModule = failureDetectionControlModule;
      this.balanceManager = managerFactory.getOrCreateBalanceManager();

      standingHeight.set(1.05);
      squattingHeight.set(0.6);

      squat.addListener(v ->
                        {
                           if (squat.getBooleanValue())
                              balanceManager.setDesiredCoMHeight(squattingHeight.getDoubleValue());
                           else
                              balanceManager.setDesiredCoMHeight(standingHeight.getDoubleValue());
                        });
      squat.notifyListeners();

      RigidBodyBasics chest = controllerToolbox.getFullRobotModel().getChest();
      RigidBodyBasics pelvis = controllerToolbox.getFullRobotModel().getPelvis();
      ReferenceFrame chestBodyFrame = chest.getBodyFixedFrame();
      ReferenceFrame pelvisBodyFrame = pelvis.getBodyFixedFrame();

      chestManager = managerFactory.getOrCreateRigidBodyManager(chest, pelvis, chestBodyFrame, pelvisBodyFrame);
      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
   }

   @Override
   public void doAction(double timeInState)
   {
      balanceManager.computeCoMPlanForStanding();
   }

   @Override
   public void onEntry()
   {
      // need to always update biped support polygons after a change to the contact states
      controllerToolbox.updateBipedSupportPolygons();

      balanceManager.initializeCoMPlanForStanding();

      if (pelvisOrientationManager != null)
         pelvisOrientationManager.initializeStanding();

      failureDetectionControlModule.setNextFootstep(null);
      controllerToolbox.reportChangeOfRobotMotionStatus(RobotMotionStatus.STANDING);
   }

   @Override
   public void onExit()
   {
      controllerToolbox.reportChangeOfRobotMotionStatus(RobotMotionStatus.IN_MOTION);
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return true;
   }
}