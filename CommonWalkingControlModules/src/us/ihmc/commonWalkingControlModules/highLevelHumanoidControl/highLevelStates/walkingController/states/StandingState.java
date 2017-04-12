package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import java.util.Collection;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.BalanceManager;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;

public class StandingState extends WalkingState
{
   private final CommandInputManager commandInputManager;
   private final WalkingMessageHandler walkingMessageHandler;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final CenterOfMassHeightManager comHeightManager;
   private final BalanceManager balanceManager;
   private final PelvisOrientationManager pelvisOrientationManager;
   private final SideDependentList<RigidBodyControlManager> handManagers = new SideDependentList<>();

   private final BooleanYoVariable doPrepareManipulationForLocomotion = new BooleanYoVariable("doPrepareManipulationForLocomotion", registry);
   private final BooleanYoVariable doPreparePelvisForLocomotion = new BooleanYoVariable("doPreparePelvisForLocomotion", registry);

   public StandingState(CommandInputManager commandInputManager, WalkingMessageHandler walkingMessageHandler, HighLevelHumanoidControllerToolbox controllerToolbox,
         HighLevelControlManagerFactory managerFactory, WalkingFailureDetectionControlModule failureDetectionControlModule,
         WalkingControllerParameters walkingControllerParameters, YoVariableRegistry parentRegistry)
   {
      super(WalkingStateEnum.STANDING, parentRegistry);

      this.commandInputManager = commandInputManager;
      this.walkingMessageHandler = walkingMessageHandler;
      this.controllerToolbox = controllerToolbox;
      this.failureDetectionControlModule = failureDetectionControlModule;

      RigidBody chest = controllerToolbox.getFullRobotModel().getChest();
      ReferenceFrame chestBodyFrame = chest.getBodyFixedFrame();
      Collection<ReferenceFrame> trajectoryFrames = controllerToolbox.getTrajectoryFrames();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody hand = controllerToolbox.getFullRobotModel().getHand(robotSide);
         ReferenceFrame handControlFrame = controllerToolbox.getFullRobotModel().getHandControlFrame(robotSide);
         RigidBodyControlManager handManager = managerFactory.getOrCreateRigidBodyManager(hand, chest, handControlFrame, chestBodyFrame, trajectoryFrames);
         handManagers.put(robotSide, handManager);
      }

      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();
      balanceManager = managerFactory.getOrCreateBalanceManager();
      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();

      doPrepareManipulationForLocomotion.set(walkingControllerParameters.doPrepareManipulationForLocomotion());
      doPreparePelvisForLocomotion.set(walkingControllerParameters.doPreparePelvisForLocomotion());
   }

   @Override
   public void doAction()
   {
      comHeightManager.setSupportLeg(RobotSide.LEFT);
   }

   @Override
   public void doTransitionIntoAction()
   {
      commandInputManager.flushAllCommands();

      balanceManager.clearICPPlan();
      balanceManager.resetPushRecovery();
      balanceManager.enablePelvisXYControl();

      controllerToolbox.updateBipedSupportPolygons(); // need to always update biped support polygons after a change to the contact states

      walkingMessageHandler.reportWalkingComplete();

      failureDetectionControlModule.setNextFootstep(null);
      controllerToolbox.reportChangeOfRobotMotionStatus(RobotMotionStatus.STANDING);
   }

   @Override
   public void doTransitionOutOfAction()
   {
      if (doPrepareManipulationForLocomotion.getBooleanValue())
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            if (handManagers.get(robotSide) != null)
               handManagers.get(robotSide).holdInJointspace();
         }
      }

      if (pelvisOrientationManager != null && doPreparePelvisForLocomotion.getBooleanValue())
         pelvisOrientationManager.prepareForLocomotion();

      balanceManager.disablePelvisXYControl();
      controllerToolbox.reportChangeOfRobotMotionStatus(RobotMotionStatus.IN_MOTION);
   }

   @Override
   public boolean isStateSafeToConsumePelvisTrajectoryCommand()
   {
      return true;
   }

   @Override
   public boolean isStateSafeToConsumeManipulationCommands()
   {
      return true;
   }

   @Override
   public boolean isDone()
   {
      return true;
   }
}