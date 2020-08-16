package us.ihmc.simpleWholeBodyWalking.states;

import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.simpleWholeBodyWalking.SimpleBalanceManager;
import us.ihmc.simpleWholeBodyWalking.SimpleControlManagerFactory;
import us.ihmc.simpleWholeBodyWalking.SimplePelvisOrientationManager;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SimpleStandingState extends SimpleWalkingState
{
   private final CommandInputManager commandInputManager;
   private final WalkingMessageHandler walkingMessageHandler;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final SimpleBalanceManager balanceManager;
   private final SimplePelvisOrientationManager pelvisOrientationManager;
   private final SideDependentList<RigidBodyControlManager> handManagers = new SideDependentList<>();

   public SimpleStandingState(CommandInputManager commandInputManager, WalkingMessageHandler walkingMessageHandler,
                              HighLevelHumanoidControllerToolbox controllerToolbox, SimpleControlManagerFactory managerFactory,
                              WalkingFailureDetectionControlModule failureDetectionControlModule,
                              YoRegistry parentRegistry)
   {
      super(SimpleWalkingStateEnum.STANDING, parentRegistry);

      this.commandInputManager = commandInputManager;
      this.walkingMessageHandler = walkingMessageHandler;
      this.controllerToolbox = controllerToolbox;
      this.failureDetectionControlModule = failureDetectionControlModule;

      RigidBodyBasics chest = controllerToolbox.getFullRobotModel().getChest();
      if (chest != null)
      {
         ReferenceFrame chestBodyFrame = chest.getBodyFixedFrame();

         for (RobotSide robotSide : RobotSide.values)
         {
            RigidBodyBasics hand = controllerToolbox.getFullRobotModel().getHand(robotSide);
            if (hand != null)
            {
               ReferenceFrame handControlFrame = controllerToolbox.getFullRobotModel().getHandControlFrame(robotSide);
               RigidBodyControlManager handManager = managerFactory.getOrCreateRigidBodyManager(hand, chest, handControlFrame, chestBodyFrame);
               handManagers.put(robotSide, handManager);
            }
         }
      }

      balanceManager = managerFactory.getOrCreateBalanceManager();
      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
   }

   @Override
   public void doAction(double timeInState)
   {
   }

   @Override
   public void onEntry()
   {
      commandInputManager.clearAllCommands();

      // need to always update biped support polygons after a change to the contact states
      controllerToolbox.updateBipedSupportPolygons();

      balanceManager.enablePelvisXYControl();
      balanceManager.setICPPlanTransferFromSide(null);
      balanceManager.initializeICPPlanForStanding();

      walkingMessageHandler.reportWalkingComplete();

      if (pelvisOrientationManager != null)
         pelvisOrientationManager.initializeStanding();

      failureDetectionControlModule.setNextFootstep(null);
      controllerToolbox.reportChangeOfRobotMotionStatus(RobotMotionStatus.STANDING);

   }

   @Override
   public void onExit()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (handManagers.get(robotSide) != null)
            handManagers.get(robotSide).prepareForLocomotion();
      }

      if (pelvisOrientationManager != null)
      {
         pelvisOrientationManager.prepareForLocomotion(walkingMessageHandler.getNextStepTime());
      }

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
   public boolean isDone(double timeInState)
   {
      return true;
   }
}