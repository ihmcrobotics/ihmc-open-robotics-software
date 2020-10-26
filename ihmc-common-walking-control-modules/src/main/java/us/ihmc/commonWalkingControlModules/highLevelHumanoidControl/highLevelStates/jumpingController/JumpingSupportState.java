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

public class JumpingSupportState extends JumpingState
{
   private final JumpingControllerToolbox controllerToolbox;
   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final JumpingGoalHandler jumpingGoalHandler;
   private final JumpingGoal jumpingGoal = new JumpingGoal();

   private final JumpingBalanceManager balanceManager;
   private final JumpingPelvisOrientationManager pelvisOrientationManager;
   private final SideDependentList<RigidBodyControlManager> handManagers = new SideDependentList<>();

   public JumpingSupportState(JumpingGoalHandler jumpingGoalHandler,
                              JumpingControllerToolbox controllerToolbox,
                              JumpingControlManagerFactory managerFactory,
                              WalkingFailureDetectionControlModule failureDetectionControlModule,
                              YoRegistry parentRegistry)
   {
      super(JumpingStateEnum.SUPPORT, parentRegistry);

      this.jumpingGoalHandler = jumpingGoalHandler;
      this.controllerToolbox = controllerToolbox;
      this.failureDetectionControlModule = failureDetectionControlModule;

      balanceManager = managerFactory.getOrCreateBalanceManager();

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

      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
   }

   @Override
   public void doAction(double timeInState)
   {
      balanceManager.computeCoMPlanForJumping(jumpingGoal);
   }

   @Override
   public void onEntry()
   {
      jumpingGoalHandler.peekNextJumpingGoal(jumpingGoal);

      // need to always update biped support polygons after a change to the contact states
      controllerToolbox.updateBipedSupportPolygons();

      balanceManager.initializeCoMPlanForSupport(jumpingGoal);

      if (pelvisOrientationManager != null)
         pelvisOrientationManager.initializeStanding();

//      failureDetectionControlModule.setNextFootstep(null);
   }

   @Override
   public void onExit()
   {

   }

   @Override
   public boolean isDone(double timeInState)

   // TODO add a fallback finished. Also add criteria on "legs are straight" and "feet are unloaded"
   {
      if (timeInState >= jumpingGoal.getSupportDuration())
         return true;
      else
         return false;
   }
}