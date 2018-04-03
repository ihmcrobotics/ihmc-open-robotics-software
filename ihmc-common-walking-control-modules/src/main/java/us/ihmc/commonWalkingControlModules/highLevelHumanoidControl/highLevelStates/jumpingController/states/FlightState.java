package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.states;

import java.util.Map;

import us.ihmc.commonWalkingControlModules.controlModules.flight.CentroidalMomentumManager;
import us.ihmc.commonWalkingControlModules.controlModules.flight.FeetJumpManager;
import us.ihmc.commonWalkingControlModules.controlModules.flight.JumpMessageHandler;
import us.ihmc.commonWalkingControlModules.controlModules.flight.PelvisControlManager;
import us.ihmc.commonWalkingControlModules.controlModules.flight.WholeBodyMotionPlanner;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class FlightState extends AbstractJumpState
{
   private static final JumpStateEnum stateEnum = JumpStateEnum.FLIGHT;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final CentroidalMomentumManager wholeBodyMomentumManager;
   private final FeetJumpManager feetManager;
   private final PelvisControlManager pelvisManager;
   private final SideDependentList<RigidBodyControlManager> handManagers;
   private final RigidBodyControlManager chestManager;
   private final RigidBodyControlManager headManager;
   private final double contactThreshold = 30.0;
   private final double footHeightThresholdForCollisionChecking = 0.005;
   private final YoBoolean isDone;

   public FlightState(WholeBodyMotionPlanner motionPlanner, JumpMessageHandler messageHandler, HighLevelHumanoidControllerToolbox controllerToolbox,
                      WholeBodyControlCoreToolbox controlCoreToolbox, CentroidalMomentumManager centroidalMomentumManager, PelvisControlManager pelvisManager,
                      SideDependentList<RigidBodyControlManager> handManagers, FeetJumpManager feetManager, Map<String, RigidBodyControlManager> bodyManagerMap,
                      YoVariableRegistry registry)
   {
      super(stateEnum, motionPlanner, messageHandler, controllerToolbox);
      this.controllerToolbox = controllerToolbox;
      this.wholeBodyMomentumManager = centroidalMomentumManager;
      this.feetManager = feetManager;
      this.pelvisManager = pelvisManager;
      this.handManagers = handManagers;
      this.chestManager = bodyManagerMap.get(controllerToolbox.getFullRobotModel().getChest().getName());
      this.headManager = bodyManagerMap.get(controllerToolbox.getFullRobotModel().getHead().getName());
      isDone = new YoBoolean(getClass().getSimpleName() + "isDone", registry);
   }

   @Override
   public boolean isDone()
   {
      isDone.set(checkIfFeetHaveCollidedWithGround());
      return isDone.getBooleanValue();
   }

   @Override
   public void doAction()
   {
      wholeBodyMomentumManager.computeMomentumRateOfChangeForFreeFall();
      pelvisManager.maintainDesiredOrientationOnly();
      feetManager.compute();
      for (RobotSide side : RobotSide.values)
         handManagers.get(side).compute();
      chestManager.compute();
      headManager.compute();
   }

   private boolean checkIfFeetHaveCollidedWithGround()
   {
      boolean checkForCollision = false;
      for (RobotSide robotSide : RobotSide.values)
      {
         double footHeightAboveGround = feetManager.getHeightOfLowestPointInFoot(robotSide);
         checkForCollision |= (footHeightAboveGround < footHeightThresholdForCollisionChecking);
      }
      if (checkForCollision)
      {
         double totalGroundReactionZ = feetManager.getGroundReactionForceZ();
         if (totalGroundReactionZ > contactThreshold)
            return true;
         else
            return false;
      }
      else
         return false;
   }

   @Override
   public void doStateSpecificTransitionIntoAction()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyControlManager handManager = handManagers.get(robotSide);
         handManager.holdInTaskspace();
         feetManager.holdInJointspace(robotSide);
         feetManager.makeFeetFullyUnconstrained(robotSide);
      }
      headManager.holdInTaskspace();
      chestManager.holdInTaskspace();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      // TODO Auto-generated method stub

   }

}
