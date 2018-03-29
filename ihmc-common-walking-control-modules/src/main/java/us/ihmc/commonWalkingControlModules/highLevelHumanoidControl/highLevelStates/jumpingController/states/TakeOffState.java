package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.states;

import java.util.Map;

import us.ihmc.commonWalkingControlModules.controlModules.flight.CentroidalMomentumManager;
import us.ihmc.commonWalkingControlModules.controlModules.flight.FeetJumpManager;
import us.ihmc.commonWalkingControlModules.controlModules.flight.GravityCompensationManager;
import us.ihmc.commonWalkingControlModules.controlModules.flight.JumpMessageHandler;
import us.ihmc.commonWalkingControlModules.controlModules.flight.PelvisControlManager;
import us.ihmc.commonWalkingControlModules.controlModules.flight.WholeBodyMotionPlanner;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.SideDependentList;

public class TakeOffState extends AbstractJumpState
{
   private static final JumpStateEnum stateEnum = JumpStateEnum.TAKE_OFF;

   private final CentroidalMomentumManager centroidalMomentumManager;
   private final GravityCompensationManager gravityCompensationManager;
   private final PelvisControlManager pelvisControlManager;
   private final SideDependentList<RigidBodyControlManager> handManagers;
   private final FeetJumpManager feetManager;
   private final Map<String, RigidBodyControlManager> bodyManagerMap;
   private final FullHumanoidRobotModel fullRobotModel;
   private final WholeBodyMotionPlanner motionPlanner;

   public TakeOffState(WholeBodyMotionPlanner motionPlanner, JumpMessageHandler messageHandler, HighLevelHumanoidControllerToolbox controllerToolbox,
                       CentroidalMomentumManager centroidalMomentumManager, GravityCompensationManager gravityCompensationManager,
                       PelvisControlManager pelvisControlManager, SideDependentList<RigidBodyControlManager> handManagers, FeetJumpManager feetManager,
                       Map<String, RigidBodyControlManager> bodyManagerMap, FullHumanoidRobotModel fullRobotModel)
   {
      super(stateEnum, motionPlanner, messageHandler, controllerToolbox);
      this.motionPlanner = motionPlanner;
      this.centroidalMomentumManager = centroidalMomentumManager;
      this.gravityCompensationManager = gravityCompensationManager;
      this.pelvisControlManager = pelvisControlManager;
      this.handManagers = handManagers;
      this.feetManager = feetManager;
      this.bodyManagerMap = bodyManagerMap;
      this.fullRobotModel = fullRobotModel;
   }

   @Override
   public boolean isDone()
   {
      return false;
   }

   @Override
   public void doAction()
   {
      double timeInCurrentState = getTimeInCurrentState();
      centroidalMomentumManager.computeMomentumRateOfChangeFromForceProfile(timeInCurrentState);
   }

   @Override
   public void doStateSpecificTransitionIntoAction()
   {
      centroidalMomentumManager.setGroundReactionForceProfile(motionPlanner.getGroundReactionForceProfile());
      centroidalMomentumManager.setCoMTrajectory(motionPlanner.getPositionTrajectory());
   }

   @Override
   public void doTransitionOutOfAction()
   {

   }
}
