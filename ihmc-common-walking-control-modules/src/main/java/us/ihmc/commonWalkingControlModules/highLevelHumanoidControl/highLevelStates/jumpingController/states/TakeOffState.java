package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.states;

import java.util.Map;

import us.ihmc.commonWalkingControlModules.controlModules.flight.CentroidalMomentumManager;
import us.ihmc.commonWalkingControlModules.controlModules.flight.FeetJumpManager;
import us.ihmc.commonWalkingControlModules.controlModules.flight.GravityCompensationManager;
import us.ihmc.commonWalkingControlModules.controlModules.flight.PelvisControlManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
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

   public TakeOffState(CentroidalMomentumManager centroidalMomentumManager, GravityCompensationManager gravityCompensationManager,
                       PelvisControlManager pelvisControlManager, SideDependentList<RigidBodyControlManager> handManagers, FeetJumpManager feetManager,
                       Map<String, RigidBodyControlManager> bodyManagerMap, FullHumanoidRobotModel fullRobotModel)
   {
      super(stateEnum);
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
   public void doTransitionIntoAction()
   {

   }

   @Override
   public void doTransitionOutOfAction()
   {

   }
}
