package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.states;

import java.util.Map;

import us.ihmc.commonWalkingControlModules.controlModules.flight.CentroidalMomentumManager;
import us.ihmc.commonWalkingControlModules.controlModules.flight.FeetJumpManager;
import us.ihmc.commonWalkingControlModules.controlModules.flight.GravityCompensationManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class StandingState extends AbstractJumpingState
{
   private static final JumpStateEnum stateEnum = JumpStateEnum.STANDING;

   private final CentroidalMomentumManager centroidalMomentumManager;
   private final GravityCompensationManager gravityCompensationManager;
   private final SideDependentList<RigidBodyControlManager> handManagers;
   private final FeetJumpManager feetManager;
   private final RigidBodyControlManager headManager;
   private final RigidBodyControlManager chestManager;

   private boolean isDone = false;

   public StandingState(CentroidalMomentumManager centroidalMomentumManager, GravityCompensationManager gravityCompensationManager,
                        SideDependentList<RigidBodyControlManager> handManagers, FeetJumpManager feetManager,
                        Map<String, RigidBodyControlManager> bodyManagerMap, FullHumanoidRobotModel fullRobotModel)
   {
      super(stateEnum);
      this.centroidalMomentumManager = centroidalMomentumManager;
      this.gravityCompensationManager = gravityCompensationManager;
      this.feetManager = feetManager;
      this.handManagers = handManagers;
      this.chestManager = bodyManagerMap.get(fullRobotModel.getChest().getName());
      this.headManager = bodyManagerMap.get(fullRobotModel.getHead().getName());
      this.isDone = false;
   }

   @Override
   public boolean isDone()
   {
      return isDone;
   }

   @Override
   public void doAction()
   {
      centroidalMomentumManager.compute();
      gravityCompensationManager.compute();
      headManager.compute();
      chestManager.compute();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyControlManager handManager = handManagers.get(robotSide);
         handManager.compute();
      }
   }

   @Override
   public void doTransitionIntoAction()
   {
      isDone = false;
      headManager.holdInJointspace();
      chestManager.holdInJointspace();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyControlManager handManager = handManagers.get(robotSide);
         handManager.holdInJointspace();
      }
   }

   @Override
   public void doTransitionOutOfAction()
   {
      
   }

   public void startJump(boolean startJump)
   {
      this.isDone = startJump;
   }
}
