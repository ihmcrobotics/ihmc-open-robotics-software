package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.states;

import java.util.Map;

import us.ihmc.commonWalkingControlModules.controlModules.flight.CentroidalMomentumManager;
import us.ihmc.commonWalkingControlModules.controlModules.flight.FeetJumpManager;
import us.ihmc.commonWalkingControlModules.controlModules.flight.GravityCompensationManager;
import us.ihmc.commonWalkingControlModules.controlModules.flight.JumpMessageHandler;
import us.ihmc.commonWalkingControlModules.controlModules.flight.WholeBodyMotionPlanner;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.Axis;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class FlightState extends AbstractJumpState
{
   private static final JumpStateEnum stateEnum = JumpStateEnum.FLIGHT;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final CentroidalMomentumManager wholeBodyMomentumManager;
   private final GravityCompensationManager gravityCompensationManager;
   private final FeetJumpManager feetManager;
   private final SideDependentList<RigidBodyControlManager> handManagers;
   private final RigidBodyControlManager chestManager;
   private final RigidBodyControlManager headManager;
   private final double contactThreshold = 100.0;
   private final double footHeightThresholdForCollisionChecking = 0.005;

   public FlightState(WholeBodyMotionPlanner motionPlanner, JumpMessageHandler messageHandler, HighLevelHumanoidControllerToolbox controllerToolbox,
                      WholeBodyControlCoreToolbox controlCoreToolbox, CentroidalMomentumManager centroidalMomentumManager,
                      GravityCompensationManager gravityCompensationManager, SideDependentList<RigidBodyControlManager> handManagers,
                      FeetJumpManager feetManager, Map<String, RigidBodyControlManager> bodyManagerMap, YoVariableRegistry registry)
   {
      super(stateEnum, motionPlanner, messageHandler, controllerToolbox);
      this.controllerToolbox = controllerToolbox;
      this.wholeBodyMomentumManager = centroidalMomentumManager;
      this.gravityCompensationManager = gravityCompensationManager;
      this.feetManager = feetManager;
      this.handManagers = handManagers;
      this.chestManager = bodyManagerMap.get(controllerToolbox.getFullRobotModel().getChest().getName());
      this.headManager = bodyManagerMap.get(controllerToolbox.getFullRobotModel().getHead().getName());
   }

   @Override
   public boolean isDone()
   {
      return checkIfFeetHaveCollidedWithGround();
   }

   @Override
   public void doAction()
   {
      wholeBodyMomentumManager.computeMomentumRateOfChangeForFreeFall();
      gravityCompensationManager.setRootJointAccelerationForFreeFall();
      feetManager.compute();
      for (RobotSide side : RobotSide.values)
         handManagers.get(side).compute();
      chestManager.compute();
      headManager.compute();
   }

   private boolean checkIfFeetHaveCollidedWithGround()
   {
      boolean checkForCollision = false;
      for(RobotSide robotSide: RobotSide.values)
      {
         double footHeightAboveGround = feetManager.getFootPosition(robotSide, Axis.Z);
         checkForCollision |= (footHeightAboveGround < footHeightThresholdForCollisionChecking);
      }
      if(checkForCollision)
      {
         double totalGroundReactionZ = feetManager.getGroundReactionForceZ();
         if(totalGroundReactionZ > contactThreshold)
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
      controllerToolbox.clearContacts();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyControlManager handManager = handManagers.get(robotSide);
         handManager.holdInTaskspace();
         feetManager.holdInJointspace(robotSide);
         feetManager.makeFeetFullyUnconstrained(robotSide);
      }
      headManager.holdInJointspace();
      chestManager.holdInJointspace();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      // TODO Auto-generated method stub

   }

}
