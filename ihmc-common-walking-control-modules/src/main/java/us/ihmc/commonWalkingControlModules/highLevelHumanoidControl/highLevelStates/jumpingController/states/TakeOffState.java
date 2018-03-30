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
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.FootSwitchInterface;

public class TakeOffState extends AbstractJumpState
{
   private static final JumpStateEnum stateEnum = JumpStateEnum.TAKE_OFF;

   private final CentroidalMomentumManager centroidalMomentumManager;
   private final GravityCompensationManager gravityCompensationManager;
   private final PelvisControlManager pelvisControlManager;
   private final SideDependentList<RigidBodyControlManager> handManagers;
   private final FeetJumpManager feetManager;
   private final RigidBodyControlManager headManager;
   private final RigidBodyControlManager chestManager;
   private final WholeBodyMotionPlanner motionPlanner;
   private final SideDependentList<FootSwitchInterface> footSwitches;
   
   private double zGroundReactionThreshold = 100;
   private double heightThreshold = 0.45;
   private double velocityThreshold = 0.1;
   private final Wrench tempWrench = new Wrench();

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
      this.headManager = bodyManagerMap.get(fullRobotModel.getHead().getName());
      this.chestManager = bodyManagerMap.get(fullRobotModel.getChest().getName());
      this.footSwitches = controllerToolbox.getFootSwitches();
   }

   @Override
   public boolean isDone()
   {
      return checkIfTakeOffIsComplete();
   }

   @Override
   public void doAction()
   {
      double timeInCurrentState = getTimeInCurrentState();
      centroidalMomentumManager.computeMomentumRateOfChangeFromForceProfile(timeInCurrentState);
      gravityCompensationManager.setRootJointAccelerationForStandardGravitationalForce();
      pelvisControlManager.maintainDesiredPositionAndOrientation();
      feetManager.makeFeetFullyConstrained();
      feetManager.computeForDampedCompliantMode();

      headManager.compute();
      chestManager.compute();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyControlManager handManager = handManagers.get(robotSide);
         handManager.compute();
      }
   }
   
   private boolean checkIfTakeOffIsComplete()
   {
      double totalForceZ = feetManager.getGroundReactionForceZ();
      double comHeightAboveGround = centroidalMomentumManager.getEstimatedCoMPositionZ(); // TODO subtract ground height. Presently works for flat ground only
      double comVelocity = centroidalMomentumManager.getEstimatedCoMVelocityZ();
      return (totalForceZ < zGroundReactionThreshold) & (comHeightAboveGround > heightThreshold) & (comVelocity > velocityThreshold);
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
