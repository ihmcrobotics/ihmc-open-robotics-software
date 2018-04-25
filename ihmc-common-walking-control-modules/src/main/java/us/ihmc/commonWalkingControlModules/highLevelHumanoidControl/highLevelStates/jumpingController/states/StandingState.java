package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.states;

import java.util.Map;

import us.ihmc.commonWalkingControlModules.controlModules.flight.CentroidalMomentumManager;
import us.ihmc.commonWalkingControlModules.controlModules.flight.FeetJumpManager;
import us.ihmc.commonWalkingControlModules.controlModules.flight.JumpMessageHandler;
import us.ihmc.commonWalkingControlModules.controlModules.flight.PelvisControlManager;
import us.ihmc.commonWalkingControlModules.controlModules.flight.WholeBodyMotionPlanner;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class StandingState extends AbstractJumpState
{
   private static final JumpStateEnum stateEnum = JumpStateEnum.STANDING;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final CentroidalMomentumManager centroidalMomentumManager;
   private final SideDependentList<RigidBodyControlManager> handManagers;
   private final FeetJumpManager feetManager;
   private final RigidBodyControlManager headManager;
   private final RigidBodyControlManager chestManager;
   private final PelvisControlManager pelvisControlManager;

   private boolean isDone = false;
   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FrameQuaternion tempOrientation = new FrameQuaternion();
   private boolean standingForFirstTime = true;

   public StandingState(WholeBodyMotionPlanner motionPlanner, JumpMessageHandler messageHandler, HighLevelHumanoidControllerToolbox controllerToolbox,
                        CentroidalMomentumManager centroidalMomentumManager, PelvisControlManager pelvisControlManager,
                        SideDependentList<RigidBodyControlManager> handManagers, FeetJumpManager feetManager,
                        Map<String, RigidBodyControlManager> bodyManagerMap, FullHumanoidRobotModel fullRobotModel)
   {
      super(stateEnum, motionPlanner, messageHandler, controllerToolbox);
      this.centroidalMomentumManager = centroidalMomentumManager;
      this.pelvisControlManager = pelvisControlManager;
      this.feetManager = feetManager;
      this.handManagers = handManagers;
      this.chestManager = bodyManagerMap.get(fullRobotModel.getChest().getName());
      this.headManager = bodyManagerMap.get(fullRobotModel.getHead().getName());
      this.isDone = false;
   }

   @Override
   public boolean isDone()
   {
      isDone = standingForFirstTime && (getTimeInCurrentState() > 0.5);
      standingForFirstTime &= (!isDone);
      return isDone;
   }

   @Override
   public void doAction()
   {
      centroidalMomentumManager.computeForZeroMomentumRateOfChange();
      pelvisControlManager.maintainDesiredPositionAndOrientation();
      feetManager.compute();
      headManager.compute();
      chestManager.compute();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyControlManager handManager = handManagers.get(robotSide);
         handManager.compute();
      }
   }

   @Override
   public void doStateSpecificTransitionIntoAction()
   {
      isDone = false;
      pelvisControlManager.getCurrentPelvisPosition(worldFrame, tempPoint);
      pelvisControlManager.setDesiredPelvisPosition(tempPoint);
      pelvisControlManager.getCurrentPelvisOrientation(worldFrame, tempOrientation);
      pelvisControlManager.setDesiredPelvisOrientation(tempOrientation);
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyControlManager handManager = handManagers.get(robotSide);
         handManager.holdInTaskspace();
         feetManager.makeFeetFullyConstrained(robotSide);
         feetManager.complyAndDamp(robotSide);
      }
      headManager.holdInJointspace();
      chestManager.holdInTaskspace();
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
