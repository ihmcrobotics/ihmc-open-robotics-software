package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyPoseController;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.trajectories.SoftTouchdownPositionTrajectoryGenerator;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.JointspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.LegTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SE3TrajectoryControllerCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class MoveViaWaypointsState extends AbstractFootControlState
{
   private final YoBoolean isPerformingTouchdown;
   private final SoftTouchdownPositionTrajectoryGenerator positionTrajectoryForDisturbanceRecovery;

   private final RigidBodyControlManager controlManager;

   private final FrameVector3DReadOnly touchdownVelocity;
   private final FrameVector3DReadOnly touchdownAcceleration;

   private ReferenceFrame controlFrame;
   private final ReferenceFrame ankleFrame;
   private final WorkspaceLimiterControlModule workspaceLimiterControlModule;

   public MoveViaWaypointsState(FootControlHelper footControlHelper, RigidBodyControlManager controlManager, YoRegistry registry)
   {
      super(footControlHelper);

      this.controlManager = controlManager;
      this.touchdownVelocity = footControlHelper.getSwingTrajectoryParameters().getDesiredTouchdownVelocity();
      this.touchdownAcceleration = footControlHelper.getSwingTrajectoryParameters().getDesiredTouchdownAcceleration();

      RigidBodyBasics foot = controllerToolbox.getFullRobotModel().getFoot(robotSide);
      String namePrefix = foot.getName() + "MoveViaWaypoints";

      isPerformingTouchdown = new YoBoolean(namePrefix + "IsPerformingTouchdown", registry);
      positionTrajectoryForDisturbanceRecovery = new SoftTouchdownPositionTrajectoryGenerator(namePrefix + "Touchdown", registry);

      ankleFrame = foot.getParentJoint().getFrameAfterJoint();
      controlFrame = ankleFrame;

      workspaceLimiterControlModule = footControlHelper.getWorkspaceLimiterControlModule();
   }

   public void handleFootTrajectoryCommand(FootTrajectoryCommand command)
   {
      SE3TrajectoryControllerCommand se3Trajectory = command.getSE3Trajectory();
      se3Trajectory.setSequenceId(command.getSequenceId());
      controlManager.handleTaskspaceTrajectoryCommand(se3Trajectory);
   }

   public void handleLegTrajectoryCommand(LegTrajectoryCommand command)
   {
      JointspaceTrajectoryCommand jointspaceTrajectory = command.getJointspaceTrajectory();
      jointspaceTrajectory.setSequenceId(command.getSequenceId());
      controlManager.handleJointspaceTrajectoryCommand(jointspaceTrajectory);
   }

   @Override
   public void onEntry()
   {
      isPerformingTouchdown.set(false);

      if (workspaceLimiterControlModule != null)
         workspaceLimiterControlModule.setCheckVelocityForSwingSingularityAvoidance(false);
   }

   @Override
   public void doSpecificAction(double timeInState)
   {
      if (isPerformingTouchdown.getBooleanValue())
      {
         positionTrajectoryForDisturbanceRecovery.compute(timeInState);
         positionTrajectoryForDisturbanceRecovery.getLinearData(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
         desiredAngularVelocity.setToZero();
         desiredAngularAcceleration.setToZero();

         packCommandForTouchdown();
      }
      else
      {
         controlManager.compute();
      }

      doSingularityAvoidance();
   }

   private void packCommandForTouchdown()
   {
      RigidBodyPoseController taskspaceControlState = (RigidBodyPoseController) controlManager.getTaskspaceControlState();
      SpatialFeedbackControlCommand spatialFeedbackControlCommand = (SpatialFeedbackControlCommand) taskspaceControlState.getFeedbackControlCommand();
      spatialFeedbackControlCommand.setInverseDynamics(desiredOrientation,
                                                       desiredPosition,
                                                       desiredAngularVelocity,
                                                       desiredLinearVelocity,
                                                       desiredAngularAcceleration,
                                                       desiredLinearAcceleration);
      spatialFeedbackControlCommand.setWeightsForSolver(taskspaceControlState.getAngularDefaultWeight(), taskspaceControlState.getLinearDefaultWeight());
      spatialFeedbackControlCommand.setOrientationGains(taskspaceControlState.getOrientationGains());
      spatialFeedbackControlCommand.setPositionGains(taskspaceControlState.getPositionGains());
   }

   public void requestTouchdownForDisturbanceRecovery(double timeInState)
   {
      if (isPerformingTouchdown.getBooleanValue())
         return;

      desiredPosition.setToZero(controlFrame);
      desiredOrientation.setToZero(controlFrame);
      desiredPosition.changeFrame(worldFrame);
      desiredOrientation.changeFrame(worldFrame);

      positionTrajectoryForDisturbanceRecovery.setLinearTrajectory(timeInState, desiredAnklePosition, touchdownVelocity, touchdownAcceleration);
      positionTrajectoryForDisturbanceRecovery.initialize();

      isPerformingTouchdown.set(true);
   }

   private final FramePoint3D desiredAnklePosition = new FramePoint3D();
   private final FramePose3D desiredPose = new FramePose3D();

   private void doSingularityAvoidance()
   {
      if (controlManager.getActiveControlMode() != RigidBodyControlMode.TASKSPACE)
         return;
      if (controlManager.getTaskspaceControlState().isHybridModeActive())
         return; // If in hybrid, let's assume that the jointspace trajectory helps with the singularity

      SpatialFeedbackControlCommand spatialFeedbackControlCommand = (SpatialFeedbackControlCommand) controlManager.getTaskspaceControlState()
                                                                                                                  .getFeedbackControlCommand();

      if (workspaceLimiterControlModule != null)
      {
         desiredPosition.setIncludingFrame(spatialFeedbackControlCommand.getReferencePosition());
         desiredOrientation.setIncludingFrame(spatialFeedbackControlCommand.getReferenceOrientation());
         desiredLinearVelocity.setIncludingFrame(spatialFeedbackControlCommand.getReferenceLinearVelocity());
         desiredAngularVelocity.setIncludingFrame(spatialFeedbackControlCommand.getReferenceAngularVelocity());
         desiredLinearAcceleration.setIncludingFrame(spatialFeedbackControlCommand.getReferenceLinearAcceleration());
         desiredAngularAcceleration.setIncludingFrame(spatialFeedbackControlCommand.getReferenceAngularAcceleration());

         desiredPose.setIncludingFrame(desiredPosition, desiredOrientation);
         changeDesiredPoseBodyFrame(controlFrame, ankleFrame, desiredPose);
         desiredAnklePosition.setIncludingFrame(desiredPose.getPosition());

         workspaceLimiterControlModule.update();
         workspaceLimiterControlModule.correctSwingFootTrajectory(desiredAnklePosition, desiredLinearVelocity, desiredLinearAcceleration);

         desiredPose.getPosition().set(desiredAnklePosition);
         changeDesiredPoseBodyFrame(ankleFrame, controlFrame, desiredPose);
         desiredPosition.setIncludingFrame(desiredPose.getPosition());
      }

      spatialFeedbackControlCommand.setInverseDynamics(desiredOrientation,
                                                       desiredPosition,
                                                       desiredAngularVelocity,
                                                       desiredLinearVelocity,
                                                       desiredAngularAcceleration,
                                                       desiredLinearAcceleration);
   }

   private final RigidBodyTransform oldBodyFrameDesiredTransform = new RigidBodyTransform();
   private final RigidBodyTransform newBodyFrameDesiredTransform = new RigidBodyTransform();
   private final RigidBodyTransform transformFromNewBodyFrameToOldBodyFrame = new RigidBodyTransform();

   private void changeDesiredPoseBodyFrame(ReferenceFrame oldBodyFrame, ReferenceFrame newBodyFrame, FramePose3D framePoseToModify)
   {
      if (oldBodyFrame == newBodyFrame)
         return;

      framePoseToModify.get(oldBodyFrameDesiredTransform);
      newBodyFrame.getTransformToDesiredFrame(transformFromNewBodyFrameToOldBodyFrame, oldBodyFrame);
      newBodyFrameDesiredTransform.set(oldBodyFrameDesiredTransform);
      newBodyFrameDesiredTransform.multiply(transformFromNewBodyFrameToOldBodyFrame);
      framePoseToModify.set(newBodyFrameDesiredTransform);
   }

   public void requestStopTrajectory()
   {
      controlManager.hold();
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return controlManager.getInverseDynamicsCommand();
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return controlManager.getFeedbackControlCommand();
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      return controlManager.createFeedbackControlTemplate();
   }

   @Override
   public void onExit(double timeInState)
   {
   }

   @Override
   public Object pollStatusToReport()
   {
      return controlManager.pollStatusToReport();
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(controlManager.getSCS2YoGraphics());
      return group;
   }

   @Override
   public boolean isLoadBearing()
   {
      return false;
   }
}