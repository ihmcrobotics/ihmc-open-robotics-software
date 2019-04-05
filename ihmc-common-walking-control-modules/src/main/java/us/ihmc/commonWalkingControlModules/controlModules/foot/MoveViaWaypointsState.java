package us.ihmc.commonWalkingControlModules.controlModules.foot;

import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyPoseController;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.trajectories.SoftTouchdownPositionTrajectoryGenerator;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SE3TrajectoryControllerCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.controllers.pidGains.PIDSE3GainsReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class MoveViaWaypointsState extends AbstractFootControlState
{
   private final YoBoolean isPerformingTouchdown;
   private final SoftTouchdownPositionTrajectoryGenerator positionTrajectoryForDisturbanceRecovery;

   private final RigidBodyPoseController poseController;
   private final SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();

   private Vector3DReadOnly angularWeight;
   private Vector3DReadOnly linearWeight;

   private final FrameVector3DReadOnly touchdownVelocity;
   private final FrameVector3DReadOnly touchdownAcceleration;

   private ReferenceFrame controlFrame;
   private final ReferenceFrame ankleFrame;
   private final LegSingularityAndKneeCollapseAvoidanceControlModule legSingularityAndKneeCollapseAvoidanceControlModule;

   private final PIDSE3GainsReadOnly gains;

   public MoveViaWaypointsState(FootControlHelper footControlHelper, FrameVector3DReadOnly touchdownVelocity, FrameVector3DReadOnly touchdownAcceleration,
                                PIDSE3GainsReadOnly gains, YoVariableRegistry registry)
   {
      super(footControlHelper);

      this.gains = gains;
      this.touchdownVelocity = touchdownVelocity;
      this.touchdownAcceleration = touchdownAcceleration;

      RigidBodyBasics foot = controllerToolbox.getFullRobotModel().getFoot(robotSide);
      String namePrefix = foot.getName() + "MoveViaWaypoints";

      isPerformingTouchdown = new YoBoolean(namePrefix + "IsPerformingTouchdown", registry);
      positionTrajectoryForDisturbanceRecovery = new SoftTouchdownPositionTrajectoryGenerator(namePrefix + "Touchdown", registry);

      YoDouble yoTime = controllerToolbox.getYoTime();
      YoGraphicsListRegistry graphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();
      ReferenceFrame pelvisFrame = pelvis.getBodyFixedFrame();
      ankleFrame = foot.getParentJoint().getFrameAfterJoint();
      controlFrame = ankleFrame;

      poseController = new RigidBodyPoseController(foot, pelvis, rootBody, controlFrame, pelvisFrame, yoTime, null, graphicsListRegistry, registry);
      poseController.setGains(gains.getOrientationGains(), gains.getPositionGains());

      spatialFeedbackControlCommand.set(rootBody, foot);
      spatialFeedbackControlCommand.setPrimaryBase(pelvis);

      legSingularityAndKneeCollapseAvoidanceControlModule = footControlHelper.getLegSingularityAndKneeCollapseAvoidanceControlModule();
   }

   public void setWeights(Vector3DReadOnly angularWeight, Vector3DReadOnly linearWeight)
   {
      this.angularWeight = angularWeight;
      this.linearWeight = linearWeight;

      poseController.setWeights(angularWeight, linearWeight);
   }

   public void holdCurrentPosition()
   {
      poseController.holdCurrent();
   }

   public void handleFootTrajectoryCommand(FootTrajectoryCommand command)
   {
      SE3TrajectoryControllerCommand se3Trajectory = command.getSE3Trajectory();
      se3Trajectory.setSequenceId(command.getSequenceId());

      if (!poseController.handleTrajectoryCommand(se3Trajectory))
      {
         poseController.holdCurrent();
      }
   }

   @Override
   public void onEntry()
   {
      poseController.onEntry();
      isPerformingTouchdown.set(false);

      if (legSingularityAndKneeCollapseAvoidanceControlModule != null)
      {
         legSingularityAndKneeCollapseAvoidanceControlModule.setCheckVelocityForSwingSingularityAvoidance(false);
      }
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
         poseController.doAction(timeInState);
         spatialFeedbackControlCommand.set((SpatialFeedbackControlCommand) poseController.getFeedbackControlCommand());

         if (poseController.abortState())
            requestTouchdownForDisturbanceRecovery(timeInState);
      }

      doSingularityAvoidance(spatialFeedbackControlCommand);
   }

   private void packCommandForTouchdown()
   {
      spatialFeedbackControlCommand.setInverseDynamics(desiredOrientation, desiredPosition, desiredAngularVelocity, desiredLinearVelocity,
                                                       desiredAngularAcceleration, desiredLinearAcceleration);
      spatialFeedbackControlCommand.setWeightsForSolver(angularWeight, linearWeight);
      spatialFeedbackControlCommand.setGains(gains);
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

   private void doSingularityAvoidance(SpatialFeedbackControlCommand spatialFeedbackControlCommand)
   {
      if (legSingularityAndKneeCollapseAvoidanceControlModule != null)
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

         legSingularityAndKneeCollapseAvoidanceControlModule.correctSwingFootTrajectory(desiredAnklePosition, desiredLinearVelocity, desiredLinearAcceleration);

         desiredPose.setPosition(desiredAnklePosition);
         changeDesiredPoseBodyFrame(ankleFrame, controlFrame, desiredPose);
         desiredPosition.setIncludingFrame(desiredPose.getPosition());
      }

      spatialFeedbackControlCommand.setInverseDynamics(desiredOrientation, desiredPosition, desiredAngularVelocity, desiredLinearVelocity,
                                                       desiredAngularAcceleration, desiredLinearAcceleration);
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
      poseController.holdCurrent();
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return null;
   }

   @Override
   public SpatialFeedbackControlCommand getFeedbackControlCommand()
   {
      return spatialFeedbackControlCommand;
   }

   @Override
   public void onExit()
   {
      poseController.onExit();
   }

   @Override
   public TaskspaceTrajectoryStatusMessage pollStatusToReport()
   {
      return poseController.pollStatusToReport();
   }
}