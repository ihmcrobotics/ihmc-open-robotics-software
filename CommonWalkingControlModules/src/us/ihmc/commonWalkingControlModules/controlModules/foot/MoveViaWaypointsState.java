package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.Collection;

import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.trajectories.SoftTouchdownPositionTrajectoryGenerator;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.trajectories.providers.SettableDoubleProvider;
import us.ihmc.robotics.trajectories.providers.SettablePositionProvider;
import us.ihmc.robotics.trajectories.providers.VectorProvider;

public class MoveViaWaypointsState extends AbstractUnconstrainedState
{
   private final ReferenceFrame footFrame;

   private final BooleanYoVariable isPerformingTouchdown;
   private final SettableDoubleProvider touchdownInitialTimeProvider = new SettableDoubleProvider(0.0);
   private final SettablePositionProvider currentDesiredFootPosition = new SettablePositionProvider();
   private final SoftTouchdownPositionTrajectoryGenerator positionTrajectoryForDisturbanceRecovery;

   private final RigidBodyTaskspaceControlState taskspaceControlState;

   public MoveViaWaypointsState(FootControlHelper footControlHelper, VectorProvider touchdownVelocityProvider, VectorProvider touchdownAccelerationProvider,
         YoSE3PIDGainsInterface gains, YoVariableRegistry registry)
   {
      super(ConstraintType.MOVE_VIA_WAYPOINTS, footControlHelper, gains, registry);

      String namePrefix = footControlHelper.getRobotSide().getCamelCaseNameForStartOfExpression() + "FootMoveViaWaypoints";

      isPerformingTouchdown = new BooleanYoVariable(namePrefix + "IsPerformingTouchdown", registry);
      positionTrajectoryForDisturbanceRecovery = new SoftTouchdownPositionTrajectoryGenerator(namePrefix + "Touchdown", worldFrame, currentDesiredFootPosition,
            touchdownVelocityProvider, touchdownAccelerationProvider, touchdownInitialTimeProvider, registry);

      RigidBody foot = controllerToolbox.getFullRobotModel().getFoot(robotSide);
      footFrame = foot.getBodyFixedFrame();
      DoubleYoVariable yoTime = controllerToolbox.getYoTime();
      ReferenceFrame soleFrame = controllerToolbox.getFullRobotModel().getSoleFrame(robotSide);
      YoGraphicsListRegistry graphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();
      Collection<ReferenceFrame> trajectoryFrames = controllerToolbox.getTrajectoryFrames();
      ReferenceFrame pelvisFrame = pelvis.getBodyFixedFrame();

      taskspaceControlState = new RigidBodyTaskspaceControlState(foot, pelvis, rootBody, trajectoryFrames, soleFrame, pelvisFrame, yoTime, graphicsListRegistry,
            registry);
      taskspaceControlState.setGains(gains.getOrientationGains(), gains.getPositionGains());
   }

   @Override
   public void setWeight(double weight)
   {
      super.setWeight(weight);
      taskspaceControlState.setWeight(weight);
   }

   @Override
   public void setWeights(Vector3D angularWeight, Vector3D linearWeight)
   {
      super.setWeights(angularWeight, linearWeight);
      taskspaceControlState.setWeights(angularWeight, linearWeight);
   }

   public void holdCurrentPosition()
   {
      taskspaceControlState.holdPose();
   }

   public void handleFootTrajectoryCommand(FootTrajectoryCommand command)
   {
      if (!taskspaceControlState.handlePoseTrajectoryCommand(command))
      {
         taskspaceControlState.holdPose();
      }
   }

   @Override
   protected void initializeTrajectory()
   {
   }

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();
      taskspaceControlState.doTransitionIntoAction();
      isPerformingTouchdown.set(false);
   };

   @Override
   protected void computeAndPackTrajectory()
   {
      if (isPerformingTouchdown.getBooleanValue())
      {
         positionTrajectoryForDisturbanceRecovery.compute(getTimeInCurrentState());
         positionTrajectoryForDisturbanceRecovery.getLinearData(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
         desiredAngularVelocity.setToZero();
         desiredAngularAcceleration.setToZero();
      }
      else
      {
         taskspaceControlState.doAction();

         if (taskspaceControlState.abortState())
            requestTouchdownForDisturbanceRecovery();
      }
   }

   public void requestTouchdownForDisturbanceRecovery()
   {
      if (isPerformingTouchdown.getBooleanValue())
         return;

      desiredPosition.setToZero(footFrame);
      desiredPosition.changeFrame(worldFrame);

      currentDesiredFootPosition.set(desiredPosition);
      touchdownInitialTimeProvider.setValue(getTimeInCurrentState());
      positionTrajectoryForDisturbanceRecovery.initialize();

      desiredOrientation.setToZero(footFrame);
      desiredOrientation.changeFrame(worldFrame);

      isPerformingTouchdown.set(true);
   }

   public void requestStopTrajectory()
   {
      taskspaceControlState.holdPose();
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      if (isPerformingTouchdown.getBooleanValue())
         return super.getInverseDynamicsCommand();

      return taskspaceControlState.getInverseDynamicsCommand();
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      if (isPerformingTouchdown.getBooleanValue())
         return super.getFeedbackControlCommand();

      return taskspaceControlState.getFeedbackControlCommand();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      super.doTransitionOutOfAction();
      taskspaceControlState.doTransitionOutOfAction();
   }
}