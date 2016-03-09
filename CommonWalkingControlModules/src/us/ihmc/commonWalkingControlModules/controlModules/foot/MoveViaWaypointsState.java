package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.FootTrajectoryControllerCommand;
import us.ihmc.commonWalkingControlModules.trajectories.SoftTouchdownPositionTrajectoryGenerator;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.ConstantVectorProvider;
import us.ihmc.robotics.trajectories.providers.SettableDoubleProvider;
import us.ihmc.robotics.trajectories.providers.SettablePositionProvider;
import us.ihmc.robotics.trajectories.providers.VectorProvider;

public class MoveViaWaypointsState extends AbstractUnconstrainedState
{
   private final MultipleWaypointsPositionTrajectoryGenerator positionTrajectoryGenerator;
   private final MultipleWaypointsOrientationTrajectoryGenerator orientationTrajectoryGenerator;

   private final ReferenceFrame footFrame;

   private final BooleanYoVariable isTrajectoryStopped;
   private final BooleanYoVariable isPerformingTouchdown;
   private final SettableDoubleProvider touchdownInitialTimeProvider = new SettableDoubleProvider(0.0);
   private final SettablePositionProvider currentDesiredFootPosition = new SettablePositionProvider();
   private final SoftTouchdownPositionTrajectoryGenerator positionTrajectoryForDisturbanceRecovery;

   private final FramePoint tempPosition = new FramePoint();
   private final FrameVector tempLinearVelocity = new FrameVector();
   private final FrameOrientation tempOrientation = new FrameOrientation();
   private final FrameVector tempAngularVelocity = new FrameVector();

   public MoveViaWaypointsState(FootControlHelper footControlHelper, YoSE3PIDGainsInterface gains, YoVariableRegistry registry)
   {
      super(ConstraintType.MOVE_VIA_WAYPOINTS, footControlHelper, gains, registry);

      String namePrefix = footControlHelper.getRobotSide().getCamelCaseNameForStartOfExpression() + "FootMoveViaWaypoints";

      footFrame = momentumBasedController.getReferenceFrames().getFootFrame(robotSide);

      positionTrajectoryGenerator = new MultipleWaypointsPositionTrajectoryGenerator(namePrefix, worldFrame, registry);
      orientationTrajectoryGenerator = new MultipleWaypointsOrientationTrajectoryGenerator(namePrefix, worldFrame, registry);

      isTrajectoryStopped = new BooleanYoVariable(namePrefix + "IsTrajectoryStopped", registry);
      isPerformingTouchdown = new BooleanYoVariable(namePrefix + "IsPerformingTouchdown", registry);

      VectorProvider touchdownVelocityProvider = new ConstantVectorProvider(new FrameVector(worldFrame, 0.0, 0.0, -0.3));
      VectorProvider touchdownAccelerationProvider = new ConstantVectorProvider(new FrameVector(worldFrame, 0.0, 0.0, -1.0));
      positionTrajectoryForDisturbanceRecovery = new SoftTouchdownPositionTrajectoryGenerator(namePrefix + "Touchdown", worldFrame, currentDesiredFootPosition,
            touchdownVelocityProvider, touchdownAccelerationProvider, touchdownInitialTimeProvider, registry);
   }

   public void handleFootTrajectoryMessage(FootTrajectoryControllerCommand footTrajectoryMessage, boolean initializeToCurrent)
   {
      positionTrajectoryGenerator.clear();
      orientationTrajectoryGenerator.clear();

      if (footTrajectoryMessage.getTrajectoryPoint(0).getTime() > 1.0e-5)
      {
         if (initializeToCurrent)
         {
            tempPosition.setToZero(footFrame);
            tempPosition.changeFrame(worldFrame);
            tempOrientation.setToZero(footFrame);
            tempOrientation.changeFrame(worldFrame);
         }
         else
         {
            positionTrajectoryGenerator.getPosition(tempPosition);
            orientationTrajectoryGenerator.getOrientation(tempOrientation);
         }

         tempLinearVelocity.setToZero(worldFrame);
         tempAngularVelocity.setToZero(worldFrame);

         positionTrajectoryGenerator.appendWaypoint(0.0, tempPosition, tempLinearVelocity);
         orientationTrajectoryGenerator.appendWaypoint(0.0, tempOrientation, tempAngularVelocity);
      }

      positionTrajectoryGenerator.appendWaypoints(footTrajectoryMessage);
      orientationTrajectoryGenerator.appendWaypoints(footTrajectoryMessage);
   }

   @Override
   protected void initializeTrajectory()
   {
      positionTrajectoryGenerator.initialize();
      orientationTrajectoryGenerator.initialize();
   }

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();
      if (getPreviousState() == this)
         legSingularityAndKneeCollapseAvoidanceControlModule.setCheckVelocityForSwingSingularityAvoidance(false);
      isTrajectoryStopped.set(false);
      isPerformingTouchdown.set(false);
   };

   @Override
   protected void computeAndPackTrajectory()
   {
      if (isPerformingTouchdown.getBooleanValue())
      {
         positionTrajectoryForDisturbanceRecovery.compute(getTimeInCurrentState());

         positionTrajectoryForDisturbanceRecovery.getLinearData(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
         orientationTrajectoryGenerator.getAngularData(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
         desiredAngularVelocity.setToZero();
         desiredAngularAcceleration.setToZero();
      }
      else if (isTrajectoryStopped.getBooleanValue())
      {
         positionTrajectoryGenerator.getPosition(desiredPosition);
         orientationTrajectoryGenerator.getOrientation(desiredOrientation);
         desiredLinearVelocity.setToZero();
         desiredAngularVelocity.setToZero();
         desiredLinearAcceleration.setToZero();
         desiredAngularAcceleration.setToZero();
      }
      else
      {
         positionTrajectoryGenerator.compute(getTimeInCurrentState());
         orientationTrajectoryGenerator.compute(getTimeInCurrentState());

         positionTrajectoryGenerator.getLinearData(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
         orientationTrajectoryGenerator.getAngularData(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
      }
   }

   public void requestTouchdownForDisturbanceRecovery()
   {
      if (isPerformingTouchdown.getBooleanValue())
         return;

      positionTrajectoryGenerator.getPosition(desiredPosition);
      currentDesiredFootPosition.set(desiredPosition);
      touchdownInitialTimeProvider.setValue(getTimeInCurrentState());
      positionTrajectoryForDisturbanceRecovery.initialize();

      isPerformingTouchdown.set(true);
   }

   public void requestStopTrajectory()
   {
      isTrajectoryStopped.set(true);
   }

   @Override
   public void doTransitionOutOfAction()
   {
      super.doTransitionOutOfAction();
      isTrajectoryStopped.set(false);
      isPerformingTouchdown.set(false);
   }
}