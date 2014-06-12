package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.StraightLinePositionTrajectoryGenerator;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.RigidBody;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleProvider;
import com.yobotics.simulationconstructionset.util.trajectory.PositionProvider;

public class MoveStraightState extends AbstractUnconstrainedState
{
   private StraightLinePositionTrajectoryGenerator positionTrajectoryGenerator;
   private OrientationInterpolationTrajectoryGenerator orientationTrajectoryGenerator;

   public MoveStraightState(DoubleProvider footTrajectoryTimeProvider, PositionProvider initialPositionProvider,
         PositionProvider finalPositionProvider, OrientationProvider initialOrientationProvider, OrientationProvider finalOrientationProvider,
         
         YoFramePoint yoDesiredPosition, YoFrameVector yoDesiredLinearVelocity, YoFrameVector yoDesiredLinearAcceleration,
         RigidBodySpatialAccelerationControlModule accelerationControlModule,
         MomentumBasedController momentumBasedController, ContactablePlaneBody contactableBody,
         EnumYoVariable<ConstraintType> requestedState, int jacobianId,
         DoubleYoVariable nullspaceMultiplier, BooleanYoVariable jacobianDeterminantInRange,
         BooleanYoVariable doSingularityEscape, BooleanYoVariable forceFootAccelerateIntoGround,
         LegSingularityAndKneeCollapseAvoidanceControlModule legSingularityAndKneeCollapseAvoidanceControlModule,
         RobotSide robotSide, YoVariableRegistry registry,
         
         BooleanYoVariable isTrajectoryDone, BooleanYoVariable isUnconstrained)
   {
      super(ConstraintType.MOVE_STRAIGHT, yoDesiredPosition, yoDesiredLinearVelocity, yoDesiredLinearAcceleration,
            accelerationControlModule, momentumBasedController, contactableBody,
            requestedState, jacobianId, nullspaceMultiplier, jacobianDeterminantInRange,
            doSingularityEscape, forceFootAccelerateIntoGround, legSingularityAndKneeCollapseAvoidanceControlModule,
            robotSide, registry,
            
            isTrajectoryDone, isUnconstrained);

      RigidBody rigidBody = contactableBody.getRigidBody();
      String namePrefix = rigidBody.getName();

      positionTrajectoryGenerator = new StraightLinePositionTrajectoryGenerator(namePrefix + "FootPosition", worldFrame, footTrajectoryTimeProvider,
            initialPositionProvider, finalPositionProvider, registry);

      orientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator(namePrefix + "Orientation", worldFrame, footTrajectoryTimeProvider,
            initialOrientationProvider, finalOrientationProvider, registry);
   }

   protected void initializeTrajectory()
   {
      positionTrajectoryGenerator.initialize();
      orientationTrajectoryGenerator.initialize();
   }

   protected void computeAndPackTrajectory()
   {
      boolean isDone = positionTrajectoryGenerator.isDone() && orientationTrajectoryGenerator.isDone();
      isTrajectoryDone.set(isDone);

      positionTrajectoryGenerator.compute(getTimeInCurrentState());
      orientationTrajectoryGenerator.compute(getTimeInCurrentState());

      positionTrajectoryGenerator.packLinearData(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
      orientationTrajectoryGenerator.packAngularData(trajectoryOrientation, desiredAngularVelocity, desiredAngularAcceleration);
   }
}