package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.trajectories.CurrentOrientationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.SimpleTwoWaypointTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.trajectories.SoftTouchdownPositionTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointPositionTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointTrajectoryGeneratorWithPushRecovery;
import us.ihmc.commonWalkingControlModules.trajectories.WrapperForMultiplePositionTrajectoryGenerators;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.trajectory.CurrentLinearVelocityProvider;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleProvider;
import com.yobotics.simulationconstructionset.util.trajectory.FrameBasedPositionSource;
import com.yobotics.simulationconstructionset.util.trajectory.PositionProvider;
import com.yobotics.simulationconstructionset.util.trajectory.PositionTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.TrajectoryParametersProvider;
import com.yobotics.simulationconstructionset.util.trajectory.VectorProvider;
import com.yobotics.simulationconstructionset.util.trajectory.YoVariableDoubleProvider;
import com.yobotics.simulationconstructionset.util.trajectory.YoVelocityProvider;

public class SwingState extends AbstractUnconstrainedState
{
   private final boolean visualizeSwingTrajectory = true;
   
   private final BooleanYoVariable replanTrajectory;
   private final YoVariableDoubleProvider swingTimeRemaining;

   private final PositionTrajectoryGenerator positionTrajectoryGenerator, pushRecoveryPositionTrajectoryGenerator;
   private final OrientationInterpolationTrajectoryGenerator orientationTrajectoryGenerator;

   public SwingState(DoubleProvider swingTimeProvider,
         PositionProvider finalPositionProvider,
         OrientationProvider finalOrientationProvider, TrajectoryParametersProvider trajectoryParametersProvider,
         
         YoFramePoint yoDesiredPosition, YoFrameVector yoDesiredLinearVelocity, YoFrameVector yoDesiredLinearAcceleration,
         RigidBodySpatialAccelerationControlModule accelerationControlModule,
         MomentumBasedController momentumBasedController, ContactablePlaneBody contactableBody,
         EnumYoVariable<ConstraintType> requestedState, int jacobianId, 
         DoubleYoVariable nullspaceMultiplier, BooleanYoVariable jacobianDeterminantInRange,
         BooleanYoVariable doSingularityEscape, BooleanYoVariable forceFootAccelerateIntoGround,
         LegSingularityAndKneeCollapseAvoidanceControlModule legSingularityAndKneeCollapseAvoidanceControlModule,
         RobotSide robotSide, YoVariableRegistry registry,
         
         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, WalkingControllerParameters walkingControllerParameters)
   {
      super(ConstraintType.SWING, yoDesiredPosition, yoDesiredLinearVelocity, yoDesiredLinearAcceleration,
            accelerationControlModule, momentumBasedController, contactableBody,
            requestedState, jacobianId, nullspaceMultiplier, jacobianDeterminantInRange,
            doSingularityEscape, forceFootAccelerateIntoGround, legSingularityAndKneeCollapseAvoidanceControlModule,
            robotSide, registry);
      
      RigidBody rigidBody = contactableBody.getRigidBody();
      String namePrefix = rigidBody.getName();
      
      replanTrajectory = new BooleanYoVariable(namePrefix + "replanTrajectory", registry);
      swingTimeRemaining = new YoVariableDoubleProvider(namePrefix + "SwingTimeRemaining", registry);
      
      ArrayList<PositionTrajectoryGenerator> positionTrajectoryGenerators = new ArrayList<PositionTrajectoryGenerator>();
      ArrayList<PositionTrajectoryGenerator> pushRecoveryPositionTrajectoryGenerators = new ArrayList<PositionTrajectoryGenerator>();

      YoVelocityProvider finalVelocityProvider =
            new YoVelocityProvider(namePrefix + "TouchdownVelocity", ReferenceFrame.getWorldFrame(), registry);
      finalVelocityProvider.set(new Vector3d(0.0, 0.0, walkingControllerParameters.getDesiredTouchdownVelocity()));
      CommonWalkingReferenceFrames referenceFrames = momentumBasedController.getReferenceFrames();
      ReferenceFrame footFrame = referenceFrames.getFootFrame(robotSide);
      VectorProvider initialVelocityProvider = new CurrentLinearVelocityProvider(
            footFrame,
            momentumBasedController.getFullRobotModel().getFoot(robotSide),
            momentumBasedController.getTwistCalculator());
      
      if (trajectoryParametersProvider == null)
      {
         trajectoryParametersProvider = new TrajectoryParametersProvider(new SimpleTwoWaypointTrajectoryParameters());
      }

      PositionProvider initialPositionProvider = new FrameBasedPositionSource(footFrame);
      OrientationProvider initialOrientationProvider = new CurrentOrientationProvider(worldFrame, footFrame);
      
      PositionTrajectoryGenerator swingTrajectoryGenerator = new TwoWaypointPositionTrajectoryGenerator(namePrefix + "Swing", worldFrame,
            swingTimeProvider, initialPositionProvider, initialVelocityProvider, finalPositionProvider, finalVelocityProvider,
            trajectoryParametersProvider, registry, dynamicGraphicObjectsListRegistry, walkingControllerParameters, visualizeSwingTrajectory);

      PositionTrajectoryGenerator touchdownTrajectoryGenerator = new SoftTouchdownPositionTrajectoryGenerator(namePrefix + "Touchdown", worldFrame,
            finalPositionProvider, finalVelocityProvider, swingTimeProvider, registry);

      PositionTrajectoryGenerator pushRecoverySwingTrajectoryGenerator = new TwoWaypointTrajectoryGeneratorWithPushRecovery(
            namePrefix + "PushRecoverySwing", worldFrame, swingTimeProvider, swingTimeRemaining, initialPositionProvider,
            initialVelocityProvider, finalPositionProvider, finalVelocityProvider, trajectoryParametersProvider, registry,
            dynamicGraphicObjectsListRegistry, swingTrajectoryGenerator, walkingControllerParameters, visualizeSwingTrajectory);

      positionTrajectoryGenerators.add(swingTrajectoryGenerator);
      positionTrajectoryGenerators.add(touchdownTrajectoryGenerator);

      pushRecoveryPositionTrajectoryGenerators.add(pushRecoverySwingTrajectoryGenerator);
      pushRecoveryPositionTrajectoryGenerators.add(touchdownTrajectoryGenerator);

      positionTrajectoryGenerator = new WrapperForMultiplePositionTrajectoryGenerators(positionTrajectoryGenerators, namePrefix, registry);

      pushRecoveryPositionTrajectoryGenerator = new WrapperForMultiplePositionTrajectoryGenerators(pushRecoveryPositionTrajectoryGenerators, namePrefix
            + "PushRecoveryTrajectoryGenerator", registry);

      orientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator(namePrefix + "SwingFootOrientation", worldFrame,
            swingTimeProvider, initialOrientationProvider, finalOrientationProvider, registry);
   }

   protected void initializeTrajectory()
   {
      positionTrajectoryGenerator.initialize();
      orientationTrajectoryGenerator.initialize();

      trajectoryWasReplanned = false;
      replanTrajectory.set(false);
   }

   protected void computeAndPackTrajectory()
   {
      if (replanTrajectory.getBooleanValue()) // This seems like a bad place for this?
      {
         pushRecoveryPositionTrajectoryGenerator.initialize();
         replanTrajectory.set(false);
         trajectoryWasReplanned = true;
      }

      if (!trajectoryWasReplanned)
      {
         positionTrajectoryGenerator.compute(getTimeInCurrentState());

         positionTrajectoryGenerator.packLinearData(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
      }
      else
      {
         pushRecoveryPositionTrajectoryGenerator.compute(getTimeInCurrentState());

         pushRecoveryPositionTrajectoryGenerator.packLinearData(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
      }

      orientationTrajectoryGenerator.compute(getTimeInCurrentState());
      orientationTrajectoryGenerator.packAngularData(trajectoryOrientation, desiredAngularVelocity, desiredAngularAcceleration);
   }
   
   public void replanTrajectory(double swingTimeRemaining)
   {
      this.swingTimeRemaining.set(swingTimeRemaining);
      this.replanTrajectory.set(true);
   }
}
