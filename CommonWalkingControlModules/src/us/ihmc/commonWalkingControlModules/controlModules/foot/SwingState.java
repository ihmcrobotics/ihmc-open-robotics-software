package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.trajectories.CurrentConfigurationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.SimpleTwoWaypointTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.trajectories.SoftTouchdownPositionTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointPositionTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointTrajectoryGeneratorWithPushRecovery;
import us.ihmc.commonWalkingControlModules.trajectories.WrapperForMultiplePositionTrajectoryGenerators;
import us.ihmc.commonWalkingControlModules.trajectories.YoSE3ConfigurationProvider;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.trajectories.providers.DoubleProvider;
import us.ihmc.utilities.screwTheory.RigidBody;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.trajectory.CurrentLinearVelocityProvider;
import com.yobotics.simulationconstructionset.util.trajectory.PositionTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.TrajectoryParameters;
import com.yobotics.simulationconstructionset.util.trajectory.TrajectoryParametersProvider;
import com.yobotics.simulationconstructionset.util.trajectory.TrajectoryWaypointGenerationMethod;
import com.yobotics.simulationconstructionset.util.trajectory.VectorProvider;
import com.yobotics.simulationconstructionset.util.trajectory.YoVariableDoubleProvider;

public class SwingState extends AbstractUnconstrainedState
{
   private final boolean visualizeSwingTrajectory = true;

   private final BooleanYoVariable replanTrajectory;
   private final YoVariableDoubleProvider swingTimeRemaining;

   private final PositionTrajectoryGenerator positionTrajectoryGenerator, pushRecoveryPositionTrajectoryGenerator;
   private final OrientationInterpolationTrajectoryGenerator orientationTrajectoryGenerator;

   private final CurrentConfigurationProvider initialConfigurationProvider;
   private final YoSE3ConfigurationProvider finalConfigurationProvider;
   private final TrajectoryParametersProvider trajectoryParametersProvider = new TrajectoryParametersProvider(new SimpleTwoWaypointTrajectoryParameters());

   public SwingState(DoubleProvider swingTimeProvider, VectorProvider touchdownVelocityProvider, RigidBodySpatialAccelerationControlModule accelerationControlModule,
         MomentumBasedController momentumBasedController, ContactablePlaneBody contactableBody, int jacobianId, DoubleYoVariable nullspaceMultiplier,
         BooleanYoVariable jacobianDeterminantInRange, BooleanYoVariable doSingularityEscape,
         LegSingularityAndKneeCollapseAvoidanceControlModule legSingularityAndKneeCollapseAvoidanceControlModule, RobotSide robotSide,
         YoVariableRegistry registry, WalkingControllerParameters walkingControllerParameters)
   {
      super(ConstraintType.SWING, accelerationControlModule, momentumBasedController,
            contactableBody, jacobianId, nullspaceMultiplier, jacobianDeterminantInRange, doSingularityEscape,
            legSingularityAndKneeCollapseAvoidanceControlModule, robotSide, registry);

      RigidBody rigidBody = contactableBody.getRigidBody();
      String namePrefix = rigidBody.getName();

      finalConfigurationProvider = new YoSE3ConfigurationProvider(namePrefix + "SwingFootFinal", worldFrame, registry);
      replanTrajectory = new BooleanYoVariable(namePrefix + "replanTrajectory", registry);
      swingTimeRemaining = new YoVariableDoubleProvider(namePrefix + "SwingTimeRemaining", registry);

      ArrayList<PositionTrajectoryGenerator> positionTrajectoryGenerators = new ArrayList<PositionTrajectoryGenerator>();
      ArrayList<PositionTrajectoryGenerator> pushRecoveryPositionTrajectoryGenerators = new ArrayList<PositionTrajectoryGenerator>();

      CommonWalkingReferenceFrames referenceFrames = momentumBasedController.getReferenceFrames();
      ReferenceFrame footFrame = referenceFrames.getFootFrame(robotSide);
      VectorProvider initialVelocityProvider = new CurrentLinearVelocityProvider(footFrame, momentumBasedController.getFullRobotModel().getFoot(robotSide),
            momentumBasedController.getTwistCalculator());

      initialConfigurationProvider = new CurrentConfigurationProvider(footFrame);

      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();
      PositionTrajectoryGenerator swingTrajectoryGenerator = new TwoWaypointPositionTrajectoryGenerator(namePrefix + "Swing", worldFrame, swingTimeProvider,
            initialConfigurationProvider, initialVelocityProvider, finalConfigurationProvider, touchdownVelocityProvider, trajectoryParametersProvider,
            registry, dynamicGraphicObjectsListRegistry, walkingControllerParameters, visualizeSwingTrajectory);

      PositionTrajectoryGenerator touchdownTrajectoryGenerator = new SoftTouchdownPositionTrajectoryGenerator(namePrefix + "Touchdown", worldFrame,
            finalConfigurationProvider, touchdownVelocityProvider, swingTimeProvider, registry);

      PositionTrajectoryGenerator pushRecoverySwingTrajectoryGenerator = new TwoWaypointTrajectoryGeneratorWithPushRecovery(namePrefix + "PushRecoverySwing",
            worldFrame, swingTimeProvider, swingTimeRemaining, initialConfigurationProvider, initialVelocityProvider, finalConfigurationProvider,
            touchdownVelocityProvider, trajectoryParametersProvider, registry, dynamicGraphicObjectsListRegistry, swingTrajectoryGenerator,
            walkingControllerParameters, visualizeSwingTrajectory);

      positionTrajectoryGenerators.add(swingTrajectoryGenerator);
      positionTrajectoryGenerators.add(touchdownTrajectoryGenerator);

      pushRecoveryPositionTrajectoryGenerators.add(pushRecoverySwingTrajectoryGenerator);
      pushRecoveryPositionTrajectoryGenerators.add(touchdownTrajectoryGenerator);

      positionTrajectoryGenerator = new WrapperForMultiplePositionTrajectoryGenerators(positionTrajectoryGenerators, namePrefix, registry);

      pushRecoveryPositionTrajectoryGenerator = new WrapperForMultiplePositionTrajectoryGenerators(pushRecoveryPositionTrajectoryGenerators, namePrefix
            + "PushRecoveryTrajectoryGenerator", registry);

      orientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator(namePrefix + "SwingFootOrientation", worldFrame, swingTimeProvider,
            initialConfigurationProvider, finalConfigurationProvider, registry);
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

   private final FramePose newFootstepPose = new FramePose();
   private final FramePoint oldFootstepPosition = new FramePoint();

   public void setFootstep(Footstep footstep, TrajectoryParameters trajectoryParameters, boolean useLowHeightTrajectory)
   {
      footstep.getPose(newFootstepPose);
      newFootstepPose.changeFrame(worldFrame);
      finalConfigurationProvider.setPose(newFootstepPose);
      initialConfigurationProvider.get(oldFootstepPosition);

      newFootstepPose.changeFrame(worldFrame);
      oldFootstepPosition.changeFrame(worldFrame);

      boolean worldFrameDeltaZAboveThreshold = Math.abs(newFootstepPose.getZ() - oldFootstepPosition.getZ()) > SimpleTwoWaypointTrajectoryParameters
            .getMinimumAnkleHeightDifferenceForStepOnOrOff();

      if (worldFrameDeltaZAboveThreshold)
      {
         trajectoryParameters = new SimpleTwoWaypointTrajectoryParameters(TrajectoryWaypointGenerationMethod.STEP_ON_OR_OFF);
      }
      else
      {
         if (useLowHeightTrajectory)
         {
            trajectoryParameters = new SimpleTwoWaypointTrajectoryParameters(TrajectoryWaypointGenerationMethod.LOW_HEIGHT);
         }
      }

      trajectoryParametersProvider.set(trajectoryParameters);
   }

   public void replanTrajectory(Footstep footstep, double swingTimeRemaining, boolean useLowHeightTrajectory)
   {
      setFootstep(footstep, trajectoryParametersProvider.getTrajectoryParameters(), useLowHeightTrajectory);
      this.swingTimeRemaining.set(swingTimeRemaining);
      this.replanTrajectory.set(true);
   }
}
