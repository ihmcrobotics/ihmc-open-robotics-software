package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.trajectories.SoftTouchdownPositionTrajectoryGenerator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.StraightLinePositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.providers.YoSE3ConfigurationProvider;
import us.ihmc.robotics.math.trajectories.providers.YoVariableDoubleProvider;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.ChangeableConfigurationProvider;
import us.ihmc.robotics.trajectories.providers.ConstantVectorProvider;
import us.ihmc.robotics.trajectories.providers.SettableDoubleProvider;
import us.ihmc.robotics.trajectories.providers.SettablePositionProvider;
import us.ihmc.robotics.trajectories.providers.VectorProvider;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoUtilities.controllers.YoSE3PIDGains;

public class MoveStraightState extends AbstractUnconstrainedState
{
   private final StraightLinePositionTrajectoryGenerator positionTrajectoryGenerator;
   private final OrientationInterpolationTrajectoryGenerator orientationTrajectoryGenerator;

   private final YoSE3ConfigurationProvider finalConfigurationProvider;
   private final YoVariableDoubleProvider trajectoryTimeProvider;

   private final ReferenceFrame footFrame;
   private final FramePose initialFootPose = new FramePose();
   private final ChangeableConfigurationProvider initialConfigurationProvider = new ChangeableConfigurationProvider();

   private final BooleanYoVariable isPerformingTouchdown;
   private final SettableDoubleProvider touchdownInitialTimeProvider = new SettableDoubleProvider(0.0);
   private final SettablePositionProvider currentDesiredFootPosition = new SettablePositionProvider();
   private final SoftTouchdownPositionTrajectoryGenerator positionTrajectoryForDisturbanceRecovery;

   public MoveStraightState(FootControlHelper footControlHelper, YoSE3PIDGains gains, YoVariableRegistry registry)
   {
      super(ConstraintType.MOVE_STRAIGHT, footControlHelper, gains, registry);

      RigidBody rigidBody = contactableFoot.getRigidBody();
      String namePrefix = rigidBody.getName();

      finalConfigurationProvider = new YoSE3ConfigurationProvider(namePrefix + "MoveStraightFootFinal", worldFrame, registry);
      trajectoryTimeProvider = new YoVariableDoubleProvider(namePrefix + "MoveStraightTrajectoryTime", registry);

      footFrame = momentumBasedController.getReferenceFrames().getFootFrame(robotSide);

      positionTrajectoryGenerator = new StraightLinePositionTrajectoryGenerator(namePrefix + "FootPosition", worldFrame, trajectoryTimeProvider,
            initialConfigurationProvider, finalConfigurationProvider, registry);

      orientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator(namePrefix + "Orientation", worldFrame, trajectoryTimeProvider,
            initialConfigurationProvider, finalConfigurationProvider, registry);

      isPerformingTouchdown = new BooleanYoVariable(namePrefix + "IsPerformingTouchdown", registry);

      VectorProvider touchdownVelocityProvider = new ConstantVectorProvider(new FrameVector(worldFrame, 0.0, 0.0, -0.3));
      VectorProvider touchdownAccelerationProvider = new ConstantVectorProvider(new FrameVector(worldFrame, 0.0, 0.0, -1.0));
      positionTrajectoryForDisturbanceRecovery = new SoftTouchdownPositionTrajectoryGenerator(namePrefix + "MoveStraightTouchdown", worldFrame,
            currentDesiredFootPosition, touchdownVelocityProvider, touchdownAccelerationProvider, touchdownInitialTimeProvider, registry);
   }

   public void setFootPose(FramePose footPose, double trajectoryTime)
   {
      finalConfigurationProvider.setPose(footPose);
      trajectoryTimeProvider.set(trajectoryTime);
   }

   @Override
   protected void initializeTrajectory()
   {
      if (getPreviousState() == this)
      {
         positionTrajectoryGenerator.get(desiredPosition);
         orientationTrajectoryGenerator.get(desiredOrientation);
         initialFootPose.setPoseIncludingFrame(desiredPosition, desiredOrientation);
      }
      else
      {
         initialFootPose.setToZero(footFrame);
      }
      initialConfigurationProvider.set(initialFootPose);

      positionTrajectoryGenerator.initialize();
      orientationTrajectoryGenerator.initialize();
   }

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();
      if (getPreviousState() == this)
         legSingularityAndKneeCollapseAvoidanceControlModule.setCheckVelocityForSwingSingularityAvoidance(false);
      isPerformingTouchdown.set(false);
   };

   @Override
   protected void computeAndPackTrajectory()
   {
      if (isPerformingTouchdown.getBooleanValue())
      {
         positionTrajectoryForDisturbanceRecovery.compute(getTimeInCurrentState());

         positionTrajectoryForDisturbanceRecovery.packLinearData(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
         orientationTrajectoryGenerator.packAngularData(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
         desiredAngularVelocity.setToZero();
         desiredAngularAcceleration.setToZero();
      }
      else
      {
         positionTrajectoryGenerator.compute(getTimeInCurrentState());
         orientationTrajectoryGenerator.compute(getTimeInCurrentState());
         
         positionTrajectoryGenerator.packLinearData(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
         orientationTrajectoryGenerator.packAngularData(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
      }
   }

   public void requestTouchdownForDisturbanceRecovery()
   {
      if (isPerformingTouchdown.getBooleanValue())
         return;

      positionTrajectoryGenerator.get(desiredPosition);
      currentDesiredFootPosition.set(desiredPosition);
      touchdownInitialTimeProvider.setValue(getTimeInCurrentState());
      positionTrajectoryForDisturbanceRecovery.initialize();

      isPerformingTouchdown.set(true);
   }

   @Override
   public void doTransitionOutOfAction()
   {
      super.doTransitionOutOfAction();
      isPerformingTouchdown.set(false);
   }
}