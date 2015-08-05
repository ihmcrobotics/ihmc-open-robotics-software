package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.trajectories.PushRecoveryTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.SoftTouchdownPositionTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointPositionTrajectoryGenerator;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.robotics.trajectories.TwoWaypointTrajectoryGeneratorParameters;
import us.ihmc.robotics.trajectories.providers.ConstantVectorProvider;
import us.ihmc.robotics.trajectories.providers.CurrentAngularVelocityProvider;
import us.ihmc.robotics.trajectories.providers.CurrentConfigurationProvider;
import us.ihmc.robotics.trajectories.providers.CurrentLinearVelocityProvider;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.TrajectoryParameters;
import us.ihmc.robotics.trajectories.providers.TrajectoryParametersProvider;
import us.ihmc.robotics.trajectories.providers.VectorProvider;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.yoUtilities.controllers.YoSE3PIDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.VelocityConstrainedOrientationTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.WrapperForMultiplePositionTrajectoryGenerators;
import us.ihmc.yoUtilities.math.trajectories.providers.YoSE3ConfigurationProvider;
import us.ihmc.yoUtilities.math.trajectories.providers.YoVariableDoubleProvider;

public class SwingState extends AbstractUnconstrainedState implements SwingStateInterface
{

   private final boolean visualizeSwingTrajectory = true;

   private final BooleanYoVariable replanTrajectory;
   private final YoVariableDoubleProvider swingTimeRemaining;

   private final PositionTrajectoryGenerator positionTrajectoryGenerator, pushRecoveryPositionTrajectoryGenerator;
   private final VelocityConstrainedOrientationTrajectoryGenerator orientationTrajectoryGenerator;

   private final CurrentConfigurationProvider stanceConfigurationProvider;
   private final CurrentConfigurationProvider initialConfigurationProvider;
   private final VectorProvider initialVelocityProvider;
   private final YoSE3ConfigurationProvider finalConfigurationProvider;
   private final TrajectoryParametersProvider trajectoryParametersProvider = new TrajectoryParametersProvider(new TrajectoryParameters());

   private final DoubleProvider swingTimeProvider;   

   private final DoubleYoVariable swingTimeSpeedUpFactor;
   private final DoubleYoVariable maxSwingTimeSpeedUpFactor;
   private final DoubleYoVariable minSwingTimeForDisturbanceRecovery;
   private final BooleanYoVariable isSwingSpeedUpEnabled;
   private final DoubleYoVariable currentTime;
   private final DoubleYoVariable currentTimeWithSwingSpeedUp;

   private final VectorProvider currentAngularVelocityProvider;
   private final FrameVector initialAngularVelocity = new FrameVector();

   private final BooleanYoVariable hasInitialAngularConfigurationBeenProvided;

   private final DoubleYoVariable finalSwingHeightOffset;
   private final double controlDT;

   public SwingState(FootControlHelper footControlHelper, DoubleProvider swingTimeProvider, VectorProvider touchdownVelocityProvider,
                     VectorProvider touchdownAccelerationProvider, YoSE3PIDGains gains, YoVariableRegistry registry)
   {
      super(ConstraintType.SWING, footControlHelper, gains, registry);

      controlDT = footControlHelper.getMomentumBasedController().getControlDT();

      String namePrefix = robotSide.getCamelCaseNameForStartOfExpression() + "Foot";

      finalConfigurationProvider = new YoSE3ConfigurationProvider(namePrefix + "SwingFinal", worldFrame, registry);
      finalSwingHeightOffset = new DoubleYoVariable(namePrefix + "SwingFinalHeightOffset", registry);
      finalSwingHeightOffset.set(footControlHelper.getWalkingControllerParameters().getDesiredTouchdownHeightOffset());
      replanTrajectory = new BooleanYoVariable(namePrefix + "SwingReplanTrajectory", registry);
      swingTimeRemaining = new YoVariableDoubleProvider(namePrefix + "SwingTimeRemaining", registry);
      this.swingTimeProvider = swingTimeProvider;

      ArrayList<PositionTrajectoryGenerator> positionTrajectoryGenerators = new ArrayList<PositionTrajectoryGenerator>();
      ArrayList<PositionTrajectoryGenerator> pushRecoveryPositionTrajectoryGenerators = new ArrayList<PositionTrajectoryGenerator>();

      CommonHumanoidReferenceFrames referenceFrames = momentumBasedController.getReferenceFrames();
      ReferenceFrame footFrame = referenceFrames.getFootFrame(robotSide);
      TwistCalculator twistCalculator = momentumBasedController.getTwistCalculator();
      RigidBody rigidBody = contactableFoot.getRigidBody();

      initialConfigurationProvider = new CurrentConfigurationProvider(footFrame);
      ReferenceFrame stanceFootFrame = referenceFrames.getFootFrame(robotSide.getOppositeSide());
      stanceConfigurationProvider = new CurrentConfigurationProvider(stanceFootFrame);
      initialVelocityProvider = new CurrentLinearVelocityProvider(footFrame, rigidBody, twistCalculator);

      YoGraphicsListRegistry yoGraphicsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();

      PositionTrajectoryGenerator swingTrajectoryGenerator;

      PositionTrajectoryGenerator touchdownTrajectoryGenerator = new SoftTouchdownPositionTrajectoryGenerator(namePrefix + "Touchdown", worldFrame,
            finalConfigurationProvider, touchdownVelocityProvider, touchdownAccelerationProvider, swingTimeProvider, registry);

      WalkingControllerParameters walkingControllerParameters = footControlHelper.getWalkingControllerParameters();
      swingTrajectoryGenerator = new TwoWaypointPositionTrajectoryGenerator(namePrefix + "Swing", worldFrame, swingTimeProvider,
            initialConfigurationProvider, initialVelocityProvider, stanceConfigurationProvider, finalConfigurationProvider, touchdownVelocityProvider, trajectoryParametersProvider,
            registry, yoGraphicsListRegistry, walkingControllerParameters, visualizeSwingTrajectory);

      pushRecoveryPositionTrajectoryGenerator = setupPushRecoveryTrajectoryGenerator(swingTimeProvider, registry, namePrefix,
            pushRecoveryPositionTrajectoryGenerators, yoGraphicsListRegistry, swingTrajectoryGenerator, touchdownTrajectoryGenerator);


      positionTrajectoryGenerators.add(swingTrajectoryGenerator);
      positionTrajectoryGenerators.add(touchdownTrajectoryGenerator);

      positionTrajectoryGenerator = new WrapperForMultiplePositionTrajectoryGenerators(positionTrajectoryGenerators, namePrefix, registry);

      currentAngularVelocityProvider = new CurrentAngularVelocityProvider(footFrame, rigidBody, twistCalculator);
      VectorProvider initialAngularVelocityProvider = new ConstantVectorProvider(initialAngularVelocity);
      VectorProvider finalAngularVelocityProvider = new ConstantVectorProvider(new FrameVector(footFrame));
      orientationTrajectoryGenerator = new VelocityConstrainedOrientationTrajectoryGenerator(namePrefix + "Swing", worldFrame, swingTimeProvider,
            initialConfigurationProvider, initialAngularVelocityProvider, finalConfigurationProvider, finalAngularVelocityProvider, registry);
      hasInitialAngularConfigurationBeenProvided = new BooleanYoVariable(namePrefix + "HasInitialAngularConfigurationBeenProvided", registry);

      swingTimeSpeedUpFactor = new DoubleYoVariable(namePrefix + "SwingTimeSpeedUpFactor", registry);
      maxSwingTimeSpeedUpFactor = new DoubleYoVariable(namePrefix + "MaxSwingTimeSpeedUpFactor", registry);
      minSwingTimeForDisturbanceRecovery = new DoubleYoVariable(namePrefix + "MinSwingTimeForDisturbanceRecovery", registry);
      minSwingTimeForDisturbanceRecovery.set(walkingControllerParameters.getMinimumSwingTimeForDisturbanceRecovery());
      currentTime = new DoubleYoVariable(namePrefix + "CurrentTime", registry);
      currentTimeWithSwingSpeedUp = new DoubleYoVariable(namePrefix + "CurrentTimeWithSwingSpeedUp", registry);
      isSwingSpeedUpEnabled = new BooleanYoVariable(namePrefix + "IsSwingSpeedUpEnabled", registry);
      isSwingSpeedUpEnabled.set(walkingControllerParameters.allowDisturbanceRecoveryBySpeedingUpSwing());
   }

   private PositionTrajectoryGenerator setupPushRecoveryTrajectoryGenerator(DoubleProvider swingTimeProvider, YoVariableRegistry registry, String namePrefix,
         ArrayList<PositionTrajectoryGenerator> pushRecoveryPositionTrajectoryGenerators, YoGraphicsListRegistry yoGraphicsListRegistry,
         PositionTrajectoryGenerator swingTrajectoryGenerator, PositionTrajectoryGenerator touchdownTrajectoryGenerator)
   {
      PositionTrajectoryGenerator pushRecoverySwingTrajectoryGenerator = new PushRecoveryTrajectoryGenerator(namePrefix + "SwingPushRecovery", worldFrame,
            swingTimeProvider, swingTimeRemaining, initialConfigurationProvider, initialVelocityProvider, finalConfigurationProvider, registry,
            yoGraphicsListRegistry, swingTrajectoryGenerator);

      pushRecoveryPositionTrajectoryGenerators.add(pushRecoverySwingTrajectoryGenerator);
      pushRecoveryPositionTrajectoryGenerators.add(touchdownTrajectoryGenerator);

      PositionTrajectoryGenerator pushRecoveryPositionTrajectoryGenerator = new WrapperForMultiplePositionTrajectoryGenerators(pushRecoveryPositionTrajectoryGenerators, namePrefix
            + "PushRecoveryTrajectoryGenerator", registry);
      return pushRecoveryPositionTrajectoryGenerator;
   }

   @Override
   public void setInitialDesireds(FrameOrientation initialOrientation, FrameVector initialAngularVelocity)
   {
      hasInitialAngularConfigurationBeenProvided.set(true);
      this.initialAngularVelocity.setIncludingFrame(initialAngularVelocity);
   }

   @Override
   protected void initializeTrajectory()
   {
      if (!hasInitialAngularConfigurationBeenProvided.getBooleanValue())
         currentAngularVelocityProvider.get(initialAngularVelocity);
      positionTrajectoryGenerator.initialize();
      orientationTrajectoryGenerator.initialize();

      trajectoryWasReplanned = false;
      replanTrajectory.set(false);
   }

   @Override
   protected void computeAndPackTrajectory()
   {
      if (replanTrajectory.getBooleanValue()) // This seems like a bad place for this?
      {
         pushRecoveryPositionTrajectoryGenerator.initialize();
         replanTrajectory.set(false);
         trajectoryWasReplanned = true;
      }

      currentTime.set(getTimeInCurrentState());

      double time;
      if (!isSwingSpeedUpEnabled.getBooleanValue() || currentTimeWithSwingSpeedUp.isNaN())
         time = currentTime.getDoubleValue();
      else
      {
         currentTimeWithSwingSpeedUp.add(swingTimeSpeedUpFactor.getDoubleValue() * controlDT);
         time = currentTimeWithSwingSpeedUp.getDoubleValue();
      }

      if (!trajectoryWasReplanned)
      {
         positionTrajectoryGenerator.compute(time);

         positionTrajectoryGenerator.packLinearData(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
      }
      else
      {
         pushRecoveryPositionTrajectoryGenerator.compute(time);

         pushRecoveryPositionTrajectoryGenerator.packLinearData(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
      }

      orientationTrajectoryGenerator.compute(getTimeInCurrentState());
      orientationTrajectoryGenerator.packAngularData(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);

      if (isSwingSpeedUpEnabled.getBooleanValue() && !currentTimeWithSwingSpeedUp.isNaN())
      {
         desiredLinearVelocity.scale(swingTimeSpeedUpFactor.getDoubleValue());
         desiredAngularVelocity.scale(swingTimeSpeedUpFactor.getDoubleValue());

         double speedUpFactorSquared = swingTimeSpeedUpFactor.getDoubleValue() * swingTimeSpeedUpFactor.getDoubleValue();
         desiredLinearAcceleration.scale(speedUpFactorSquared);
         desiredAngularAcceleration.scale(speedUpFactorSquared);
      }
   }

   private final FramePose newFootstepPose = new FramePose();
   private final FramePoint oldFootstepPosition = new FramePoint();
   @Override
   public void setFootstep(Footstep footstep)
   {
      footstep.getPose(newFootstepPose);
      newFootstepPose.changeFrame(worldFrame);

      newFootstepPose.setZ(newFootstepPose.getZ() + finalSwingHeightOffset.getDoubleValue());
      finalConfigurationProvider.setPose(newFootstepPose);
      initialConfigurationProvider.get(oldFootstepPosition);

      newFootstepPose.changeFrame(worldFrame);
      oldFootstepPosition.changeFrame(worldFrame);

      boolean worldFrameDeltaZAboveThreshold = Math.abs(newFootstepPose.getZ() - oldFootstepPosition.getZ()) > TwoWaypointTrajectoryGeneratorParameters.getMinimumHeightDifferenceForStepOnOrOff();

      if (footstep.getTrajectoryType() == TrajectoryType.PUSH_RECOVERY)
      {
         trajectoryParametersProvider.set(new TrajectoryParameters(TrajectoryType.PUSH_RECOVERY));
      }
      else if (worldFrameDeltaZAboveThreshold)
      {
         trajectoryParametersProvider.set(new TrajectoryParameters(TrajectoryType.OBSTACLE_CLEARANCE, footstep.getSwingHeight()));
      }
      else
      {
         trajectoryParametersProvider.set(new TrajectoryParameters(footstep.getTrajectoryType(), footstep.getSwingHeight()));
      }
   }

   @Override
   public void replanTrajectory(Footstep newFootstep)
   {
      setFootstep(newFootstep);
      if (!currentTimeWithSwingSpeedUp.isNaN())
         this.swingTimeRemaining.set(swingTimeProvider.getValue() - currentTimeWithSwingSpeedUp.getDoubleValue());
      else
         this.swingTimeRemaining.set(swingTimeProvider.getValue() - getTimeInCurrentState());
      this.replanTrajectory.set(true);
   }

   @Override
   public void requestSwingSpeedUp(double speedUpFactor)
   {
      if (isSwingSpeedUpEnabled.getBooleanValue())
      {
         if (speedUpFactor <= 1.1 || speedUpFactor <= swingTimeSpeedUpFactor.getDoubleValue())
            return;
         speedUpFactor = MathTools.clipToMinMax(speedUpFactor, swingTimeSpeedUpFactor.getDoubleValue(), maxSwingTimeSpeedUpFactor.getDoubleValue());
         swingTimeSpeedUpFactor.set(speedUpFactor);
         if (currentTimeWithSwingSpeedUp.isNaN())
            currentTimeWithSwingSpeedUp.set(currentTime.getDoubleValue());
      }
   }

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();

      maxSwingTimeSpeedUpFactor.set(Math.max(swingTimeProvider.getValue() / minSwingTimeForDisturbanceRecovery.getDoubleValue(), 1.0));
      swingTimeSpeedUpFactor.set(1.0);
      currentTimeWithSwingSpeedUp.set(Double.NaN);
   }

   @Override
   public void doTransitionOutOfAction()
   {
      super.doTransitionOutOfAction();

      hasInitialAngularConfigurationBeenProvided.set(false);
      currentTimeWithSwingSpeedUp.set(Double.NaN);
   }
}
