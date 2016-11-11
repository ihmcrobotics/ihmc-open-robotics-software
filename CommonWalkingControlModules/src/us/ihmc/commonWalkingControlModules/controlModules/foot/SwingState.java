package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.trajectories.PushRecoveryTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.SoftTouchdownPositionTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointPositionTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointSwingGenerator;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.VelocityConstrainedOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.WrapperForMultiplePositionTrajectoryGenerators;
import us.ihmc.robotics.math.trajectories.providers.YoSE3ConfigurationProvider;
import us.ihmc.robotics.math.trajectories.providers.YoVariableDoubleProvider;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.robotics.trajectories.TwoWaypointTrajectoryGeneratorParameters;
import us.ihmc.robotics.trajectories.providers.CurrentAngularVelocityProvider;
import us.ihmc.robotics.trajectories.providers.CurrentConfigurationProvider;
import us.ihmc.robotics.trajectories.providers.CurrentLinearVelocityProvider;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.SettableDoubleProvider;
import us.ihmc.robotics.trajectories.providers.TrajectoryParameters;
import us.ihmc.robotics.trajectories.providers.TrajectoryParametersProvider;
import us.ihmc.robotics.trajectories.providers.VectorProvider;

public class SwingState extends AbstractUnconstrainedState
{
   private final boolean useNewSwingTrajectoyOptimization;
   private static final boolean CONTROL_TOE = false;

   private final BooleanYoVariable replanTrajectory;
   private final BooleanYoVariable doContinuousReplanning;
   private final YoVariableDoubleProvider swingTimeRemaining;

   private final TwoWaypointPositionTrajectoryGenerator swingTrajectoryGenerator;
   private final CurrentConfigurationProvider stanceConfigurationProvider;

   private final TwoWaypointSwingGenerator swingTrajectoryGeneratorNew;
   private final VectorProvider touchdownVelocityProvider;
   private final FramePoint initialPosition = new FramePoint();
   private final FrameVector initialVelocity = new FrameVector();
   private final FramePoint finalPosition = new FramePoint();
   private final FrameVector finalVelocity = new FrameVector();
   private final FramePoint stanceFootPosition = new FramePoint();

   private final PositionTrajectoryGenerator positionTrajectoryGenerator, pushRecoveryPositionTrajectoryGenerator;
   private final VelocityConstrainedOrientationTrajectoryGenerator orientationTrajectoryGenerator;

   private final CurrentConfigurationProvider initialConfigurationProvider;
   private final VectorProvider initialVelocityProvider;
   private final YoSE3ConfigurationProvider finalConfigurationProvider;
   private final TrajectoryParametersProvider trajectoryParametersProvider = new TrajectoryParametersProvider(new TrajectoryParameters());

   private final SettableDoubleProvider swingTimeProvider = new SettableDoubleProvider();

   private final DoubleYoVariable swingTimeSpeedUpFactor;
   private final DoubleYoVariable maxSwingTimeSpeedUpFactor;
   private final DoubleYoVariable minSwingTimeForDisturbanceRecovery;
   private final BooleanYoVariable isSwingSpeedUpEnabled;
   private final DoubleYoVariable currentTime;
   private final DoubleYoVariable currentTimeWithSwingSpeedUp;

   private final VectorProvider currentAngularVelocityProvider;
   private final FrameOrientation initialOrientation = new FrameOrientation();
   private final FrameVector initialAngularVelocity = new FrameVector();

   private final BooleanYoVariable hasInitialAngularConfigurationBeenProvided;

   private final DoubleYoVariable finalSwingHeightOffset;
   private final double controlDT;

   private final ReferenceFrame footFrame;
   private final ReferenceFrame toeFrame;
   private final ReferenceFrame stanceFootFrame;
   private final ReferenceFrame controlFrame;

   private final RigidBodyTransform transformFromToeToAnkle = new RigidBodyTransform();

   private final DoubleYoVariable velocityAdjustmentDamping;
   private final YoFrameVector adjustmentVelocityCorrection;
   private final FramePoint unadjustedPosition = new FramePoint(worldFrame);

   public SwingState(FootControlHelper footControlHelper, VectorProvider touchdownVelocityProvider, VectorProvider touchdownAccelerationProvider,
         YoSE3PIDGainsInterface gains, YoVariableRegistry registry)
   {
      super(ConstraintType.SWING, footControlHelper, gains, registry);

      controlDT = footControlHelper.getMomentumBasedController().getControlDT();

      String namePrefix = robotSide.getCamelCaseNameForStartOfExpression() + "Foot";

      finalConfigurationProvider = new YoSE3ConfigurationProvider(namePrefix + "SwingFinal", worldFrame, registry);
      finalSwingHeightOffset = new DoubleYoVariable(namePrefix + "SwingFinalHeightOffset", registry);
      finalSwingHeightOffset.set(footControlHelper.getWalkingControllerParameters().getDesiredTouchdownHeightOffset());
      replanTrajectory = new BooleanYoVariable(namePrefix + "SwingReplanTrajectory", registry);
      swingTimeRemaining = new YoVariableDoubleProvider(namePrefix + "SwingTimeRemaining", registry);

      velocityAdjustmentDamping = new DoubleYoVariable(namePrefix + "VelocityAdjustmentDamping", registry);
      velocityAdjustmentDamping.set(footControlHelper.getWalkingControllerParameters().getSwingFootVelocityAdjustmentDamping());
      adjustmentVelocityCorrection = new YoFrameVector(namePrefix + "AdjustmentVelocityCorrection", worldFrame, registry);

      // todo make a smarter distinction on this as a way to work with the push recovery module
      doContinuousReplanning = new BooleanYoVariable(namePrefix + "DoContinuousReplanning", registry);

      footFrame = contactableFoot.getFrameAfterParentJoint();
      toeFrame = createToeFrame(robotSide);

      controlFrame = CONTROL_TOE ? toeFrame : footFrame;
      stanceFootFrame = CONTROL_TOE ? createToeFrame(robotSide.getOppositeSide()) : momentumBasedController.getReferenceFrames().getFootFrame(robotSide.getOppositeSide());

      TwistCalculator twistCalculator = momentumBasedController.getTwistCalculator();
      RigidBody rigidBody = contactableFoot.getRigidBody();

      stanceConfigurationProvider = new CurrentConfigurationProvider(stanceFootFrame);
      initialConfigurationProvider = new CurrentConfigurationProvider(controlFrame);
      initialVelocityProvider = new CurrentLinearVelocityProvider(controlFrame, rigidBody, twistCalculator);

      YoGraphicsListRegistry yoGraphicsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();

      PositionTrajectoryGenerator touchdownTrajectoryGenerator = new SoftTouchdownPositionTrajectoryGenerator(namePrefix + "Touchdown", worldFrame,
            finalConfigurationProvider, touchdownVelocityProvider, touchdownAccelerationProvider, swingTimeProvider, registry);

      WalkingControllerParameters walkingControllerParameters = footControlHelper.getWalkingControllerParameters();
      double maxSwingHeightFromStanceFoot;
      if (walkingControllerParameters != null)
         maxSwingHeightFromStanceFoot = walkingControllerParameters.getMaxSwingHeightFromStanceFoot();
      else
         maxSwingHeightFromStanceFoot = 0.0;

      this.touchdownVelocityProvider = touchdownVelocityProvider;

      ArrayList<PositionTrajectoryGenerator> positionTrajectoryGenerators = new ArrayList<PositionTrajectoryGenerator>();
      ArrayList<PositionTrajectoryGenerator> pushRecoveryPositionTrajectoryGenerators = new ArrayList<PositionTrajectoryGenerator>();

      useNewSwingTrajectoyOptimization = walkingControllerParameters.useSwingTrajectoryOptimizer();
      if (useNewSwingTrajectoyOptimization)
      {
         swingTrajectoryGeneratorNew = new TwoWaypointSwingGenerator(namePrefix + "SwingNew", maxSwingHeightFromStanceFoot, registry, yoGraphicsListRegistry);
         swingTrajectoryGenerator = null;

         positionTrajectoryGenerators.add(swingTrajectoryGeneratorNew);
         pushRecoveryPositionTrajectoryGenerator = setupPushRecoveryTrajectoryGenerator(swingTimeProvider, registry, namePrefix,
               pushRecoveryPositionTrajectoryGenerators, yoGraphicsListRegistry, swingTrajectoryGeneratorNew, touchdownTrajectoryGenerator);
      }
      else
      {
         swingTrajectoryGeneratorNew = null;
         swingTrajectoryGenerator = new TwoWaypointPositionTrajectoryGenerator(namePrefix + "Swing", worldFrame, swingTimeProvider, initialConfigurationProvider,
               initialVelocityProvider, stanceConfigurationProvider, finalConfigurationProvider, touchdownVelocityProvider, trajectoryParametersProvider, registry,
               yoGraphicsListRegistry, maxSwingHeightFromStanceFoot, true);

         positionTrajectoryGenerators.add(swingTrajectoryGenerator);
         pushRecoveryPositionTrajectoryGenerator = setupPushRecoveryTrajectoryGenerator(swingTimeProvider, registry, namePrefix,
               pushRecoveryPositionTrajectoryGenerators, yoGraphicsListRegistry, swingTrajectoryGenerator, touchdownTrajectoryGenerator);
      }
      positionTrajectoryGenerators.add(touchdownTrajectoryGenerator);

      positionTrajectoryGenerator = new WrapperForMultiplePositionTrajectoryGenerators(positionTrajectoryGenerators, namePrefix, registry);

      currentAngularVelocityProvider = new CurrentAngularVelocityProvider(controlFrame, rigidBody, twistCalculator);
      orientationTrajectoryGenerator = new VelocityConstrainedOrientationTrajectoryGenerator(namePrefix + "Swing", worldFrame, registry);
      hasInitialAngularConfigurationBeenProvided = new BooleanYoVariable(namePrefix + "HasInitialAngularConfigurationBeenProvided", registry);

      swingTimeSpeedUpFactor = new DoubleYoVariable(namePrefix + "SwingTimeSpeedUpFactor", registry);
      minSwingTimeForDisturbanceRecovery = new DoubleYoVariable(namePrefix + "MinSwingTimeForDisturbanceRecovery", registry);
      minSwingTimeForDisturbanceRecovery.set(walkingControllerParameters.getMinimumSwingTimeForDisturbanceRecovery());
      maxSwingTimeSpeedUpFactor = new DoubleYoVariable(namePrefix + "MaxSwingTimeSpeedUpFactor", registry);
      maxSwingTimeSpeedUpFactor.set(Math.max(swingTimeProvider.getValue() / minSwingTimeForDisturbanceRecovery.getDoubleValue(), 1.0));
      currentTime = new DoubleYoVariable(namePrefix + "CurrentTime", registry);
      currentTimeWithSwingSpeedUp = new DoubleYoVariable(namePrefix + "CurrentTimeWithSwingSpeedUp", registry);
      isSwingSpeedUpEnabled = new BooleanYoVariable(namePrefix + "IsSwingSpeedUpEnabled", registry);
      isSwingSpeedUpEnabled.set(walkingControllerParameters.allowDisturbanceRecoveryBySpeedingUpSwing());

      FramePose controlFramePose = new FramePose(controlFrame);
      controlFramePose.changeFrame(contactableFoot.getRigidBody().getBodyFixedFrame());
      changeControlFrame(controlFramePose);
   }

   private ReferenceFrame createToeFrame(RobotSide robotSide)
   {
      ContactableFoot contactableFoot = momentumBasedController.getContactableFeet().get(robotSide);
      ReferenceFrame footFrame = momentumBasedController.getReferenceFrames().getFootFrame(robotSide);
      FramePoint2d toeContactPoint2d = new FramePoint2d();
      contactableFoot.getToeOffContactPoint(toeContactPoint2d);
      FramePoint toeContactPoint = new FramePoint();
      toeContactPoint.setXYIncludingFrame(toeContactPoint2d);
      toeContactPoint.changeFrame(footFrame);

      transformFromToeToAnkle.setTranslation(toeContactPoint.getVectorCopy());
      return ReferenceFrame.constructFrameWithUnchangingTransformToParent(robotSide.getCamelCaseNameForStartOfExpression() + "ToeFrame", footFrame, transformFromToeToAnkle);
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

      PositionTrajectoryGenerator pushRecoveryPositionTrajectoryGenerator = new WrapperForMultiplePositionTrajectoryGenerators(
            pushRecoveryPositionTrajectoryGenerators, namePrefix + "PushRecoveryTrajectoryGenerator", registry);
      return pushRecoveryPositionTrajectoryGenerator;
   }

   public void setInitialDesireds(FrameOrientation initialOrientation, FrameVector initialAngularVelocity)
   {
      hasInitialAngularConfigurationBeenProvided.set(true);
      orientationTrajectoryGenerator.setInitialConditions(initialOrientation, initialAngularVelocity);
   }

   protected void initializeTrajectory()
   {
      if (!hasInitialAngularConfigurationBeenProvided.getBooleanValue())
      {
         currentAngularVelocityProvider.get(initialAngularVelocity);
         initialOrientation.setToZero(controlFrame);
         orientationTrajectoryGenerator.setInitialConditions(initialOrientation, initialAngularVelocity);
      }

      orientationTrajectoryGenerator.setTrajectoryTime(swingTimeProvider.getValue());

      if (useNewSwingTrajectoyOptimization)
      {
         initialConfigurationProvider.getPosition(initialPosition);
         initialVelocityProvider.get(initialVelocity);
         finalConfigurationProvider.getPosition(finalPosition);
         touchdownVelocityProvider.get(finalVelocity);
         stanceConfigurationProvider.getPosition(stanceFootPosition);
         swingTrajectoryGeneratorNew.setInitialConditions(initialPosition, initialVelocity);
         swingTrajectoryGeneratorNew.setFinalConditions(finalPosition, finalVelocity);
         swingTrajectoryGeneratorNew.setStepTime(swingTimeProvider.getValue());
         swingTrajectoryGeneratorNew.setTrajectoryType(trajectoryParametersProvider.getTrajectoryParameters().getTrajectoryType());
         swingTrajectoryGeneratorNew.setSwingHeight(trajectoryParametersProvider.getTrajectoryParameters().getSwingHeight());
         swingTrajectoryGeneratorNew.setStanceFootPosition(stanceFootPosition);
      }

      positionTrajectoryGenerator.initialize();
      orientationTrajectoryGenerator.initialize();

      trajectoryWasReplanned = false;
      replanTrajectory.set(false);
   }

   protected void reinitializeTrajectory()
   {
      orientationTrajectoryGenerator.setTrajectoryTime(swingTimeProvider.getValue());

      if (useNewSwingTrajectoyOptimization)
      {
         finalConfigurationProvider.getPosition(finalPosition);
         touchdownVelocityProvider.get(finalVelocity);
         swingTrajectoryGeneratorNew.setFinalConditions(finalPosition, finalVelocity);
         swingTrajectoryGeneratorNew.setStepTime(swingTimeProvider.getValue());
         swingTrajectoryGeneratorNew.setTrajectoryType(trajectoryParametersProvider.getTrajectoryParameters().getTrajectoryType());
      }

      positionTrajectoryGenerator.initialize();
      orientationTrajectoryGenerator.initialize();

      trajectoryWasReplanned = false;
      replanTrajectory.set(false);
   }

   protected void computeAndPackTrajectory()
   {
      if (this.replanTrajectory.getBooleanValue()) // This seems like a bad place for this?
      {
         if (!doContinuousReplanning.getBooleanValue())
         {
            pushRecoveryPositionTrajectoryGenerator.initialize();
            this.replanTrajectory.set(false);
            trajectoryWasReplanned = true;
         }
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

      if (!trajectoryWasReplanned || doContinuousReplanning.getBooleanValue())
      {
         boolean footstepWasAdjusted = false;
         positionTrajectoryGenerator.compute(time);

         if (replanTrajectory.getBooleanValue())
         {
            footstepWasAdjusted = true;
            positionTrajectoryGenerator.getLinearData(unadjustedPosition, desiredLinearVelocity, desiredAngularAcceleration);

            reinitializeTrajectory();
            positionTrajectoryGenerator.compute(time);
         }

         positionTrajectoryGenerator.getLinearData(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);

         if (footstepWasAdjusted)
         {
            adjustmentVelocityCorrection.set(desiredPosition);
            adjustmentVelocityCorrection.sub(unadjustedPosition);
            adjustmentVelocityCorrection.scale(1.0 / controlDT);
            adjustmentVelocityCorrection.setZ(0.0);
            adjustmentVelocityCorrection.scale(velocityAdjustmentDamping.getDoubleValue());

            desiredLinearVelocity.add(adjustmentVelocityCorrection.getFrameTuple());
         }
         else
         {
            adjustmentVelocityCorrection.setToZero();
         }
      }
      else
      {
         pushRecoveryPositionTrajectoryGenerator.compute(time);

         pushRecoveryPositionTrajectoryGenerator.getLinearData(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
      }

      orientationTrajectoryGenerator.compute(getTimeInCurrentState());
      orientationTrajectoryGenerator.getAngularData(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);

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

   public void setFootstep(Footstep footstep, double swingTime)
   {
      swingTimeProvider.setValue(swingTime);
      if (CONTROL_TOE)
      {
         computeToeDesiredPose(footstep, newFootstepPose);
      }
      else
      {
         footstep.getPose(newFootstepPose);
      }
      newFootstepPose.changeFrame(worldFrame);

      newFootstepPose.setZ(newFootstepPose.getZ() + finalSwingHeightOffset.getDoubleValue());
      finalConfigurationProvider.setPose(newFootstepPose);
      initialConfigurationProvider.getPosition(oldFootstepPosition);
      orientationTrajectoryGenerator.setFinalOrientation(newFootstepPose);
      orientationTrajectoryGenerator.setFinalVelocityToZero();

      newFootstepPose.changeFrame(worldFrame);
      oldFootstepPosition.changeFrame(worldFrame);

      boolean worldFrameDeltaZAboveThreshold = Math.abs(newFootstepPose.getZ() - oldFootstepPosition.getZ()) > TwoWaypointTrajectoryGeneratorParameters
            .getMinimumHeightDifferenceForStepOnOrOff();

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

   private final RigidBodyTransform toePose = new RigidBodyTransform();
   private final RigidBodyTransform footstepPose = new RigidBodyTransform();

   private void computeToeDesiredPose(Footstep footstep, FramePose desiredToePoseToPack)
   {
      footstep.getPose(footstepPose);
      toePose.multiply(footstepPose, transformFromToeToAnkle);
      desiredToePoseToPack.setPoseIncludingFrame(worldFrame, toePose);
   }

   public void replanTrajectory(Footstep newFootstep, double swingTime, boolean continuousReplan)
   {
      setFootstep(newFootstep, swingTime);
      computeSwingTimeRemaining();

      replanTrajectory.set(true);
      doContinuousReplanning.set(continuousReplan);
   }

   private void computeSwingTimeRemaining()
   {
      if (!currentTimeWithSwingSpeedUp.isNaN())
         this.swingTimeRemaining.set(swingTimeProvider.getValue() - currentTimeWithSwingSpeedUp.getDoubleValue());
      else
         this.swingTimeRemaining.set(swingTimeProvider.getValue() - getTimeInCurrentState());
   }

   /**
    * Request the swing trajectory to speed up using the given speed up factor.
    * It is clamped w.r.t. to {@link WalkingControllerParameters#getMinimumSwingTimeForDisturbanceRecovery()}.
    * @param speedUpFactor
    * @return the current swing time remaining for the swing foot trajectory
    */
   public double requestSwingSpeedUp(double speedUpFactor)
   {
      if (isSwingSpeedUpEnabled.getBooleanValue() && (speedUpFactor > 1.1 && speedUpFactor > swingTimeSpeedUpFactor.getDoubleValue()))
      {
         speedUpFactor = MathTools.clipToMinMax(speedUpFactor, swingTimeSpeedUpFactor.getDoubleValue(), maxSwingTimeSpeedUpFactor.getDoubleValue());

         //         speedUpFactor = MathTools.clipToMinMax(speedUpFactor, 0.7, maxSwingTimeSpeedUpFactor.getDoubleValue());
         //         if (speedUpFactor < 1.0) speedUpFactor = 1.0 - 0.5 * (1.0 - speedUpFactor);

         swingTimeSpeedUpFactor.set(speedUpFactor);
         if (currentTimeWithSwingSpeedUp.isNaN())
            currentTimeWithSwingSpeedUp.set(currentTime.getDoubleValue());
      }

      computeSwingTimeRemaining();
      return swingTimeRemaining.getValue();
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
      swingTimeSpeedUpFactor.set(Double.NaN);
      currentTimeWithSwingSpeedUp.set(Double.NaN);

      if (useNewSwingTrajectoyOptimization)
         swingTrajectoryGeneratorNew.informDone();
      else
         swingTrajectoryGenerator.informDone();

      adjustmentVelocityCorrection.setToZero();
   }
}
