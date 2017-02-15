package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.trajectories.PushRecoveryTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.SoftTouchdownPositionTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointPositionTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointSwingGenerator;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
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
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.VelocityConstrainedOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.WrapperForMultiplePositionTrajectoryGenerators;
import us.ihmc.robotics.math.trajectories.providers.YoSE3ConfigurationProvider;
import us.ihmc.robotics.math.trajectories.providers.YoVariableDoubleProvider;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.trajectories.TrajectoryType;
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
   private final RecyclingArrayList<FramePoint> swingWaypointsForSole = new RecyclingArrayList<>(FramePoint.class);

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
   private final DoubleYoVariable percentOfSwingToStraightenLeg;

   private final VectorProvider currentAngularVelocityProvider;
   private final FrameOrientation initialOrientation = new FrameOrientation();
   private final FrameVector initialAngularVelocity = new FrameVector();

   private final BooleanYoVariable hasInitialAngularConfigurationBeenProvided;

   private final DoubleYoVariable finalSwingHeightOffset;
   private final double controlDT;

   private final DoubleYoVariable minHeightDifferenceForObstacleClearance;

   private final ReferenceFrame soleFrame;
   private final ReferenceFrame controlFrame;

   private final PoseReferenceFrame desiredSoleFrame = new PoseReferenceFrame("desiredSoleFrame", worldFrame);
   private final PoseReferenceFrame desiredControlFrame = new PoseReferenceFrame("desiredControlFrame", desiredSoleFrame);
   private final RigidBodyTransform soleToControlFrameTransform = new RigidBodyTransform();
   private final FramePose desiredPose = new FramePose();
   private final Twist desiredTwist = new Twist();
   private final SpatialAccelerationVector desiredSpatialAcceleration = new SpatialAccelerationVector();

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
      WalkingControllerParameters walkingControllerParameters = footControlHelper.getWalkingControllerParameters();

      finalConfigurationProvider = new YoSE3ConfigurationProvider(namePrefix + "SwingFinal", worldFrame, registry);
      finalSwingHeightOffset = new DoubleYoVariable(namePrefix + "SwingFinalHeightOffset", registry);
      finalSwingHeightOffset.set(footControlHelper.getWalkingControllerParameters().getDesiredTouchdownHeightOffset());
      replanTrajectory = new BooleanYoVariable(namePrefix + "SwingReplanTrajectory", registry);
      swingTimeRemaining = new YoVariableDoubleProvider(namePrefix + "SwingTimeRemaining", registry);

      minHeightDifferenceForObstacleClearance = new DoubleYoVariable(namePrefix + "MinHeightDifferenceForObstacleClearance", registry);
      minHeightDifferenceForObstacleClearance.set(walkingControllerParameters.getMinHeightDifferenceForStepUpOrDown());

      velocityAdjustmentDamping = new DoubleYoVariable(namePrefix + "VelocityAdjustmentDamping", registry);
      velocityAdjustmentDamping.set(footControlHelper.getWalkingControllerParameters().getSwingFootVelocityAdjustmentDamping());
      adjustmentVelocityCorrection = new YoFrameVector(namePrefix + "AdjustmentVelocityCorrection", worldFrame, registry);

      percentOfSwingToStraightenLeg = new DoubleYoVariable(namePrefix + "PercentOfSwingToStraightenLeg", registry);
      percentOfSwingToStraightenLeg.set(footControlHelper.getWalkingControllerParameters().getPercentOfSwingToStraightenLeg());

      // todo make a smarter distinction on this as a way to work with the push recovery module
      doContinuousReplanning = new BooleanYoVariable(namePrefix + "DoContinuousReplanning", registry);

      soleFrame = footControlHelper.getMomentumBasedController().getReferenceFrames().getSoleFrame(robotSide);
      ReferenceFrame footFrame = contactableFoot.getFrameAfterParentJoint();
      ReferenceFrame toeFrame = createToeFrame(robotSide);
      controlFrame = walkingControllerParameters.controlToeDuringSwing() ? toeFrame : footFrame;
      controlFrame.getTransformToDesiredFrame(soleToControlFrameTransform, soleFrame);
      desiredControlFrame.setPoseAndUpdate(soleToControlFrameTransform);

      TwistCalculator twistCalculator = momentumBasedController.getTwistCalculator();
      RigidBody rigidBody = contactableFoot.getRigidBody();

      stanceConfigurationProvider = new CurrentConfigurationProvider(momentumBasedController.getReferenceFrames().getFootFrame(robotSide.getOppositeSide()));
      initialConfigurationProvider = new CurrentConfigurationProvider(soleFrame);
      initialVelocityProvider = new CurrentLinearVelocityProvider(soleFrame, rigidBody, twistCalculator);

      YoGraphicsListRegistry yoGraphicsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();

      PositionTrajectoryGenerator touchdownTrajectoryGenerator = new SoftTouchdownPositionTrajectoryGenerator(namePrefix + "Touchdown", worldFrame,
            finalConfigurationProvider, touchdownVelocityProvider, touchdownAccelerationProvider, swingTimeProvider, registry);

      double maxSwingHeightFromStanceFoot = 0.0;
      double minSwingHeightFromStanceFoot = 0.0;
      if (walkingControllerParameters != null)
      {
         maxSwingHeightFromStanceFoot = walkingControllerParameters.getMaxSwingHeightFromStanceFoot();
         minSwingHeightFromStanceFoot = walkingControllerParameters.getMinSwingHeightFromStanceFoot();
      }

      this.touchdownVelocityProvider = touchdownVelocityProvider;

      ArrayList<PositionTrajectoryGenerator> positionTrajectoryGenerators = new ArrayList<PositionTrajectoryGenerator>();
      ArrayList<PositionTrajectoryGenerator> pushRecoveryPositionTrajectoryGenerators = new ArrayList<PositionTrajectoryGenerator>();

      useNewSwingTrajectoyOptimization = walkingControllerParameters.useSwingTrajectoryOptimizer();
      if (useNewSwingTrajectoyOptimization)
      {
         swingTrajectoryGeneratorNew = new TwoWaypointSwingGenerator(namePrefix + "SwingNew", minSwingHeightFromStanceFoot, maxSwingHeightFromStanceFoot, registry, yoGraphicsListRegistry);
         swingTrajectoryGenerator = null;

         positionTrajectoryGenerators.add(swingTrajectoryGeneratorNew);
         pushRecoveryPositionTrajectoryGenerator = swingTrajectoryGeneratorNew;
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
         TrajectoryType trajectoryType = trajectoryParametersProvider.getTrajectoryParameters().getTrajectoryType();
         double swingHeight = trajectoryParametersProvider.getTrajectoryParameters().getSwingHeight();
         initialConfigurationProvider.getPosition(initialPosition);
         initialVelocityProvider.get(initialVelocity);
         finalConfigurationProvider.getPosition(finalPosition);
         touchdownVelocityProvider.get(finalVelocity);
         stanceConfigurationProvider.getPosition(stanceFootPosition);
         swingTrajectoryGeneratorNew.setInitialConditions(initialPosition, initialVelocity);
         swingTrajectoryGeneratorNew.setFinalConditions(finalPosition, finalVelocity);
         swingTrajectoryGeneratorNew.setStepTime(swingTimeProvider.getValue());
         swingTrajectoryGeneratorNew.setTrajectoryType(trajectoryType, swingWaypointsForSole);
         swingTrajectoryGeneratorNew.setSwingHeight(swingHeight);
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
         TrajectoryType trajectoryType = trajectoryParametersProvider.getTrajectoryParameters().getTrajectoryType();
         finalConfigurationProvider.getPosition(finalPosition);
         touchdownVelocityProvider.get(finalVelocity);
         swingTrajectoryGeneratorNew.setFinalConditions(finalPosition, finalVelocity);
         swingTrajectoryGeneratorNew.setStepTime(swingTimeProvider.getValue());
         swingTrajectoryGeneratorNew.setTrajectoryType(trajectoryType, swingWaypointsForSole);
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
            if (useNewSwingTrajectoyOptimization)
            {
               finalConfigurationProvider.getPosition(finalPosition);
               touchdownVelocityProvider.get(finalVelocity);
               swingTrajectoryGeneratorNew.setFinalConditions(finalPosition, finalVelocity);
            }
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

      updatePrivilegedConfiguration();

      transformDesiredsFromSoleFrameToControlFrame();
   }

   private void transformDesiredsFromSoleFrameToControlFrame()
   {
      desiredSoleFrame.setPoseAndUpdate(desiredPosition, desiredOrientation);

      // change pose
      desiredPose.setToZero(desiredControlFrame);
      desiredPose.changeFrame(worldFrame);
      desiredPose.getPosition(desiredPosition.getPoint());
      desiredPose.getOrientation(desiredOrientation.getQuaternion());

      // change twist
      desiredLinearVelocity.changeFrame(desiredSoleFrame);
      desiredAngularVelocity.changeFrame(desiredSoleFrame);
      desiredTwist.set(desiredSoleFrame, worldFrame, desiredSoleFrame, desiredLinearVelocity, desiredAngularVelocity);
      desiredTwist.changeFrame(desiredControlFrame);
      desiredTwist.getLinearPart(desiredLinearVelocity);
      desiredTwist.getAngularPart(desiredAngularVelocity);
      desiredLinearVelocity.changeFrame(worldFrame);
      desiredAngularVelocity.changeFrame(worldFrame);

      // change spatial acceleration
      desiredLinearAcceleration.changeFrame(desiredSoleFrame);
      desiredAngularAcceleration.changeFrame(desiredSoleFrame);
      desiredSpatialAcceleration.set(desiredSoleFrame, worldFrame, desiredSoleFrame, desiredLinearAcceleration, desiredAngularAcceleration);
      desiredSpatialAcceleration.changeFrameNoRelativeMotion(desiredControlFrame);
      desiredSpatialAcceleration.getLinearPart(desiredLinearAcceleration);
      desiredSpatialAcceleration.getAngularPart(desiredAngularAcceleration);
      desiredLinearAcceleration.changeFrame(worldFrame);
      desiredAngularAcceleration.changeFrame(worldFrame);
   }

   private final FramePose footstepSolePose = new FramePose();
   private final FramePoint oldFootstepPosition = new FramePoint();

   public void setFootstep(Footstep footstep, double swingTime)
   {
      swingTimeProvider.setValue(swingTime);
      footstep.getSolePose(footstepSolePose);

      footstepSolePose.setZ(footstepSolePose.getZ() + finalSwingHeightOffset.getDoubleValue());
      finalConfigurationProvider.setPose(footstepSolePose);
      orientationTrajectoryGenerator.setFinalOrientation(footstepSolePose);
      orientationTrajectoryGenerator.setFinalVelocityToZero();

      // if replanning do not change the original trajectory type
      if (replanTrajectory.getBooleanValue())
         return;

      // if the trajectory is custom trust the waypoints...
      TrajectoryType trajectoryType = footstep.getTrajectoryType();
      this.swingWaypointsForSole.clear();
      if (trajectoryType == TrajectoryType.CUSTOM)
      {
         List<Point3d> swingWaypoints = footstep.getSwingWaypoints();
         for (int i = 0; i < swingWaypoints.size(); i++)
            this.swingWaypointsForSole.add().setIncludingFrame(worldFrame, swingWaypoints.get(i));
         trajectoryParametersProvider.set(new TrajectoryParameters(trajectoryType));
         return;
      }

      // ... otherwise switch the trajectory type to obstacle clearance if the robot steps up or down
      // TODO: using the initialConfigurationProvider is not ideal since a high toe off might trigger obstacle clearance mode
      initialConfigurationProvider.getPosition(oldFootstepPosition);

      footstepSolePose.changeFrame(worldFrame);
      oldFootstepPosition.changeFrame(worldFrame);
      double zDifference = Math.abs(footstepSolePose.getZ() - oldFootstepPosition.getZ());
      boolean stepUpOrDown = zDifference > minHeightDifferenceForObstacleClearance.getDoubleValue();

      if (stepUpOrDown)
      {
         trajectoryParametersProvider.set(new TrajectoryParameters(TrajectoryType.OBSTACLE_CLEARANCE, footstep.getSwingHeight()));
      }
      else
      {
         trajectoryParametersProvider.set(new TrajectoryParameters(trajectoryType, footstep.getSwingHeight()));
      }
   }

   public void replanTrajectory(Footstep newFootstep, double swingTime, boolean continuousReplan)
   {
      replanTrajectory.set(true);
      setFootstep(newFootstep, swingTime);
      computeSwingTimeRemaining();
      doContinuousReplanning.set(continuousReplan);
   }

   private void computeSwingTimeRemaining()
   {
      if (!currentTimeWithSwingSpeedUp.isNaN())
      {
         double swingTimeRemaining = (swingTimeProvider.getValue() - currentTimeWithSwingSpeedUp.getDoubleValue()) / swingTimeSpeedUpFactor.getDoubleValue();
         this.swingTimeRemaining.set(swingTimeRemaining);
      }
      else
      {
         this.swingTimeRemaining.set(swingTimeProvider.getValue() - getTimeInCurrentState());
      }
   }

   private void updatePrivilegedConfiguration()
   {
      if (currentTime.getDoubleValue() > (percentOfSwingToStraightenLeg.getDoubleValue() * swingTimeProvider.getValue()) && attemptToStraightenLegs &&
            !hasSwitchedToStraightLegs.getBooleanValue())
         hasSwitchedToStraightLegs.set(true);
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
