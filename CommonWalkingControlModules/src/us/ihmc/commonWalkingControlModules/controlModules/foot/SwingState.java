package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.ArrayList;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.trajectories.SoftTouchdownPositionTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointPositionTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointTrajectoryGeneratorWithPushRecovery;
import us.ihmc.utilities.humanoidRobot.frames.CommonHumanoidReferenceFrames;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.trajectories.SimpleTwoWaypointTrajectoryParameters;
import us.ihmc.utilities.math.trajectories.TrajectoryWaypointGenerationMethod;
import us.ihmc.utilities.math.trajectories.providers.ConstantVectorProvider;
import us.ihmc.utilities.math.trajectories.providers.CurrentAngularVelocityProvider;
import us.ihmc.utilities.math.trajectories.providers.CurrentConfigurationProvider;
import us.ihmc.utilities.math.trajectories.providers.CurrentLinearVelocityProvider;
import us.ihmc.utilities.math.trajectories.providers.DoubleProvider;
import us.ihmc.utilities.math.trajectories.providers.TrajectoryParameters;
import us.ihmc.utilities.math.trajectories.providers.TrajectoryParametersProvider;
import us.ihmc.utilities.math.trajectories.providers.VectorProvider;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.yoUtilities.controllers.YoSE3PIDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.humanoidRobot.footstep.Footstep;
import us.ihmc.yoUtilities.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.TwoViaPointTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.VelocityConstrainedOrientationTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.WrapperForMultiplePositionTrajectoryGenerators;
import us.ihmc.yoUtilities.math.trajectories.providers.YoSE3ConfigurationProvider;
import us.ihmc.yoUtilities.math.trajectories.providers.YoVariableDoubleProvider;

public class SwingState extends AbstractUnconstrainedState implements SwingStateInterface
{
   private static final boolean USE_NEW_CONTINUOUS_TRAJECTORY = false;
   
   private final FrameVector initialSwingDirection = new FrameVector(worldFrame, 0.0, 0.0, 1.0);
   private final FrameVector finalSwingDirection = new FrameVector(worldFrame, 0.0, 0.0, -1.0);

   private final boolean visualizeSwingTrajectory = true;

   private final BooleanYoVariable replanTrajectory;
   private final YoVariableDoubleProvider swingTimeRemaining;

   private final PositionTrajectoryGenerator positionTrajectoryGenerator, pushRecoveryPositionTrajectoryGenerator;
   private final VelocityConstrainedOrientationTrajectoryGenerator orientationTrajectoryGenerator;

   private final CurrentConfigurationProvider initialConfigurationProvider;
   private final YoSE3ConfigurationProvider finalConfigurationProvider;
   private final DoubleProvider swingTimeProvider;
   private final TrajectoryParametersProvider trajectoryParametersProvider = new TrajectoryParametersProvider(new SimpleTwoWaypointTrajectoryParameters());
   
   private final TwoViaPointTrajectoryGenerator continuousTrajectory;
   private final DoubleYoVariable swingClearanceAngle, swingLandingAngle, defaultHeightClearance;
   
   private final VectorProvider currentAngularVelocityProvider;
   private final FrameVector initialAngularVelocity = new FrameVector();
   private boolean hasInitialAngularVelocityBeenProvided = false;

   public SwingState(FootControlHelper footControlHelper, DoubleProvider swingTimeProvider, VectorProvider touchdownVelocityProvider, YoSE3PIDGains gains, YoVariableRegistry registry)
   {
      super(ConstraintType.SWING, footControlHelper, gains, registry);

      this.swingTimeProvider = swingTimeProvider;

      String namePrefix = robotSide.getCamelCaseNameForStartOfExpression() + "Foot";

      finalConfigurationProvider = new YoSE3ConfigurationProvider(namePrefix + "SwingFinal", worldFrame, registry);
      replanTrajectory = new BooleanYoVariable(namePrefix + "SwingReplanTrajectory", registry);
      swingTimeRemaining = new YoVariableDoubleProvider(namePrefix + "SwingTimeRemaining", registry);

      ArrayList<PositionTrajectoryGenerator> positionTrajectoryGenerators = new ArrayList<PositionTrajectoryGenerator>();
      ArrayList<PositionTrajectoryGenerator> pushRecoveryPositionTrajectoryGenerators = new ArrayList<PositionTrajectoryGenerator>();

      CommonHumanoidReferenceFrames referenceFrames = momentumBasedController.getReferenceFrames();
      ReferenceFrame footFrame = referenceFrames.getFootFrame(robotSide);
      TwistCalculator twistCalculator = momentumBasedController.getTwistCalculator();
      RigidBody rigidBody = contactableBody.getRigidBody();
      VectorProvider initialVelocityProvider = new CurrentLinearVelocityProvider(footFrame, rigidBody, twistCalculator);

      initialConfigurationProvider = new CurrentConfigurationProvider(footFrame);

      YoGraphicsListRegistry yoGraphicsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();

      PositionTrajectoryGenerator swingTrajectoryGenerator;

      PositionTrajectoryGenerator touchdownTrajectoryGenerator = new SoftTouchdownPositionTrajectoryGenerator(namePrefix + "Touchdown", worldFrame,
            finalConfigurationProvider, touchdownVelocityProvider, swingTimeProvider, registry);

      if (!USE_NEW_CONTINUOUS_TRAJECTORY)
      {
         WalkingControllerParameters walkingControllerParameters = footControlHelper.getWalkingControllerParameters();
         swingTrajectoryGenerator = new TwoWaypointPositionTrajectoryGenerator(namePrefix + "Swing", worldFrame, swingTimeProvider,
               initialConfigurationProvider, initialVelocityProvider, finalConfigurationProvider, touchdownVelocityProvider, trajectoryParametersProvider,
               registry, yoGraphicsListRegistry, walkingControllerParameters, visualizeSwingTrajectory);

         PositionTrajectoryGenerator pushRecoverySwingTrajectoryGenerator = new TwoWaypointTrajectoryGeneratorWithPushRecovery(
               namePrefix + "SwingPushRecovery", worldFrame, swingTimeProvider, swingTimeRemaining, initialConfigurationProvider, initialVelocityProvider,
               finalConfigurationProvider, touchdownVelocityProvider, trajectoryParametersProvider, registry, yoGraphicsListRegistry, swingTrajectoryGenerator,
               walkingControllerParameters, visualizeSwingTrajectory);

         pushRecoveryPositionTrajectoryGenerators.add(pushRecoverySwingTrajectoryGenerator);
         pushRecoveryPositionTrajectoryGenerators.add(touchdownTrajectoryGenerator);

         pushRecoveryPositionTrajectoryGenerator = new WrapperForMultiplePositionTrajectoryGenerators(pushRecoveryPositionTrajectoryGenerators, namePrefix
               + "PushRecoveryTrajectoryGenerator", registry);
         
         continuousTrajectory = null;
      }
      else
      {
         continuousTrajectory = new TwoViaPointTrajectoryGenerator(namePrefix + "Swing", worldFrame, registry, visualizeSwingTrajectory,
               yoGraphicsListRegistry);
         swingTrajectoryGenerator = continuousTrajectory;

         // Needs to be implemented
         pushRecoveryPositionTrajectoryGenerator = null;
      }

      if (USE_NEW_CONTINUOUS_TRAJECTORY)
      {
         swingClearanceAngle = new DoubleYoVariable(namePrefix + "SwingClearanceAngle", registry);
         swingClearanceAngle.set(3.0*0.1);
         swingLandingAngle = new DoubleYoVariable(namePrefix + "SwingLandingAngle", registry);
         swingLandingAngle.set(1.0);
         defaultHeightClearance = new DoubleYoVariable(namePrefix + "DefaultHeightClearance", registry);
         defaultHeightClearance.set(0.04);
      }
      else
      {
         swingClearanceAngle = null;
         swingLandingAngle = null;
         defaultHeightClearance = null;
      }

      positionTrajectoryGenerators.add(swingTrajectoryGenerator);
      positionTrajectoryGenerators.add(touchdownTrajectoryGenerator);

      positionTrajectoryGenerator = new WrapperForMultiplePositionTrajectoryGenerators(positionTrajectoryGenerators, namePrefix, registry);

      currentAngularVelocityProvider = new CurrentAngularVelocityProvider(footFrame, rigidBody, twistCalculator);
      VectorProvider initialAngularVelocityProvider = new ConstantVectorProvider(initialAngularVelocity);
      VectorProvider finalAngularVelocityProvider = new ConstantVectorProvider(new FrameVector(footFrame));
      orientationTrajectoryGenerator = new VelocityConstrainedOrientationTrajectoryGenerator(namePrefix + "Swing", worldFrame, swingTimeProvider,
            initialConfigurationProvider, initialAngularVelocityProvider, finalConfigurationProvider, finalAngularVelocityProvider, registry);
   }

   @Override
   public void setInitialDesireds(FrameOrientation initialOrientation, FrameVector initialAngularVelocity)
   {
      hasInitialAngularVelocityBeenProvided = true;
      this.initialAngularVelocity.setIncludingFrame(initialAngularVelocity);
   }

   @Override
   protected void initializeTrajectory()
   {
      if (!hasInitialAngularVelocityBeenProvided)
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
      orientationTrajectoryGenerator.packAngularData(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
   }

   private final FramePose newFootstepPose = new FramePose();
   private final FramePoint oldFootstepPosition = new FramePoint();
   
   private final FramePoint initialSwingPosition = new FramePoint();
   private final FramePoint finalSwingPosition = new FramePoint();
   private final FrameVector swingTranslation = new FrameVector();

   private final Vector3d tempVector = new Vector3d();
   private final Matrix3d rotationMatrix = new Matrix3d();

   @Override
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
      
      if (USE_NEW_CONTINUOUS_TRAJECTORY)
      {
         footstep.getPositionIncludingFrame(finalSwingPosition);
         finalSwingPosition.changeFrame(worldFrame);
         initialConfigurationProvider.get(initialSwingPosition);
         initialSwingPosition.changeFrame(worldFrame);
         
         swingTranslation.sub(finalSwingPosition, initialSwingPosition);
         double stepLength = swingTranslation.length();
         
         swingTranslation.normalize();
         tempVector.set(0.0, 0.0, -1.0);
         swingTranslation.cross(tempVector);

         double angle = swingClearanceAngle.getDoubleValue() * stepLength;
         AxisAngle4d axisAngle4d = new AxisAngle4d(swingTranslation.getX(), swingTranslation.getY(), swingTranslation.getZ(), angle);
         rotationMatrix.set(axisAngle4d);
         
         initialSwingDirection.setIncludingFrame(worldFrame, 0.0, 0.0, 1.0);
         rotationMatrix.transform(initialSwingDirection.getVector());
         
         angle = swingLandingAngle.getDoubleValue() * stepLength;
         axisAngle4d.set(swingTranslation.getX(), swingTranslation.getY(), swingTranslation.getZ(), -angle);
         rotationMatrix.set(axisAngle4d);
         finalSwingDirection.setIncludingFrame(worldFrame, 0.0, 0.0, -1.0);
         rotationMatrix.transform(finalSwingDirection.getVector());
         
         continuousTrajectory.setTrajectoryTime(swingTimeProvider.getValue());
         continuousTrajectory.setInitialLeadOut(initialSwingPosition, initialSwingDirection, defaultHeightClearance.getDoubleValue());
         continuousTrajectory.setFinalVelocity(0.3);
         continuousTrajectory.setFinalLeadIn(finalSwingPosition, finalSwingDirection, defaultHeightClearance.getDoubleValue());
      }
   }

   @Override
   public void replanTrajectory(Footstep footstep, double swingTimeRemaining, boolean useLowHeightTrajectory)
   {
      setFootstep(footstep, trajectoryParametersProvider.getTrajectoryParameters(), useLowHeightTrajectory);
      this.swingTimeRemaining.set(swingTimeRemaining);
      this.replanTrajectory.set(true);
   }

   @Override
   public void doTransitionIntoAction()
   {
      if (USE_NEW_CONTINUOUS_TRAJECTORY)
         continuousTrajectory.showVisualization();
      super.doTransitionIntoAction();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      hasInitialAngularVelocityBeenProvided = false;
//      if (USE_NEW_CONTINUOUS_TRAJECTORY)
//         continuousTrajectory.hideVisualization();
      super.doTransitionOutOfAction();
   }
}
