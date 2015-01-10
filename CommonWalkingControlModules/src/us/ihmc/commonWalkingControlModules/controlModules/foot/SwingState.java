package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.ArrayList;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.trajectories.SoftTouchdownPositionTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointPositionTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointTrajectoryGeneratorWithPushRecovery;
import us.ihmc.utilities.humanoidRobot.frames.CommonHumanoidReferenceFrames;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.trajectories.SimpleTwoWaypointTrajectoryParameters;
import us.ihmc.utilities.math.trajectories.TrajectoryWaypointGenerationMethod;
import us.ihmc.utilities.math.trajectories.providers.CurrentConfigurationProvider;
import us.ihmc.utilities.math.trajectories.providers.CurrentLinearVelocityProvider;
import us.ihmc.utilities.math.trajectories.providers.DoubleProvider;
import us.ihmc.utilities.math.trajectories.providers.TrajectoryParameters;
import us.ihmc.utilities.math.trajectories.providers.TrajectoryParametersProvider;
import us.ihmc.utilities.math.trajectories.providers.VectorProvider;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.controllers.YoSE3PIDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoUtilities.humanoidRobot.footstep.Footstep;
import us.ihmc.yoUtilities.math.trajectories.LeadInOutPositionTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.WrapperForMultiplePositionTrajectoryGenerators;
import us.ihmc.yoUtilities.math.trajectories.providers.YoSE3ConfigurationProvider;
import us.ihmc.yoUtilities.math.trajectories.providers.YoVariableDoubleProvider;

public class SwingState extends AbstractUnconstrainedState
{
   private static final boolean USE_NEW_CONTINUOUS_TRAJECTORY = false;
   
   private final FrameVector initialSwingDirection = new FrameVector(worldFrame, 0.0, 0.0, 1.0);
   private final FrameVector finalSwingDirection = new FrameVector(worldFrame, 0.0, 0.0, -1.0);

   private final boolean visualizeSwingTrajectory = true;

   private final BooleanYoVariable replanTrajectory;
   private final YoVariableDoubleProvider swingTimeRemaining;

   private final PositionTrajectoryGenerator positionTrajectoryGenerator, pushRecoveryPositionTrajectoryGenerator;
   private final OrientationInterpolationTrajectoryGenerator orientationTrajectoryGenerator;

   private final CurrentConfigurationProvider initialConfigurationProvider;
   private final YoSE3ConfigurationProvider finalConfigurationProvider;
   private final DoubleProvider swingTimeProvider;
   private final TrajectoryParametersProvider trajectoryParametersProvider = new TrajectoryParametersProvider(new SimpleTwoWaypointTrajectoryParameters());
   
   private final LeadInOutPositionTrajectoryGenerator leadInOutPositionTrajectoryGenerator;
   private final DoubleYoVariable swingClearanceAngle, swingLandingAngle, defaultHeightClearance;

   public SwingState(DoubleProvider swingTimeProvider, VectorProvider touchdownVelocityProvider,
         RigidBodySpatialAccelerationControlModule accelerationControlModule, MomentumBasedController momentumBasedController,
         ContactablePlaneBody contactableBody, int jacobianId, DoubleYoVariable nullspaceMultiplier, BooleanYoVariable jacobianDeterminantInRange,
         BooleanYoVariable doSingularityEscape, LegSingularityAndKneeCollapseAvoidanceControlModule legSingularityAndKneeCollapseAvoidanceControlModule,
         YoSE3PIDGains gains, RobotSide robotSide, YoVariableRegistry registry, WalkingControllerParameters walkingControllerParameters)
   {
      super(ConstraintType.SWING, accelerationControlModule, momentumBasedController, contactableBody, jacobianId, nullspaceMultiplier,
            jacobianDeterminantInRange, doSingularityEscape, legSingularityAndKneeCollapseAvoidanceControlModule, gains, robotSide, registry);

      this.swingTimeProvider = swingTimeProvider;

      String namePrefix = robotSide.getCamelCaseNameForStartOfExpression() + "Foot";

      finalConfigurationProvider = new YoSE3ConfigurationProvider(namePrefix + "SwingFinal", worldFrame, registry);
      replanTrajectory = new BooleanYoVariable(namePrefix + "SwingReplanTrajectory", registry);
      swingTimeRemaining = new YoVariableDoubleProvider(namePrefix + "SwingTimeRemaining", registry);

      ArrayList<PositionTrajectoryGenerator> positionTrajectoryGenerators = new ArrayList<PositionTrajectoryGenerator>();
      ArrayList<PositionTrajectoryGenerator> pushRecoveryPositionTrajectoryGenerators = new ArrayList<PositionTrajectoryGenerator>();

      CommonHumanoidReferenceFrames referenceFrames = momentumBasedController.getReferenceFrames();
      ReferenceFrame footFrame = referenceFrames.getFootFrame(robotSide);
      VectorProvider initialVelocityProvider = new CurrentLinearVelocityProvider(footFrame, momentumBasedController.getFullRobotModel().getFoot(robotSide),
            momentumBasedController.getTwistCalculator());

      initialConfigurationProvider = new CurrentConfigurationProvider(footFrame);

      YoGraphicsListRegistry yoGraphicsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();

      PositionTrajectoryGenerator swingTrajectoryGenerator;

      PositionTrajectoryGenerator touchdownTrajectoryGenerator = new SoftTouchdownPositionTrajectoryGenerator(namePrefix + "Touchdown", worldFrame,
            finalConfigurationProvider, touchdownVelocityProvider, swingTimeProvider, registry);

      if (!USE_NEW_CONTINUOUS_TRAJECTORY)
      {
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
         
         leadInOutPositionTrajectoryGenerator = null;
      }
      else
      {
         leadInOutPositionTrajectoryGenerator = new LeadInOutPositionTrajectoryGenerator(namePrefix + "Swing", worldFrame, registry, visualizeSwingTrajectory,
               yoGraphicsListRegistry);
         swingTrajectoryGenerator = leadInOutPositionTrajectoryGenerator;

         // Needs to be implemented
         pushRecoveryPositionTrajectoryGenerator = null;
      }

      if (USE_NEW_CONTINUOUS_TRAJECTORY)
      {
         swingClearanceAngle = new DoubleYoVariable(namePrefix + "SwingClearanceAngle", registry);
         swingClearanceAngle.set(0.1);
         swingLandingAngle = new DoubleYoVariable(namePrefix + "SwingLandingAngle", registry);
         swingLandingAngle.set(0.8);
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

      orientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator(namePrefix + "Swing", worldFrame, swingTimeProvider,
            initialConfigurationProvider, finalConfigurationProvider, registry);
   }

   @Override
   protected void initializeTrajectory()
   {
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
      orientationTrajectoryGenerator.packAngularData(trajectoryOrientation, desiredAngularVelocity, desiredAngularAcceleration);
   }

   private final FramePose newFootstepPose = new FramePose();
   private final FramePoint oldFootstepPosition = new FramePoint();
   
   private final FramePoint initialSwingPosition = new FramePoint();
   private final FramePoint finalSwingPosition = new FramePoint();
   private final FrameVector swingTranslation = new FrameVector();

   private final Vector3d tempVector = new Vector3d();
   private final Matrix3d rotationMatrix = new Matrix3d();

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
         initialSwingPosition.setToZero(contactableBody.getFrameAfterParentJoint());
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
         
         leadInOutPositionTrajectoryGenerator.setTrajectoryTime(swingTimeProvider.getValue());
         leadInOutPositionTrajectoryGenerator.setInitialLeadOut(initialSwingPosition, initialSwingDirection, defaultHeightClearance.getDoubleValue());
         leadInOutPositionTrajectoryGenerator.setFinalLeadIn(finalSwingPosition, finalSwingDirection, defaultHeightClearance.getDoubleValue());
      }
   }

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
         leadInOutPositionTrajectoryGenerator.showVisualization();
      super.doTransitionIntoAction();
   }

//   @Override
//   public void doTransitionOutOfAction()
//   {
//      if (USE_NEW_CONTINUOUS_TRAJECTORY)
//         twoViaPointTrajectoryGenerator.hideVisualization();
//      super.doTransitionOutOfAction();
//   }
}
