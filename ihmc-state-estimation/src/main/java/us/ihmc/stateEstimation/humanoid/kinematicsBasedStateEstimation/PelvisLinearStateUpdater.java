package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Collectors;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.IntegratorBiasCompensatorYoFrameVector3D;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

/**
 * PelvisLinearUpdater estimates the pelvis position and linear velocity using leg kinematics and
 * IMU acceleration data. If enabled, it also estimates the IMU drift on the estimation of the
 * pelvis yaw angle and velocity.
 * 
 * @author Sylvain
 */
public class PelvisLinearStateUpdater implements SCS2YoGraphicHolder
{
   private static final boolean MORE_YOVARIABLES = false;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final List<RigidBodyBasics> feet = new ArrayList<RigidBodyBasics>();

   private final YoFramePoint3D rootJointPosition = new YoFramePoint3D("estimatedRootJointPosition", worldFrame, registry);
   private final YoFrameVector3D rootJointVelocity = new YoFrameVector3D("estimatedRootJointLinearVelocity", worldFrame, registry);

   private final FixedFrameVector3DBasics rootJointVelocityIMUPart;
   private final FixedFramePoint3DBasics rootJointPositionIMUPart;
   private final FixedFrameVector3DBasics rootJointVelocityKinPart;
   private final FixedFramePoint3DBasics rootJointPositionKinPart;

   private final DoubleProvider imuAgainstKinematicsForVelocityBreakFrequency;
   private final DoubleProvider imuAgainstKinematicsForPositionBreakFrequency;

   private final BooleanProvider useNewFusingFilter;

   private final DoubleProvider linearVelocityFusingKp, linearVelocityFusingKi;
   private final IntegratorBiasCompensatorYoFrameVector3D mainIMULinearVelocityEstimate;
   private final YoFrameVector3D rootJointNewLinearVelocityEstimate;

   private final DoubleProvider positionFusingKp, positionFusingKi;
   private final IntegratorBiasCompensatorYoFrameVector3D rootJointPositionEstimate;

   private final MovingReferenceFrame rootJointFrame;
   private final Map<RigidBodyBasics, ReferenceFrame> footFrames;

   private final double estimatorDT;

   private final YoBoolean requestStopEstimationOfPelvisLinearState = new YoBoolean("userRequestStopEstimationOfPelvisLinearState", registry);

   private final PelvisKinematicsBasedLinearStateCalculator kinematicsBasedLinearStateCalculator;
   private final PelvisIMUBasedLinearStateCalculator imuBasedLinearStateCalculator;

   private final FloatingJointBasics rootJoint;

   private boolean initializeToActual = false;
   private final FramePoint3D initialRootJointPosition = new FramePoint3D(worldFrame);

   // Temporary variables
   private final FramePoint3D footPositionInWorld = new FramePoint3D();

   private final BooleanParameter zeroRootXYPositionAtInitialization = new BooleanParameter("zeroRootXYPositionAtInitialization", registry, false);
   private final BooleanParameter zeroFootHeightAtInitialization = new BooleanParameter("zeroFootHeightAtInitialization", registry, true);

   private final FootContactStateEstimator contactStateEstimator;

   public PelvisLinearStateUpdater(FullInverseDynamicsStructure inverseDynamicsStructure,
                                   List<? extends IMUSensorReadOnly> imuProcessedOutputs,
                                   IMUBiasProvider imuBiasProvider,
                                   BooleanProvider cancelGravityFromAccelerationMeasurement,
                                   Map<RigidBodyBasics, FootSwitchInterface> footSwitches,
                                   CenterOfPressureDataHolder centerOfPressureDataHolderFromController,
                                   Map<RigidBodyBasics, ? extends ContactablePlaneBody> feetContactablePlaneBodies,
                                   double gravitationalAcceleration,
                                   StateEstimatorParameters stateEstimatorParameters,
                                   YoGraphicsListRegistry yoGraphicsListRegistry,
                                   YoRegistry parentRegistry)
   {
      this.estimatorDT = stateEstimatorParameters.getEstimatorDT();
      this.feet.addAll(footSwitches.keySet());

      rootJoint = inverseDynamicsStructure.getRootJoint();
      rootJointFrame = rootJoint.getFrameAfterJoint();

      contactStateEstimator = new DRCFootContactStateEstimator(footSwitches, stateEstimatorParameters, registry);

      footFrames = feet.stream().collect(Collectors.toMap(Function.identity(), foot -> feetContactablePlaneBodies.get(foot).getSoleFrame()));

      kinematicsBasedLinearStateCalculator = new PelvisKinematicsBasedLinearStateCalculator(inverseDynamicsStructure,
                                                                                            feetContactablePlaneBodies,
                                                                                            footSwitches,
                                                                                            centerOfPressureDataHolderFromController,
                                                                                            estimatorDT,
                                                                                            stateEstimatorParameters,
                                                                                            yoGraphicsListRegistry,
                                                                                            registry);

      imuBasedLinearStateCalculator = new PelvisIMUBasedLinearStateCalculator(rootJoint,
                                                                              imuProcessedOutputs,
                                                                              imuBiasProvider,
                                                                              cancelGravityFromAccelerationMeasurement,
                                                                              estimatorDT,
                                                                              gravitationalAcceleration,
                                                                              stateEstimatorParameters,
                                                                              yoGraphicsListRegistry,
                                                                              registry);

      imuAgainstKinematicsForVelocityBreakFrequency = new DoubleParameter("imuAgainstKinematicsForVelocityBreakFrequency",
                                                                          registry,
                                                                          stateEstimatorParameters.getPelvisLinearVelocityFusingFrequency());
      imuAgainstKinematicsForPositionBreakFrequency = new DoubleParameter("imuAgainstKinematicsForPositionBreakFrequency",
                                                                          registry,
                                                                          stateEstimatorParameters.getPelvisPositionFusingFrequency());
      useNewFusingFilter = new BooleanParameter("usePelvisLinearStateNewFusingFilter",
                                                registry,
                                                stateEstimatorParameters.usePelvisLinearStateNewFusingFilter());

      linearVelocityFusingKp = new DoubleParameter("pelvisLinearStateLinearVelocityFusingKp",
                                                   registry,
                                                   stateEstimatorParameters.getPelvisLinearVelocityNewFusingFilterKp());
      linearVelocityFusingKi = new DoubleParameter("pelvisLinearStateLinearVelocityFusingKi",
                                                   registry,
                                                   stateEstimatorParameters.getPelvisLinearVelocityNewFusingFilterKi());
      positionFusingKp = new DoubleParameter("pelvisLinearStatePositionFusingKp", registry, stateEstimatorParameters.getPelvisPositionNewFusingFilterKp());
      positionFusingKi = new DoubleParameter("pelvisLinearStatePositionFusingKi", registry, stateEstimatorParameters.getPelvisPositionNewFusingFilterKi());
      mainIMULinearVelocityEstimate = new IntegratorBiasCompensatorYoFrameVector3D("newMainIMULinearVelocityEstimate",
                                                                                   registry,
                                                                                   linearVelocityFusingKp,
                                                                                   linearVelocityFusingKi,
                                                                                   imuBasedLinearStateCalculator.getIMUMeasurementFrame(),
                                                                                   estimatorDT);
      rootJointNewLinearVelocityEstimate = new YoFrameVector3D("newEstimatedRootJointLinearVelocity", worldFrame, registry);

      rootJointPositionEstimate = new IntegratorBiasCompensatorYoFrameVector3D("newRootJointPositionEstimate",
                                                                               registry,
                                                                               positionFusingKp,
                                                                               positionFusingKi,
                                                                               worldFrame,
                                                                               rootJointFrame, // Keep the bias in the local frame instead of world.
                                                                               estimatorDT);

      if (MORE_YOVARIABLES)
      {
         rootJointVelocityIMUPart = new YoFrameVector3D("estimatedRootJointLinearVelocity_IMUPart", worldFrame, registry);
         rootJointPositionIMUPart = new YoFramePoint3D("estimatedRootJointLinearPosition_IMUPart", worldFrame, registry);
         rootJointVelocityKinPart = new YoFrameVector3D("estimatedRootJointLinearVelocity_KinPart", worldFrame, registry);
         rootJointPositionKinPart = new YoFramePoint3D("estimatedRootJointLinearPosition_KinPart", worldFrame, registry);
      }
      else
      {
         rootJointVelocityIMUPart = new FrameVector3D();
         rootJointPositionIMUPart = new FramePoint3D();
         rootJointVelocityKinPart = new FrameVector3D();
         rootJointPositionKinPart = new FramePoint3D();
      }

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      initializeRobotState();
      updateRootJoint();
   }

   private void initializeRobotState()
   {
      if (!initializeToActual)
      {
         rootJointPosition.set(worldFrame, rootJoint.getJointPose().getPosition());

         if (zeroRootXYPositionAtInitialization.getValue())
         {
            rootJointPosition.setX(0.0);
            rootJointPosition.setY(0.0);
         }

         if (zeroFootHeightAtInitialization.getValue())
         {
            RigidBodyBasics foot = feet.get(0);
            // We're interested in the delta-z between the foot and the root joint frame.
            footPositionInWorld.setToZero(footFrames.get(foot));
            footPositionInWorld.changeFrame(rootJointFrame);

            // By setting the root joint to be at -footZ, the foot will be at a height of zero.
            rootJointPosition.setZ(-footPositionInWorld.getZ());
         }
      }
      else
      {
         rootJointPosition.set(initialRootJointPosition);
      }

      rootJointVelocity.setToZero();
      kinematicsBasedLinearStateCalculator.initialize(rootJointPosition);

      imuBasedLinearStateCalculator.initialize();
      mainIMULinearVelocityEstimate.getPositionEstimation().setToZero();
      mainIMULinearVelocityEstimate.getRateEstimation().setToZero();
      rootJointPositionEstimate.getPositionEstimation().set(rootJointPosition);
      rootJointPositionEstimate.getRateEstimation().setToZero();
   }

   public void initializeRootJointPosition(Tuple3DReadOnly rootJointPosition)
   {
      initializeToActual = true;
      initialRootJointPosition.setIncludingFrame(worldFrame, rootJointPosition);
   }

   public void updateForFrozenState()
   {
      // Keep setting the position so the localization updater works properly.
      rootJoint.setJointPosition(rootJointPosition);
      kinematicsBasedLinearStateCalculator.updateKinematics();
      kinematicsBasedLinearStateCalculator.updateNoTrustedFeet(rootJointPosition, null);
      // Reset the IMU updater
      imuBasedLinearStateCalculator.initialize();

      // Set the rootJoint twist to zero.
      rootJoint.getJointTwist().setToZero();
      rootJoint.updateFramesRecursively();
      rootJointVelocity.setToZero();
   }

   public void updateRootJointPositionAndLinearVelocity()
   {
      if (requestStopEstimationOfPelvisLinearState.getBooleanValue())
         return;

      contactStateEstimator.update(); // TODO Not sure if that should be here

      kinematicsBasedLinearStateCalculator.updateKinematics();

      if (imuBasedLinearStateCalculator.isEstimationEnabled())
         imuBasedLinearStateCalculator.updateLinearAccelerationMeasurement();

      if (contactStateEstimator.getNumberOfEndEffectorsTrusted() == 0)
      {
         FixedFrameVector3DBasics estimatedRootJointAngularVelocity = rootJoint.getJointTwist().getAngularPart();
         imuBasedLinearStateCalculator.estimateRootJointLinearVelocity(estimatedRootJointAngularVelocity, rootJointVelocity);
         imuBasedLinearStateCalculator.estimateRootJointPosition(rootJointPosition, estimatedRootJointAngularVelocity, rootJointPosition);

         mainIMULinearVelocityEstimate.update(null, imuBasedLinearStateCalculator.getLinearAccelerationMeasurement());
         tempTwist.setToZero(rootJointFrame, rootJointFrame.getRootFrame(), imuBasedLinearStateCalculator.getIMUMeasurementFrame());
         tempTwist.getAngularPart().setMatchingFrame(estimatedRootJointAngularVelocity);
         tempTwist.getLinearPart().set(mainIMULinearVelocityEstimate);
         tempTwist.changeFrame(rootJointFrame);
         rootJointNewLinearVelocityEstimate.setMatchingFrame(tempTwist.getLinearPart());
         rootJointPositionEstimate.update(null, rootJointNewLinearVelocityEstimate);
         kinematicsBasedLinearStateCalculator.updateNoTrustedFeet(rootJointPosition, rootJointVelocity);
      }
      else if (contactStateEstimator.getNumberOfEndEffectorsTrusted() > 0)
      {
         kinematicsBasedLinearStateCalculator.estimatePelvisLinearState(contactStateEstimator.getListOfTrustedFeet(),
                                                                        contactStateEstimator.getListOfUnTrustedFeet(),
                                                                        rootJointPosition);

         if (imuBasedLinearStateCalculator.isEstimationEnabled())
         {
            computeLinearStateFromMergingMeasurements();
         }
         else
         {
            rootJointPosition.set(kinematicsBasedLinearStateCalculator.getPelvisPosition());
            rootJointVelocity.set(kinematicsBasedLinearStateCalculator.getPelvisVelocity());
         }
      }
      else
      {
         throw new RuntimeException("Computation of the number of end effectors to be trusted for state estimation is broken, computed: "
                                    + contactStateEstimator.getNumberOfEndEffectorsTrusted());
      }

      updateRootJoint();
   }

   private void updateRootJoint()
   {
      rootJoint.getJointPose().getPosition().set(rootJointPosition);
      rootJoint.getJointTwist().getLinearPart().setMatchingFrame(rootJointVelocity);
      rootJoint.updateFrame();
      imuBasedLinearStateCalculator.saveMeasurementFrameTwist(rootJoint.getJointTwist());
   }

   private void computeLinearStateFromMergingMeasurements()
   {
      computeLinearVelocityFromMergingMeasurements();
      computePositionFromMergingMeasurements();
   }

   private final Twist tempTwist = new Twist();

   private void computeLinearVelocityFromMergingMeasurements()
   {
      FixedFrameVector3DBasics estimatedRootJointAngularVelocity = rootJoint.getJointTwist().getAngularPart();

      if (!useNewFusingFilter.getValue())
      {
         imuBasedLinearStateCalculator.estimateRootJointLinearVelocity(estimatedRootJointAngularVelocity, rootJointVelocityIMUPart);
         rootJointVelocityKinPart.setMatchingFrame(kinematicsBasedLinearStateCalculator.getPelvisVelocity());

         double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(imuAgainstKinematicsForVelocityBreakFrequency.getValue(), estimatorDT);
         rootJointVelocity.interpolate(rootJointVelocityKinPart, rootJointVelocityIMUPart, alpha);
      }

      tempTwist.setToZero(rootJointFrame, rootJointFrame.getRootFrame(), rootJointFrame);
      tempTwist.getAngularPart().setMatchingFrame(estimatedRootJointAngularVelocity);
      tempTwist.getLinearPart().setMatchingFrame(kinematicsBasedLinearStateCalculator.getPelvisVelocity());
      tempTwist.changeFrame(imuBasedLinearStateCalculator.getIMUMeasurementFrame());

      mainIMULinearVelocityEstimate.update(tempTwist.getLinearPart(), imuBasedLinearStateCalculator.getLinearAccelerationMeasurement());
      tempTwist.getLinearPart().set(mainIMULinearVelocityEstimate);
      tempTwist.changeFrame(rootJointFrame);
      rootJointNewLinearVelocityEstimate.setMatchingFrame(tempTwist.getLinearPart());

      if (useNewFusingFilter.getValue())
      {
         rootJointVelocity.set(rootJointNewLinearVelocityEstimate);
      }
   }

   private void computePositionFromMergingMeasurements()
   {
      if (!useNewFusingFilter.getValue())
      {
         imuBasedLinearStateCalculator.estimateRootJointPosition(rootJointPosition, rootJoint.getJointTwist().getAngularPart(), rootJointPositionIMUPart);
         rootJointPositionKinPart.setMatchingFrame(kinematicsBasedLinearStateCalculator.getPelvisPosition());

         double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(imuAgainstKinematicsForPositionBreakFrequency.getValue(), estimatorDT);
         rootJointPosition.interpolate(rootJointPositionKinPart, rootJointPositionIMUPart, alpha);
      }

      rootJointPositionEstimate.update(kinematicsBasedLinearStateCalculator.getPelvisPosition(), rootJointNewLinearVelocityEstimate);

      if (useNewFusingFilter.getValue())
      {
         rootJointPosition.set(rootJointPositionEstimate);
      }
   }

   public void getEstimatedPelvisPosition(FramePoint3D pelvisPositionToPack)
   {
      pelvisPositionToPack.setIncludingFrame(rootJointPosition);
   }

   public void getEstimatedPelvisLinearVelocity(FrameVector3D pelvisLinearVelocityToPack)
   {
      pelvisLinearVelocityToPack.setIncludingFrame(rootJointVelocity);
   }

   public List<RigidBodyBasics> getCurrentListOfTrustedFeet()
   {
      return contactStateEstimator.getListOfTrustedFeet();
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(kinematicsBasedLinearStateCalculator.getSCS2YoGraphics());
      group.addChild(imuBasedLinearStateCalculator.getSCS2YoGraphics());
      return group;
   }
}
