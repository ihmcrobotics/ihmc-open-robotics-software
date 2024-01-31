package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commons.MathTools;
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
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.math.filters.GlitchFilteredYoInteger;
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
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

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

   private static final double minForceZInPercentThresholdToFilterFoot = 0.0;
   private static final double maxForceZInPercentThresholdToFilterFoot = 0.45;

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

   private final YoInteger numberOfEndEffectorsTrusted = new YoInteger("numberOfEndEffectorsTrusted", registry);
   private final YoInteger numberOfEndEffectorsFilteredByLoad = new YoInteger("numberOfEndEffectorsFilteredByLoad", registry);

   private final Map<RigidBodyBasics, YoDouble> footForcesZInPercentOfTotalForce = new LinkedHashMap<>();
   private final IntegerProvider optimalNumberOfTrustedFeet;
   private final DoubleProvider forceZInPercentThresholdToTrustFoot;
   private final DoubleProvider forceZInPercentThresholdToNotTrustFoot;

   private final Map<RigidBodyBasics, FootSwitchInterface> footSwitches;
   private final Map<RigidBodyBasics, FixedFrameVector3DBasics> footForces = new LinkedHashMap<>();
   private final Map<RigidBodyBasics, Wrench> footWrenches = new LinkedHashMap<>();
   private final DoubleProvider delayTimeBeforeTrustingFoot;
   private final Map<RigidBodyBasics, GlitchFilteredYoBoolean> haveFeetHitGroundFiltered = new LinkedHashMap<>();
   private final Map<RigidBodyBasics, YoBoolean> areFeetTrusted = new LinkedHashMap<>();
   private final Map<RigidBodyBasics, YoBoolean> wereFeetTrustedLastTick = new LinkedHashMap<>();
   private final List<RigidBodyBasics> listOfTrustedFeet = new ArrayList<>();
   private final List<RigidBodyBasics> listOfUnTrustedFeet = new ArrayList<>();

   private final MovingReferenceFrame rootJointFrame;
   private final Map<RigidBodyBasics, ReferenceFrame> footFrames;

   private final double estimatorDT;

   private final Map<RigidBodyBasics, ? extends ContactablePlaneBody> feetContactablePlaneBodies;

   private final BooleanProvider trustImuWhenNoFeetAreInContact;

   private enum SlippageCompensatorMode
   {
      LOAD_THRESHOLD, MIN_PELVIS_ACCEL
   };

   private final YoEnum<SlippageCompensatorMode> slippageCompensatorMode = new YoEnum<SlippageCompensatorMode>("slippageCompensatorMode",
                                                                                                               registry,
                                                                                                               SlippageCompensatorMode.class);

   private final YoBoolean requestStopEstimationOfPelvisLinearState = new YoBoolean("userRequestStopEstimationOfPelvisLinearState", registry);

   private final PelvisKinematicsBasedLinearStateCalculator kinematicsBasedLinearStateCalculator;
   private final PelvisIMUBasedLinearStateCalculator imuBasedLinearStateCalculator;

   private final FloatingJointBasics rootJoint;

   private boolean initializeToActual = false;
   private final FramePoint3D initialRootJointPosition = new FramePoint3D(worldFrame);

   // Temporary variables
   private final FramePoint3D footPositionInWorld = new FramePoint3D();

   private final BooleanProvider trustOnlyLowestFoot = new BooleanParameter("TrustOnlyLowestFoot", registry, false);
   private final IntegerProvider lowestFootWindowSize = new IntegerParameter("LowestFootWindowSize", registry, 0);
   private final GlitchFilteredYoInteger lowestFootInContactIndex = new GlitchFilteredYoInteger("LowestFootInContact", lowestFootWindowSize, registry);

   private final BooleanParameter zeroRootXYPositionAtInitialization = new BooleanParameter("zeroRootXYPositionAtInitialization", registry, true);
   private final BooleanParameter zeroFootHeightAtInitialization = new BooleanParameter("zeroFootHeightAtInitialization", registry, true);

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
      this.footSwitches = footSwitches;
      this.feetContactablePlaneBodies = feetContactablePlaneBodies;
      this.feet.addAll(footSwitches.keySet());

      rootJoint = inverseDynamicsStructure.getRootJoint();
      rootJointFrame = rootJoint.getFrameAfterJoint();

      footFrames = new LinkedHashMap<RigidBodyBasics, ReferenceFrame>();

      setupBunchOfVariables();

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

      delayTimeBeforeTrustingFoot = new DoubleParameter("delayTimeBeforeTrustingFoot", registry, stateEstimatorParameters.getDelayTimeForTrustingFoot());
      optimalNumberOfTrustedFeet = new IntegerParameter("optimalNumberOfTrustedFeet", registry, 2);
      forceZInPercentThresholdToTrustFoot = new DoubleParameter("forceZInPercentThresholdToTrustFoot",
                                                                registry,
                                                                stateEstimatorParameters.getForceInPercentOfWeightThresholdToTrustFoot());
      forceZInPercentThresholdToNotTrustFoot = new DoubleParameter("forceZInPercentThresholdToNotTrustFoot",
                                                                   registry,
                                                                   stateEstimatorParameters.getForceInPercentOfWeightThresholdToNotTrustFoot());
      trustImuWhenNoFeetAreInContact = new BooleanParameter("trustImuWhenNoFeetAreInContact",
                                                            registry,
                                                            stateEstimatorParameters.getPelvisLinearStateUpdaterTrustImuWhenNoFeetAreInContact());

      slippageCompensatorMode.set(SlippageCompensatorMode.LOAD_THRESHOLD);

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

   private void setupBunchOfVariables()
   {
      for (int i = 0; i < feet.size(); i++)
      {
         RigidBodyBasics foot = feet.get(i);
         ReferenceFrame footFrame = feetContactablePlaneBodies.get(foot).getSoleFrame();
         footFrames.put(foot, footFrame);

         String footPrefix = foot.getName();

         final GlitchFilteredYoBoolean hasFootHitTheGroundFiltered = new GlitchFilteredYoBoolean("has" + footPrefix + "FootHitGroundFiltered", registry, 0);
         hasFootHitTheGroundFiltered.set(true);
         haveFeetHitGroundFiltered.put(foot, hasFootHitTheGroundFiltered);

         YoBoolean isFootTrusted = new YoBoolean("is" + footPrefix + "FootTrusted", registry);
         YoBoolean wasFootTrusted = new YoBoolean("was" + footPrefix + "FootTrustedLastTick", registry);
         if (i == 0)
         {
            isFootTrusted.set(true);
            wasFootTrusted.set(true);
         }
         areFeetTrusted.put(foot, isFootTrusted);
         wereFeetTrustedLastTick.put(foot, wasFootTrusted);

         YoDouble footForceZInPercentOfTotalForce = new YoDouble(footPrefix + "FootForceZInPercentOfTotalForce", registry);
         footForcesZInPercentOfTotalForce.put(foot, footForceZInPercentOfTotalForce);

         footForces.put(foot, new FrameVector3D(worldFrame));
         footWrenches.put(foot, new Wrench());
      }
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
            System.out.println(-footPositionInWorld.getZ());
            System.out.println(rootJointPosition.getZ());
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

      kinematicsBasedLinearStateCalculator.updateKinematics();

      numberOfEndEffectorsTrusted.set(setTrustedFeetUsingFootSwitches());
      numberOfEndEffectorsFilteredByLoad.set(0);

      if (numberOfEndEffectorsTrusted.getIntegerValue() >= optimalNumberOfTrustedFeet.getValue())
      {
         switch (slippageCompensatorMode.getEnumValue())
         {
            case LOAD_THRESHOLD:
               int filteredNumberOfEndEffectorsTrusted = filterTrustedFeetBasedOnContactForces(numberOfEndEffectorsTrusted.getIntegerValue());
               numberOfEndEffectorsTrusted.set(filteredNumberOfEndEffectorsTrusted);
               break;
            case MIN_PELVIS_ACCEL:
               throw new RuntimeException("Implement me if possible!");
            default:
               throw new RuntimeException("Should not get there");
         }
      }

      if (imuBasedLinearStateCalculator.isEstimationEnabled())
         imuBasedLinearStateCalculator.updateLinearAccelerationMeasurement();

      if (numberOfEndEffectorsTrusted.getIntegerValue() == 0)
      {
         if (trustImuWhenNoFeetAreInContact.getValue())
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
         else
            throw new RuntimeException("No foot trusted!");
      }
      else if (numberOfEndEffectorsTrusted.getIntegerValue() > 0)
      {
         updateTrustedFeetLists();
         kinematicsBasedLinearStateCalculator.estimatePelvisLinearState(listOfTrustedFeet, listOfUnTrustedFeet, rootJointPosition);

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
               + numberOfEndEffectorsTrusted);
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

   private int setTrustedFeetUsingFootSwitches()
   {
      int numberOfEndEffectorsTrusted = 0;

      int windowSize = (int) (delayTimeBeforeTrustingFoot.getValue() / estimatorDT);

      for (int i = 0; i < feet.size(); i++)
      {
         RigidBodyBasics foot = feet.get(i);
         wereFeetTrustedLastTick.get(foot).set(areFeetTrusted.get(foot).getValue());
         haveFeetHitGroundFiltered.get(foot).setWindowSize(windowSize);

         if (footSwitches.get(foot).hasFootHitGroundFiltered())
            haveFeetHitGroundFiltered.get(foot).update(true);
         else
            haveFeetHitGroundFiltered.get(foot).set(false);

         if (haveFeetHitGroundFiltered.get(foot).getBooleanValue())
            numberOfEndEffectorsTrusted++;
      }

      // Update only if at least one foot hit the ground
      if (numberOfEndEffectorsTrusted > 0)
      {
         if (trustOnlyLowestFoot.getValue())
         {
            numberOfEndEffectorsTrusted = filterAndTrustLowestFoot();
         }
         else
         {
            for (int i = 0; i < feet.size(); i++)
            {
               RigidBodyBasics foot = feet.get(i);
               boolean isFootOnGround = haveFeetHitGroundFiltered.get(foot).getBooleanValue();
               areFeetTrusted.get(foot).set(isFootOnGround);
            }
         }
      }
      // Else if there is a foot with a force past the threshold trust the force and not the CoP
      else
      {
         RigidBodyBasics trustedFoot = null;

         for (int i = 0; i < feet.size(); i++)
         {
            RigidBodyBasics foot = feet.get(i);
            if (footSwitches.get(foot).hasFootHitGroundSensitive())
            {
               trustedFoot = foot;
               numberOfEndEffectorsTrusted = 1;
               break;
            }
         }

         if (trustedFoot != null)
         {
            for (int i = 0; i < feet.size(); i++)
            {
               RigidBodyBasics foot = feet.get(i);
               areFeetTrusted.get(foot).set(foot == trustedFoot);
            }
         }
      }

      if (numberOfEndEffectorsTrusted == 0)
      {
         if (trustImuWhenNoFeetAreInContact.getValue())
         {
            for (int i = 0; i < feet.size(); i++)
            {
               RigidBodyBasics foot = feet.get(i);
               areFeetTrusted.get(foot).set(false);
            }
         }
         else
         {
            for (int i = 0; i < feet.size(); i++)
            {
               RigidBodyBasics foot = feet.get(i);
               if (areFeetTrusted.get(foot).getBooleanValue())
                  numberOfEndEffectorsTrusted++;
            }
         }
      }

      return numberOfEndEffectorsTrusted;
   }

   private int filterAndTrustLowestFoot()
   {
      int lastLowestFootIdx = lowestFootInContactIndex.getValue();
      int lowestFootIdx = findLowestFootInContact();

      if (haveFeetHitGroundFiltered.get(feet.get(lastLowestFootIdx)).getValue())
      {
         // If the previously trusted foot is still in contact glitch filter the trusted foot to avoid
         // jumping between the feet.
         lowestFootInContactIndex.update(lowestFootIdx);
      }
      else
      {
         // In case the previously trusted foot is not in contact anymore we do not need to glitch filter.
         // Just use the new lowest foot.
         lowestFootInContactIndex.set(lowestFootIdx);
      }

      for (int footIdx = 0; footIdx < feet.size(); footIdx++)
      {
         areFeetTrusted.get(feet.get(footIdx)).set(footIdx == lowestFootInContactIndex.getValue());
      }

      return 1;
   }

   FramePoint3D tmpFramePoint = new FramePoint3D();

   private int findLowestFootInContact()
   {
      int lowestFootInContact = -1;
      double lowestFootZ = Double.MAX_VALUE;
      for (int footIdx = 0; footIdx < feet.size(); footIdx++)
      {
         RigidBodyBasics foot = feet.get(footIdx);
         tmpFramePoint.setToZero(foot.getBodyFixedFrame());
         tmpFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
         double footZ = tmpFramePoint.getZ();

         if (haveFeetHitGroundFiltered.get(foot).getBooleanValue())
         {
            if (footZ < lowestFootZ)
            {
               lowestFootZ = footZ;
               lowestFootInContact = footIdx;
            }
         }
      }
      return lowestFootInContact;
   }

   private int filterTrustedFeetBasedOnContactForces(int numberOfEndEffectorsTrusted)
   {
      double totalForceZ = 0.0;
      for (int i = 0; i < feet.size(); i++)
      {
         RigidBodyBasics foot = feet.get(i);
         if (!areFeetTrusted.get(foot).getBooleanValue())
            continue;
         Wrench footWrench = footWrenches.get(foot);
         footSwitches.get(foot).getMeasuredWrench(footWrench);
         FixedFrameVector3DBasics footForce = footForces.get(foot);
         footForce.setMatchingFrame(footWrench.getLinearPart());
         totalForceZ += footForce.getZ();
      }

      int filteredNumberOfEndEffectorsTrusted = 0;

      for (int i = 0; i < feet.size(); i++)
      {
         RigidBodyBasics foot = feet.get(i);
         if (!areFeetTrusted.get(foot).getBooleanValue())
            continue;

         FixedFrameVector3DBasics footForce = footForces.get(foot);
         YoDouble footLoad = footForcesZInPercentOfTotalForce.get(foot);
         footLoad.set(footForce.getZ() / totalForceZ);

         double percentForceToTrustFootAgain = forceZInPercentThresholdToTrustFoot.getValue();
         double percentForceToNotTrustFoot = forceZInPercentThresholdToNotTrustFoot.getValue();
         percentForceToTrustFootAgain = MathTools.clamp(percentForceToTrustFootAgain,
                                                        minForceZInPercentThresholdToFilterFoot,
                                                        maxForceZInPercentThresholdToFilterFoot);
         percentForceToNotTrustFoot = MathTools.clamp(percentForceToNotTrustFoot,
                                                      minForceZInPercentThresholdToFilterFoot,
                                                      maxForceZInPercentThresholdToFilterFoot);

         double magnitudeForTrust;
         if (wereFeetTrustedLastTick.get(foot).getValue())
            magnitudeForTrust = percentForceToNotTrustFoot;
         else
            magnitudeForTrust = percentForceToTrustFootAgain;

         if (footLoad.getValue() < magnitudeForTrust)
            areFeetTrusted.get(foot).set(false);
         else
            filteredNumberOfEndEffectorsTrusted++;
      }

      numberOfEndEffectorsFilteredByLoad.set(numberOfEndEffectorsTrusted - filteredNumberOfEndEffectorsTrusted);

      return filteredNumberOfEndEffectorsTrusted;
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

   private void updateTrustedFeetLists()
   {
      listOfTrustedFeet.clear();
      listOfUnTrustedFeet.clear();

      for (int i = 0; i < feet.size(); i++)
      {
         RigidBodyBasics foot = feet.get(i);
         if (areFeetTrusted.get(foot).getBooleanValue())
            listOfTrustedFeet.add(foot);
         else
            listOfUnTrustedFeet.add(foot);
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
      return listOfTrustedFeet;
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
