package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.math.filters.GlitchFilteredYoInteger;
import us.ihmc.robotics.math.filters.IntegratorBiasCompensatorYoFrameVector3D;
import us.ihmc.robotics.sensors.FootSwitchInterface;
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
public class PelvisLinearStateUpdater
{
   private static final double minForceZInPercentThresholdToFilterFoot = 0.0;
   private static final double maxForceZInPercentThresholdToFilterFoot = 0.45;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final List<RigidBodyBasics> feet = new ArrayList<RigidBodyBasics>();

   private final YoFramePoint3D yoRootJointPosition = new YoFramePoint3D("estimatedRootJointPosition", worldFrame, registry);
   private final YoFrameVector3D yoRootJointVelocity = new YoFrameVector3D("estimatedRootJointVelocity", worldFrame, registry);

   private final DoubleProvider imuAgainstKinematicsForVelocityBreakFrequency;
   private final DoubleProvider imuAgainstKinematicsForPositionBreakFrequency;

   private final BooleanProvider useNewFusingFilter;

   private final DoubleProvider linearVelocityFusingKp, linearVelocityFusingKi;
   private final IntegratorBiasCompensatorYoFrameVector3D mainIMULinearVelocityEstimate;
   private final YoFrameVector3D pelvisNewLinearVelocityEstimate;

   private final DoubleProvider positionFusingKp, positionFusingKi;
   private final IntegratorBiasCompensatorYoFrameVector3D pelvisPositionEstimate;

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
   private final FramePoint3D rootJointPosition = new FramePoint3D(worldFrame);
   private final FrameVector3D rootJointVelocity = new FrameVector3D(worldFrame);
   private final Vector3D tempRootJointTranslation = new Vector3D();
   private final FramePoint3D footPositionInWorld = new FramePoint3D();

   private final BooleanProvider trustOnlyLowestFoot = new BooleanParameter("TrustOnlyLowestFoot", registry, false);
   private final IntegerProvider lowestFootWindowSize = new IntegerParameter("LowestFootWindowSize", registry, 0);
   private final GlitchFilteredYoInteger lowestFootInContactIndex = new GlitchFilteredYoInteger("LowestFootInContact", lowestFootWindowSize, registry);

   private final BooleanParameter zeroRootXYPositionAtInitialization = new BooleanParameter("zeroRootXYPositionAtInitialization", registry, false);
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

      imuBasedLinearStateCalculator = new PelvisIMUBasedLinearStateCalculator(inverseDynamicsStructure,
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
      pelvisNewLinearVelocityEstimate = new YoFrameVector3D("newEstimatedRootJointLinearVelocity", worldFrame, registry);

      pelvisPositionEstimate = new IntegratorBiasCompensatorYoFrameVector3D("newPelvisPositionEstimate",
                                                                            registry,
                                                                            positionFusingKp,
                                                                            positionFusingKi,
                                                                            worldFrame,
                                                                            rootJointFrame, // Keep the bias in the local frame instead of world.
                                                                            estimatorDT);

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
         rootJointPosition.setIncludingFrame(worldFrame, rootJoint.getJointPose().getPosition());

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

      rootJointVelocity.setToZero(worldFrame);
      yoRootJointPosition.set(rootJointPosition);
      yoRootJointVelocity.setToZero();

      kinematicsBasedLinearStateCalculator.initialize(rootJointPosition);

      imuBasedLinearStateCalculator.initialize();
   }

   public void initializeRootJointPosition(Tuple3DReadOnly rootJointPosition)
   {
      initializeToActual = true;
      initialRootJointPosition.setIncludingFrame(worldFrame, rootJointPosition);
   }

   public void updateForFrozenState()
   {
      // Keep setting the position so the localization updater works properly.
      tempRootJointTranslation.set(yoRootJointPosition);
      rootJoint.setJointPosition(tempRootJointTranslation);
      kinematicsBasedLinearStateCalculator.updateKinematics();
      kinematicsBasedLinearStateCalculator.updateFeetPositionsWhenTrustingIMUOnly(yoRootJointPosition);
      kinematicsBasedLinearStateCalculator.setPelvisLinearVelocityToZero();
      // Reset the IMU updater
      imuBasedLinearStateCalculator.initialize();

      // Set the rootJoint twist to zero.
      rootJoint.getJointTwist().setToZero();
      rootJoint.updateFramesRecursively();
      yoRootJointVelocity.setToZero();
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

      if (numberOfEndEffectorsTrusted.getIntegerValue() == 0)
      {
         if (trustImuWhenNoFeetAreInContact.getValue())
         {
            rootJointPosition.set(yoRootJointPosition);
            kinematicsBasedLinearStateCalculator.updateFeetPositionsWhenTrustingIMUOnly(rootJointPosition);

            imuBasedLinearStateCalculator.updateIMUAndRootJointLinearVelocity(rootJointVelocity);
            yoRootJointVelocity.set(rootJointVelocity);
            imuBasedLinearStateCalculator.correctIMULinearVelocity(rootJointVelocity);

            rootJointPosition.set(yoRootJointPosition);
            imuBasedLinearStateCalculator.updatePelvisPosition(rootJointPosition, pelvisPositionIMUPart);
            rootJointPosition.set(pelvisPositionIMUPart);
            yoRootJointPosition.set(rootJointPosition);
         }
         else
            throw new RuntimeException("No foot trusted!");
      }
      else if (numberOfEndEffectorsTrusted.getIntegerValue() > 0)
      {
         updateTrustedFeetLists();
         rootJointPosition.set(yoRootJointPosition);
         kinematicsBasedLinearStateCalculator.estimatePelvisLinearState(listOfTrustedFeet, listOfUnTrustedFeet, rootJointPosition);

         if (imuBasedLinearStateCalculator.isEstimationEnabled())
         {
            computeLinearStateFromMergingMeasurements();
         }
         else
         {
            rootJointPosition.set(yoRootJointPosition);
            yoRootJointPosition.set(kinematicsBasedLinearStateCalculator.getPelvisPosition());
            yoRootJointVelocity.set(kinematicsBasedLinearStateCalculator.getPelvisVelocity());
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
      rootJoint.getJointPose().getPosition().set(yoRootJointPosition);
      rootJoint.getJointTwist().getLinearPart().setMatchingFrame(rootJointVelocity);
      rootJoint.updateFrame();
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

         if (footSwitches.get(foot).hasFootHitGround())
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
            if (footSwitches.get(foot).getForceMagnitudePastThreshhold())
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
         footSwitches.get(foot).computeAndPackFootWrench(footWrench);
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

   private final FrameVector3D pelvisVelocityIMUPart = new FrameVector3D();
   private final FramePoint3D pelvisPositionIMUPart = new FramePoint3D();

   private final FrameVector3D pelvisVelocityKinPart = new FrameVector3D();
   private final FramePoint3D pelvisPositionKinPart = new FramePoint3D();

   private void computeLinearStateFromMergingMeasurements()
   {
      computeLinearVelocityFromMergingMeasurements();
      computePositionFromMergingMeasurements();
   }

   private final Twist tempTwist = new Twist();

   private void computeLinearVelocityFromMergingMeasurements()
   {
      imuBasedLinearStateCalculator.updateIMUAndRootJointLinearVelocity(pelvisVelocityIMUPart);

      if (!useNewFusingFilter.getValue())
      {
         // TODO Check out AlphaFusedYoVariable to that
         pelvisVelocityKinPart.setIncludingFrame(kinematicsBasedLinearStateCalculator.getPelvisVelocity());

         double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(imuAgainstKinematicsForVelocityBreakFrequency.getValue(), estimatorDT);
         pelvisVelocityIMUPart.scale(alpha);
         pelvisVelocityKinPart.scale(1.0 - alpha);

         rootJointVelocity.add(pelvisVelocityIMUPart, pelvisVelocityKinPart);
         yoRootJointVelocity.set(rootJointVelocity);
      }

      tempTwist.setToZero(rootJointFrame, rootJointFrame.getRootFrame(), rootJointFrame);
      tempTwist.getAngularPart().setMatchingFrame(rootJointFrame.getTwistOfFrame().getAngularPart());
      tempTwist.getLinearPart().setMatchingFrame(kinematicsBasedLinearStateCalculator.getPelvisVelocity());
      tempTwist.changeFrame(imuBasedLinearStateCalculator.getIMUMeasurementFrame());

      mainIMULinearVelocityEstimate.update(tempTwist.getLinearPart(), imuBasedLinearStateCalculator.getLinearAccelerationMeasurement());
      tempTwist.getLinearPart().set((Vector3DReadOnly) mainIMULinearVelocityEstimate);
      tempTwist.changeFrame(rootJointFrame);
      pelvisNewLinearVelocityEstimate.setMatchingFrame(tempTwist.getLinearPart());

      if (useNewFusingFilter.getValue())
      {
         rootJointVelocity.set(pelvisNewLinearVelocityEstimate);
         yoRootJointVelocity.set(pelvisNewLinearVelocityEstimate);
      }

      imuBasedLinearStateCalculator.correctIMULinearVelocity(rootJointVelocity);
   }

   private void computePositionFromMergingMeasurements()
   {
      if (!useNewFusingFilter.getValue())
      {
         rootJointPosition.set(yoRootJointPosition);
         imuBasedLinearStateCalculator.updatePelvisPosition(rootJointPosition, pelvisPositionIMUPart);
         pelvisPositionKinPart.setIncludingFrame(kinematicsBasedLinearStateCalculator.getPelvisPosition());

         double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(imuAgainstKinematicsForPositionBreakFrequency.getValue(), estimatorDT);
         pelvisPositionIMUPart.scale(alpha);
         pelvisPositionKinPart.scale(1.0 - alpha);

         rootJointPosition.set(pelvisPositionIMUPart);
         rootJointPosition.add(pelvisPositionKinPart);
         yoRootJointPosition.set(rootJointPosition);
      }

      pelvisPositionEstimate.update(kinematicsBasedLinearStateCalculator.getPelvisPosition(), pelvisNewLinearVelocityEstimate);

      if (useNewFusingFilter.getValue())
      {
         rootJointPosition.set(pelvisPositionEstimate);
         yoRootJointPosition.set(pelvisPositionEstimate);
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
      pelvisPositionToPack.setIncludingFrame(yoRootJointPosition);
   }

   public void getEstimatedPelvisLinearVelocity(FrameVector3D pelvisLinearVelocityToPack)
   {
      pelvisLinearVelocityToPack.setIncludingFrame(yoRootJointVelocity);
   }

   public List<RigidBodyBasics> getCurrentListOfTrustedFeet()
   {
      return listOfTrustedFeet;
   }
}
