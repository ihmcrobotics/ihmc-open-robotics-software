package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import static us.ihmc.robotics.math.filters.AlphaFilteredYoVariable.*;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.screwTheory.CenterOfMassCalculator;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;

/**
 * PelvisLinearUpdater estimates the pelvis position and linear velocity using leg kinematics and IMU acceleration data.
 * If enabled, it also estimates the IMU drift on the estimation of the pelvis yaw angle and velocity.
 * @author Sylvain
 *
 */
public class PelvisLinearStateUpdater
{
   private static boolean VISUALIZE = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final CenterOfMassCalculator centerOfMassCalculator;
   private final CenterOfMassJacobian centerOfMassJacobianWorld;

   private final List<RigidBody> feet = new ArrayList<RigidBody>();

   private final YoFramePoint yoRootJointPosition = new YoFramePoint("estimatedRootJointPosition", worldFrame, registry);
   private final YoFrameVector yoRootJointVelocity = new YoFrameVector("estimatedRootJointVelocity", worldFrame, registry);

   private final YoDouble yoInitialComHeight = new YoDouble("initialComHeight", registry);
   private final YoFramePoint yoInitialCenterOfMassPosition = new YoFramePoint("initialCenterOfMassPosition", worldFrame, registry);
   private final YoFramePoint yoInitialFootPosition = new YoFramePoint("initialFootPosition", worldFrame, registry);

   private final YoFramePoint yoCenterOfMassPosition = new YoFramePoint("estimatedCenterOfMassPosition", worldFrame, registry);
   private final YoFrameVector yoCenterOfMassVelocityUsingPelvisAndKinematics = new YoFrameVector("estimatedCenterOfMassVelocityPelvisAndKin", worldFrame, registry);
   private final YoFrameVector yoCenterOfMassVelocityIntegrateGRF = new YoFrameVector("estimatedCenterOfMassVelocityGRF", worldFrame, registry);
   private final YoFrameVector yoCenterOfMassVelocity = new YoFrameVector("estimatedCenterOfMassVelocity", worldFrame, registry);

   private final CenterOfMassDataHolder estimatorCenterOfMassDataHolderToUpdate;
   
   private final YoFrameVector totalGroundReactionForce = new YoFrameVector("totalGroundForce", worldFrame, registry);
   private final YoDouble robotMass = new YoDouble("robotMass", registry);
   private final YoFrameVector comAcceleration = new YoFrameVector("comAcceleration", worldFrame, registry);

   private final YoDouble alphaIMUAgainstKinematicsForVelocity = new YoDouble("alphaIMUAgainstKinematicsForVelocity", registry);
   private final YoDouble alphaGRFAgainstIMUAndKinematicsForVelocity = new YoDouble("alphaGRFAgainstIMUAndKinematicsForVelocity", registry);
   private final YoDouble alphaIMUAgainstKinematicsForPosition = new YoDouble("alphaIMUAgainstKinematicsForPosition", registry);

   private final YoBoolean useGroundReactionForcesToComputeCenterOfMassVelocity = new YoBoolean("useGRFToComputeCoMVelocity", registry);
   private final YoInteger numberOfEndEffectorsTrusted = new YoInteger("numberOfEndEffectorsTrusted", registry);

   private final Map<RigidBody, YoDouble> footForcesZInPercentOfTotalForce = new LinkedHashMap<RigidBody, YoDouble>();
   private final YoDouble forceZInPercentThresholdToFilterFoot = new YoDouble("forceZInPercentThresholdToFilterFootUserParameter", registry);

   private final Map<RigidBody, FootSwitchInterface> footSwitches;
   private final Map<RigidBody, Wrench> footWrenches = new LinkedHashMap<RigidBody, Wrench>();
   private final YoDouble delayTimeBeforeTrustingFoot = new YoDouble("delayTimeBeforeTrustingFoot", registry);
   private final Map<RigidBody, GlitchFilteredYoBoolean> haveFeetHitGroundFiltered = new LinkedHashMap<>();
   private final Map<RigidBody, YoBoolean> areFeetTrusted = new LinkedHashMap<>();
   private final List<RigidBody> listOfTrustedFeet = new ArrayList<RigidBody>();
   private final List<RigidBody> listOfUnTrustedFeet = new ArrayList<RigidBody>();

   private final ReferenceFrame rootJointFrame;
   private final Map<RigidBody, ReferenceFrame> footFrames;

   private final double estimatorDT;

   private final Map<RigidBody, ? extends ContactablePlaneBody> feetContactablePlaneBodies;

   private final YoBoolean reinitialize = new YoBoolean("reinitialize", registry);
   private final YoBoolean trustImuWhenNoFeetAreInContact = new YoBoolean("trustImuWhenNoFeetAreInContact", registry);

   private enum SlippageCompensatorMode
   {
      LOAD_THRESHOLD, MIN_PELVIS_ACCEL
   };

   private final YoEnum<SlippageCompensatorMode> slippageCompensatorMode = new YoEnum<SlippageCompensatorMode>("slippageCompensatorMode",
         registry, SlippageCompensatorMode.class);

   private final YoBoolean requestStopEstimationOfPelvisLinearState = new YoBoolean("userRequestStopEstimationOfPelvisLinearState", registry);

   private final PelvisKinematicsBasedLinearStateCalculator kinematicsBasedLinearStateCalculator;
   private final PelvisIMUBasedLinearStateCalculator imuBasedLinearStateCalculator;

   private final FloatingInverseDynamicsJoint rootJoint;

   private boolean initializeToActual = false;

   // Temporary variables
   private final FramePoint3D rootJointPosition = new FramePoint3D(worldFrame);
   private final FrameVector3D rootJointVelocity = new FrameVector3D(worldFrame);
   private final FramePoint3D centerOfMassPosition = new FramePoint3D(worldFrame);
   private final FrameVector3D centerOfMassVelocityUsingPelvisIMUAndKinematics = new FrameVector3D(worldFrame);
   private final Vector3D tempRootJointTranslation = new Vector3D();
   private final FrameVector3D tempFrameVector = new FrameVector3D();
   private final FramePoint3D tempCenterOfMassPosition = new FramePoint3D();
   private final FramePoint3D footPositionInWorld = new FramePoint3D();
   private final FrameVector3D tempVelocity = new FrameVector3D();

   private final double gravitationalAcceleration;
   
   public PelvisLinearStateUpdater(FullInverseDynamicsStructure inverseDynamicsStructure, List<? extends IMUSensorReadOnly> imuProcessedOutputs,
         IMUBiasProvider imuBiasProvider, Map<RigidBody, FootSwitchInterface> footSwitches, 
         CenterOfMassDataHolder estimatorCenterOfMassDataHolderToUpdate, CenterOfPressureDataHolder centerOfPressureDataHolderFromController,
         Map<RigidBody, ? extends ContactablePlaneBody> feetContactablePlaneBodies, double gravitationalAcceleration, StateEstimatorParameters stateEstimatorParameters,
         YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.estimatorDT = stateEstimatorParameters.getEstimatorDT();
      this.footSwitches = footSwitches;
      this.feetContactablePlaneBodies = feetContactablePlaneBodies;
      this.feet.addAll(footSwitches.keySet());
      this.trustImuWhenNoFeetAreInContact.set(stateEstimatorParameters.getPelvisLinearStateUpdaterTrustImuWhenNoFeetAreInContact());

      this.estimatorCenterOfMassDataHolderToUpdate = estimatorCenterOfMassDataHolderToUpdate;
      
      rootJoint = inverseDynamicsStructure.getRootJoint();
      rootJointFrame = rootJoint.getFrameAfterJoint();

      footFrames = new LinkedHashMap<RigidBody, ReferenceFrame>();

      RigidBody elevator = inverseDynamicsStructure.getElevator();
      this.centerOfMassCalculator = new CenterOfMassCalculator(elevator, rootJointFrame);
      this.centerOfMassJacobianWorld = new CenterOfMassJacobian(elevator);

      this.gravitationalAcceleration = gravitationalAcceleration;
      
      robotMass.set(TotalMassCalculator.computeSubTreeMass(elevator));
      setupBunchOfVariables();

      forceZInPercentThresholdToFilterFoot.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            double currentValue = forceZInPercentThresholdToFilterFoot.getDoubleValue();
            if (currentValue < 0.0)
               forceZInPercentThresholdToFilterFoot.set(0.0);
            else if (currentValue > 0.45)
               forceZInPercentThresholdToFilterFoot.set(0.45);
         }
      });

      reinitialize.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            if (reinitialize.getBooleanValue())
               initialize();
         }
      });

      kinematicsBasedLinearStateCalculator = new PelvisKinematicsBasedLinearStateCalculator(inverseDynamicsStructure, feetContactablePlaneBodies, footSwitches,
            centerOfPressureDataHolderFromController, estimatorDT, yoGraphicsListRegistry, registry);
      kinematicsBasedLinearStateCalculator.setAlphaPelvisPosition(computeAlphaGivenBreakFrequencyProperly(stateEstimatorParameters.getKinematicsPelvisPositionFilterFreqInHertz(), estimatorDT));
      kinematicsBasedLinearStateCalculator.setPelvisLinearVelocityAlphaNewTwist(stateEstimatorParameters.getPelvisLinearVelocityAlphaNewTwist());
      kinematicsBasedLinearStateCalculator.setTrustCoPAsNonSlippingContactPoint(stateEstimatorParameters.trustCoPAsNonSlippingContactPoint());
      kinematicsBasedLinearStateCalculator.useControllerDesiredCoP(stateEstimatorParameters.useControllerDesiredCenterOfPressure());
      kinematicsBasedLinearStateCalculator.setAlphaCenterOfPressure(computeAlphaGivenBreakFrequencyProperly(stateEstimatorParameters.getCoPFilterFreqInHertz(), estimatorDT));
      kinematicsBasedLinearStateCalculator.setCorrectTrustedFeetPositions(stateEstimatorParameters.correctTrustedFeetPositions());

      imuBasedLinearStateCalculator = new PelvisIMUBasedLinearStateCalculator(inverseDynamicsStructure, imuProcessedOutputs, imuBiasProvider, estimatorDT,
            gravitationalAcceleration, yoGraphicsListRegistry, registry);
      boolean cancelGravityFromAccelerationMeasurement = stateEstimatorParameters.cancelGravityFromAccelerationMeasurement();
      imuBasedLinearStateCalculator.cancelGravityFromAccelerationMeasurement(cancelGravityFromAccelerationMeasurement);
      imuBasedLinearStateCalculator.enableEstimationModule(stateEstimatorParameters.useAccelerometerForEstimation());

      alphaIMUAgainstKinematicsForVelocity
            .set(computeAlphaGivenBreakFrequencyProperly(stateEstimatorParameters.getPelvisLinearVelocityFusingFrequency(), estimatorDT));
      alphaGRFAgainstIMUAndKinematicsForVelocity.set(computeAlphaGivenBreakFrequencyProperly(stateEstimatorParameters.getCenterOfMassVelocityFusingFrequency(), estimatorDT));
      alphaIMUAgainstKinematicsForPosition
            .set(computeAlphaGivenBreakFrequencyProperly(stateEstimatorParameters.getPelvisPositionFusingFrequency(), estimatorDT));

      useGroundReactionForcesToComputeCenterOfMassVelocity.set(stateEstimatorParameters.useGroundReactionForcesToComputeCenterOfMassVelocity());

      delayTimeBeforeTrustingFoot.set(stateEstimatorParameters.getDelayTimeForTrustingFoot());

      forceZInPercentThresholdToFilterFoot.set(stateEstimatorParameters.getForceInPercentOfWeightThresholdToTrustFoot());

      slippageCompensatorMode.set(SlippageCompensatorMode.LOAD_THRESHOLD);

      //      requestStopEstimationOfPelvisLinearState.set(true);

      if (VISUALIZE)
      {
         if (yoGraphicsListRegistry != null)
         {
            YoArtifactPosition comArtifact = new YoGraphicPosition("Meas CoM", yoCenterOfMassPosition, 0.006, YoAppearance.Black(), GraphicType.CROSS)
                  .createArtifact();
            yoGraphicsListRegistry.registerArtifact("StateEstimator", comArtifact);
         }
      }
      parentRegistry.addChild(registry);
   }

   private void setupBunchOfVariables()
   {
      int windowSize = (int) (delayTimeBeforeTrustingFoot.getDoubleValue() / estimatorDT);

      for (int i = 0; i < feet.size(); i++)
      {
         RigidBody foot = feet.get(i);
         ReferenceFrame footFrame = feetContactablePlaneBodies.get(foot).getSoleFrame();
         footFrames.put(foot, footFrame);

         String footPrefix = foot.getName();

         final GlitchFilteredYoBoolean hasFootHitTheGroundFiltered = new GlitchFilteredYoBoolean("has" + footPrefix + "FootHitGroundFiltered",
               registry, windowSize);
         hasFootHitTheGroundFiltered.set(true);
         haveFeetHitGroundFiltered.put(foot, hasFootHitTheGroundFiltered);

         YoBoolean isFootTrusted = new YoBoolean("is" + footPrefix + "FootTrusted", registry);
         if (i == 0)
         {
            isFootTrusted.set(true);
         }
         areFeetTrusted.put(foot, isFootTrusted);

         YoDouble footForceZInPercentOfTotalForce = new YoDouble(footPrefix + "FootForceZInPercentOfTotalForce", registry);
         footForcesZInPercentOfTotalForce.put(foot, footForceZInPercentOfTotalForce);

         footWrenches.put(foot, new Wrench());

         delayTimeBeforeTrustingFoot.addVariableChangedListener(new VariableChangedListener()
         {
            @Override
            public void notifyOfVariableChange(YoVariable<?> v)
            {
               int windowSize = (int) (delayTimeBeforeTrustingFoot.getDoubleValue() / estimatorDT);
               hasFootHitTheGroundFiltered.setWindowSize(windowSize);
            }
         });
      }
   }

   public void initialize()
   {
      initializeRobotState();
      updateRootJoint();
   }

   private void initializeRobotState()
   {
      reinitialize.set(false);

      centerOfMassCalculator.compute();
      centerOfMassCalculator.getCenterOfMass(tempCenterOfMassPosition);

      tempCenterOfMassPosition.changeFrame(worldFrame);
      yoInitialCenterOfMassPosition.set(tempCenterOfMassPosition);

      if (!initializeToActual && DRCKinematicsBasedStateEstimator.INITIALIZE_HEIGHT_WITH_FOOT)
      {
         RigidBody foot = feet.get(0);
         footPositionInWorld.setToZero(footFrames.get(foot));
         footPositionInWorld.changeFrame(worldFrame);
         yoInitialFootPosition.set(footPositionInWorld);

         double footToCoMZ = tempCenterOfMassPosition.getZ() - footPositionInWorld.getZ();
         yoInitialComHeight.set(footToCoMZ);

         tempCenterOfMassPosition.setZ(tempCenterOfMassPosition.getZ() - footToCoMZ);
      }
      tempFrameVector.setIncludingFrame(tempCenterOfMassPosition);

      rootJointPosition.set(centerOfMassPosition);
      rootJointPosition.sub(tempFrameVector);
      yoRootJointPosition.set(rootJointPosition);
      rootJointVelocity.setToZero(worldFrame);
      yoRootJointVelocity.setToZero();

      kinematicsBasedLinearStateCalculator.initialize(rootJointPosition);
   }

   public void updateForFrozenState()
   {
      // Keep setting the position so the localization updater works properly.
      tempRootJointTranslation.set(yoRootJointPosition);
      rootJoint.setPosition(tempRootJointTranslation);

      // Set the rootJoint twist to zero.
      rootJoint.getJointTwist(rootJointTwist);
      rootJointTwist.setToZero();
      rootJoint.setJointTwist(rootJointTwist);
      rootJoint.updateFramesRecursively();

      updateCoMState();
   }

   public void updateRootJointPositionAndLinearVelocity()
   {
      if (requestStopEstimationOfPelvisLinearState.getBooleanValue())
         return;

      kinematicsBasedLinearStateCalculator.updateKinematics();

      numberOfEndEffectorsTrusted.set(setTrustedFeetUsingFootSwitches());

      if (numberOfEndEffectorsTrusted.getIntegerValue() >= 2)
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
         if (trustImuWhenNoFeetAreInContact.getBooleanValue())
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
            kinematicsBasedLinearStateCalculator.getRootJointPositionAndVelocity(rootJointPosition, rootJointVelocity);
            yoRootJointPosition.set(rootJointPosition);
            yoRootJointVelocity.set(rootJointVelocity);
         }
      }
      else
      {
         throw new RuntimeException(
               "Computation of the number of end effectors to be trusted for state estimation is broken, computed: " + numberOfEndEffectorsTrusted);
      }

      updateRootJoint();

      updateCoMState();
   }

   private Twist rootJointTwist = new Twist();

   private void updateRootJoint()
   {
      tempRootJointTranslation.set(yoRootJointPosition);
      rootJoint.setPosition(tempRootJointTranslation);

      tempVelocity.setIncludingFrame(rootJointVelocity);
      rootJoint.getJointTwist(rootJointTwist);
      tempVelocity.changeFrame(rootJointTwist.getExpressedInFrame());
      rootJointTwist.setLinearPart(tempVelocity);
      rootJoint.setJointTwist(rootJointTwist);
      rootJoint.updateFramesRecursively();
   }

   private int setTrustedFeetUsingFootSwitches()
   {
      int numberOfEndEffectorsTrusted = 0;

      for (int i = 0; i < feet.size(); i++)
      {
         RigidBody foot = feet.get(i);
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
         for (int i = 0; i < feet.size(); i++)
         {
            RigidBody foot = feet.get(i);
            areFeetTrusted.get(foot).set(haveFeetHitGroundFiltered.get(foot).getBooleanValue());
         }
      }

      // Else if there is a foot with a force past the threshold trust the force and not the CoP
      else
      {
         RigidBody trustedFoot = null;

         for (int i = 0; i < feet.size(); i++)
         {
            RigidBody foot = feet.get(i);
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
               RigidBody foot = feet.get(i);
               areFeetTrusted.get(foot).set(foot == trustedFoot);
            }
         }
      }

      if (numberOfEndEffectorsTrusted == 0)
      {
         if (trustImuWhenNoFeetAreInContact.getBooleanValue())
         {
            for (int i = 0; i < feet.size(); i++)
            {
               RigidBody foot = feet.get(i);
               areFeetTrusted.get(foot).set(false);
            }
         }
         else
         {
            for (int i = 0; i < feet.size(); i++)
            {
               RigidBody foot = feet.get(i);
               if (areFeetTrusted.get(foot).getBooleanValue())
                  numberOfEndEffectorsTrusted++;
            }
         }
      }

      return numberOfEndEffectorsTrusted;
   }

   private int filterTrustedFeetBasedOnContactForces(int numberOfEndEffectorsTrusted)
   {
      double totalForceZ = 0.0;
      for (int i = 0; i < feet.size(); i++)
      {
         RigidBody foot = feet.get(i);
         Wrench footWrench = footWrenches.get(foot);
         footSwitches.get(foot).computeAndPackFootWrench(footWrench);
         totalForceZ += footWrench.getLinearPartZ();
      }

      for (int i = 0; i < feet.size(); i++)
      {
         RigidBody foot = feet.get(i);
         Wrench footWrench = footWrenches.get(foot);
         footForcesZInPercentOfTotalForce.get(foot).set(footWrench.getLinearPartZ() / totalForceZ);

         if (footForcesZInPercentOfTotalForce.get(foot).getDoubleValue() < forceZInPercentThresholdToFilterFoot.getDoubleValue())
         {
            numberOfEndEffectorsTrusted--;
            areFeetTrusted.get(foot).set(false);

            return numberOfEndEffectorsTrusted;
         }
      }

      return numberOfEndEffectorsTrusted;
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

   private void computeLinearVelocityFromMergingMeasurements()
   {
      // TODO Check out AlphaFusedYoVariable to that
      imuBasedLinearStateCalculator.updateIMUAndRootJointLinearVelocity(pelvisVelocityIMUPart);
      kinematicsBasedLinearStateCalculator.getPelvisVelocity(pelvisVelocityKinPart);

      pelvisVelocityIMUPart.scale(alphaIMUAgainstKinematicsForVelocity.getDoubleValue());
      pelvisVelocityKinPart.scale(1.0 - alphaIMUAgainstKinematicsForVelocity.getDoubleValue());

      rootJointVelocity.add(pelvisVelocityIMUPart, pelvisVelocityKinPart);
      yoRootJointVelocity.set(rootJointVelocity);

      imuBasedLinearStateCalculator.correctIMULinearVelocity(rootJointVelocity);
   }

   private void computePositionFromMergingMeasurements()
   {
      rootJointPosition.set(yoRootJointPosition);
      imuBasedLinearStateCalculator.updatePelvisPosition(rootJointPosition, pelvisPositionIMUPart);
      kinematicsBasedLinearStateCalculator.getPelvisPosition(pelvisPositionKinPart);

      pelvisPositionIMUPart.scale(alphaIMUAgainstKinematicsForPosition.getDoubleValue());
      pelvisPositionKinPart.scale(1.0 - alphaIMUAgainstKinematicsForPosition.getDoubleValue());

      rootJointPosition.set(pelvisPositionIMUPart);
      rootJointPosition.add(pelvisPositionKinPart);
      yoRootJointPosition.set(rootJointPosition);
   }

   private final FrameVector3D comVelocityGRFPart = new FrameVector3D();
   private final FrameVector3D comVelocityPelvisAndKinPart = new FrameVector3D();

   private void updateCoMState()
   {
      centerOfMassCalculator.compute();
      centerOfMassCalculator.getCenterOfMass(centerOfMassPosition);
      centerOfMassPosition.changeFrame(worldFrame);
      yoCenterOfMassPosition.set(centerOfMassPosition);

      centerOfMassJacobianWorld.compute();
      centerOfMassVelocityUsingPelvisIMUAndKinematics.setToZero(ReferenceFrame.getWorldFrame());
      centerOfMassJacobianWorld.getCenterOfMassVelocity(centerOfMassVelocityUsingPelvisIMUAndKinematics);
      centerOfMassVelocityUsingPelvisIMUAndKinematics.changeFrame(ReferenceFrame.getWorldFrame());
      yoCenterOfMassVelocityUsingPelvisAndKinematics.set(centerOfMassVelocityUsingPelvisIMUAndKinematics);

      if (useGroundReactionForcesToComputeCenterOfMassVelocity.getBooleanValue())
      {
         //centerOfMassVelocity at this point is from pelvis velocity and joint velocities, where pelvis velocity is estimated from pelvis imu at high freq and leg kinematics at low freq.

         //TODO: Lots of duplicated computation with reading the force sensors and changing frames. Just do it once and share...
         computeTotalGroundReactionForce();
         
         double totalMass = robotMass.getDoubleValue();
         if (totalMass < 0.01) totalMass = 0.01;

         comAcceleration.scale(1.0/totalMass);

         comAcceleration.scale(estimatorDT);
         yoCenterOfMassVelocityIntegrateGRF.add(comAcceleration);

         comVelocityGRFPart.set(yoCenterOfMassVelocity);
         comVelocityGRFPart.add(comAcceleration);
         comAcceleration.scale(1.0/estimatorDT);

         comVelocityPelvisAndKinPart.set(centerOfMassVelocityUsingPelvisIMUAndKinematics);

         comVelocityGRFPart.scale(alphaGRFAgainstIMUAndKinematicsForVelocity.getDoubleValue());
         comVelocityPelvisAndKinPart.scale(1.0 - alphaGRFAgainstIMUAndKinematicsForVelocity.getDoubleValue());


         yoCenterOfMassVelocity.add(comVelocityGRFPart, comVelocityPelvisAndKinPart);         
      }
      else
      {
         yoCenterOfMassVelocity.set(centerOfMassVelocityUsingPelvisIMUAndKinematics);
      }
      
      if (estimatorCenterOfMassDataHolderToUpdate != null) estimatorCenterOfMassDataHolderToUpdate.setCenterOfMassVelocity(yoCenterOfMassVelocity);
   }


   private final FrameVector3D tempCoMAcceleration = new FrameVector3D(ReferenceFrame.getWorldFrame());
   private final FrameVector3D tempFootForce = new FrameVector3D(ReferenceFrame.getWorldFrame());

   private void computeTotalGroundReactionForce()
   {
      totalGroundReactionForce.setToZero();

      for (int i = 0; i < feet.size(); i++)
      {
         RigidBody foot = feet.get(i);
         Wrench footWrench = footWrenches.get(foot);
         footSwitches.get(foot).computeAndPackFootWrench(footWrench);
         footWrench.getLinearPartIncludingFrame(tempFootForce);
         tempFootForce.changeFrame(worldFrame);

         totalGroundReactionForce.add(tempFootForce);
      }

      tempCoMAcceleration.set(totalGroundReactionForce);
      comAcceleration.set(tempCoMAcceleration);
      comAcceleration.setZ(comAcceleration.getZ() - robotMass.getDoubleValue() * gravitationalAcceleration);
   }

   private void updateTrustedFeetLists()
   {
      listOfTrustedFeet.clear();
      listOfUnTrustedFeet.clear();

      for (int i = 0; i < feet.size(); i++)
      {
         RigidBody foot = feet.get(i);
         if (areFeetTrusted.get(foot).getBooleanValue())
            listOfTrustedFeet.add(foot);
         else
            listOfUnTrustedFeet.add(foot);
      }
   }

   public void initializeCoMPositionToActual(Tuple3DReadOnly initialCoMPosition)
   {
      initializeToActual = true;
      centerOfMassPosition.setIncludingFrame(worldFrame, initialCoMPosition);
      yoCenterOfMassPosition.set(initialCoMPosition);
   }

   public void initializeCoMPositionToActual(FramePoint3D initialCoMPosition)
   {
      initializeToActual = true;
      centerOfMassPosition.set(initialCoMPosition);
      yoCenterOfMassPosition.set(initialCoMPosition);
   }

   public void getEstimatedPelvisPosition(FramePoint3D pelvisPositionToPack)
   {
      pelvisPositionToPack.setIncludingFrame(yoRootJointPosition);
   }

   public void getEstimatedPelvisLinearVelocity(FrameVector3D pelvisLinearVelocityToPack)
   {
      pelvisLinearVelocityToPack.setIncludingFrame(yoRootJointVelocity);
   }

   public void getEstimatedCoMPosition(FramePoint3D comPositionToPack)
   {
      comPositionToPack.setIncludingFrame(yoCenterOfMassPosition);
   }

   public void getEstimatedCoMVelocity(FrameVector3D comVelocityToPack)
   {
      comVelocityToPack.setIncludingFrame(yoCenterOfMassVelocity);
   }

   public List<RigidBody> getCurrentListOfTrustedFeet()
   {
      return listOfTrustedFeet;
   }
}
