package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;


import static us.ihmc.robotics.math.filters.AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.filters.GlitchFilteredBooleanYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.CenterOfMassCalculator;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactPosition;


/**
 * PelvisLinearUpdater estimates the pelvis position and linear velocity using leg kinematics and IMU acceleration data.
 * If enabled, it also estimates the IMU drift on the estimation of the pelvis yaw angle and velocity.
 * @author Sylvain
 *
 */
public class PelvisLinearStateUpdater
{
   private static boolean VISUALIZE = false;
   private static final boolean ALLOW_USING_IMU_ONLY = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final CenterOfMassCalculator centerOfMassCalculator;
   private final CenterOfMassJacobian centerOfMassJacobianWorld;
   
   private final List<RigidBody> feet = new ArrayList<RigidBody>();

   private final YoFramePoint yoRootJointPosition = new YoFramePoint("estimatedRootJointPosition", worldFrame, registry);
   private final YoFrameVector yoRootJointVelocity = new YoFrameVector("estimatedRootJointVelocity", worldFrame, registry);
   
   private final YoFramePoint yoCenterOfMassPosition = new YoFramePoint("estimatedCenterOfMassPosition", worldFrame, registry);
   private final YoFrameVector yoCenterOfMassVelocity = new YoFrameVector("estimatedCenterOfMassVelocity", worldFrame, registry);

   private final DoubleYoVariable alphaIMUAgainstKinematicsForVelocity = new DoubleYoVariable("alphaIMUAgainstKinematicsForVelocity", registry);
   private final DoubleYoVariable alphaIMUAgainstKinematicsForPosition = new DoubleYoVariable("alphaIMUAgainstKinematicsForPosition", registry);
   
   private final Map<RigidBody, DoubleYoVariable> footForcesZInPercentOfTotalForce = new LinkedHashMap<RigidBody, DoubleYoVariable>();
   private final DoubleYoVariable forceZInPercentThresholdToFilterFoot = new DoubleYoVariable("forceZInPercentThresholdToFilterFootUserParameter", registry);

   private final Map<RigidBody, FootSwitchInterface> footSwitches;
   private final Map<RigidBody, Wrench> footWrenches = new LinkedHashMap<RigidBody, Wrench>();
   private final DoubleYoVariable delayTimeBeforeTrustingFoot = new DoubleYoVariable("delayTimeBeforeTrustingFoot", registry);
   private final Map<RigidBody, GlitchFilteredBooleanYoVariable> haveFeetHitGroundFiltered = new LinkedHashMap<>();
   private final Map<RigidBody, BooleanYoVariable> areFeetTrusted = new LinkedHashMap<>();
   private final List<RigidBody> listOfTrustedFeet = new ArrayList<RigidBody>();
   private final List<RigidBody> listOfUnTrustedFeet = new ArrayList<RigidBody>();
   
   private final ReferenceFrame rootJointFrame;
   private final Map<RigidBody, ReferenceFrame> footFrames;
   
   private final double estimatorDT;
      
   private final Map<RigidBody, ContactablePlaneBody> feetContactablePlaneBodies;
   
   private final BooleanYoVariable reinitialize = new BooleanYoVariable("reinitialize", registry);

   private enum SlippageCompensatorMode {LOAD_THRESHOLD, MIN_PELVIS_ACCEL};
   private final EnumYoVariable<SlippageCompensatorMode> slippageCompensatorMode = new EnumYoVariable<SlippageCompensatorMode>("slippageCompensatorMode", registry, SlippageCompensatorMode.class);
   
   private final BooleanYoVariable requestStopEstimationOfPelvisLinearState = new BooleanYoVariable("userRequestStopEstimationOfPelvisLinearState", registry);

   private final TwistCalculator twistCalculator;
   
   private final PelvisKinematicsBasedLinearStateCalculator kinematicsBasedLinearStateCalculator;
   private final PelvisIMUBasedLinearStateCalculator imuBasedLinearStateCalculator;
   private final IMUDriftCompensator imuDriftCompensator;
   
   private final SixDoFJoint rootJoint;
   
   private boolean initializeToActual = false;
   
   // Temporary variables
   private final FramePoint rootJointPosition = new FramePoint(worldFrame);
   private final FrameVector rootJointVelocity = new FrameVector(worldFrame);
   private final FramePoint centerOfMassPosition = new FramePoint(worldFrame);
   private final FrameVector centerOfMassVelocity = new FrameVector(worldFrame);
   private final Vector3d tempRootJointTranslation = new Vector3d();
   private final FrameVector tempFrameVector = new FrameVector();
   private final FramePoint tempPosition = new FramePoint();
   private final FrameVector tempVelocity = new FrameVector();
   
   public PelvisLinearStateUpdater(FullInverseDynamicsStructure inverseDynamicsStructure, List<? extends IMUSensorReadOnly> imuProcessedOutputs,
         Map<RigidBody, FootSwitchInterface> footSwitches, CenterOfPressureDataHolder centerOfPressureDataHolderFromController,
         Map<RigidBody, ContactablePlaneBody> feetContactablePlaneBodies, double gravitationalAcceleration, DoubleYoVariable yoTime,
         StateEstimatorParameters stateEstimatorParameters, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.estimatorDT = stateEstimatorParameters.getEstimatorDT();
      this.footSwitches = footSwitches;
      this.feetContactablePlaneBodies = feetContactablePlaneBodies;
      this.feet.addAll(footSwitches.keySet());
      
      twistCalculator = inverseDynamicsStructure.getTwistCalculator();
      rootJoint = inverseDynamicsStructure.getRootJoint();
      rootJointFrame = rootJoint.getFrameAfterJoint();
      
      footFrames = new LinkedHashMap<RigidBody, ReferenceFrame>();
      
      RigidBody elevator = inverseDynamicsStructure.getElevator();
      this.centerOfMassCalculator = new CenterOfMassCalculator(elevator, rootJointFrame);
      this.centerOfMassJacobianWorld = new CenterOfMassJacobian(elevator);

      setupBunchOfVariables();
      
      forceZInPercentThresholdToFilterFoot.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            double currentValue = forceZInPercentThresholdToFilterFoot.getDoubleValue();
            if (currentValue < 0.0) forceZInPercentThresholdToFilterFoot.set(0.0);
            else if (currentValue > 0.45) forceZInPercentThresholdToFilterFoot.set(0.45);
         }
      });
      
      reinitialize.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            if (reinitialize.getBooleanValue())
               initialize();
         }
      });

      kinematicsBasedLinearStateCalculator = new PelvisKinematicsBasedLinearStateCalculator(inverseDynamicsStructure, feetContactablePlaneBodies, footSwitches,
            centerOfPressureDataHolderFromController, estimatorDT, yoGraphicsListRegistry, registry);
      kinematicsBasedLinearStateCalculator.setAlphaPelvisPosition(computeAlphaGivenBreakFrequencyProperly(stateEstimatorParameters.getKinematicsPelvisPositionFilterFreqInHertz(), estimatorDT));
      double alphaFilter = computeAlphaGivenBreakFrequencyProperly(stateEstimatorParameters.getKinematicsPelvisLinearVelocityFilterFreqInHertz(), estimatorDT);
      kinematicsBasedLinearStateCalculator.setPelvisLinearVelocityAlphaNewTwist(stateEstimatorParameters.getPelvisLinearVelocityAlphaNewTwist());
      kinematicsBasedLinearStateCalculator.setPelvisLinearVelocityBacklashParameters(alphaFilter, stateEstimatorParameters.getPelvisVelocityBacklashSlopTime());
      kinematicsBasedLinearStateCalculator.setTrustCoPAsNonSlippingContactPoint(stateEstimatorParameters.trustCoPAsNonSlippingContactPoint());
      kinematicsBasedLinearStateCalculator.useControllerDesiredCoP(stateEstimatorParameters.useControllerDesiredCenterOfPressure());
      kinematicsBasedLinearStateCalculator.setAlphaCenterOfPressure(computeAlphaGivenBreakFrequencyProperly(stateEstimatorParameters.getCoPFilterFreqInHertz(), estimatorDT));
      kinematicsBasedLinearStateCalculator.enableTwistEstimation(stateEstimatorParameters.useTwistForPelvisLinearStateEstimation());

      imuBasedLinearStateCalculator = new PelvisIMUBasedLinearStateCalculator(inverseDynamicsStructure, imuProcessedOutputs, estimatorDT, gravitationalAcceleration, registry);
      imuBasedLinearStateCalculator.cancelGravityFromAccelerationMeasurement(stateEstimatorParameters.cancelGravityFromAccelerationMeasurement());
      imuBasedLinearStateCalculator.enableEsimationModule(stateEstimatorParameters.useAccelerometerForEstimation());
      imuBasedLinearStateCalculator.enableAccelerationBiasEstimation(stateEstimatorParameters.estimateAccelerationBias());
      imuBasedLinearStateCalculator.setAlphaAccelerationBiasEstimation(computeAlphaGivenBreakFrequencyProperly(stateEstimatorParameters.getAccelerationBiasFilterFreqInHertz(), estimatorDT));

      alphaIMUAgainstKinematicsForVelocity.set(computeAlphaGivenBreakFrequencyProperly(stateEstimatorParameters.getPelvisLinearVelocityFusingFrequency(), estimatorDT));
      alphaIMUAgainstKinematicsForPosition.set(computeAlphaGivenBreakFrequencyProperly(stateEstimatorParameters.getPelvisPositionFusingFrequency(), estimatorDT));

      delayTimeBeforeTrustingFoot.set(stateEstimatorParameters.getDelayTimeForTrustingFoot());

      forceZInPercentThresholdToFilterFoot.set(stateEstimatorParameters.getForceInPercentOfWeightThresholdToTrustFoot()); 
      
      slippageCompensatorMode.set(SlippageCompensatorMode.LOAD_THRESHOLD);
      
//      requestStopEstimationOfPelvisLinearState.set(true);

      imuDriftCompensator = new IMUDriftCompensator(footFrames, inverseDynamicsStructure, footSwitches, estimatorDT, registry);
      imuDriftCompensator.activateEstimation(stateEstimatorParameters.estimateIMUDrift());
      imuDriftCompensator.activateCompensation(stateEstimatorParameters.compensateIMUDrift());
      imuDriftCompensator.setAlphaIMUDrift(computeAlphaGivenBreakFrequencyProperly(stateEstimatorParameters.getIMUDriftFilterFreqInHertz(), estimatorDT));
      imuDriftCompensator.setAlphaFootAngularVelocity(computeAlphaGivenBreakFrequencyProperly(stateEstimatorParameters.getFootVelocityUsedForImuDriftFilterFreqInHertz(), estimatorDT));
      imuDriftCompensator.setFootAngularVelocityThreshold(stateEstimatorParameters.getFootVelocityThresholdToEnableIMUDriftCompensation());

      if (VISUALIZE)
      {
         if (yoGraphicsListRegistry != null)
         {
            YoArtifactPosition comArtifact = new YoGraphicPosition("Meas CoM", yoCenterOfMassPosition, 0.006, YoAppearance.Black(), GraphicType.CROSS).createArtifact();
            yoGraphicsListRegistry.registerArtifact("StateEstimator", comArtifact);
         }
      }
      parentRegistry.addChild(registry);
   }

   private void setupBunchOfVariables()
   {
      int windowSize = (int)(delayTimeBeforeTrustingFoot.getDoubleValue() / estimatorDT);
      
      for (RigidBody foot : feet)
      {
         ReferenceFrame footFrame = feetContactablePlaneBodies.get(foot).getSoleFrame();
         footFrames.put(foot, footFrame);
         
         String footPrefix = foot.getName();
         
         final GlitchFilteredBooleanYoVariable hasFootHitTheGroundFiltered = new GlitchFilteredBooleanYoVariable("has" + footPrefix + "FootHitGroundFiltered", registry, windowSize);
         hasFootHitTheGroundFiltered.set(true);
         haveFeetHitGroundFiltered.put(foot, hasFootHitTheGroundFiltered);
         
         BooleanYoVariable isFootTrusted = new BooleanYoVariable("is" + footPrefix + "FootTrusted", registry);
         isFootTrusted.set(true);
         areFeetTrusted.put(foot, isFootTrusted);
         
         DoubleYoVariable footForceZInPercentOfTotalForce = new DoubleYoVariable(footPrefix + "FootForceZInPercentOfTotalForce", registry);
         footForcesZInPercentOfTotalForce.put(foot, footForceZInPercentOfTotalForce);
         
         footWrenches.put(foot, new Wrench());
         
         delayTimeBeforeTrustingFoot.addVariableChangedListener(new VariableChangedListener()
         {            
            @Override
            public void variableChanged(YoVariable<?> v)
            {
             int windowSize = (int)(delayTimeBeforeTrustingFoot.getDoubleValue() / estimatorDT);
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
      
      imuDriftCompensator.initialize();
      
      centerOfMassCalculator.compute();
      centerOfMassCalculator.packCenterOfMass(tempPosition);
      if (!initializeToActual && DRCKinematicsBasedStateEstimator.INITIALIZE_HEIGHT_WITH_FOOT)
      {
         RigidBody foot = feet.get(0);
         tempPosition.changeFrame(footFrames.get(foot));
         double footToCoMZ = tempPosition.getZ();
         tempPosition.changeFrame(worldFrame);
         tempPosition.setZ(tempPosition.getZ() - footToCoMZ);
      }
      tempFrameVector.setIncludingFrame(tempPosition);
      tempFrameVector.changeFrame(worldFrame);

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
      yoRootJointPosition.get(tempRootJointTranslation);
      rootJoint.setPosition(tempRootJointTranslation);

      // Set the rootJoint twist to zero.
      rootJoint.packJointTwist(rootJointTwist);
      rootJointTwist.setToZero();
      rootJoint.setJointTwist(rootJointTwist);

      rootJointFrame.update();
      twistCalculator.compute();

      updateCoMState();
   }

   public void updateRootJointPositionAndLinearVelocity()
   {
	   if (requestStopEstimationOfPelvisLinearState.getBooleanValue())
		   return;
	   
	   // adapted defaultActionIntoStates
	   imuDriftCompensator.updateAndCompensateDrift();
	   
	   kinematicsBasedLinearStateCalculator.updateKinematics();
	   
	   int numberOfEndEffectorsTrusted = setTrustedFeetUsingFootSwitches();

	   if (numberOfEndEffectorsTrusted >= 2)
	   {
	      switch(slippageCompensatorMode.getEnumValue())
	      {
	      case LOAD_THRESHOLD:
	         numberOfEndEffectorsTrusted = filterTrustedFeetBasedOnContactForces(numberOfEndEffectorsTrusted);
	         break;
	      case MIN_PELVIS_ACCEL:
	         throw new RuntimeException("Implement me if possible!");
	      default:
	         throw new RuntimeException("Should not get there");
	      }
	   }
	   
	   if (numberOfEndEffectorsTrusted == 0)
	   {
	      if (ALLOW_USING_IMU_ONLY)
	      {
            yoRootJointPosition.getFrameTuple(rootJointPosition);
            kinematicsBasedLinearStateCalculator.updateFeetPositionsWhenTrustingIMUOnly(rootJointPosition);	     
            
            imuBasedLinearStateCalculator.updateIMUAndRootJointLinearVelocity(rootJointVelocity);
            yoRootJointVelocity.set(rootJointVelocity);
            imuBasedLinearStateCalculator.correctIMULinearVelocity(rootJointVelocity);

            yoRootJointPosition.getFrameTuple(rootJointPosition);
            imuBasedLinearStateCalculator.updatePelvisPosition(rootJointPosition, pelvisPositionIMUPart);
            rootJointPosition.set(pelvisPositionIMUPart);
            yoRootJointPosition.set(rootJointPosition);
	      }
	      else
	         throw new RuntimeException("No foot trusted!");
	   }
	   else if (numberOfEndEffectorsTrusted > 0)
	   {
	      updateTrustedFeetLists();
         yoRootJointPosition.getFrameTuple(rootJointPosition);
	      kinematicsBasedLinearStateCalculator.estimatePelvisLinearState(listOfTrustedFeet, listOfUnTrustedFeet, rootJointPosition);
	      
	      if(numberOfEndEffectorsTrusted == 1)
	         imuDriftCompensator.esimtateDriftIfPossible(listOfTrustedFeet.get(0));
	      
	      if (imuBasedLinearStateCalculator.isEstimationEnabled())
	      {
	         computeLinearStateFromMergingMeasurements();
	      }
	      else 
	      {
	         yoRootJointPosition.getFrameTuple(rootJointPosition);
	         kinematicsBasedLinearStateCalculator.getRootJointPositionAndVelocity(rootJointPosition, rootJointVelocity);
	         yoRootJointPosition.set(rootJointPosition);
	         yoRootJointVelocity.set(rootJointVelocity);
	      }
	   }
	   else
	   {
	      throw new RuntimeException("Computation of the number of end effectors to be trusted for state estimation is broken, computed: " + numberOfEndEffectorsTrusted);
	   }
	   
	   updateRootJoint();
      
	   updateCoMState();
   }
   
   private Twist rootJointTwist = new Twist();

   private void updateRootJoint()
   {
      yoRootJointPosition.get(tempRootJointTranslation);
      rootJoint.setPosition(tempRootJointTranslation);

      tempVelocity.setIncludingFrame(rootJointVelocity);
      rootJoint.packJointTwist(rootJointTwist);
      tempVelocity.changeFrame(rootJointTwist.getExpressedInFrame());
      rootJointTwist.setLinearPart(tempVelocity);
      rootJoint.setJointTwist(rootJointTwist);

      rootJointFrame.update();
      twistCalculator.compute();
   }
   
   private int setTrustedFeetUsingFootSwitches()
   {
      int numberOfEndEffectorsTrusted = 0;
      
      for(RigidBody foot : feet)
      {
         if (footSwitches.get(foot).hasFootHitGround())
            haveFeetHitGroundFiltered.get(foot).set(true);
         else
            haveFeetHitGroundFiltered.get(foot).set(false);
         
         if (haveFeetHitGroundFiltered.get(foot).getBooleanValue())
            numberOfEndEffectorsTrusted++;
      }
      
      // Update only if at least one foot hit the ground
      if(numberOfEndEffectorsTrusted > 0)
      {
         for(RigidBody foot : feet)
            areFeetTrusted.get(foot).set(haveFeetHitGroundFiltered.get(foot).getBooleanValue());
      }
      
      // Else if there is a foot with a force past the threshold trust the force and not the CoP      
      else
      {
         RigidBody trustedFoot = null;
         
         for(RigidBody foot : feet)
         {
            if(footSwitches.get(foot).getForceMagnitudePastThreshhold())
            {
               trustedFoot = foot;
               numberOfEndEffectorsTrusted = 1;
               break;
            }
         }
         
         if(trustedFoot != null)
         {
            for(RigidBody foot : feet)
            {
               areFeetTrusted.get(foot).set(foot == trustedFoot);
            }            
         }
      }     
      
      if(numberOfEndEffectorsTrusted == 0)
      {
         if(ALLOW_USING_IMU_ONLY)
         {
            for(RigidBody foot : feet)
               areFeetTrusted.get(foot).set(false);
         }
         else
         {
            for (RigidBody foot : feet)
            {
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
      for (RigidBody foot : feet)
      {
         Wrench footWrench = footWrenches.get(foot);
         footSwitches.get(foot).computeAndPackFootWrench(footWrench);
         totalForceZ += footWrench.getLinearPartZ();
      }

      for (RigidBody foot : feet)
      {
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

   private final FrameVector pelvisVelocityIMUPart = new FrameVector();
   private final FramePoint pelvisPositionIMUPart = new FramePoint();

   private final FrameVector pelvisVelocityKinPart = new FrameVector();
   private final FramePoint pelvisPositionKinPart = new FramePoint();

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
      yoRootJointPosition.getFrameTuple(rootJointPosition);
      imuBasedLinearStateCalculator.updatePelvisPosition(rootJointPosition, pelvisPositionIMUPart);
      kinematicsBasedLinearStateCalculator.getPelvisPosition(pelvisPositionKinPart);

      pelvisPositionIMUPart.scale(alphaIMUAgainstKinematicsForPosition.getDoubleValue());
      pelvisPositionKinPart.scale(1.0 - alphaIMUAgainstKinematicsForPosition.getDoubleValue());

      rootJointPosition.set(pelvisPositionIMUPart);
      rootJointPosition.add(pelvisPositionKinPart);
      yoRootJointPosition.set(rootJointPosition);
   }

   private void updateCoMState()
   {
      centerOfMassCalculator.compute();
      centerOfMassCalculator.packCenterOfMass(centerOfMassPosition);
      centerOfMassPosition.changeFrame(worldFrame);
      yoCenterOfMassPosition.set(centerOfMassPosition);
      
      centerOfMassJacobianWorld.compute();
      centerOfMassVelocity.setToZero(ReferenceFrame.getWorldFrame());
      centerOfMassJacobianWorld.packCenterOfMassVelocity(centerOfMassVelocity);
      centerOfMassVelocity.changeFrame(ReferenceFrame.getWorldFrame());
      yoCenterOfMassVelocity.set(centerOfMassVelocity);
   }
   
   private void updateTrustedFeetLists()
   {
      listOfTrustedFeet.clear();
      listOfUnTrustedFeet.clear();
      
      for(RigidBody foot : feet)
      {
         if(areFeetTrusted.get(foot).getBooleanValue())
            listOfTrustedFeet.add(foot);
         else
            listOfUnTrustedFeet.add(foot);
      }
   }

   public void initializeCoMPositionToActual(Point3d initialCoMPosition)
   {
      initializeToActual = true;
      centerOfMassPosition.setIncludingFrame(worldFrame, initialCoMPosition);
      yoCenterOfMassPosition.set(initialCoMPosition);
   }

   public void initializeCoMPositionToActual(FramePoint initialCoMPosition)
   {
      initializeToActual = true;
      centerOfMassPosition.set(initialCoMPosition);
      yoCenterOfMassPosition.set(initialCoMPosition);
   }
   
   public void getEstimatedPelvisPosition(FramePoint pelvisPositionToPack)
   {
      yoRootJointPosition.getFrameTupleIncludingFrame(pelvisPositionToPack);
   }

   public void getEstimatedPelvisLinearVelocity(FrameVector pelvisLinearVelocityToPack)
   {
      yoRootJointVelocity.getFrameTupleIncludingFrame(pelvisLinearVelocityToPack);
   }

   public void getEstimatedCoMPosition(FramePoint comPositionToPack)
   {
      yoCenterOfMassPosition.getFrameTupleIncludingFrame(comPositionToPack);
   }

   public void getEstimatedCoMVelocity(FrameVector comVelocityToPack)
   {
      yoCenterOfMassVelocity.getFrameTupleIncludingFrame(comVelocityToPack);
   }   
}
