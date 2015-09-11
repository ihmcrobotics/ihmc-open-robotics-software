package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;


import static us.ihmc.robotics.math.filters.AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly;

import java.util.List;

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
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.CenterOfMassCalculator;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.stateMachines.State;
import us.ihmc.robotics.stateMachines.StateMachine;
import us.ihmc.robotics.stateMachines.StateMachineTools;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.plotting.YoArtifactPosition;


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

   private final YoFramePoint yoRootJointPosition = new YoFramePoint("estimatedRootJointPosition", worldFrame, registry);
   private final YoFrameVector yoRootJointVelocity = new YoFrameVector("estimatedRootJointVelocity", worldFrame, registry);
   
   private final YoFramePoint yoCenterOfMassPosition = new YoFramePoint("estimatedCenterOfMassPosition", worldFrame, registry);
   private final YoFrameVector yoCenterOfMassVelocity = new YoFrameVector("estimatedCenterOfMassVelocity", worldFrame, registry);

   private final DoubleYoVariable alphaIMUAgainstKinematicsForVelocity = new DoubleYoVariable("alphaIMUAgainstKinematicsForVelocity", registry);
   private final DoubleYoVariable alphaIMUAgainstKinematicsForPosition = new DoubleYoVariable("alphaIMUAgainstKinematicsForPosition", registry);
   
   private final SideDependentList<DoubleYoVariable> footForcesZInPercentOfTotalForce = new SideDependentList<DoubleYoVariable>();
   private final DoubleYoVariable forceZInPercentThresholdToFilterFoot = new DoubleYoVariable("forceZInPercentThresholdToFilterFootUserParameter", registry);

   private final SideDependentList<FootSwitchInterface> footSwitches;
   private final SideDependentList<Wrench> footWrenches = new SideDependentList<Wrench>(new Wrench(), new Wrench());
   private final DoubleYoVariable delayTimeBeforeTrustingFoot = new DoubleYoVariable("delayTimeBeforeTrustingFoot", registry);
   private final SideDependentList<GlitchFilteredBooleanYoVariable> haveFeetHitGroundFiltered = new SideDependentList<GlitchFilteredBooleanYoVariable>();
   private final SideDependentList<BooleanYoVariable> areFeetTrusted = new SideDependentList<BooleanYoVariable>();
   
   private final ReferenceFrame rootJointFrame;
   private final SideDependentList<ReferenceFrame> footFrames;
   
   private final double estimatorDT;
      
   private final SideDependentList<ContactablePlaneBody> bipedFeet;
   
   private final BooleanYoVariable reinitialize = new BooleanYoVariable("reinitialize", registry);

   private enum SlippageCompensatorMode {LOAD_THRESHOLD, MIN_PELVIS_ACCEL};
   private final EnumYoVariable<SlippageCompensatorMode> slippageCompensatorMode = new EnumYoVariable<SlippageCompensatorMode>("slippageCompensatorMode", registry, SlippageCompensatorMode.class);
   
   private enum EstimationState {TRUST_BOTH_FEET, TRUST_LEFT_FOOT, TRUST_RIGHT_FOOT, IMU_ONLY};

   private final SideDependentList<EstimationState> robotSideToEstimationState = new SideDependentList<EstimationState>(
         EstimationState.TRUST_LEFT_FOOT, EstimationState.TRUST_RIGHT_FOOT);
   
   private final EnumYoVariable<EstimationState> requestedState = new EnumYoVariable<EstimationState>("requestedEstimationState", "", registry, EstimationState.class, true);
   private final BooleanYoVariable requestStopEstimationOfPelvisLinearState = new BooleanYoVariable("userRequestStopEstimationOfPelvisLinearState", registry);
   
   private final StateMachine<EstimationState> stateMachine;

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
         SideDependentList<FootSwitchInterface> footSwitches, CenterOfPressureDataHolder centerOfPressureDataHolderFromController,
         SideDependentList<ContactablePlaneBody> bipedFeet, double gravitationalAcceleration, DoubleYoVariable yoTime,
         StateEstimatorParameters stateEstimatorParameters, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.estimatorDT = stateEstimatorParameters.getEstimatorDT();
      this.footSwitches = footSwitches;
      this.bipedFeet = bipedFeet;
      
      twistCalculator = inverseDynamicsStructure.getTwistCalculator();
      rootJoint = inverseDynamicsStructure.getRootJoint();
      rootJointFrame = rootJoint.getFrameAfterJoint();
      
      footFrames = new SideDependentList<ReferenceFrame>();
      
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

      stateMachine = new StateMachine<EstimationState>("LinearEstimationStateMachine", "switchTime", EstimationState.class, yoTime, registry);
      setupStateMachine();
      
      kinematicsBasedLinearStateCalculator = new PelvisKinematicsBasedLinearStateCalculator(inverseDynamicsStructure, bipedFeet, footSwitches,
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
      imuBasedLinearStateCalculator.enableEsimationModule(stateEstimatorParameters.useAccelerometerForEstimation());
      imuBasedLinearStateCalculator.enableGravityEstimation(stateEstimatorParameters.estimateGravity());
      imuBasedLinearStateCalculator.setAlphaGravityEstimation(computeAlphaGivenBreakFrequencyProperly(stateEstimatorParameters.getGravityFilterFreqInHertz(), estimatorDT));

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
      
      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame footFrame = bipedFeet.get(robotSide).getSoleFrame();
         footFrames.put(robotSide, footFrame);
         
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         
         final GlitchFilteredBooleanYoVariable hasFootHitTheGroundFiltered = new GlitchFilteredBooleanYoVariable("has" + robotSide.getCamelCaseNameForMiddleOfExpression() + "FootHitGroundFiltered", registry, windowSize);
         hasFootHitTheGroundFiltered.set(true);
         haveFeetHitGroundFiltered.put(robotSide, hasFootHitTheGroundFiltered);
         
         BooleanYoVariable isFootTrusted = new BooleanYoVariable("is" + robotSide.getCamelCaseNameForMiddleOfExpression() + "FootTrusted", registry);
         isFootTrusted.set(true);
         areFeetTrusted.put(robotSide, isFootTrusted);
         
         DoubleYoVariable footForceZInPercentOfTotalForce = new DoubleYoVariable(sidePrefix + "FootForceZInPercentOfTotalForce", registry);
         footForcesZInPercentOfTotalForce.put(robotSide, footForceZInPercentOfTotalForce);
         
         delayTimeBeforeTrustingFoot.addVariableChangedListener(new VariableChangedListener()
         {
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
         tempPosition.changeFrame(footFrames.get(RobotSide.LEFT));
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
   
   private void setupStateMachine()
   {
      TrustBothFeetState trustBothFeetState = new TrustBothFeetState();
      TrustOneFootState trustLeftFootState = new TrustOneFootState(RobotSide.LEFT);
      TrustOneFootState trustRightFootState = new TrustOneFootState(RobotSide.RIGHT);

      StateMachineTools.addRequestedStateTransition(requestedState, false, trustBothFeetState, trustLeftFootState);
      StateMachineTools.addRequestedStateTransition(requestedState, false, trustBothFeetState, trustRightFootState);

      StateMachineTools.addRequestedStateTransition(requestedState, false, trustLeftFootState, trustBothFeetState);
      StateMachineTools.addRequestedStateTransition(requestedState, false, trustRightFootState, trustBothFeetState);

      StateMachineTools.addRequestedStateTransition(requestedState, false, trustLeftFootState, trustRightFootState);
      StateMachineTools.addRequestedStateTransition(requestedState, false, trustRightFootState, trustLeftFootState);
      
      stateMachine.addState(trustBothFeetState);
      stateMachine.addState(trustLeftFootState);
      stateMachine.addState(trustRightFootState);

      if (ALLOW_USING_IMU_ONLY)
      {
         TrustIMUOnlyState trustIMUOnlyState = new TrustIMUOnlyState();

         StateMachineTools.addRequestedStateTransition(requestedState, false, trustBothFeetState, trustIMUOnlyState);
         StateMachineTools.addRequestedStateTransition(requestedState, false, trustLeftFootState, trustIMUOnlyState);
         StateMachineTools.addRequestedStateTransition(requestedState, false, trustRightFootState, trustIMUOnlyState);

         StateMachineTools.addRequestedStateTransition(requestedState, false, trustIMUOnlyState, trustBothFeetState);
         StateMachineTools.addRequestedStateTransition(requestedState, false, trustIMUOnlyState, trustLeftFootState);
         StateMachineTools.addRequestedStateTransition(requestedState, false, trustIMUOnlyState, trustRightFootState);

         stateMachine.addState(trustIMUOnlyState);
      }
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

      defaultActionIntoStates();
      
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
      
      defaultActionOutOfStates();

      updateRootJoint();
      updateCoMState();
   }

   private void defaultActionIntoStates()
   {
      imuDriftCompensator.updateAndCompensateDrift();
      
      kinematicsBasedLinearStateCalculator.updateKinematics();
      
      int numberOfEndEffectorsTrusted = setTrustedFeetUsingFootSwitches();

      if (numberOfEndEffectorsTrusted >= 2)
      {
         switch(slippageCompensatorMode.getEnumValue())
         {
         case LOAD_THRESHOLD:
            numberOfEndEffectorsTrusted = filterTrustedFeetBasedOnContactForces();
            break;
         case MIN_PELVIS_ACCEL:
            throw new RuntimeException("Implement me if possible!");
         default:
            throw new RuntimeException("Should not get there");
         }
      }
      
      if (numberOfEndEffectorsTrusted == 2)
      {
         requestedState.set(EstimationState.TRUST_BOTH_FEET);
      }
      else if (numberOfEndEffectorsTrusted == 1)
      {
         if (areFeetTrusted.get(RobotSide.LEFT).getBooleanValue())
            requestedState.set(EstimationState.TRUST_LEFT_FOOT);
         else
            requestedState.set(EstimationState.TRUST_RIGHT_FOOT);
      }
      else if (numberOfEndEffectorsTrusted == 0)
      {
         if (ALLOW_USING_IMU_ONLY)
            requestedState.set(EstimationState.IMU_ONLY);
         else
            throw new RuntimeException("No foot trusted!");
      }
      else
      {
         throw new RuntimeException("Computation of the number of end effectors to be trusted for state estimation is broken, computed: " + numberOfEndEffectorsTrusted);
      }
      
      if (stateMachine.getCurrentStateEnum() == requestedState.getEnumValue())
         requestedState.set(null);
   }

   private void defaultActionOutOfStates()
   {
      if (stateMachine.isCurrentState(EstimationState.IMU_ONLY))
      {
         if (!ALLOW_USING_IMU_ONLY)
            throw new RuntimeException("Should not be in IMU_ONLY state with ALLOW_USING_IMU_ONLY set to false.");

         imuBasedLinearStateCalculator.updateIMUAndRootJointLinearVelocity(rootJointVelocity);
         yoRootJointVelocity.set(rootJointVelocity);
         imuBasedLinearStateCalculator.correctIMULinearVelocity(rootJointVelocity);

         yoRootJointPosition.getFrameTuple(rootJointPosition);
         imuBasedLinearStateCalculator.updatePelvisPosition(rootJointPosition, pelvisPositionIMUPart);
         rootJointPosition.set(pelvisPositionIMUPart);
         yoRootJointPosition.set(rootJointPosition);
      }
      else if (imuBasedLinearStateCalculator.isEstimationEnabled())
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

      for (RobotSide robotSide : RobotSide.values)
      {
         if (footSwitches.get(robotSide).hasFootHitGround())
            haveFeetHitGroundFiltered.get(robotSide).update(true);
         else
            haveFeetHitGroundFiltered.get(robotSide).set(false);

         if (haveFeetHitGroundFiltered.get(robotSide).getBooleanValue())
            numberOfEndEffectorsTrusted++;
      }

      // Update only if at least one foot hit the ground
      if (numberOfEndEffectorsTrusted > 0)
      {
         for (RobotSide robotSide : RobotSide.values)
            areFeetTrusted.get(robotSide).set(haveFeetHitGroundFiltered.get(robotSide).getBooleanValue());
      }
      // Else if there is a foot with a force past the threshold trust the force and not the CoP
      else if (footSwitches.get(RobotSide.LEFT).getForceMagnitudePastThreshhold())
      {
         areFeetTrusted.get(RobotSide.LEFT).set(true);
         areFeetTrusted.get(RobotSide.RIGHT).set(false);
         numberOfEndEffectorsTrusted = 1;
      }
      else if (footSwitches.get(RobotSide.RIGHT).getForceMagnitudePastThreshhold())
      {
         areFeetTrusted.get(RobotSide.LEFT).set(false);
         areFeetTrusted.get(RobotSide.RIGHT).set(true);
         numberOfEndEffectorsTrusted = 1;
      }
      else if (ALLOW_USING_IMU_ONLY)
      {
         areFeetTrusted.get(RobotSide.LEFT).set(false);
         areFeetTrusted.get(RobotSide.RIGHT).set(false);
         numberOfEndEffectorsTrusted = 0;
      }
      // Else keep the old states
      else
      {
         numberOfEndEffectorsTrusted = 0;
         for (RobotSide robotSide : RobotSide.values)
         {
            if (areFeetTrusted.get(robotSide).getBooleanValue())
               numberOfEndEffectorsTrusted++;
         }
      }

      return numberOfEndEffectorsTrusted;
   }
   
   private int filterTrustedFeetBasedOnContactForces()
   {
      int numberOfEndEffectorsTrusted = 2;

      double totalForceZ = 0.0;
      for (RobotSide robotSide : RobotSide.values)
      {
         Wrench footWrench = footWrenches.get(robotSide);
         footSwitches.get(robotSide).computeAndPackFootWrench(footWrench);
         totalForceZ += footWrench.getLinearPartZ();
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         Wrench footWrench = footWrenches.get(robotSide);
         footForcesZInPercentOfTotalForce.get(robotSide).set(footWrench.getLinearPartZ() / totalForceZ);

         if (footForcesZInPercentOfTotalForce.get(robotSide).getDoubleValue() < forceZInPercentThresholdToFilterFoot.getDoubleValue())
         {
            numberOfEndEffectorsTrusted--;
            areFeetTrusted.get(robotSide).set(false);

            return numberOfEndEffectorsTrusted;
         }
      }

      return numberOfEndEffectorsTrusted;
   }

   private class TrustBothFeetState extends State<EstimationState>
   {
      public TrustBothFeetState()
      {
         super(EstimationState.TRUST_BOTH_FEET);
      }

      @Override
      public void doAction()
      {
         kinematicsBasedLinearStateCalculator.estimatePelvisLinearStateForDoubleSupport();
         imuDriftCompensator.esimtateDriftIfPossible(null);
      }

      @Override
      public void doTransitionIntoAction()
      {
         imuDriftCompensator.resetFootAngularVelocitiesFiltered();
         requestedState.set(null);
      }

      @Override
      public void doTransitionOutOfAction()
      {
      }
   }

   private class TrustOneFootState extends State<EstimationState>
   {
      private final RobotSide trustedSide;
      private final FramePoint tempRootJointPosition = new FramePoint(worldFrame);

      public TrustOneFootState(RobotSide trustedSide)
      {
         super(robotSideToEstimationState.get(trustedSide));
         this.trustedSide = trustedSide;
      }

      @Override
      public void doAction()
      {
         yoRootJointPosition.getFrameTuple(tempRootJointPosition);
         kinematicsBasedLinearStateCalculator.estimatePelvisLinearStateForSingleSupport(tempRootJointPosition, trustedSide);
         imuDriftCompensator.esimtateDriftIfPossible(trustedSide);
      }
      
      @Override
      public void doTransitionIntoAction()
      {
         imuDriftCompensator.resetFootAngularVelocitiesFiltered();
         requestedState.set(null);
      }

      @Override
      public void doTransitionOutOfAction()
      {
      }
   }

   private class TrustIMUOnlyState extends State<EstimationState>
   {
      private final FramePoint tempRootJointPosition = new FramePoint(worldFrame);

      public TrustIMUOnlyState()
      {
         super(EstimationState.IMU_ONLY);
      }

      @Override
      public void doAction()
      {
         yoRootJointPosition.getFrameTuple(tempRootJointPosition);
         kinematicsBasedLinearStateCalculator.updateFeetPositionsWhenTrustingIMUOnly(tempRootJointPosition);
      }
      
      @Override
      public void doTransitionIntoAction()
      {
         requestedState.set(null);
      }

      @Override
      public void doTransitionOutOfAction()
      {
      }
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
