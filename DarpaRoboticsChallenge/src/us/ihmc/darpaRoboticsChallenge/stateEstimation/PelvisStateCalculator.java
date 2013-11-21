package us.ihmc.darpaRoboticsChallenge.stateEstimation;


import java.util.Collection;

import javax.vecmath.Point2d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.darpaRoboticsChallenge.sensors.WrenchBasedFootSwitch;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.stateEstimation.JointAndIMUSensorDataSource;
import us.ihmc.sensorProcessing.stateEstimation.SimplePositionStateCalculatorInterface;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimationDataFromController;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameLine2d;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CenterOfMassCalculator;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.VariableChangedListener;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoFramePoint2d;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoFrameVector;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoVariable;
import com.yobotics.simulationconstructionset.util.math.filter.BacklashCompensatingVelocityYoFrameVector;
import com.yobotics.simulationconstructionset.util.math.filter.FilteredVelocityYoFrameVector;
import com.yobotics.simulationconstructionset.util.math.filter.GlitchFilteredBooleanYoVariable;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint2d;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.statemachines.State;
import com.yobotics.simulationconstructionset.util.statemachines.StateMachine;
import com.yobotics.simulationconstructionset.util.statemachines.StateMachineTools;

public class PelvisStateCalculator implements SimplePositionStateCalculatorInterface
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final boolean VISUALIZE = true;
   
   private final YoFrameVector imuAccelerationInWorld = new YoFrameVector("imuAccelerationInWorld", worldFrame, registry);
   private final YoFrameVector pelvisVelocityByIntegrating = new YoFrameVector("pelvisVelocityByIntegrating", worldFrame, registry);  

   private final DoubleYoVariable delayTimeBeforeTrustingFoot = new DoubleYoVariable("delayTimeBeforeTrustingFoot", registry);

   private final CenterOfMassCalculator centerOfMassCalculator;
   private final CenterOfMassJacobian centerOfMassJacobianBody;

   private final YoFramePoint pelvisPosition = new YoFramePoint("estimatedPelvisPosition", worldFrame, registry);
   private final YoFrameVector pelvisVelocity = new YoFrameVector("estimatedPelvisVelocity", worldFrame, registry);
   
   private final YoFramePoint centerOfMassPosition = new YoFramePoint("estimatedCenterOfMassPosition", worldFrame, registry);
   private final YoFrameVector centerOfMassVelocity = new YoFrameVector("estimatedCenterOfMassVelocity", worldFrame, registry);

   private final YoFramePoint pelvisPositionKinematics = new YoFramePoint("pelvisPositionKinematics", worldFrame, registry);   
   private final YoFrameVector pelvisVelocityKinematics = new YoFrameVector("pelvisVelocityKinematics", worldFrame, registry);
   
   private final DoubleYoVariable alphaPelvisVelocityBacklashKinematics = new DoubleYoVariable("alphaPelvisVelocityBacklashKinematics", registry);
   private final DoubleYoVariable slopTimePelvisVelocityBacklashKinematics = new DoubleYoVariable("slopTimePelvisVelocityBacklashKinematics", registry);
   
   private final BacklashCompensatingVelocityYoFrameVector pelvisVelocityBacklashKinematics;
   
   private final YoFrameVector pelvisVelocityTwist = new YoFrameVector("pelvisVelocityTwist", worldFrame, registry);
   private final BooleanYoVariable useTwistToComputePelvisVelocity = new BooleanYoVariable("useTwistToComputePelvisVelocity", registry);
   
   private final DoubleYoVariable alphaPelvisAccelerometerIntegrationToVelocity = new DoubleYoVariable("alphaPelvisAccelerometerIntegrationToVelocity", registry);
   private final DoubleYoVariable alphaPelvisAccelerometerIntegrationToPosition = new DoubleYoVariable("alphaPelvisAccelerometerIntegrationToPosition", registry);
   
   private final DoubleYoVariable alphaFootToPelvisPosition = new DoubleYoVariable("alphaFootToPelvisPosition", registry);
   private final DoubleYoVariable alphaFootToPelvisVelocity = new DoubleYoVariable("alphaFootToPelvisVelocity", registry);
   private final DoubleYoVariable alphaFootToPelvisAccel = new DoubleYoVariable("alphaFootToPelvisAcceleration", registry);
   
   private final DoubleYoVariable alphaCoPFilter = new DoubleYoVariable("alphaCoPFilter", registry);
   private final SideDependentList<AlphaFilteredYoFramePoint2d> copsFilteredInFootFrame;
   private final SideDependentList<YoFramePoint2d> copsRawInFootFrame;
   
   private final SideDependentList<DoubleYoVariable> footForcesZInPercentOfTotalForce = new SideDependentList<DoubleYoVariable>();
   private final DoubleYoVariable forceZInPercentThresholdToFilterFoot = new DoubleYoVariable("forceZInPercentThresholdToFilterFootUserParameter", registry);

   private final SideDependentList<AlphaFilteredYoFrameVector> footToPelvisPositions;
   private final SideDependentList<FilteredVelocityYoFrameVector> footToPelvisVelocities;
   private final SideDependentList<FilteredVelocityYoFrameVector> footToPelvisAccels;
   private final SideDependentList<Twist> pelvisToFootTwists = new SideDependentList<Twist>(new Twist(), new Twist());

   private final DoubleYoVariable feetSpacing = new DoubleYoVariable("estimatedFeetSpacing", registry);
   private final DoubleYoVariable alphaFeetSpacingVelocity = new DoubleYoVariable("alphaFeetSpacingVelocity", registry);
   private final AlphaFilteredYoVariable feetSpacingVelocity = new AlphaFilteredYoVariable("estimatedFeetSpacingVelocity", registry, alphaFeetSpacingVelocity);
   private final DoubleYoVariable footVelocityThreshold = new DoubleYoVariable("footVelocityThresholdToFilterTrustedFoot", registry);
   
   private final SideDependentList<YoFramePoint> footPositionsInWorld = new SideDependentList<YoFramePoint>();

   private final SideDependentList<YoFramePoint> copPositionsInWorld = new SideDependentList<YoFramePoint>();
   
   private final SideDependentList<WrenchBasedFootSwitch> footSwitches;
   
   private final SideDependentList<GlitchFilteredBooleanYoVariable> haveFeetHitGroundFiltered = new SideDependentList<GlitchFilteredBooleanYoVariable>();

   private final SideDependentList<BooleanYoVariable> areFeetTrusted = new SideDependentList<BooleanYoVariable>();
   
   private final ReferenceFrame pelvisFrame;
   private final SideDependentList<ReferenceFrame> footFrames;
   
   private final RigidBody pelvis;
   
   private final SideDependentList<Wrench> footWrenches = new SideDependentList<Wrench>(new Wrench(), new Wrench());
   private ControlFlowOutputPort<Vector3d> linearAccelerationPort;

   private final double estimatorDT;
   
   private final DoubleYoVariable alphaGravityEstimation = new DoubleYoVariable("alphaGravityEstimation", registry);
   private final AlphaFilteredYoVariable gravityEstimation = new AlphaFilteredYoVariable("gravityEstimation", registry, alphaGravityEstimation);

   private final SideDependentList<ContactablePlaneBody> bipedFeet;
   
   // temporary variables
   private final FramePoint tempFramePoint = new FramePoint();
   private final FrameVector tempFrameVector = new FrameVector();
   private final FramePoint tempPosition = new FramePoint();
   private final FrameVector tempVelocity = new FrameVector();

   private final BooleanYoVariable reinitialize = new BooleanYoVariable("reinitialize", registry);

   private final SideDependentList<FrameConvexPolygon2d> footPolygons = new SideDependentList<FrameConvexPolygon2d>();
   private final SideDependentList<FrameLine2d> ankleToCoPLines = new SideDependentList<FrameLine2d>();
   private final SideDependentList<FrameLineSegment2d> tempLineSegments = new SideDependentList<FrameLineSegment2d>();
   
   private enum SlippageCompensatorMode {LOAD_THRESHOLD, MIN_PELVIS_ACCEL};
   private final EnumYoVariable<SlippageCompensatorMode> slippageCompensatorMode = new EnumYoVariable<SlippageCompensatorMode>("slippageCompensatorMode", registry, SlippageCompensatorMode.class);
   
   private enum PelvisEstimationState {TRUST_BOTH_FEET, TRUST_LEFT_FOOT, TRUST_RIGHT_FOOT};

   private final SideDependentList<PelvisEstimationState> robotSideToPelvisEstimationState = new SideDependentList<PelvisStateCalculator.PelvisEstimationState>(
         PelvisEstimationState.TRUST_LEFT_FOOT, PelvisEstimationState.TRUST_RIGHT_FOOT);
   
   private final DoubleYoVariable yoTime = new DoubleYoVariable("t_pelvisStateEstimator", registry);
   
   private final EnumYoVariable<PelvisEstimationState> requestedState = new EnumYoVariable<>("requestedPelvisEstimationState", "", registry, PelvisEstimationState.class, true);
   
   private final StateMachine<PelvisEstimationState> stateMachine;

   private final TwistCalculator twistCalculator;
   
   public PelvisStateCalculator(StateEstimationDataFromController stateEstimatorDataFromControllerSource, SDFFullRobotModel estimatorModel,
         FullInverseDynamicsStructure inverseDynamicsStructure, SideDependentList<WrenchBasedFootSwitch> footSwitches,
         SideDependentList<ContactablePlaneBody> bipedFeet, double gravitationalAcceleration, final double estimatorDT,
         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, YoVariableRegistry parentRegistry)
   {      
      this.estimatorDT = estimatorDT;
      this.footSwitches = footSwitches;
      this.bipedFeet = bipedFeet;
      
      alphaPelvisVelocityBacklashKinematics.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(16.0, estimatorDT));
      slopTimePelvisVelocityBacklashKinematics.set(0.03);
      pelvisVelocityBacklashKinematics = BacklashCompensatingVelocityYoFrameVector.createBacklashCompensatingVelocityYoFrameVector("pelvisVelocityBacklashKin", "", 
            alphaPelvisVelocityBacklashKinematics, estimatorDT, slopTimePelvisVelocityBacklashKinematics, registry, pelvisPositionKinematics);

      twistCalculator = inverseDynamicsStructure.getTwistCalculator();
      
      gravityEstimation.reset();
      gravityEstimation.update(Math.abs(gravitationalAcceleration));
      
      pelvisFrame = estimatorModel.getRootJoint().getFrameAfterJoint();
      footFrames = new SideDependentList<ReferenceFrame>();
      
      
      RigidBody elevator = inverseDynamicsStructure.getElevator();
      pelvis = inverseDynamicsStructure.getRootJoint().getSuccessor();
      this.centerOfMassCalculator = new CenterOfMassCalculator(elevator, pelvisFrame);
      this.centerOfMassJacobianBody = new CenterOfMassJacobian(ScrewTools.computeSupportAndSubtreeSuccessors(elevator),
              ScrewTools.computeSubtreeJoints(pelvis), pelvisFrame);

      footToPelvisPositions = new SideDependentList<AlphaFilteredYoFrameVector>();
      footToPelvisVelocities = new SideDependentList<FilteredVelocityYoFrameVector>();
      footToPelvisAccels = new SideDependentList<FilteredVelocityYoFrameVector>();
      
      copsRawInFootFrame = new SideDependentList<YoFramePoint2d>();
      copsFilteredInFootFrame = new SideDependentList<AlphaFilteredYoFramePoint2d>();
      
      setupBunchOfVariables();
      
      forceZInPercentThresholdToFilterFoot.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable v)
         {
            double valueClipped = MathTools.clipToMinMax(forceZInPercentThresholdToFilterFoot.getDoubleValue(), 0.0, 0.45);
            forceZInPercentThresholdToFilterFoot.set(valueClipped);
         }
      });
      
      reinitialize.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable v)
         {
            if (reinitialize.getBooleanValue())
               initialize();
         }
      });

      stateMachine = new StateMachine<PelvisEstimationState>("PelvisEstimationStateMachine", "switchTime", PelvisEstimationState.class, yoTime, registry);
      setupStateMachine();
      
      parentRegistry.addChild(registry);

      if (VISUALIZE && dynamicGraphicObjectsListRegistry != null)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
            DynamicGraphicPosition copInWorld = new DynamicGraphicPosition(sidePrefix + "StateEstimatorCoP", copPositionsInWorld.get(robotSide), 0.005, YoAppearance.DeepPink());
            dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("StateEstimatorCoP", copInWorld);
            dynamicGraphicObjectsListRegistry.registerArtifact("StateEstimatorCoP", copInWorld.createArtifact());
//            DynamicGraphicVector footToPelvis = new DynamicGraphicVector(sidePrefix + "FootToPelvis", footPositionsInWorld.get(robotSide), footToPelvisPositions.get(robotSide), 1.0, YoAppearance.Blue());
//            dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("footToPelvis", footToPelvis);
         }
      }
   }

   private void setupBunchOfVariables()
   {
      int windowSize = (int)(delayTimeBeforeTrustingFoot.getDoubleValue() / estimatorDT);
      
      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame footFrame = bipedFeet.get(robotSide).getPlaneFrame();
         footFrames.put(robotSide, footFrame);
         
         FrameConvexPolygon2d footPolygon = new FrameConvexPolygon2d(bipedFeet.get(robotSide).getContactPoints2d());
         footPolygons.put(robotSide, footPolygon);
         
         FrameLine2d ankleToCoPLine = new FrameLine2d(footFrame, new Point2d(), new Point2d(1.0, 0.0));
         ankleToCoPLines.put(robotSide, ankleToCoPLine);
         
         FrameLineSegment2d tempLineSegment = new FrameLineSegment2d(new FramePoint2d(footFrame), new FramePoint2d(footFrame, 1.0, 1.0)); // TODO need to give distinct points that's not convenient
         tempLineSegments.put(robotSide, tempLineSegment);
         
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         
         AlphaFilteredYoFrameVector footToPelvisPosition = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector(sidePrefix + "FootToPelvisPosition", "", registry, alphaFootToPelvisPosition, worldFrame);
         footToPelvisPositions.set(robotSide, footToPelvisPosition);
         
         FilteredVelocityYoFrameVector footToPelvisVelocity = FilteredVelocityYoFrameVector.createFilteredVelocityYoFrameVector(sidePrefix + "FootToPelvisVelocity", "", alphaFootToPelvisVelocity, estimatorDT, registry, footToPelvisPosition);
         footToPelvisVelocities.set(robotSide, footToPelvisVelocity);
         
         FilteredVelocityYoFrameVector footToPelvisAccel = FilteredVelocityYoFrameVector.createFilteredVelocityYoFrameVector(sidePrefix + "FootToPelvisAcceleration", "", alphaFootToPelvisAccel, estimatorDT, registry, footToPelvisVelocity);
         footToPelvisAccels.set(robotSide, footToPelvisAccel);
         
         YoFramePoint footPositionInWorld = new YoFramePoint(sidePrefix + "FootPositionsInWorld", worldFrame, registry);
         footPositionsInWorld.put(robotSide, footPositionInWorld);
         
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
            public void variableChanged(YoVariable v)
            {
               int windowSize = (int)(delayTimeBeforeTrustingFoot.getDoubleValue() / estimatorDT);
               hasFootHitTheGroundFiltered.setWindowSize(windowSize);
            }
         });

         YoFramePoint2d copRawInFootFrame = new YoFramePoint2d(sidePrefix + "CoPRawInFootFrame", footFrames.get(robotSide), registry);
         copsRawInFootFrame.put(robotSide, copRawInFootFrame);
         
         AlphaFilteredYoFramePoint2d copFilteredInFootFrame = AlphaFilteredYoFramePoint2d.createAlphaFilteredYoFramePoint2d(sidePrefix + "CoPFilteredInFootFrame", "", registry, alphaCoPFilter, copRawInFootFrame);
         copFilteredInFootFrame.update(0.0, 0.0); // So the next point will be filtered
         copsFilteredInFootFrame.put(robotSide, copFilteredInFootFrame);

         YoFramePoint copPositionInWorld = new YoFramePoint(sidePrefix + "CoPPositionsInWorld", worldFrame, registry);
         copPositionsInWorld.put(robotSide, copPositionInWorld);
      }
   }

   public void initialize()
   {
      alphaGravityEstimation.set(0.99999);
      
      alphaFootToPelvisPosition.set(0.0); 
      alphaFootToPelvisVelocity.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(16.0, estimatorDT)); //0.85);
      
      alphaPelvisAccelerometerIntegrationToVelocity.set(0.992);
      alphaPelvisAccelerometerIntegrationToPosition.set(0.8);
      
      alphaCoPFilter.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(4.0, estimatorDT));
      
      delayTimeBeforeTrustingFoot.set(0.02);

      footVelocityThreshold.set(Double.MAX_VALUE); // 0.01);

      forceZInPercentThresholdToFilterFoot.set(0.3); 
      
      slippageCompensatorMode.set(SlippageCompensatorMode.LOAD_THRESHOLD);
      
      initializeRobotState();
   }

   public void initializeRobotState()
   {
      reinitialize.set(false);
      
      updateKinematics();

      centerOfMassCalculator.compute();
      centerOfMassCalculator.packCenterOfMass(tempPosition);
      tempFrameVector.setAndChangeFrame(tempPosition);
      tempFrameVector.changeFrame(worldFrame);

      pelvisPosition.set(centerOfMassPosition);
      pelvisPosition.sub(tempFrameVector);

      pelvisPositionKinematics.set(pelvisPosition);
      pelvisVelocity.setToZero();
      
      for(RobotSide robotSide : RobotSide.values)
         updateFootPosition(robotSide);
   }

   @Override
   public void initializeCoMPositionToActual(FramePoint estimatedCoMPosition)
   {
      centerOfMassPosition.set(estimatedCoMPosition);
   }
   
   @SuppressWarnings("unchecked")
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
   }
   
   private void defaultActionIntoStates()
   {
      yoTime.add(estimatorDT); //Hack to have a yoTime for the state machine
      
      pelvisPositionKinematics.setToZero();
      pelvisVelocityKinematics.setToZero();
      pelvisVelocityTwist.setToZero();

      updateKinematics();
      
      updatePelvisAccelerationWithIMU();
      
      int numberOfEndEffectorsTrusted = setTrustedFeetUsingFootSwitches();

      computeFeetSpacingLengthAndVelocity();

      if (numberOfEndEffectorsTrusted >= 2)
      {
         switch(slippageCompensatorMode.getEnumValue())
         {
         case LOAD_THRESHOLD:
            numberOfEndEffectorsTrusted = filterTrustedFeetBasedOnContactForces();
            break;
         case MIN_PELVIS_ACCEL:
            numberOfEndEffectorsTrusted = filterTrustedFeetBasedOnMinPelvisAcceleration();
            break;
         default:
            throw new RuntimeException("Should not get there");
         }
      }
      
      if (numberOfEndEffectorsTrusted == 2)
         requestedState.set(PelvisEstimationState.TRUST_BOTH_FEET);
      else if (areFeetTrusted.get(RobotSide.LEFT).getBooleanValue())
         requestedState.set(PelvisEstimationState.TRUST_LEFT_FOOT);
      else
         requestedState.set(PelvisEstimationState.TRUST_RIGHT_FOOT);
      
      if (stateMachine.getCurrentStateEnum() == requestedState.getEnumValue())
         requestedState.set(null);

      if (numberOfEndEffectorsTrusted == 0)
         throw new RuntimeException("No foot trusted!");
}

   private void updateKinematics()
   {
      twistCalculator.compute();
      
      for(RobotSide robotSide : RobotSide.values)
      {
         tempFramePoint.setToZero(pelvisFrame);
         tempFramePoint.changeFrame(footFrames.get(robotSide));

         tempFrameVector.setAndChangeFrame(tempFramePoint);
         tempFrameVector.changeFrame(worldFrame);

         footToPelvisPositions.get(robotSide).update(tempFrameVector);
         footToPelvisVelocities.get(robotSide).update();
         footToPelvisAccels.get(robotSide).update();
         
         Twist pelvisToFootTwist = pelvisToFootTwists.get(robotSide);
         twistCalculator.packRelativeTwist(pelvisToFootTwist, pelvis, bipedFeet.get(robotSide).getRigidBody());
      }
   }

   private void updatePelvisAccelerationWithIMU()
   {
      if (linearAccelerationPort == null)
         return;
      
      tempIMUAcceleration.set(pelvisFrame, linearAccelerationPort.getData());
      gravityEstimation.update(tempIMUAcceleration.length());
      
      tempIMUAcceleration.changeFrame(worldFrame);
      tempIMUAcceleration.setZ(tempIMUAcceleration.getZ() - gravityEstimation.getDoubleValue());
      imuAccelerationInWorld.set(tempIMUAcceleration);
   }

   private int setTrustedFeetUsingFootSwitches()
   {
      int numberOfEndEffectorsTrusted = 0;
      
      for(RobotSide robotSide : RobotSide.values)
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
         for(RobotSide robotSide : RobotSide.values)
            areFeetTrusted.get(robotSide).set(haveFeetHitGroundFiltered.get(robotSide).getBooleanValue());
      }
      // Else keep the old states
      else
      {
         numberOfEndEffectorsTrusted = 0;
         for(RobotSide robotSide : RobotSide.values)
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

   private final SideDependentList<Double> accelerationMagnitudeErrors = new SideDependentList<Double>(0.0, 0.0);
   
   private int filterTrustedFeetBasedOnMinPelvisAcceleration()
   {
      int numberOfEndEffectorsTrusted = 2;
      
      if (feetSpacingVelocity.getDoubleValue() < footVelocityThreshold.getDoubleValue())
         return numberOfEndEffectorsTrusted;
      
      for (RobotSide robotSide : RobotSide.values)
      {
         imuAccelerationInWorld.getFrameVector(tempIMUAcceleration);
         footToPelvisAccels.get(robotSide).getFrameVector(tempFrameVector);
         tempFrameVector.sub(tempIMUAcceleration);
         accelerationMagnitudeErrors.put(robotSide, tempFrameVector.length());
      }
      
      for (RobotSide robotSide : RobotSide.values)
      {
         if (accelerationMagnitudeErrors.get(robotSide) > accelerationMagnitudeErrors.get(robotSide.getOppositeSide()))
         {
            areFeetTrusted.get(robotSide).set(false);
            numberOfEndEffectorsTrusted--;
            return numberOfEndEffectorsTrusted;
         }
      }
      
      return numberOfEndEffectorsTrusted;
   }

   private void computeFeetSpacingLengthAndVelocity()
   {
      tempFrameVector.setToZero(worldFrame);
      
      for (RobotSide robotSide : RobotSide.values)
      {
         footToPelvisPositions.get(robotSide).getFramePointAndChangeFrameOfPackedPoint(tempPosition);
         tempPosition.scale(robotSide.negateIfRightSide(1.0));
         tempFrameVector.add(tempPosition);
      }
      
      feetSpacing.set(tempFrameVector.length());

      tempFrameVector.setToZero(worldFrame);
      
      for (RobotSide robotSide : RobotSide.values)
      {
         footToPelvisVelocities.get(robotSide).getFrameVector(tempVelocity);
         tempVelocity.scale(robotSide.negateIfRightSide(1.0));
         tempFrameVector.add(tempVelocity);
      }
      feetSpacingVelocity.update(tempFrameVector.length());
   }
   
   private class TrustBothFeetState extends State<PelvisEstimationState>
   {

      public TrustBothFeetState()
      {
         super(PelvisEstimationState.TRUST_BOTH_FEET);
      }

      @Override
      public void doAction()
      {
         for(RobotSide robotSide : RobotSide.values)
         {
            updateCoPPosition(robotSide);
            correctFootPositionsUsingCoP(robotSide);
            updatePelvisWithKinematics(robotSide, 2);
         }
         
         System.out.println("Updating in TrustBothFeet at time " + yoTime.getDoubleValue());
         pelvisVelocityBacklashKinematics.update();
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

   private class TrustOneFootState extends State<PelvisEstimationState>
   {
      private final RobotSide trustedSide;
      private final RobotSide ignoredSide;

      public TrustOneFootState(RobotSide trustedSide)
      {
         super(robotSideToPelvisEstimationState.get(trustedSide));
         this.trustedSide = trustedSide;
         this.ignoredSide = trustedSide.getOppositeSide();
      }

      @Override
      public void doAction()
      {
         doActionForTrustedFoot();
         doActionForIgnoredFoot();
      }
      
      private void doActionForTrustedFoot()
      {
         updateCoPPosition(trustedSide);
         correctFootPositionsUsingCoP(trustedSide);
         updatePelvisWithKinematics(trustedSide, 1);
         
         System.out.println("Updating in TrustOneFoot at time " + yoTime.getDoubleValue());
         pelvisVelocityBacklashKinematics.update();
      }
      
      private void doActionForIgnoredFoot()
      {
         updateFootPosition(ignoredSide);
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

   private void updateCoPPosition(RobotSide trustedSide)
   {
         AlphaFilteredYoFramePoint2d copFilteredInFootFrame = copsFilteredInFootFrame.get(trustedSide);
         ReferenceFrame footFrame = footFrames.get(trustedSide);
         
         footSwitches.get(trustedSide).packCoP(tempCoP);
         
         if (tempCoP.containsNaN())
         {
            tempCoP.setToZero();
         }
         else
         {
            FrameConvexPolygon2d footPolygon = footPolygons.get(trustedSide);
            boolean isCoPInsideFoot = footPolygon.isPointInside(tempCoP);
            if (!isCoPInsideFoot)
            {
               FrameLineSegment2d tempLineSegment = tempLineSegments.get(trustedSide);
               footPolygon.getClosestEdge(tempLineSegment, tempCoP);

               FrameLine2d ankleToCoPLine = ankleToCoPLines.get(trustedSide);
               ankleToCoPLine.set(footFrame, 0.0, 0.0, tempCoP.getX(), tempCoP.getY());
               tempLineSegment.intersectionWith(tempCoP, ankleToCoPLine);
            }
         }
         
         copsRawInFootFrame.get(trustedSide).set(tempCoP);
         
         tempCoPOffset.set(footFrame, copFilteredInFootFrame.getX(), copFilteredInFootFrame.getY(), 0.0);
         copFilteredInFootFrame.update();
         tempCoPOffset.set(footFrame, copFilteredInFootFrame.getX() - tempCoPOffset.getX(), copFilteredInFootFrame.getY() - tempCoPOffset.getY(), 0.0);

         tempCoPOffset.changeFrame(worldFrame);
         copPositionsInWorld.get(trustedSide).add(tempCoPOffset);
   }

   private void correctFootPositionsUsingCoP(RobotSide trustedSide)
   {
      AlphaFilteredYoFramePoint2d copFilteredInFootFrame = copsFilteredInFootFrame.get(trustedSide);
      tempCoPOffset.set(copFilteredInFootFrame.getReferenceFrame(), copFilteredInFootFrame.getX(), copFilteredInFootFrame.getY(), 0.0);

      tempCoPOffset.changeFrame(worldFrame);

      YoFramePoint footPositionIWorld = footPositionsInWorld.get(trustedSide);
      footPositionIWorld.set(copPositionsInWorld.get(trustedSide));
      footPositionIWorld.sub(tempCoPOffset);
   }

   private void updatePelvisWithKinematics(RobotSide robotSide, int numberOfFootInContact)
   {
      double scaleFactor = 1.0 / numberOfFootInContact;
      
      footToPelvisPositions.get(robotSide).getFramePoint(tempPosition);
      tempPosition.scale(scaleFactor);
      pelvisPositionKinematics.add(tempPosition);
      footPositionsInWorld.get(robotSide).getFramePoint(tempPosition);
      tempPosition.scale(scaleFactor);
      pelvisPositionKinematics.add(tempPosition);
      
      footToPelvisVelocities.get(robotSide).getFrameVector(tempVelocity);
      tempVelocity.scale(scaleFactor);
      pelvisVelocityKinematics.add(tempVelocity);
      
      YoFramePoint2d copPosition2d = copsFilteredInFootFrame.get(robotSide);
      tempFramePoint.set(copPosition2d.getReferenceFrame(), copPosition2d.getX(), copPosition2d.getY(), 0.0);
      tempFramePoint.changeFrame(pelvisToFootTwists.get(robotSide).getBaseFrame());
      pelvisToFootTwists.get(robotSide).changeFrame(pelvisToFootTwists.get(robotSide).getBaseFrame());
      pelvisToFootTwists.get(robotSide).packVelocityOfPointFixedInBodyFrame(tempVelocity, tempFramePoint);
      tempVelocity.changeFrame(worldFrame);
      tempVelocity.scale(-scaleFactor); // We actually want footToPelvis velocity
      pelvisVelocityTwist.add(tempVelocity);
   }

   private void updateFootPosition(RobotSide ignoredSide)
   {
      YoFramePoint footPositionInWorld = footPositionsInWorld.get(ignoredSide);
      footPositionInWorld.set(footToPelvisPositions.get(ignoredSide));
      footPositionInWorld.scale(-1.0);
      footPositionInWorld.add(pelvisPosition);

      copPositionsInWorld.get(ignoredSide).set(footPositionInWorld);

      copsRawInFootFrame.get(ignoredSide).setToZero();
      copsFilteredInFootFrame.get(ignoredSide).setToZero();
   }

   private final FrameVector tempIMUAcceleration = new FrameVector();
   private final FrameVector tempPelvisVelocityIntegrated = new FrameVector();
   private final FrameVector tempEstimatedVelocityIMUPart = new FrameVector();
   private final FramePoint tempEstimatedPositionIMUPart = new FramePoint();
   private final FramePoint2d tempCoP = new FramePoint2d();
   private final FrameVector tempCoPOffset = new FrameVector();
   
   public void run()
   {
      defaultActionIntoStates();
      
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
      
      defaultActionOutOfStates();
   }
   
   private void defaultActionOutOfStates()
   {
      if (linearAccelerationPort != null)
      {
         computePelvisStateByIntegratingAccelerometerAndMergeWithKinematics();
      
      }   
      else
      {
         if (useTwistToComputePelvisVelocity.getBooleanValue())
            pelvisVelocity.set(pelvisVelocityTwist);
         else
            pelvisVelocity.set(pelvisVelocityBacklashKinematics); //pelvisVelocityKinematics);
         pelvisPosition.set(pelvisPositionKinematics);
      }
      
      updateCoMState();
   }
   
   private void computePelvisStateByIntegratingAccelerometerAndMergeWithKinematics()
   {
      imuAccelerationInWorld.getFrameVector(tempIMUAcceleration);
      tempIMUAcceleration.scale(estimatorDT);
      pelvisVelocityByIntegrating.add(tempIMUAcceleration);
      
      pelvisVelocity.getFrameVectorAndChangeFrameOfPackedVector(tempEstimatedVelocityIMUPart);
      tempEstimatedVelocityIMUPart.add(tempIMUAcceleration);
      tempEstimatedVelocityIMUPart.scale(alphaPelvisAccelerometerIntegrationToVelocity.getDoubleValue());
      
      if (useTwistToComputePelvisVelocity.getBooleanValue())
         pelvisVelocity.set(pelvisVelocityTwist);
      else
         pelvisVelocity.set(pelvisVelocityBacklashKinematics);
      
      pelvisVelocity.scale(1.0 - alphaPelvisAccelerometerIntegrationToVelocity.getDoubleValue());
      pelvisVelocity.add(tempEstimatedVelocityIMUPart);
      
      pelvisVelocity.getFrameVectorAndChangeFrameOfPackedVector(tempPelvisVelocityIntegrated);
      tempPelvisVelocityIntegrated.scale(estimatorDT);
      pelvisPosition.getFramePointAndChangeFrameOfPackedPoint(tempEstimatedPositionIMUPart);
      tempEstimatedPositionIMUPart.add(tempPelvisVelocityIntegrated);
      tempEstimatedPositionIMUPart.scale(alphaPelvisAccelerometerIntegrationToPosition.getDoubleValue());

      pelvisPosition.set(pelvisPositionKinematics);
      pelvisPosition.scale(1.0 - alphaPelvisAccelerometerIntegrationToPosition.getDoubleValue());
      pelvisPosition.add(tempEstimatedPositionIMUPart);
   }

   private final FramePoint tempCenterOfMassPositionWorld = new FramePoint(worldFrame);
   private final FrameVector tempCenterOfMassVelocityWorld = new FrameVector(worldFrame);
   
   private void updateCoMState()
   {
      centerOfMassCalculator.compute();
      centerOfMassCalculator.packCenterOfMass(tempCenterOfMassPositionWorld);
      tempCenterOfMassPositionWorld.changeFrame(worldFrame);
      centerOfMassPosition.set(tempCenterOfMassPositionWorld);

      centerOfMassJacobianBody.compute();
      tempCenterOfMassVelocityWorld.setToZero(pelvisFrame);
      centerOfMassJacobianBody.packCenterOfMassVelocity(tempCenterOfMassVelocityWorld);
      tempCenterOfMassVelocityWorld.changeFrame(worldFrame);
      pelvisVelocity.getFrameVectorAndChangeFrameOfPackedVector(tempFrameVector);
      tempCenterOfMassVelocityWorld.add(tempFrameVector);
      centerOfMassVelocity.set(tempCenterOfMassVelocityWorld);
   }

   @Override
   public void getEstimatedPelvisPosition(FramePoint pelvisPositionToPack)
   {
      pelvisPosition.getFramePointAndChangeFrameOfPackedPoint(pelvisPositionToPack);
   }

   @Override
   public void getEstimatedPelvisLinearVelocity(FrameVector pelvisLinearVelocityToPack)
   {
      pelvisVelocity.getFrameVectorAndChangeFrameOfPackedVector(pelvisLinearVelocityToPack);
   }

   @Override
   public void getEstimatedCoMPosition(FramePoint comPositionToPack)
   {
      centerOfMassPosition.getFramePointAndChangeFrameOfPackedPoint(comPositionToPack);
   }

   @Override
   public void getEstimatedCoMVelocity(FrameVector comVelocityToPack)
   {
      centerOfMassVelocity.getFrameVectorAndChangeFrameOfPackedVector(comVelocityToPack);
   }

   @Override
   public void setJointAndIMUSensorDataSource(JointAndIMUSensorDataSource jointAndIMUSensorDataSource)
   {
      Collection<ControlFlowOutputPort<Vector3d>> linearAccelerationOutputPorts = jointAndIMUSensorDataSource.getSensorMap().getLinearAccelerationOutputPorts();

      for (ControlFlowOutputPort<Vector3d> controlFlowOutputPort : linearAccelerationOutputPorts)
      {
         linearAccelerationPort = controlFlowOutputPort;
      }
   }
}
