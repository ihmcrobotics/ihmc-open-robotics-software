package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.calculators.EquivalentConstantCoPCalculator;
import us.ihmc.commonWalkingControlModules.calculators.GainCalculator;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.ChestOrientationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.DegenerateOrientationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.endEffector.EndEffectorControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.endEffector.EndEffectorControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controlModules.head.DesiredHeadOrientationProvider;
import us.ihmc.commonWalkingControlModules.controlModules.head.HeadOrientationControlModule;
import us.ihmc.commonWalkingControlModules.controllers.HandControllerInterface;
import us.ihmc.commonWalkingControlModules.controllers.LidarControllerInterface;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorTools;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepProvider;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepUtils;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.desiredFootStep.UpcomingFootstepList;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.DesiredHandPoseProvider;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.ManipulationStateMachine;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.FinalDesiredICPCalculator;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPBasedMomentumRateOfChangeControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointTrajectoryData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.OrientationTrajectoryData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LimbName;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.sensors.HeelSwitch;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightPartialDerivativesData;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTimeDerivativesCalculator;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTimeDerivativesData;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTimeDerivativesSmoother;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.CoMXYTimeDerivativesData;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantCoPInstantaneousCapturePointTrajectory;
import us.ihmc.commonWalkingControlModules.trajectories.ContactStatesAndUpcomingFootstepData;
import us.ihmc.commonWalkingControlModules.trajectories.CubicPolynomialTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.CurrentOrientationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.FlatThenPolynomialCoMHeightTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.HeelPitchTouchdownProvidersManager;
import us.ihmc.commonWalkingControlModules.trajectories.InstantaneousCapturePointTrajectory;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.SettableOrientationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.SimpleTwoWaypointTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.trajectories.SwingTimeCalculationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.YoVariableDoubleProvider;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.PDController;
import com.yobotics.simulationconstructionset.util.errorHandling.WalkingStatusReporter;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameOrientation;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint2d;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector2d;
import com.yobotics.simulationconstructionset.util.statemachines.State;
import com.yobotics.simulationconstructionset.util.statemachines.StateMachine;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransition;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransitionAction;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransitionCondition;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleProvider;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.PositionTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.TrajectoryParameters;
import com.yobotics.simulationconstructionset.util.trajectory.TrajectoryParametersProvider;
import com.yobotics.simulationconstructionset.util.trajectory.TrajectoryWaypointGenerationMethod;
import com.yobotics.simulationconstructionset.util.trajectory.YoPositionProvider;

public class WalkingHighLevelHumanoidController extends State<HighLevelState>
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final WalkingControllerParameters walkingControllerParameters;
   
   private final WalkingStatusReporter walkingStatusReporter;

   private static enum WalkingState
   {
      LEFT_SUPPORT, RIGHT_SUPPORT, TRANSFER_TO_LEFT_SUPPORT, TRANSFER_TO_RIGHT_SUPPORT, DOUBLE_SUPPORT
   }

   private final static boolean DEBUG = false;
   private final StateMachine<WalkingState> stateMachine;
   private final SideDependentList<ManipulationStateMachine> manipulationStateMachines = new SideDependentList<ManipulationStateMachine>();
   private final CenterOfMassJacobian centerOfMassJacobian;

   private final CoMHeightTrajectoryGenerator centerOfMassHeightTrajectoryGenerator;
   private final CoMHeightTimeDerivativesCalculator coMHeightTimeDerivativesCalculator = new CoMHeightTimeDerivativesCalculator();
   private final CoMHeightTimeDerivativesSmoother coMHeightTimeDerivativesSmoother;

   private final PDController centerOfMassHeightController;
   private final SideDependentList<WalkingState> singleSupportStateEnums = new SideDependentList<WalkingState>(WalkingState.LEFT_SUPPORT,
                                                                              WalkingState.RIGHT_SUPPORT);

   private final SideDependentList<WalkingState> transferStateEnums = new SideDependentList<WalkingState>(WalkingState.TRANSFER_TO_LEFT_SUPPORT,
                                                                         WalkingState.TRANSFER_TO_RIGHT_SUPPORT);

   private final YoVariableDoubleProvider transferTimeProvider = new YoVariableDoubleProvider("transferTime", registry);
   private final DoubleYoVariable stopInDoubleSupporTrajectoryTime = new DoubleYoVariable("stopInDoubleSupporTrajectoryTime", registry);

   private final FinalDesiredICPCalculator finalDesiredICPCalculator;

   private final BooleanYoVariable rememberFinalICPFromSingleSupport = new BooleanYoVariable("rememberFinalICPFromSingleSupport", registry);
   private final YoFramePoint2d finalDesiredICPInWorld = new YoFramePoint2d("finalDesiredICPInWorld", "", worldFrame, registry);

   protected final SideDependentList<FootSwitchInterface> footSwitches;

   private final DoubleYoVariable minOrbitalEnergyForSingleSupport = new DoubleYoVariable("minOrbitalEnergyForSingleSupport", registry);
   private final DoubleYoVariable amountToBeInsideSingleSupport = new DoubleYoVariable("amountToBeInsideSingleSupport", registry);
   private final DoubleYoVariable amountToBeInsideDoubleSupport = new DoubleYoVariable("amountToBeInsideDoubleSupport", registry);

   private final DegenerateOrientationControlModule chestOrientationControlModule;

   private final DoubleYoVariable userDesiredPelvisPitch = new DoubleYoVariable("userDesiredPelvisPitch", registry);
   private final SettableOrientationProvider initialPelvisOrientationProvider;
   private final SettableOrientationProvider finalPelvisOrientationProvider;
   private final OrientationTrajectoryGenerator pelvisOrientationTrajectoryGenerator;

   private final LinkedHashMap<ContactablePlaneBody, EndEffectorControlModule> endEffectorControlModules = new LinkedHashMap<ContactablePlaneBody, EndEffectorControlModule>();

   private final SwingTimeCalculationProvider swingTimeCalculationProvider;
   private final TrajectoryParametersProvider trajectoryParametersProvider;
   private final SideDependentList<YoVariableDoubleProvider> onToesInitialAngleProviders = new SideDependentList<YoVariableDoubleProvider>();
   private final SideDependentList<YoVariableDoubleProvider> onToesFinalAngleProviders = new SideDependentList<YoVariableDoubleProvider>();

   private final BooleanYoVariable stayOnToes = new BooleanYoVariable("stayOnToes", registry);
   private final DoubleYoVariable trailingFootPitch = new DoubleYoVariable("trailingFootPitch", registry);

   private final DoubleYoVariable kUpperBody = new DoubleYoVariable("kUpperBody", registry);
   private final DoubleYoVariable zetaUpperBody = new DoubleYoVariable("zetaUpperBody", registry);
   private final YoPositionProvider finalPositionProvider;

   private final DoubleYoVariable swingAboveSupportAnkle = new DoubleYoVariable("swingAboveSupportAnkle", registry);
   private final BooleanYoVariable readyToGrabNextFootstep = new BooleanYoVariable("readyToGrabNextFootstep", registry);

   private final HashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters;
   private final InstantaneousCapturePointTrajectory icpTrajectoryGenerator;
   private final SideDependentList<SettableOrientationProvider> finalFootOrientationProviders = new SideDependentList<SettableOrientationProvider>();

   private final OneDoFJoint[] positionControlJoints;

   private final OneDoFJoint jointForExtendedNeckPitchRange;
   private final HeadOrientationControlModule headOrientationControlModule;
   private final DesiredHeadOrientationProvider desiredHeadOrientationProvider;

   private final SideDependentList<EnumMap<LimbName, GeometricJacobian>> jacobians = SideDependentList.createListOfEnumMaps(LimbName.class);
   private final ControlFlowInputPort<OrientationTrajectoryData> desiredPelvisOrientationTrajectoryInputPort;
   private final ICPBasedMomentumRateOfChangeControlModule icpBasedMomentumRateOfChangeControlModule;
   private final YoFrameOrientation desiredPelvisOrientation = new YoFrameOrientation("desiredPelvis", worldFrame, registry);
   private final YoFrameVector desiredPelvisAngularVelocity;
   private final YoFrameVector desiredPelvisAngularAcceleration;


   private final DoubleYoVariable coefficientOfFriction = new DoubleYoVariable("coefficientOfFriction", registry);

// private final OneDoFJoint lidarJoint;
   private final LidarControllerInterface lidarControllerInterface;
   private final List<OneDoFJoint> torqueControlJoints = new ArrayList<OneDoFJoint>();
   private final BooleanYoVariable toeOff = new BooleanYoVariable("toeOff", registry);
   private final BooleanYoVariable icpTrajectoryHasBeenInitialized;
   private final BooleanYoVariable doToeOffIfPossible = new BooleanYoVariable("doToeOffIfPossible", registry);
   private final DoubleYoVariable onToesTriangleArea = new DoubleYoVariable("onToesTriangleArea", registry);
   private final DoubleYoVariable onToesTriangleAreaLimit = new DoubleYoVariable("onToesTriangleAreaLimit", registry);
   private final HeelPitchTouchdownProvidersManager heelPitchTouchdownProvidersManager;

   private final UpcomingFootstepList upcomingFootstepList;

   // TODO: New variables
   private final LinkedHashMap<ContactablePlaneBody, YoPlaneContactState> contactStates;
   private final ICPAndMomentumBasedController icpAndMomentumBasedController;
   private final EnumYoVariable<RobotSide> upcomingSupportLeg;
   private final EnumYoVariable<RobotSide> supportLeg;
   private final SideDependentList<? extends ContactablePlaneBody> bipedFeet;
   private final BipedSupportPolygons bipedSupportPolygons;
   protected final CommonWalkingReferenceFrames referenceFrames;
   private final YoFramePoint capturePoint;
   private final YoFramePoint2d desiredICP;
   private final YoFrameVector2d desiredICPVelocity;
   private final FullRobotModel fullRobotModel;
   protected final double gravity;
   private final DoubleYoVariable desiredCoMHeightAcceleration;
   private final DoubleYoVariable yoTime;
   
   public WalkingHighLevelHumanoidController(FullRobotModel fullRobotModel, TwistCalculator twistCalculator, CenterOfMassJacobian centerOfMassJacobian,
         SideDependentList<FootSwitchInterface> footSwitches, double gravityZ, DoubleYoVariable yoTime, double controlDT,
         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, FootstepProvider footstepProvider, DesiredHandPoseProvider handPoseProvider,
         HashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters, DesiredHeadOrientationProvider desiredHeadOrientationProvider,
         CoMHeightTrajectoryGenerator centerOfMassHeightTrajectoryGenerator, SideDependentList<PositionTrajectoryGenerator> footPositionTrajectoryGenerators,
         SideDependentList<DoubleTrajectoryGenerator> heelPitchTrajectoryGenerators, HeelPitchTouchdownProvidersManager heelPitchTouchdownProvidersManager,
         SwingTimeCalculationProvider swingTimeCalculationProvider, YoPositionProvider finalPositionProvider,
         TrajectoryParametersProvider trajectoryParametersProvider, boolean stayOntoes, double desiredPelvisPitch, double trailingFootPitch,
         WalkingControllerParameters walkingControllerParameters, ICPBasedMomentumRateOfChangeControlModule momentumRateOfChangeControlModule,
         ControlFlowInputPort<OrientationTrajectoryData> desiredPelvisOrientationTrajectoryInputPort, LidarControllerInterface lidarControllerInterface,
         FinalDesiredICPCalculator finalDesiredICPCalculator, SideDependentList<HandControllerInterface> handControllers,
         ICPAndMomentumBasedController icpAndMomentumBasedController, WalkingStatusReporter walkingStatusReporter)
   {
      super(HighLevelState.WALKING);
      
      this.yoTime = yoTime;
      coMHeightTimeDerivativesSmoother = new CoMHeightTimeDerivativesSmoother(controlDT, registry);
      this.icpAndMomentumBasedController = icpAndMomentumBasedController;
      contactStates = icpAndMomentumBasedController.getContactStates();
      upcomingSupportLeg = icpAndMomentumBasedController.getUpcomingSupportLeg();
      supportLeg = icpAndMomentumBasedController.getYoSupportLeg();
      this.bipedFeet = icpAndMomentumBasedController.getBipedFeet();
      this.referenceFrames = icpAndMomentumBasedController.getReferenceFrames();
      capturePoint = icpAndMomentumBasedController.getCapturePoint();
      desiredICP = icpAndMomentumBasedController.getDesiredICP();
      desiredICPVelocity = icpAndMomentumBasedController.getDesiredICPVelocity();
      this.bipedSupportPolygons = icpAndMomentumBasedController.getBipedSupportPolygons();
      this.fullRobotModel = fullRobotModel;
      this.gravity = gravityZ;
      desiredCoMHeightAcceleration = icpAndMomentumBasedController.getDesiredCoMHeightAcceleration();

      this.finalDesiredICPCalculator = finalDesiredICPCalculator;
      this.icpBasedMomentumRateOfChangeControlModule = momentumRateOfChangeControlModule;
      this.desiredPelvisOrientationTrajectoryInputPort = desiredPelvisOrientationTrajectoryInputPort;
      this.centerOfMassJacobian = centerOfMassJacobian;
      this.centerOfMassHeightTrajectoryGenerator = centerOfMassHeightTrajectoryGenerator;
      this.swingTimeCalculationProvider = swingTimeCalculationProvider;
      this.trajectoryParametersProvider = trajectoryParametersProvider;
      this.mapFromFootstepsToTrajectoryParameters = mapFromFootstepsToTrajectoryParameters;
      this.footSwitches = footSwitches;
      this.heelPitchTouchdownProvidersManager = heelPitchTouchdownProvidersManager;

      this.upcomingFootstepList = new UpcomingFootstepList(footstepProvider);
      
      this.centerOfMassHeightController = new PDController("comHeight", registry);
      centerOfMassHeightController.setProportionalGain(40.0);
      double zeta = 1.0;
      centerOfMassHeightController.setDerivativeGain(GainCalculator.computeDerivativeGain(centerOfMassHeightController.getProportionalGain(), zeta));

      String namePrefix = "walking";
      icpTrajectoryGenerator = new ConstantCoPInstantaneousCapturePointTrajectory(namePrefix, bipedSupportPolygons, registry);

      this.stateMachine = new StateMachine<WalkingState>(namePrefix + "State", namePrefix + "SwitchTime", WalkingState.class, yoTime, registry);    // this is used by name, and it is ugly.

      this.finalPositionProvider = finalPositionProvider;

      this.icpTrajectoryHasBeenInitialized = new BooleanYoVariable("icpTrajectoryHasBeenInitialized", registry);

      rememberFinalICPFromSingleSupport.set(true);
      finalDesiredICPInWorld.set(Double.NaN, Double.NaN);

      coefficientOfFriction.set(0.6);    // TODO: tune?

      setupLimbJacobians(fullRobotModel);

      for (RobotSide robotSide : RobotSide.values())
      {
         ContactablePlaneBody bipedFoot = bipedFeet.get(robotSide);
         contactStates.get(bipedFoot).set(bipedFoot.getContactPoints2d(), coefficientOfFriction.getDoubleValue());    // flat feet
         String sideString = robotSide.getCamelCaseNameForStartOfExpression();

         PositionTrajectoryGenerator swingPositionTrajectoryGenerator = footPositionTrajectoryGenerators.get(robotSide);
         DoubleTrajectoryGenerator heelPitchTrajectoryGenerator = (heelPitchTrajectoryGenerators == null) ? null : heelPitchTrajectoryGenerators.get(robotSide);

         OrientationProvider initialOrientationProvider = new CurrentOrientationProvider(worldFrame, bipedFoot.getBodyFrame());
         SettableOrientationProvider finalFootOrientationProvider = new SettableOrientationProvider(sideString + "FinalFootOrientation", worldFrame, registry);
         finalFootOrientationProviders.put(robotSide, finalFootOrientationProvider);

         OrientationTrajectoryGenerator swingOrientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator(sideString
                                                                                 + "SwingFootOrientation", worldFrame, swingTimeCalculationProvider,
                                                                                    initialOrientationProvider, finalFootOrientationProvider, registry);

         YoVariableDoubleProvider onToesInitialPitchProvider = new YoVariableDoubleProvider(sideString + "OnToesInitialPitch", registry);
         YoVariableDoubleProvider onToesFinalPitchProvider = new YoVariableDoubleProvider(sideString + "OnToesFinalPitch", registry);
         DoubleProvider onToesTrajectoryTimeProvider = transferTimeProvider;
         DoubleTrajectoryGenerator onToesPitchTrajectoryGenerator = new CubicPolynomialTrajectoryGenerator(sideString + "OnToesPitch",
                                                                       onToesInitialPitchProvider, onToesFinalPitchProvider, onToesTrajectoryTimeProvider,
                                                                       registry);
         onToesInitialAngleProviders.put(robotSide, onToesInitialPitchProvider);
         onToesFinalAngleProviders.put(robotSide, onToesFinalPitchProvider);

         GeometricJacobian jacobian = jacobians.get(robotSide).get(LimbName.LEG);
         EndEffectorControlModule endEffectorControlModule = new EndEffectorControlModule(bipedFoot, jacobian, swingPositionTrajectoryGenerator,
                                                                heelPitchTrajectoryGenerator, swingOrientationTrajectoryGenerator,
                                                                onToesPitchTrajectoryGenerator, yoTime, twistCalculator, registry);
         endEffectorControlModule.setParameters(3e-2, 500.0);
         endEffectorControlModules.put(bipedFoot, endEffectorControlModule);
      }

      initialPelvisOrientationProvider = new SettableOrientationProvider("initialPelvis", worldFrame, registry);
      finalPelvisOrientationProvider = new SettableOrientationProvider("finalPelvis", worldFrame, registry);
      this.pelvisOrientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator("pelvis", worldFrame, swingTimeCalculationProvider,
              initialPelvisOrientationProvider, finalPelvisOrientationProvider, registry);

      setUpStateMachine();
      readyToGrabNextFootstep.set(true);

      minOrbitalEnergyForSingleSupport.set(0.007);    // 0.008
      amountToBeInsideSingleSupport.set(0.0);
      amountToBeInsideDoubleSupport.set(0.03);    // 0.02);    // TODO: necessary for stairs...
      transferTimeProvider.set(0.2);    // 0.5);    // 0.2);    // 0.6;    // 0.3
      stopInDoubleSupporTrajectoryTime.set(0.5);
      this.userDesiredPelvisPitch.set(desiredPelvisPitch);
      this.stayOnToes.set(stayOntoes);
      this.trailingFootPitch.set(trailingFootPitch);
      kUpperBody.set(100.0);
      zetaUpperBody.set(1.0);
      onToesTriangleAreaLimit.set(0.01);

      this.walkingControllerParameters = walkingControllerParameters;
      
      this.walkingStatusReporter = walkingStatusReporter;

      double initialLeadingFootPitch = 0.05;
      upcomingSupportLeg.set(RobotSide.RIGHT); // TODO: stairs hack, so that the following lines use the correct leading leg
      RobotSide leadingLeg = getUpcomingSupportLeg();
      onToesInitialAngleProviders.get(leadingLeg).set(initialLeadingFootPitch);
      onToesFinalAngleProviders.get(leadingLeg).set(initialLeadingFootPitch);

      RobotSide trailingLeg = leadingLeg.getOppositeSide();
      onToesInitialAngleProviders.get(trailingLeg).set(this.trailingFootPitch.getDoubleValue());


      for (RobotSide robotSide : RobotSide.values())
      {
         HandControllerInterface handControllerInterface = null;
         if (handControllers != null)
         {
            handControllerInterface = handControllers.get(robotSide);

         }

         manipulationStateMachines.put(robotSide,
                                       new ManipulationStateMachine(yoTime, robotSide, fullRobotModel, twistCalculator, icpAndMomentumBasedController.getInverseDynamicsCalculator(),
                                          walkingControllerParameters, handPoseProvider, dynamicGraphicObjectsListRegistry, handControllerInterface, gravityZ,
                                          controlDT, registry));
      }

      this.desiredHeadOrientationProvider = desiredHeadOrientationProvider;

      String[] headOrientationControlJointNames = walkingControllerParameters.getHeadOrientationControlJointNames();
      String[] chestOrientationControlJointNames = walkingControllerParameters.getChestOrientationControlJointNames();

      RigidBody elevator = fullRobotModel.getElevator();
      RigidBody pelvis = fullRobotModel.getPelvis();
      InverseDynamicsJoint[] allJoints = ScrewTools.computeSupportAndSubtreeJoints(fullRobotModel.getRootJoint().getSuccessor());
      InverseDynamicsJoint[] headOrientationControlJoints = ScrewTools.findJointsWithNames(allJoints, headOrientationControlJointNames);
      InverseDynamicsJoint[] chestOrientationControlJoints = ScrewTools.findJointsWithNames(allJoints, chestOrientationControlJointNames);

      if (headOrientationControlJoints.length > 0)
      {
         final RigidBody head = fullRobotModel.getHead();
         GeometricJacobian neckJacobian = new GeometricJacobian(headOrientationControlJoints, head.getBodyFixedFrame());
         ReferenceFrame pelvisFrame = pelvis.getBodyFixedFrame();
         ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
         ReferenceFrame chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
         ReferenceFrame[] availableHeadOrientationControlFrames = new ReferenceFrame[] {pelvisZUpFrame, pelvisFrame, ReferenceFrame.getWorldFrame()};
         headOrientationControlModule = new HeadOrientationControlModule(neckJacobian, pelvis, elevator, twistCalculator,
                 availableHeadOrientationControlFrames, chestFrame, walkingControllerParameters, registry, dynamicGraphicObjectsListRegistry);

         // Setting initial head pitch
         // This magic number (0.67) is a good default head pitch for getting good LIDAR point coverage of ground by feet
         // it would be in DRC config parameters, but the would require updating several nested constructors with an additional parameter
         FrameOrientation orientation = new FrameOrientation(pelvisZUpFrame, 0.0, 0.67, 0.0);
         headOrientationControlModule.setOrientationToTrack(new FrameOrientation(orientation), pelvis);
         double headKp = 40.0;
         double headZeta = 1.0;
         double headKd = GainCalculator.computeDerivativeGain(headKp, headZeta);
         headOrientationControlModule.setProportionalGains(headKp, headKp, headKp);
         headOrientationControlModule.setDerivativeGains(headKd, headKd, headKd);

         if (desiredHeadOrientationProvider != null)
         {
            desiredHeadOrientationProvider.setHeadOrientationControlModule(headOrientationControlModule);
         }
      }
      else
      {
         headOrientationControlModule = null;
      }

      if (walkingControllerParameters.getJointNameForExtendedPitchRange() != null)
      {
         InverseDynamicsJoint[] inverseDynamicsJointForExtendedNeckPitchControl = ScrewTools.findJointsWithNames(allJoints,
                                                                                     walkingControllerParameters.getJointNameForExtendedPitchRange());
         OneDoFJoint[] jointForExtendedNeckPitchControl = ScrewTools.filterJoints(inverseDynamicsJointForExtendedNeckPitchControl, OneDoFJoint.class);
         if (jointForExtendedNeckPitchControl.length == 1)
         {
            this.jointForExtendedNeckPitchRange = jointForExtendedNeckPitchControl[0];
         }
         else
         {
            this.jointForExtendedNeckPitchRange = null;
         }
      }
      else
      {
         this.jointForExtendedNeckPitchRange = null;
      }

      if (chestOrientationControlJoints.length > 0)
      {
         RigidBody chest = fullRobotModel.getChest();
         GeometricJacobian spineJacobian = new GeometricJacobian(chestOrientationControlJoints, chest.getBodyFixedFrame());
         chestOrientationControlModule = new ChestOrientationControlModule(pelvis, fullRobotModel.getChest(), spineJacobian, twistCalculator, registry);
         chestOrientationControlModule.setProportionalGains(100.0, 100.0, 100.0);
         chestOrientationControlModule.setDerivativeGains(20.0, 20.0, 20.0);
      }
      else
      {
         chestOrientationControlModule = null;
      }

      List<InverseDynamicsJoint> unconstrainedJoints = new ArrayList<InverseDynamicsJoint>(Arrays.asList(allJoints));
      for (RobotSide robotSide : RobotSide.values())
      {
         RigidBody hand = fullRobotModel.getHand(robotSide);
         InverseDynamicsJoint[] armJoints = ScrewTools.createJointPath(fullRobotModel.getChest(), hand);
         unconstrainedJoints.removeAll(Arrays.asList(armJoints));

         RigidBody foot = fullRobotModel.getFoot(robotSide);
         InverseDynamicsJoint[] legJoints = ScrewTools.createJointPath(pelvis, foot);
         unconstrainedJoints.removeAll(Arrays.asList(legJoints));

         InverseDynamicsJoint[] handJoints = ScrewTools.computeSubtreeJoints(hand);
         OneDoFJoint[] handJointsArray = new OneDoFJoint[ScrewTools.computeNumberOfJointsOfType(OneDoFJoint.class, handJoints)];
         ScrewTools.filterJoints(handJoints, handJointsArray, OneDoFJoint.class);

         List<OneDoFJoint> handJointsList = Arrays.asList(handJointsArray);

         unconstrainedJoints.removeAll(handJointsList);
         torqueControlJoints.addAll(handJointsList);
      }

      this.lidarControllerInterface = lidarControllerInterface;

      if (lidarControllerInterface != null)
      {
         unconstrainedJoints.remove(lidarControllerInterface.getLidarJoint());
      }

      unconstrainedJoints.removeAll(Arrays.asList(headOrientationControlJoints));
      unconstrainedJoints.removeAll(Arrays.asList(chestOrientationControlJoints));

      if (jointForExtendedNeckPitchRange != null)
      {
         unconstrainedJoints.remove(jointForExtendedNeckPitchRange);
      }

      unconstrainedJoints.remove(fullRobotModel.getRootJoint());
      InverseDynamicsJoint[] unconstrainedJointsArray = new InverseDynamicsJoint[unconstrainedJoints.size()];
      unconstrainedJoints.toArray(unconstrainedJointsArray);
      positionControlJoints = new OneDoFJoint[unconstrainedJointsArray.length];
      ScrewTools.filterJoints(unconstrainedJointsArray, positionControlJoints, OneDoFJoint.class);

      unconstrainedJoints.removeAll(Arrays.asList(positionControlJoints));
      if (unconstrainedJoints.size() > 0)
         throw new RuntimeException("Joints unconstrained: " + unconstrainedJoints);

      this.desiredPelvisAngularVelocity = new YoFrameVector("desiredPelvisAngularVelocity", worldFrame, registry);
      this.desiredPelvisAngularAcceleration = new YoFrameVector("desiredPelvisAngularAcceleration", worldFrame, registry);
   }

   private void setupLimbJacobians(FullRobotModel fullRobotModel)
   {
      EnumMap<LimbName, RigidBody> bases = new EnumMap<LimbName, RigidBody>(LimbName.class);
      bases.put(LimbName.LEG, fullRobotModel.getPelvis());
      bases.put(LimbName.ARM, fullRobotModel.getChest());

      for (RobotSide robotSide : RobotSide.values())
      {
         for (LimbName limbName : LimbName.values())
         {
            RigidBody endEffector = fullRobotModel.getEndEffector(robotSide, limbName);
            GeometricJacobian jacobian = new GeometricJacobian(bases.get(limbName), endEffector, endEffector.getBodyFixedFrame());
            jacobians.get(robotSide).put(limbName, jacobian);
         }
      }
   }

   public void setDoToeOffIfPossible(boolean doToeOffIfPossible)
   {
      this.doToeOffIfPossible.set(doToeOffIfPossible);
   }

   private RobotSide getUpcomingSupportLeg()
   {
      return upcomingSupportLeg.getEnumValue();
   }

   private RobotSide getSupportLeg()
   {
      return supportLeg.getEnumValue();
   }

   private void setUpStateMachine()
   {
      DoubleSupportState doubleSupportState = new DoubleSupportState(null);

      stateMachine.addState(doubleSupportState);

      ResetICPTrajectoryAction resetICPTrajectoryAction = new ResetICPTrajectoryAction();
      for (RobotSide robotSide : RobotSide.values())
      {
         EndEffectorControlModule swingEndEffectorControlModule = endEffectorControlModules.get(bipedFeet.get(robotSide.getOppositeSide()));
         StopWalkingCondition stopWalkingCondition = new StopWalkingCondition(swingEndEffectorControlModule);
         ResetSwingTrajectoryDoneAction resetSwingTrajectoryDoneAction = new ResetSwingTrajectoryDoneAction(swingEndEffectorControlModule);

         ArrayList<StateTransitionAction> stopWalkingStateTransitionActions = new ArrayList<StateTransitionAction>();
         stopWalkingStateTransitionActions.add(resetICPTrajectoryAction);
         stopWalkingStateTransitionActions.add(resetSwingTrajectoryDoneAction);

         State<WalkingState> transferState = new DoubleSupportState(robotSide);
         StateTransition<WalkingState> toDoubleSupport = new StateTransition<WalkingState>(doubleSupportState.getStateEnum(), stopWalkingCondition,
                                                            stopWalkingStateTransitionActions);
         transferState.addStateTransition(toDoubleSupport);
         StateTransition<WalkingState> toSingleSupport = new StateTransition<WalkingState>(singleSupportStateEnums.get(robotSide),
                                                            new DoneWithTransferCondition());
         transferState.addStateTransition(toSingleSupport);
         stateMachine.addState(transferState);

         State<WalkingState> singleSupportState = new SingleSupportState(robotSide);
         StateTransition<WalkingState> toDoubleSupport2 = new StateTransition<WalkingState>(doubleSupportState.getStateEnum(), stopWalkingCondition,
                                                             stopWalkingStateTransitionActions);
         singleSupportState.addStateTransition(toDoubleSupport2);

         DoneWithSingleSupportCondition doneWithSingleSupportCondition = new DoneWithSingleSupportCondition(swingEndEffectorControlModule);
         StateTransition<WalkingState> toTransfer = new StateTransition<WalkingState>(transferStateEnums.get(robotSide.getOppositeSide()),
                                                       doneWithSingleSupportCondition, resetSwingTrajectoryDoneAction);
         singleSupportState.addStateTransition(toTransfer);
         stateMachine.addState(singleSupportState);
      }

      for (RobotSide robotSide : RobotSide.values())
      {
         StateTransition<WalkingState> toTransfer = new StateTransition<WalkingState>(transferStateEnums.get(robotSide),
                                                       new DoneWithDoubleSupportCondition(robotSide));
         doubleSupportState.addStateTransition(toTransfer);
      }
   }

   public void initialize()
   {
      icpAndMomentumBasedController.initialize();
      
      FrameOrientation initialDesiredPelvisOrientation = new FrameOrientation(referenceFrames.getAnkleZUpFrame(getUpcomingSupportLeg()));
      initialDesiredPelvisOrientation.changeFrame(worldFrame);
      double yaw = initialDesiredPelvisOrientation.getYawPitchRoll()[0];
      initialDesiredPelvisOrientation.setYawPitchRoll(yaw, userDesiredPelvisPitch.getDoubleValue(), 0.0);
      desiredPelvisOrientation.set(initialDesiredPelvisOrientation);
      finalPelvisOrientationProvider.setOrientation(initialDesiredPelvisOrientation);    // yes, final. To make sure that the first swing phase has the right initial

      icpAndMomentumBasedController.computeCapturePoint();
      desiredICP.set(capturePoint.getFramePoint2dCopy());

      stateMachine.setCurrentState(WalkingState.DOUBLE_SUPPORT);
   }

   private class DoubleSupportState extends State<WalkingState>
   {
      private final RobotSide transferToSide;

      public DoubleSupportState(RobotSide transferToSide)
      {
         super((transferToSide == null) ? WalkingState.DOUBLE_SUPPORT : transferStateEnums.get(transferToSide));
         this.transferToSide = transferToSide;
      }

      @Override
      public void doAction()
      {
         ContactablePlaneBody transferFoot = bipedFeet.get(transferToSide);

         if ((endEffectorControlModules.get(transferFoot) != null) && endEffectorControlModules.get(transferFoot).onHeel()
                 && footSwitches.get(transferToSide).hasFootHitGround())
         {
            setFlatFootContactState(transferFoot);
         }

         // note: this has to be done before the ICP trajectory generator is initialized, since it is using nextFootstep
         upcomingFootstepList.checkForFootsteps(icpAndMomentumBasedController.getPointPositionGrabber(), readyToGrabNextFootstep, upcomingSupportLeg, bipedFeet);
         checkForHighSteps(transferToSide);

         if (icpTrajectoryGenerator.isDone())
         {
            if (!icpTrajectoryHasBeenInitialized.getBooleanValue())
            {
               Pair<FramePoint2d, Double> finalDesiredICPAndTrajectoryTime = computeFinalDesiredICPAndTrajectoryTime();

               FramePoint2d finalDesiredICP = finalDesiredICPAndTrajectoryTime.first();
               double trajectoryTime = finalDesiredICPAndTrajectoryTime.second();

               finalDesiredICP.changeFrame(desiredICP.getReferenceFrame());

               if (!stayOnToes.getBooleanValue() && (transferToSide != null))    // the only case left for determining the contact state of the trailing foot
               {
                  RobotSide trailingLeg = transferToSide.getOppositeSide();
                  ContactablePlaneBody supportFoot = bipedFeet.get(trailingLeg);
                  FrameConvexPolygon2d onToesTriangle = getOnToesTriangle(finalDesiredICP, supportFoot);

                  boolean desiredICPOK = onToesTriangle.isPointInside(desiredICP.getFramePoint2dCopy())
                                         && (onToesTriangle.getArea() > onToesTriangleAreaLimit.getDoubleValue());
                  toeOff.set(desiredICPOK && doToeOffIfPossible.getBooleanValue());

                  if (toeOff.getBooleanValue())
                  {
                     setOnToesContactState(supportFoot);
                  }
                  else
                  {
                     setFlatFootContactState(supportFoot);
                  }
               }

               icpAndMomentumBasedController.updateBipedSupportPolygons(bipedSupportPolygons);    // need to always update biped support polygons after a change to the contact states

               icpTrajectoryGenerator.initialize(desiredICP.getFramePoint2dCopy(), finalDesiredICP, trajectoryTime, icpAndMomentumBasedController.getOmega0(),
                                                 amountToBeInsideDoubleSupport.getDoubleValue(), getSupportLeg(), yoTime.getDoubleValue());

               icpTrajectoryHasBeenInitialized.set(true);
            }
         }

         if (icpTrajectoryGenerator.isDone() && (transferToSide == null))
         {
            desiredICPVelocity.set(0.0, 0.0);
         }
         else
         {
            FramePoint2d desiredICPLocal = new FramePoint2d(desiredICP.getReferenceFrame());
            FrameVector2d desiredICPVelocityLocal = new FrameVector2d(desiredICPVelocity.getReferenceFrame());
            icpTrajectoryGenerator.getCurrentDesiredICPPositionAndVelocity(desiredICPLocal, desiredICPVelocityLocal, icpAndMomentumBasedController.getOmega0(), yoTime.getDoubleValue());
            desiredICP.set(desiredICPLocal);
            desiredICPVelocity.set(desiredICPVelocityLocal);
         }

         // keep desired pelvis orientation as it is
         desiredPelvisAngularVelocity.set(0.0, 0.0, 0.0);
         desiredPelvisAngularAcceleration.set(0.0, 0.0, 0.0);

         doFootcontrol();
      }

      private Pair<FramePoint2d, Double> computeFinalDesiredICPAndTrajectoryTime()
      {
         Pair<FramePoint2d, Double> finalDesiredICPAndTrajectoryTime;

         if (transferToSide == null)
         {
            FramePoint2d finalDesiredICP = getDoubleSupportFinalDesiredICPForDoubleSupportStance();
            double trajectoryTime = stopInDoubleSupporTrajectoryTime.getDoubleValue();

            finalDesiredICPInWorld.set(Double.NaN, Double.NaN);

            finalDesiredICPAndTrajectoryTime = new Pair<FramePoint2d, Double>(finalDesiredICP, trajectoryTime);
         }

         else if (rememberFinalICPFromSingleSupport.getBooleanValue() && !finalDesiredICPInWorld.containsNaN())
         {
            FramePoint2d finalDesiredICP = finalDesiredICPInWorld.getFramePoint2dCopy();
            double trajectoryTime = transferTimeProvider.getValue();

            finalDesiredICPAndTrajectoryTime = new Pair<FramePoint2d, Double>(finalDesiredICP, trajectoryTime);
         }

         else
         {

            Footstep transferFromFootstep = createFootstepFromFootAndContactablePlaneBody(referenceFrames.getFootFrame(transferToSide.getOppositeSide()),
                  bipedFeet.get(transferToSide.getOppositeSide()));
            Footstep transferToFootstep = createFootstepFromFootAndContactablePlaneBody(referenceFrames.getFootFrame(transferToSide),
                  bipedFeet.get(transferToSide));

            
            FrameConvexPolygon2d transferToFootPolygon = computeFootPolygon(transferToSide, referenceFrames.getSoleFrame(transferToSide));

            Footstep nextFootstep = upcomingFootstepList.getNextFootstep();
            Footstep nextNextFootstep = upcomingFootstepList.getNextNextFootstep();

            TransferToAndNextFootstepsData transferToAndNextFootstepsData = new TransferToAndNextFootstepsData();
            transferToAndNextFootstepsData.setTransferFromFootstep(transferFromFootstep);
            transferToAndNextFootstepsData.setTransferToFootstep(transferToFootstep);
            transferToAndNextFootstepsData.setTransferToFootPolygonInSoleFrame(transferToFootPolygon);
            transferToAndNextFootstepsData.setTransferToSide(transferToSide);
            transferToAndNextFootstepsData.setNextFootstep(nextFootstep);
            transferToAndNextFootstepsData.setNextNextFootstep(nextNextFootstep);
            transferToAndNextFootstepsData.setEstimatedStepTime(swingTimeCalculationProvider.getValue() + transferTimeProvider.getValue());
            transferToAndNextFootstepsData.setW0(icpAndMomentumBasedController.getOmega0());

            finalDesiredICPCalculator.initialize(transferToAndNextFootstepsData);
            FramePoint2d finalDesiredICP = finalDesiredICPCalculator.getFinalDesiredICP();
            double trajectoryTime = transferTimeProvider.getValue();

            finalDesiredICPInWorld.set(finalDesiredICP.changeFrameCopy(worldFrame));
            finalDesiredICPAndTrajectoryTime = new Pair<FramePoint2d, Double>(finalDesiredICP, trajectoryTime);
         }

         return finalDesiredICPAndTrajectoryTime;
      }

      @Override
      public void doTransitionIntoAction()
      {
         icpTrajectoryHasBeenInitialized.set(false);
         if (DEBUG)
            System.out.println("WalkingHighLevelHumanoidController: enteringDoubleSupportState");
         setSupportLeg(null);    // TODO: check if necessary

         // TODO: simplify the following
         if (transferToSide != null)
         {
            RobotSide trailingLeg = transferToSide.getOppositeSide();
            onToesFinalAngleProviders.get(trailingLeg).set(trailingFootPitch.getDoubleValue());
         }

         if (stayOnToes.getBooleanValue())
         {
            for (RobotSide robotSide : RobotSide.values())
            {
               setOnToesContactState(bipedFeet.get(robotSide));
            }
         }
         else
         {
            if (transferToSide == null)
            {
               for (RobotSide robotSide : RobotSide.values())
               {
                  setFlatFootContactState(bipedFeet.get(robotSide));
               }
            }
            else if (landOnHeels())
            {
               setOnHeelContactState(bipedFeet.get(transferToSide));
            }
            else
            {
               setFlatFootContactState(bipedFeet.get(transferToSide));

               // still need to determine contact state for trailing leg. This is done in doAction as soon as the previous ICP trajectory is done
            }
         }

         icpAndMomentumBasedController.updateBipedSupportPolygons(bipedSupportPolygons);    // need to always update biped support polygons after a change to the contact states

         centerOfMassHeightTrajectoryGenerator.initialize(getSupportLeg(), null, getContactStatesList());
      }

      @Override
      public void doTransitionOutOfAction()
      {
         if (DEBUG)
            System.out.println("WalkingHighLevelHumanoidController: leavingDoubleSupportState");
         desiredICPVelocity.set(0.0, 0.0);
      }
   }


   private FrameConvexPolygon2d getOnToesTriangle(FramePoint2d finalDesiredICP, ContactablePlaneBody supportFoot)
   {
      List<FramePoint> toePoints = getToePoints(supportFoot);
      Collection<FramePoint2d> points = new ArrayList<FramePoint2d>();
      for (FramePoint toePoint : toePoints)
      {
         points.add(toePoint.toFramePoint2d());
      }

      points.add(finalDesiredICP);

      return new FrameConvexPolygon2d(points);
   }


   private class SingleSupportState extends State<WalkingState>
   {
      private final RobotSide swingSide;
      private final FrameOrientation desiredPelvisOrientationToPack;
      private final FrameVector desiredPelvisAngularVelocityToPack;
      private final FrameVector desiredPelvisAngularAccelerationToPack;

      public SingleSupportState(RobotSide robotSide)
      {
         super(singleSupportStateEnums.get(robotSide));
         this.swingSide = robotSide.getOppositeSide();
         this.desiredPelvisOrientationToPack = new FrameOrientation(worldFrame);
         this.desiredPelvisAngularVelocityToPack = new FrameVector(fullRobotModel.getRootJoint().getFrameAfterJoint());
         this.desiredPelvisAngularAccelerationToPack = new FrameVector(fullRobotModel.getRootJoint().getFrameAfterJoint());
      }

      @Override
      public void doAction()
      {
//       if (!icpTrajectoryGenerator.isDone())
//       {
//          FramePoint2d desiredICPLocal = new FramePoint2d(desiredICP.getReferenceFrame());
//          FrameVector2d desiredICPVelocityLocal = new FrameVector2d(desiredICPVelocity.getReferenceFrame());
//          icpTrajectoryGenerator.pack(desiredICPLocal, desiredICPVelocityLocal, getOmega0());
//          desiredICP.set(desiredICPLocal);
//          desiredICPVelocity.set(desiredICPVelocityLocal);
//       }
//       else
//       {
//          // don't change desiredICP
//          desiredICPVelocity.set(0.0, 0.0);
//       }

         FramePoint2d desiredICPLocal = new FramePoint2d(desiredICP.getReferenceFrame());
         FrameVector2d desiredICPVelocityLocal = new FrameVector2d(desiredICPVelocity.getReferenceFrame());
         icpTrajectoryGenerator.getCurrentDesiredICPPositionAndVelocity(desiredICPLocal, desiredICPVelocityLocal, icpAndMomentumBasedController.getOmega0(), yoTime.getDoubleValue());
         desiredICP.set(desiredICPLocal);
         desiredICPVelocity.set(desiredICPVelocityLocal);

         pelvisOrientationTrajectoryGenerator.compute(stateMachine.timeInCurrentState());
         pelvisOrientationTrajectoryGenerator.get(desiredPelvisOrientationToPack);
         pelvisOrientationTrajectoryGenerator.packAngularVelocity(desiredPelvisAngularVelocityToPack);
         pelvisOrientationTrajectoryGenerator.packAngularAcceleration(desiredPelvisAngularAccelerationToPack);
         desiredPelvisOrientation.set(desiredPelvisOrientationToPack);
         desiredPelvisAngularVelocity.set(desiredPelvisAngularVelocityToPack);
         desiredPelvisAngularAcceleration.set(desiredPelvisAngularAccelerationToPack);

         doFootcontrol();
      }

      @Override
      public void doTransitionIntoAction()
      {
         footSwitches.get(swingSide).reset();
         if(footSwitches.get(swingSide) instanceof HeelSwitch) ((HeelSwitch) footSwitches.get(swingSide)).resetHeelSwitch();

         Footstep nextFootstep = upcomingFootstepList.getNextFootstep();
         boolean nextFootstepHasBeenReplaced = false;
         Footstep oldNextFootstep = nextFootstep;

         if (!nextFootstep.getTrustHeight())
         {
            // TODO: This might be better placed somewhere else.
            // TODO: Do more than just step at the previous ankle height.
            // Probably do something a little smarter like take a cautious high step.
            // Or we should have a mode that the user can set on how cautious to step.

            FramePoint supportAnklePosition = new FramePoint(referenceFrames.getAnkleZUpFrame(swingSide.getOppositeSide()));
            supportAnklePosition.changeFrame(nextFootstep.getReferenceFrame());
            double newHeight = supportAnklePosition.getZ() + swingAboveSupportAnkle.getDoubleValue();

            nextFootstep = Footstep.copyButChangeHeight(nextFootstep, newHeight);
            nextFootstepHasBeenReplaced = true;
         }

         if (landOnHeels())
         {
            nextFootstep = Footstep.copyButChangePitch(nextFootstep, heelPitchTouchdownProvidersManager.getInitialAngle());
            heelPitchTouchdownProvidersManager.updateInitialAngularSpeed();
            nextFootstepHasBeenReplaced = true;
         }

         if (nextFootstepHasBeenReplaced)
            switchTrajectoryParametersMapping(oldNextFootstep, nextFootstep);

         if (DEBUG)
            System.out.println("WalkingHighLevelHumanoidController: enteringSingleSupportState");
         RobotSide supportSide = swingSide.getOppositeSide();

         setSupportLeg(supportSide);

         if (stayOnToes.getBooleanValue())
         {
            setOnToesContactState(bipedFeet.get(supportSide));
         }
         else
         {
            setFlatFootContactState(bipedFeet.get(supportSide));
         }

         initializeTrajectory(nextFootstep, swingSide, null);
         if (DEBUG)
            System.out.println("WalkingHighLevelHumanoidController: nextFootstep will change now!");
         readyToGrabNextFootstep.set(true);
      }

      private void switchTrajectoryParametersMapping(Footstep oldFootstep, Footstep newFootstep)
      {
         mapFromFootstepsToTrajectoryParameters.put(newFootstep, mapFromFootstepsToTrajectoryParameters.get(oldFootstep));
         mapFromFootstepsToTrajectoryParameters.remove(oldFootstep);
      }

      @Override
      public void doTransitionOutOfAction()
      {
         if (DEBUG)
            System.out.println("WalkingHighLevelController: leavingDoubleSupportState");

         upcomingFootstepList.notifyComplete();

         // ContactableBody swingFoot = contactablePlaneBodies.get(swingSide);
         // Footstep desiredFootstep = desiredFootstepCalculator.updateAndGetDesiredFootstep(swingSide.getOppositeSide());
         // contactStates.get(swingFoot).setContactPoints(desiredFootstep.getExpectedContactPoints());
         // updateFootStateMachines(swingFoot);
      }
   }


   public class DoneWithDoubleSupportCondition implements StateTransitionCondition
   {
      private final RobotSide transferToSide;

      public DoneWithDoubleSupportCondition(RobotSide robotSide)
      {
         this.transferToSide = robotSide;
      }

      public boolean checkCondition()
      {
         if (readyToGrabNextFootstep.getBooleanValue())
            return false;
         else
         {
            boolean doubleSupportTimeHasPassed = stateMachine.timeInCurrentState() > transferTimeProvider.getValue();
            boolean transferringToThisRobotSide = transferToSide == getUpcomingSupportLeg();

            return transferringToThisRobotSide && doubleSupportTimeHasPassed;
         }
      }
   }


   public class DoneWithTransferCondition implements StateTransitionCondition
   {
      public boolean checkCondition()
      {
         if (walkingControllerParameters.checkOrbitalEnergyCondition())
         {
            // TODO: not really nice, but it'll do:
            FlatThenPolynomialCoMHeightTrajectoryGenerator flatThenPolynomialCoMHeightTrajectoryGenerator =
               (FlatThenPolynomialCoMHeightTrajectoryGenerator) centerOfMassHeightTrajectoryGenerator;
            double orbitalEnergy = flatThenPolynomialCoMHeightTrajectoryGenerator.computeOrbitalEnergyIfInitializedNow(getUpcomingSupportLeg());

            // return transferICPTrajectoryDone.getBooleanValue() && orbitalEnergy > minOrbitalEnergy;
            return icpTrajectoryHasBeenInitialized.getBooleanValue() && (orbitalEnergy > minOrbitalEnergyForSingleSupport.getDoubleValue());
         }
         else
         {
            return icpTrajectoryHasBeenInitialized.getBooleanValue() && icpTrajectoryGenerator.isDone();
         }
      }
   }

   private class DoneWithSingleSupportCondition implements StateTransitionCondition
   {
      public DoneWithSingleSupportCondition(EndEffectorControlModule endEffectorControlModule)
      {
      }

      public boolean checkCondition()
      {
         RobotSide swingSide = getSupportLeg().getOppositeSide();
         boolean hasMinimumTimePassed = hasMinimumTimePassed();

         FootSwitchInterface footSwitch = footSwitches.get(swingSide);

         // TODO probably make all FootSwitches in this class be HeelSwitches and get rid of instanceof
         boolean footSwitchActivated;
         if (landOnHeels())
         {
            if (!(footSwitch instanceof HeelSwitch))
            {
               throw new RuntimeException("landOnHeels should not be set to true if Robot is not using a HeelSwitch.");
            }

            HeelSwitch heelSwitch = (HeelSwitch) footSwitch;
            footSwitchActivated = heelSwitch.hasHeelHitGround();
         }
         else
         {
            footSwitchActivated = footSwitch.hasFootHitGround();
         }

         if (walkingControllerParameters.finishSwingWhenTrajectoryDone())
         {
            boolean trajectoryDone = icpTrajectoryGenerator.isDone();    // endEffectorControlModule.isTrajectoryDone();

            return hasMinimumTimePassed && (trajectoryDone || footSwitchActivated);
         }
         else
         {
            return hasMinimumTimePassed && footSwitchActivated;
         }
      }

      private boolean hasMinimumTimePassed()
      {
         double minimumSwingFraction = 0.5;
         double minimumSwingTime = swingTimeCalculationProvider.getValue() * minimumSwingFraction;
         return stateMachine.timeInCurrentState() > minimumSwingTime;
      }
   }


   private class ResetSwingTrajectoryDoneAction implements StateTransitionAction
   {
      private EndEffectorControlModule endEffectorControlModule;

      public ResetSwingTrajectoryDoneAction(EndEffectorControlModule endEffectorControlModule)
      {
         this.endEffectorControlModule = endEffectorControlModule;
      }

      public void doTransitionAction()
      {
         endEffectorControlModule.resetTrajectoryDone();
      }
   }


   private class StopWalkingCondition extends DoneWithSingleSupportCondition
   {
      public StopWalkingCondition(EndEffectorControlModule endEffectorControlModule)
      {
         super(endEffectorControlModule);
      }

      public boolean checkCondition()
      {
         Footstep nextFootstep = upcomingFootstepList.getNextFootstep();
         boolean readyToStopWalking = (upcomingFootstepList.isFootstepProviderEmpty() && (nextFootstep == null)) && ((getSupportLeg() == null) || super.checkCondition());

         return readyToStopWalking;
      }
   }


   public class ResetICPTrajectoryAction implements StateTransitionAction
   {
      public void doTransitionAction()
      {
         icpTrajectoryGenerator.reset();
      }
   }


   private FramePoint2d getDoubleSupportFinalDesiredICPForDoubleSupportStance()
   {
      FramePoint2d ret = new FramePoint2d(worldFrame);
      double trailingFootToLeadingFootFactor = 0.5;    // 0.25;
      for (RobotSide robotSide : RobotSide.values())
      {
         FramePoint2d centroid = new FramePoint2d(ret.getReferenceFrame());
         FrameConvexPolygon2d footPolygon = computeFootPolygon(robotSide, referenceFrames.getAnkleZUpFrame(robotSide));
         footPolygon.getCentroid(centroid);
         centroid.changeFrame(ret.getReferenceFrame());
         if (robotSide == getUpcomingSupportLeg())
            centroid.scale(trailingFootToLeadingFootFactor);
         else
            centroid.scale(1.0 - trailingFootToLeadingFootFactor);
         ret.add(centroid);
      }

      return ret;
   }

   private FramePoint2d getSingleSupportFinalDesiredICPForWalking(Footstep transferToFootstep, RobotSide swingSide)
   {
      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
      FramePoint2d initialDesiredICP = desiredICP.getFramePoint2dCopy();
      initialDesiredICP.changeFrame(referenceFrame);

      Footstep transferFromFootstep = createFootstepFromFootAndContactablePlaneBody(referenceFrames.getFootFrame(swingSide.getOppositeSide()),
            bipedFeet.get(swingSide.getOppositeSide()));
      
      FrameConvexPolygon2d footPolygon;
      ContactablePlaneBody contactableBody = bipedFeet.get(swingSide);
      if (stayOnToes.getBooleanValue())
      {
         List<FramePoint> contactPoints = getContactPointsForWalkingOnEdge(contactableBody, ConstraintType.TOES);
         footPolygon = FrameConvexPolygon2d.constructByProjectionOntoXYPlane(contactPoints, referenceFrames.getSoleFrame(swingSide));
      }
      else
      {
         footPolygon = new FrameConvexPolygon2d(contactableBody.getContactPoints2d());
      }

      TransferToAndNextFootstepsData transferToAndNextFootstepsData = new TransferToAndNextFootstepsData();

      transferToAndNextFootstepsData.setTransferFromFootstep(transferFromFootstep);
      transferToAndNextFootstepsData.setTransferToFootstep(transferToFootstep);

      transferToAndNextFootstepsData.setTransferToFootPolygonInSoleFrame(footPolygon);
      transferToAndNextFootstepsData.setTransferToSide(swingSide);
      transferToAndNextFootstepsData.setNextFootstep(upcomingFootstepList.getNextNextFootstep());
      transferToAndNextFootstepsData.setNextNextFootstep(upcomingFootstepList.getNextNextNextFootstep());
      transferToAndNextFootstepsData.setEstimatedStepTime(swingTimeCalculationProvider.getValue() + transferTimeProvider.getValue());
      transferToAndNextFootstepsData.setW0(icpAndMomentumBasedController.getOmega0());

      finalDesiredICPCalculator.initialize(transferToAndNextFootstepsData);
      FramePoint2d finalDesiredICP = finalDesiredICPCalculator.getFinalDesiredICP();
      finalDesiredICP.changeFrame(referenceFrame);

      finalDesiredICPInWorld.set(finalDesiredICP);

      ContactablePlaneBody supportFoot = bipedFeet.get(swingSide.getOppositeSide());

      FrameConvexPolygon2d onToesTriangle = getOnToesTriangle(finalDesiredICP, supportFoot);
      double onToesTriangleArea = onToesTriangle.getArea();
      WalkingHighLevelHumanoidController.this.onToesTriangleArea.set(onToesTriangleArea);

      FramePoint2d icpWayPoint;
      if (doToeOffIfPossible.getBooleanValue() && (onToesTriangleArea > onToesTriangleAreaLimit.getDoubleValue()))
      {
         FramePoint toeOffPoint = new FramePoint(worldFrame);
         List<FramePoint> toePoints = getToePoints(supportFoot);
         toeOffPoint.interpolate(toePoints.get(0), toePoints.get(1), 0.5);
         FramePoint2d toeOffPoint2d = toeOffPoint.toFramePoint2d();
         double toeOffPointToFinalDesiredFactor = 0.2;    // TODO: magic number
         FramePoint2d desiredToeOffCoP = new FramePoint2d(worldFrame);
         desiredToeOffCoP.interpolate(toeOffPoint2d, finalDesiredICP, toeOffPointToFinalDesiredFactor);
         icpWayPoint = EquivalentConstantCoPCalculator.computeICPPositionWithConstantCMP(finalDesiredICP, desiredToeOffCoP, -transferTimeProvider.getValue(),
                 icpAndMomentumBasedController.getOmega0());
      }
      else
      {
         icpWayPoint = EquivalentConstantCoPCalculator.computeIntermediateICPWithConstantCMP(desiredICP.getFramePoint2dCopy(), finalDesiredICP,
                 swingTimeCalculationProvider.getValue() + transferTimeProvider.getValue(), swingTimeCalculationProvider.getValue(), icpAndMomentumBasedController.getOmega0());
      }

      return icpWayPoint;
   }

   private List<FramePoint> getToePoints(ContactablePlaneBody supportFoot)
   {
      FrameVector forward = new FrameVector(supportFoot.getPlaneFrame(), 1.0, 0.0, 0.0);
      int nToePoints = 2;
      List<FramePoint> toePoints = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(supportFoot.getContactPoints(), forward, nToePoints);
      for (FramePoint toePoint : toePoints)
      {
         toePoint.changeFrame(worldFrame);
      }

      return toePoints;
   }

   private void setSupportLeg(RobotSide supportLeg)
   {
      this.supportLeg.set(supportLeg);
   }

   public void initializeTrajectory(Footstep nextFootstep, RobotSide swingSide, SpatialAccelerationVector taskSpaceAcceleration)
   {
      RobotSide supportSide = swingSide.getOppositeSide();

//    ReferenceFrame trajectoryGeneratorFrame = ReferenceFrame.getWorldFrame();

      // ReferenceFrame swingAnkleZUpFrame = referenceFrames.getAnkleZUpFrame(swingSide);
      // ReferenceFrame swingFootFrame = referenceFrames.getFootFrame(swingSide);
      // FramePoint initialPosition = new FramePoint(swingAnkleZUpFrame);
      // initialPosition.changeFrame(trajectoryGeneratorFrame);
      //
      // Twist footTwist = new Twist();
      // twistCalculator.packTwistOfBody(footTwist, fullRobotModel.getFoot(swingSide));

      // SpatialAccelerationVector taskSpaceAccelerationWithRespectToWorld = new SpatialAccelerationVector();
      // spatialAccelerationCalculator.compute();
      // spatialAccelerationCalculator.packAccelerationOfBody(taskSpaceAccelerationWithRespectToWorld, fullRobotModel.getPelvis());
      //
      // Twist pelvisTwist = new Twist();
      // twistCalculator.packTwistOfBody(pelvisTwist, fullRobotModel.getPelvis());
      // taskSpaceAccelerationWithRespectToWorld.changeFrame(taskSpaceAccelerationWithRespectToWorld.getBaseFrame(), pelvisTwist, pelvisTwist);
      //
      // Twist footPelvisTwist = new Twist();
      // twistCalculator.packRelativeTwist(footPelvisTwist, fullRobotModel.getPelvis(), fullRobotModel.getFoot(swingSide));
      //
      // taskSpaceAcceleration.changeFrame(taskSpaceAccelerationWithRespectToWorld.getExpressedInFrame(), footTwist, footPelvisTwist);
      // taskSpaceAccelerationWithRespectToWorld.add(taskSpaceAcceleration);
      // FramePoint swingAnkle = new FramePoint(swingFootFrame);
      // swingAnkle.changeFrame(taskSpaceAccelerationWithRespectToWorld.getBaseFrame());
      // footTwist.changeFrame(taskSpaceAccelerationWithRespectToWorld.getExpressedInFrame());

      // footTwist.changeFrame(swingFootFrame);
      // FrameVector initialVelocity = new FrameVector(trajectoryGeneratorFrame);
      // footTwist.packLinearPart(initialVelocity);
      // initialVelocity.changeFrame(trajectoryGeneratorFrame);

      // footTwist.changeFrame(taskSpaceAccelerationWithRespectToWorld.getExpressedInFrame());
      // FrameVector initialAcceleration = new FrameVector(worldFrame);

      // taskSpaceAccelerationWithRespectToWorld.packAccelerationOfPointFixedInBodyFrame(footTwist, swingAnkle, initialAcceleration);
      // initialAcceleration.changeFrame(trajectoryGeneratorFrame);

      // initialAcceleration.setToZero(trajectoryGeneratorFrame);    // TODO

      finalPositionProvider.set(nextFootstep.getPositionInFrame(worldFrame));

      SideDependentList<Transform3D> footToWorldTransform = new SideDependentList<Transform3D>();
      for (RobotSide robotSide : RobotSide.values)
      {
         Transform3D transform = bipedFeet.get(robotSide).getBodyFrame().getTransformToDesiredFrame(worldFrame);
         footToWorldTransform.set(robotSide, transform);
      }

      Vector3d initialVectorPosition = new Vector3d();
      footToWorldTransform.get(supportSide.getOppositeSide()).get(initialVectorPosition);
      FramePoint initialFramePosition = new FramePoint(worldFrame, initialVectorPosition);
      FramePoint finalPosition = new FramePoint(worldFrame);
      finalPositionProvider.get(finalPosition);
      double stepDistance = initialFramePosition.distance(finalPosition);
      swingTimeCalculationProvider.setSwingTime(stepDistance);

      trajectoryParametersProvider.set(mapFromFootstepsToTrajectoryParameters.get(nextFootstep));
      finalFootOrientationProviders.get(swingSide).setOrientation(nextFootstep.getOrientationInFrame(worldFrame));

      FrameOrientation initialPelvisOrientation = new FrameOrientation(worldFrame);
      finalPelvisOrientationProvider.get(initialPelvisOrientation);
      initialPelvisOrientation.changeFrame(worldFrame);
      initialPelvisOrientationProvider.setOrientation(initialPelvisOrientation);

      FrameOrientation finalPelvisOrientation = nextFootstep.getOrientationInFrame(worldFrame);
      finalPelvisOrientation.setYawPitchRoll(finalPelvisOrientation.getYawPitchRoll()[0], userDesiredPelvisPitch.getDoubleValue(), 0.0);
      finalPelvisOrientationProvider.setOrientation(finalPelvisOrientation);
      pelvisOrientationTrajectoryGenerator.initialize();

      double stepPitch = nextFootstep.getOrientationInFrame(worldFrame).getYawPitchRoll()[1];
      onToesInitialAngleProviders.get(swingSide).set(stepPitch);
      onToesFinalAngleProviders.get(swingSide).set(stepPitch);

      FramePoint centerOfMass = new FramePoint(referenceFrames.getCenterOfMassFrame());
      centerOfMass.changeFrame(worldFrame);
      ContactablePlaneBody supportFoot = bipedFeet.get(supportSide);
      Transform3D supportFootToWorldTransform = footToWorldTransform.get(supportSide);
      double footHeight = DesiredFootstepCalculatorTools.computeMinZPointInFrame(supportFootToWorldTransform, supportFoot, worldFrame).getZ();
      double comHeight = centerOfMass.getZ() - footHeight;
      double omega0 = CapturePointCalculator.computeOmega0ConstantHeight(gravity, comHeight);
      icpAndMomentumBasedController.setOmega0(omega0);
      icpAndMomentumBasedController.computeCapturePoint();

      if (walkingControllerParameters.resetDesiredICPToCurrentAtStartOfSwing())
      {
         desiredICP.set(capturePoint.getFramePoint2dCopy());    // TODO: currently necessary for stairs because of the omega0 jump, but should get rid of this
      }

      FramePoint2d finalDesiredICP = getSingleSupportFinalDesiredICPForWalking(nextFootstep, swingSide);

      ContactablePlaneBody swingFoot = bipedFeet.get(swingSide);
      setContactStateForSwing(swingFoot);
      setSupportLeg(supportSide);
      icpAndMomentumBasedController.updateBipedSupportPolygons(bipedSupportPolygons);

      icpTrajectoryGenerator.initialize(desiredICP.getFramePoint2dCopy(), finalDesiredICP, swingTimeCalculationProvider.getValue(), omega0,
                                        amountToBeInsideSingleSupport.getDoubleValue(), getSupportLeg(), yoTime.getDoubleValue());

      centerOfMassHeightTrajectoryGenerator.initialize(getSupportLeg(), nextFootstep, getContactStatesList());
   }

   public void doMotionControl()
   {
      icpAndMomentumBasedController.doPrioritaryControl();
      
      for (ContactablePlaneBody contactablePlaneBody : endEffectorControlModules.keySet())
      {
         EndEffectorControlModule endEffectorControlModule = endEffectorControlModules.get(contactablePlaneBody);
         FramePoint2d cop = icpAndMomentumBasedController.getCoP(contactablePlaneBody);
         endEffectorControlModule.setCenterOfPressure(cop);
      }

      icpAndMomentumBasedController.computeCapturePoint();
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
      desiredCoMHeightAcceleration.set(computeDesiredCoMHeightAcceleration(desiredICPVelocity.getFrameVector2dCopy()));

      double kUpperBody = this.kUpperBody.getDoubleValue();
      double dUpperBody = GainCalculator.computeDerivativeGain(kUpperBody, zetaUpperBody.getDoubleValue());
      for (OneDoFJoint joint : positionControlJoints)
      {
         icpAndMomentumBasedController.doPDControl(joint, kUpperBody, dUpperBody, 0.0, 0.0);
      }

      if (chestOrientationControlModule != null)
         doChestControl();

      if (headOrientationControlModule != null)
         doHeadControl();

      doArmControl();

      setICPBasedMomentumRateOfChangeControlModuleInputs();

      setTorqueControlJointsToZeroDersiredAcceleration();

      if (lidarControllerInterface != null)
      {
         icpAndMomentumBasedController.setOneDoFJointAcceleration(lidarControllerInterface.getLidarJoint(), 0.0);
      }
      
      icpAndMomentumBasedController.doSecondaryControl();
   }

   private void setTorqueControlJointsToZeroDersiredAcceleration()
   {
      for (OneDoFJoint joint : torqueControlJoints)
      {
         icpAndMomentumBasedController.setOneDoFJointAcceleration(joint, 0.0);
      }
   }

   // TODO: connect ports instead
   private void setICPBasedMomentumRateOfChangeControlModuleInputs()
   {
      icpBasedMomentumRateOfChangeControlModule.getBipedSupportPolygonsInputPort().setData(bipedSupportPolygons);

      CapturePointData capturePointData = new CapturePointData();
      capturePointData.set(capturePoint.getFramePoint2dCopy(), icpAndMomentumBasedController.getOmega0());
      icpBasedMomentumRateOfChangeControlModule.getCapturePointInputPort().setData(capturePointData);

      CapturePointTrajectoryData capturePointTrajectoryData = new CapturePointTrajectoryData();
      capturePointTrajectoryData.set(desiredICP.getFramePoint2dCopy(), desiredICPVelocity.getFrameVector2dCopy());
      icpBasedMomentumRateOfChangeControlModule.getDesiredCapturePointTrajectoryInputPort().setData(capturePointTrajectoryData);

      icpBasedMomentumRateOfChangeControlModule.getSupportLegInputPort().setData(getSupportLeg());

      icpBasedMomentumRateOfChangeControlModule.getDesiredCenterOfMassHeightAccelerationInputPort().setData(desiredCoMHeightAcceleration.getDoubleValue());

      OrientationTrajectoryData desiredPelvisOrientationTrajectoryData = new OrientationTrajectoryData();
      desiredPelvisOrientationTrajectoryData.set(desiredPelvisOrientation.getFrameOrientationCopy(), desiredPelvisAngularVelocity.getFrameVectorCopy(),
              desiredPelvisAngularAcceleration.getFrameVectorCopy());
      desiredPelvisOrientationTrajectoryInputPort.setData(desiredPelvisOrientationTrajectoryData);
   }

   private void doChestControl()
   {
      chestOrientationControlModule.compute();
      icpAndMomentumBasedController.setDesiredSpatialAcceleration(chestOrientationControlModule.getJacobian(),
            chestOrientationControlModule.getTaskspaceConstraintData());
   }

   public void doHeadControl()
   {
      headOrientationControlModule.compute();
      icpAndMomentumBasedController.setDesiredSpatialAcceleration(headOrientationControlModule.getJacobian(),
              headOrientationControlModule.getTaskspaceConstraintData());

      if (jointForExtendedNeckPitchRange != null)
      {
         double kUpperBody = this.kUpperBody.getDoubleValue();
         double dUpperBody = GainCalculator.computeDerivativeGain(kUpperBody, zetaUpperBody.getDoubleValue());
         double angle = 0.0;
         if (desiredHeadOrientationProvider != null)
         {
            angle = desiredHeadOrientationProvider.getDesiredExtendedNeckPitchJointAngle();
         }

         icpAndMomentumBasedController.doPDControl(jointForExtendedNeckPitchRange, kUpperBody, dUpperBody, angle, 0.0);
      }
   }

   public void doArmControl()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         GeometricJacobian jacobian = jacobians.get(robotSide).get(LimbName.ARM);

         manipulationStateMachines.get(robotSide).startComputation();
         manipulationStateMachines.get(robotSide).waitUntilComputationIsDone();

         TaskspaceConstraintData taskspaceConstraintData = manipulationStateMachines.get(robotSide).getTaskspaceConstraintData();
         icpAndMomentumBasedController.setDesiredSpatialAcceleration(jacobian, taskspaceConstraintData);
      }
   }

   // Temporary objects to reduce garbage collection.
   private final CoMHeightPartialDerivativesData coMHeightPartialDerivatives = new CoMHeightPartialDerivativesData();
   private final ContactStatesAndUpcomingFootstepData centerOfMassHeightInputData = new ContactStatesAndUpcomingFootstepData();

   private double computeDesiredCoMHeightAcceleration(FrameVector2d desiredICPVelocity)
   {
      ReferenceFrame frame = worldFrame;

      centerOfMassHeightInputData.setCenterOfMassFrame(icpAndMomentumBasedController.getCenterOfMassFrame());

      List<? extends PlaneContactState> contactStatesList = getContactStatesList();

      centerOfMassHeightInputData.setContactStates(contactStatesList);

      centerOfMassHeightInputData.setSupportLeg(getSupportLeg());

      Footstep nextFootstep = upcomingFootstepList.getNextFootstep();
      centerOfMassHeightInputData.setUpcomingFootstep(nextFootstep);

      centerOfMassHeightTrajectoryGenerator.solve(coMHeightPartialDerivatives, centerOfMassHeightInputData);

      FramePoint comPosition = new FramePoint(referenceFrames.getCenterOfMassFrame());
      FrameVector comVelocity = new FrameVector(frame);
      centerOfMassJacobian.packCenterOfMassVelocity(comVelocity);
      comPosition.changeFrame(frame);
      comVelocity.changeFrame(frame);

      // TODO: use current omega0 instead of previous
      FrameVector2d comXYVelocity = comVelocity.toFrameVector2d();
      FrameVector2d comXYAcceleration = new FrameVector2d(desiredICPVelocity);
      comXYAcceleration.sub(comXYVelocity);
      comXYAcceleration.scale(icpAndMomentumBasedController.getOmega0());    // MathTools.square(omega0.getDoubleValue()) * (com.getX() - copX);

      // FrameVector2d comd2dSquared = new FrameVector2d(comXYVelocity.getReferenceFrame(), comXYVelocity.getX() * comXYVelocity.getX(), comXYVelocity.getY() * comXYVelocity.getY());

      CoMHeightTimeDerivativesData comHeightDataBeforeSmoothing = new CoMHeightTimeDerivativesData();
      CoMHeightTimeDerivativesData comHeightDataAfterSmoothing = new CoMHeightTimeDerivativesData();

      CoMXYTimeDerivativesData comXYTimeDerivatives = new CoMXYTimeDerivativesData();

      comXYTimeDerivatives.setCoMXYPosition(comPosition.toFramePoint2d());
      comXYTimeDerivatives.setCoMXYVelocity(comXYVelocity);
      comXYTimeDerivatives.setCoMXYAcceleration(comXYAcceleration);

      coMHeightTimeDerivativesCalculator.computeCoMHeightTimeDerivatives(comHeightDataBeforeSmoothing, comXYTimeDerivatives, coMHeightPartialDerivatives);

      coMHeightTimeDerivativesSmoother.smooth(comHeightDataAfterSmoothing, comHeightDataBeforeSmoothing);

      FramePoint centerOfMassHeightPoint = new FramePoint(ReferenceFrame.getWorldFrame());
      comHeightDataAfterSmoothing.getComHeight(centerOfMassHeightPoint);
      double zDesired = centerOfMassHeightPoint.getZ();

      double zdDesired = comHeightDataAfterSmoothing.getComHeightVelocity();
      double zddFeedForward = comHeightDataAfterSmoothing.getComHeightAcceleration();

      double zCurrent = comPosition.getZ();
      double zdCurrent = comVelocity.getZ();

      double zddDesired = centerOfMassHeightController.compute(zCurrent, zDesired, zdCurrent, zdDesired) + zddFeedForward;

      double epsilon = 1e-12;
      zddDesired = MathTools.clipToMinMax(zddDesired, -gravity + epsilon, Double.POSITIVE_INFINITY);

      return zddDesired;
   }

   private List<PlaneContactState> getContactStatesList()
   {
      List<PlaneContactState> contactStatesList = new ArrayList<PlaneContactState>();

      for (ContactablePlaneBody contactablePlaneBody : bipedFeet)
      {
         YoPlaneContactState contactState = contactStates.get(contactablePlaneBody);
         if (contactState.inContact())
            contactStatesList.add(contactState);
      }

      return contactStatesList;
   }

   private void doFootcontrol()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         ContactablePlaneBody contactablePlaneBody = bipedFeet.get(robotSide);
         EndEffectorControlModule endEffectorControlModule = endEffectorControlModules.get(contactablePlaneBody);
         endEffectorControlModule.startComputation();
         endEffectorControlModule.waitUntilComputationIsDone();
         TaskspaceConstraintData taskspaceConstraintData = endEffectorControlModule.getTaskSpaceConstraintOutputPort().getData();
         GeometricJacobian jacobian = endEffectorControlModule.getJacobian();
         icpAndMomentumBasedController.setDesiredSpatialAcceleration(jacobian, taskspaceConstraintData);
      }
   }

   private void setOnToesContactState(ContactablePlaneBody contactableBody)
   {
      List<FramePoint> contactPoints = getContactPointsForWalkingOnEdge(contactableBody, ConstraintType.TOES);
      List<FramePoint2d> contactPoints2d = getContactPoints2d(contactableBody, contactPoints);
      setContactState(contactableBody, contactPoints2d, ConstraintType.TOES);
   }

   private void setOnHeelContactState(ContactablePlaneBody contactableBody)
   {
      List<FramePoint> contactPoints = getContactPointsForWalkingOnEdge(contactableBody, ConstraintType.HEEL);
      List<FramePoint2d> contactPoints2d = getContactPoints2d(contactableBody, contactPoints);
      setContactState(contactableBody, contactPoints2d, ConstraintType.HEEL);
   }

   private void setFlatFootContactState(ContactablePlaneBody contactableBody)
   {
      setContactState(contactableBody, contactableBody.getContactPoints2d(), ConstraintType.FULL);
   }

   private void setContactStateForSwing(ContactablePlaneBody contactableBody)
   {
      setContactState(contactableBody, new ArrayList<FramePoint2d>(), ConstraintType.UNCONSTRAINED);
   }

   private void setContactState(ContactablePlaneBody contactableBody, List<FramePoint2d> contactPoints, ConstraintType constraintType)
   {
      if (contactPoints.size() == 0)
      {
         endEffectorControlModules.get(contactableBody).doSingularityEscapeBeforeTransitionToNextState();
      }

      YoPlaneContactState contactState = contactStates.get(contactableBody);
      int oldNumberOfContactPoints = contactState.getNumberOfContactPoints();
      contactState.set(contactPoints, coefficientOfFriction.getDoubleValue());
      updateEndEffectorControlModule(contactableBody, contactState, constraintType);

      if (contactPoints.size() < oldNumberOfContactPoints)
      {
//         resetGroundReactionWrenchFilter();    // so that we don't end up with CoPs outside of the base of support
      }
   }

   private List<FramePoint2d> getContactPoints2d(ContactablePlaneBody contactableBody, List<FramePoint> contactPoints)
   {
      List<FramePoint2d> contactPoints2d = new ArrayList<FramePoint2d>(contactPoints.size());
      for (FramePoint contactPoint : contactPoints)
      {
         contactPoint.changeFrame(contactableBody.getPlaneFrame());
         contactPoints2d.add(contactPoint.toFramePoint2d());
      }

      return contactPoints2d;
   }

   private List<FramePoint> getContactPointsForWalkingOnEdge(ContactablePlaneBody contactableBody, ConstraintType constraintType)
   {
      FrameVector direction = new FrameVector(contactableBody.getBodyFrame(), 1.0, 0.0, 0.0);
      if (constraintType == ConstraintType.HEEL)
         direction.scale(-1.0);

      return DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(contactableBody.getContactPoints(), direction, 2);
   }

   private void updateEndEffectorControlModule(ContactablePlaneBody contactablePlaneBody, PlaneContactState contactState, ConstraintType constraintType)
   {
      List<FramePoint2d> contactPoints = contactState.getContactPoints2d();
      endEffectorControlModules.get(contactablePlaneBody).setContactPoints(contactPoints, constraintType);
   }

   // TODO: should probably precompute this somewhere else
   private FrameConvexPolygon2d computeFootPolygon(RobotSide robotSide, ReferenceFrame referenceFrame)
   {
      List<FramePoint> contactPoints = contactStates.get(bipedFeet.get(robotSide)).getContactPoints();
      FrameConvexPolygon2d footPolygon = FrameConvexPolygon2d.constructByProjectionOntoXYPlane(contactPoints, referenceFrame);

      return footPolygon;
   }

   private boolean landOnHeels()
   {
      return (heelPitchTouchdownProvidersManager != null) && (heelPitchTouchdownProvidersManager.getInitialAngle() != 0.0);
   }

   private void checkForHighSteps(RobotSide transferToSide)
   {      
      if(transferToSide != null && upcomingFootstepList.hasNextFootsteps())
      {
         boolean footstepZIncreaseAboveThresholdInFinalFrame;

         double zIncreaseThreshold = 0.05;

         ReferenceFrame initialSoleFrame;
         // NOTE: the foot may have moved so its ideal to get the previous footstep, rather than the current foot frame, if possible
         if (upcomingFootstepList.doesNextFootstepListHaveFewerThanTwoElements())
         {
            initialSoleFrame = bipedFeet.get(transferToSide.getOppositeSide()).getRigidBody().getParentJoint().getFrameAfterJoint();
         }
         else
         {
            initialSoleFrame = upcomingFootstepList.getFootstepTwoBackFromNextFootstepList().getSoleReferenceFrame();
         }

         Footstep nextFootstep = upcomingFootstepList.getNextFootstep(); 
         ReferenceFrame finalSoleFrame = nextFootstep.getSoleReferenceFrame();
         
         FramePoint initialSole = new FramePoint(initialSoleFrame);
         initialSole.changeFrame(finalSoleFrame);
         footstepZIncreaseAboveThresholdInFinalFrame = - initialSole.getZ() > zIncreaseThreshold;
         
         boolean useHighSteps = footstepZIncreaseAboveThresholdInFinalFrame;
         
         if(useHighSteps)
         {
            TrajectoryParameters highStepTrajectoryParameters = new SimpleTwoWaypointTrajectoryParameters(TrajectoryWaypointGenerationMethod.HIGH_STEP);
            mapFromFootstepsToTrajectoryParameters.put(nextFootstep, highStepTrajectoryParameters);                
         }    
      }
   }   
   
   public static Footstep createFootstepFromFootAndContactablePlaneBody(ReferenceFrame footReferenceFrame, ContactablePlaneBody contactablePlaneBody)
   {
      FramePose framePose = new FramePose(footReferenceFrame);
      framePose.changeFrame(ReferenceFrame.getWorldFrame());

      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("poseReferenceFrame", framePose);

      ReferenceFrame soleReferenceFrame = FootstepUtils.createSoleFrame(poseReferenceFrame, contactablePlaneBody);
      List<FramePoint> expectedContactPoints = FootstepUtils.getContactPointsInFrame(contactablePlaneBody, soleReferenceFrame);
      boolean trustHeight = true;

      Footstep footstep = new Footstep(contactablePlaneBody, poseReferenceFrame, soleReferenceFrame, expectedContactPoints, trustHeight);
      return footstep;
   }

   //TODO: New methods coming from extending State class
   public void doAction()
   {
      doMotionControl();
   }

   public void doTransitionIntoAction()
   {
      initialize();
   }

   public void doTransitionOutOfAction()
   {
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }
}
