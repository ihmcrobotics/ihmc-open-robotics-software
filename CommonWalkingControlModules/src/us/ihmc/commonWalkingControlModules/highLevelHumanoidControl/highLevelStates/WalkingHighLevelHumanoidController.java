package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.calculators.EquivalentConstantCoPCalculator;
import us.ihmc.commonWalkingControlModules.calculators.GainCalculator;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.ChestOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.endEffector.EndEffectorControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.endEffector.EndEffectorControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controlModules.head.HeadOrientationManager;
import us.ihmc.commonWalkingControlModules.controllers.LidarControllerInterface;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorTools;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepProvider;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepUtils;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsDataVisualizer;
import us.ihmc.commonWalkingControlModules.desiredFootStep.UpcomingFootstepList;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingManagers;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPBasedMomentumRateOfChangeControlModule;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.InstantaneousCapturePointPlanner;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointTrajectoryData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController.MomentumControlModuleType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.RootJointAngularAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.packetConsumers.ReinitializeWalkingControllerProvider;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.sensors.HeelSwitch;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightPartialDerivativesData;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTimeDerivativesCalculator;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTimeDerivativesData;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTimeDerivativesSmoother;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.CoMXYTimeDerivativesData;
import us.ihmc.commonWalkingControlModules.trajectories.ContactStatesAndUpcomingFootstepData;
import us.ihmc.commonWalkingControlModules.trajectories.CurrentOrientationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.FlatThenPolynomialCoMHeightTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.HeelPitchTouchdownProvidersManager;
import us.ihmc.commonWalkingControlModules.trajectories.MaximumConstantJerkFinalToeOffAngleComputer;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.QuinticPolynomialTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.SettableOrientationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.SimpleTwoWaypointTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.trajectories.SwingTimeCalculationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.ThirdOrderPolynomialTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.TransferTimeCalculationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointTrajectoryUtils;
import us.ihmc.commonWalkingControlModules.trajectories.YoVariableDoubleProvider;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.kinematics.AverageOrientationCalculator;
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
import us.ihmc.utilities.screwTheory.RigidBody;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.util.PDController;
import com.yobotics.simulationconstructionset.util.errorHandling.WalkingStatusReporter;
import com.yobotics.simulationconstructionset.util.errorHandling.WalkingStatusReporter.ErrorType;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint2d;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector2d;
import com.yobotics.simulationconstructionset.util.statemachines.State;
import com.yobotics.simulationconstructionset.util.statemachines.StateMachine;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransition;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransitionAction;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransitionCondition;
import com.yobotics.simulationconstructionset.util.trajectory.ConstantDoubleProvider;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleProvider;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.PositionTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.TrajectoryParameters;
import com.yobotics.simulationconstructionset.util.trajectory.TrajectoryParametersProvider;
import com.yobotics.simulationconstructionset.util.trajectory.TrajectoryWaypointGenerationMethod;
import com.yobotics.simulationconstructionset.util.trajectory.YoPositionProvider;

public class WalkingHighLevelHumanoidController extends AbstractHighLevelHumanoidControlPattern
{
   private boolean VISUALIZE = true;

   private final static HighLevelState controllerState = HighLevelState.WALKING;
   private final static MomentumControlModuleType MOMENTUM_CONTROL_MODULE_TO_USE = MomentumControlModuleType.OLD;
   private final static double DELAY_TIME_BEFORE_TRUSTING_CONTACTS = 0.12;
   
   private final double PELVIS_YAW_INITIALIZATION_TIME = 1.5;

   private final BooleanYoVariable alreadyBeenInDoubleSupportOnce;

   private static enum WalkingState {LEFT_SUPPORT, RIGHT_SUPPORT, TRANSFER_TO_LEFT_SUPPORT, TRANSFER_TO_RIGHT_SUPPORT, DOUBLE_SUPPORT}

   private final static boolean DEBUG = false;
   private final StateMachine<WalkingState> stateMachine;
   private final CenterOfMassJacobian centerOfMassJacobian;

   private final CoMHeightTrajectoryGenerator centerOfMassHeightTrajectoryGenerator;
   private final CoMHeightTimeDerivativesCalculator coMHeightTimeDerivativesCalculator = new CoMHeightTimeDerivativesCalculator();
   private final CoMHeightTimeDerivativesSmoother coMHeightTimeDerivativesSmoother;

   private final PDController centerOfMassHeightController;
   private final SideDependentList<WalkingState> singleSupportStateEnums = new SideDependentList<WalkingState>(WalkingState.LEFT_SUPPORT,
                                                                              WalkingState.RIGHT_SUPPORT);

   private final SideDependentList<WalkingState> transferStateEnums = new SideDependentList<WalkingState>(WalkingState.TRANSFER_TO_LEFT_SUPPORT,
                                                                         WalkingState.TRANSFER_TO_RIGHT_SUPPORT);

   private final DoubleYoVariable stopInDoubleSupporTrajectoryTime = new DoubleYoVariable("stopInDoubleSupporTrajectoryTime", registry);

   private final BooleanYoVariable loopControllerForever = new BooleanYoVariable("loopControllerForever", "For checking memory and profiling", registry);
   private final BooleanYoVariable justFall = new BooleanYoVariable("justFall", registry);
   
   private final BooleanYoVariable stepOnOrOff = new BooleanYoVariable("stepOnOrOff", registry);

   private final BooleanYoVariable hasMinimumTimePassed = new BooleanYoVariable("hasMinimumTimePassed", registry);
   private final DoubleYoVariable minimumSwingFraction = new DoubleYoVariable("minimumSwingFraction", registry);
   
   // private final FinalDesiredICPCalculator finalDesiredICPCalculator;

   private final BooleanYoVariable rememberFinalICPFromSingleSupport = new BooleanYoVariable("rememberFinalICPFromSingleSupport", registry);
   private final YoFramePoint2d finalDesiredICPInWorld = new YoFramePoint2d("finalDesiredICPInWorld", "", worldFrame, registry);

   private final SideDependentList<FootSwitchInterface> footSwitches;

   private final DoubleYoVariable minOrbitalEnergyForSingleSupport = new DoubleYoVariable("minOrbitalEnergyForSingleSupport", registry);
   private final DoubleYoVariable amountToBeInsideSingleSupport = new DoubleYoVariable("amountToBeInsideSingleSupport", registry);
   private final DoubleYoVariable amountToBeInsideDoubleSupport = new DoubleYoVariable("amountToBeInsideDoubleSupport", registry);

   private final DoubleYoVariable userDesiredPelvisPitch = new DoubleYoVariable("userDesiredPelvisPitch", registry);
   private final SettableOrientationProvider initialPelvisOrientationProvider;
   private final SettableOrientationProvider finalPelvisOrientationProvider;
   private final OrientationTrajectoryGenerator pelvisOrientationTrajectoryGenerator;

   private final SwingTimeCalculationProvider swingTimeCalculationProvider;
   private final TransferTimeCalculationProvider transferTimeCalculationProvider;
   
   private final TrajectoryParametersProvider trajectoryParametersProvider;
   private final SideDependentList<YoVariableDoubleProvider> onToesInitialAngleProviders = new SideDependentList<YoVariableDoubleProvider>();
   private final SideDependentList<YoVariableDoubleProvider> onToesFinalAngleProviders = new SideDependentList<YoVariableDoubleProvider>();

   private final DoubleYoVariable additionalSwingTimeForICP = new DoubleYoVariable("additionalSwingTimeForICP", registry);
   private final BooleanYoVariable stayOnToes = new BooleanYoVariable("stayOnToes", registry);
   private final DoubleYoVariable trailingFootPitch = new DoubleYoVariable("trailingFootPitch", registry);

   private final YoPositionProvider finalPositionProvider;

   private final DoubleYoVariable swingAboveSupportAnkle = new DoubleYoVariable("swingAboveSupportAnkle", registry);
   private final BooleanYoVariable readyToGrabNextFootstep = new BooleanYoVariable("readyToGrabNextFootstep", registry);

   private final HashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters;
   private final InstantaneousCapturePointPlanner instantaneousCapturePointPlanner;
   private final ReinitializeWalkingControllerProvider reinitializeControllerProvider;

   private final SideDependentList<SettableOrientationProvider> finalFootOrientationProviders = new SideDependentList<SettableOrientationProvider>();

   private final ICPBasedMomentumRateOfChangeControlModule icpBasedMomentumRateOfChangeControlModule;

   private final BooleanYoVariable toeOff = new BooleanYoVariable("toeOff", registry);
   private final BooleanYoVariable icpTrajectoryHasBeenInitialized;
   private final BooleanYoVariable doToeOffIfPossible = new BooleanYoVariable("doToeOffIfPossible", registry);
   private final DoubleYoVariable onToesTriangleArea = new DoubleYoVariable("onToesTriangleArea", registry);
   private final DoubleYoVariable onToesTriangleAreaLimit = new DoubleYoVariable("onToesTriangleAreaLimit", registry);
   private final HeelPitchTouchdownProvidersManager heelPitchTouchdownProvidersManager;

   private final UpcomingFootstepList upcomingFootstepList;

   private final AverageOrientationCalculator averageOrientationCalculator = new AverageOrientationCalculator();

   private final ICPAndMomentumBasedController icpAndMomentumBasedController;
   private final EnumYoVariable<RobotSide> upcomingSupportLeg;
   private final EnumYoVariable<RobotSide> supportLeg;
   private final BipedSupportPolygons bipedSupportPolygons;
   private final YoFramePoint capturePoint;
   private final YoFramePoint2d desiredICP;
   private final YoFrameVector2d desiredICPVelocity;

   private final DoubleYoVariable desiredCoMHeightAcceleration;
   private final DoubleYoVariable controllerInitializationTime;

   private final TransferToAndNextFootstepsDataVisualizer transferToAndNextFootstepsDataVisualizer;

   private final BooleanYoVariable ecmpBasedToeOffHasBeenInitialized = new BooleanYoVariable("ecmpBasedToeOffHasBeenInitialized", registry);
   private final boolean useECMPinToeSupportPolygonInsteadICPTriangle = true;
   private final YoFramePoint2d desiredECMP = new YoFramePoint2d("desiredECMP", "", worldFrame, registry);
   private final BooleanYoVariable desiredECMPinSupportPolygon = new BooleanYoVariable("desiredECMPinSupportPolygon", registry);
   private YoFramePoint ecmpViz = new YoFramePoint("ecmpViz", ReferenceFrame.getWorldFrame(), registry);
//   private final DoubleYoVariable trailingLegJacobianDeterminant = new DoubleYoVariable("trailingLegJacobianDeterminant", registry);
   private final DoubleYoVariable trailingLegKneeAngle = new DoubleYoVariable("trailingLegKneeAngle", registry);
   private final DoubleYoVariable toeOffKneeAngleThreashold = new DoubleYoVariable("toeOffKneeAngleThreashold", registry);
   private final BooleanYoVariable useTrailingLegKneeAngleAsToeOffTrigger = new BooleanYoVariable("useLegDeterminantAsToeOffTrigger", registry);
   
   private final DoubleYoVariable trailingLegAnklePitchAngle = new DoubleYoVariable("trailingLegAnklePitchAngle", registry);
   private final DoubleYoVariable toeOffAnklePitchThreashold = new DoubleYoVariable("toeOffAnklePitchThreashold", registry);
   private final BooleanYoVariable useAnklePitchAngleAsToeOffTrigger = new BooleanYoVariable("useAnklePitchAngleAsToeOffTrigger", registry);
   
   
   private final boolean replaceQuinticToeOffSplineByCubicSpline = true;
   private final YoVariableDoubleProvider totalEstimatedToeOffTimeProvider = new YoVariableDoubleProvider("totalEstimatedToeOffTimeProvider", registry);
   
   
   private final BooleanYoVariable toeOffTrigger = new BooleanYoVariable("toeOffTrigger", registry);
   

   private double minimumKneeAngleAfterToeOff = 60*Math.PI/180; 
   private double maximumToeOffAngle = 0.5; 
   private double referenceTime = 0.22; 
   private MaximumConstantJerkFinalToeOffAngleComputer maximumConstantJerkFinalToeOffAngleComputer = new MaximumConstantJerkFinalToeOffAngleComputer();  
   private YoVariableDoubleProvider onToesFinalPitchProvider = new YoVariableDoubleProvider("OnToesFinalPitch", registry);

   private final VariousWalkingProviders variousWalkingProviders;
   private final VariousWalkingManagers variousWalkingManagers;

   public WalkingHighLevelHumanoidController(VariousWalkingProviders variousWalkingProviders, VariousWalkingManagers variousWalkingManagers,
         SideDependentList<FootSwitchInterface> footSwitches,
           DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, 
           CoMHeightTrajectoryGenerator centerOfMassHeightTrajectoryGenerator,
           SideDependentList<PositionTrajectoryGenerator> footPositionTrajectoryGenerators,
           SideDependentList<DoubleTrajectoryGenerator> heelPitchTrajectoryGenerators, HeelPitchTouchdownProvidersManager heelPitchTouchdownProvidersManager,
           SwingTimeCalculationProvider swingTimeCalculationProvider, TransferTimeCalculationProvider transferTimeCalculationProvider, 
           YoPositionProvider finalPositionProvider,
           TrajectoryParametersProvider trajectoryParametersProvider, boolean stayOntoes, double desiredPelvisPitch, double trailingFootPitch,
           WalkingControllerParameters walkingControllerParameters, ICPBasedMomentumRateOfChangeControlModule momentumRateOfChangeControlModule,
           RootJointAngularAccelerationControlModule rootJointAngularAccelerationControlModule, LidarControllerInterface lidarControllerInterface,
           InstantaneousCapturePointPlanner instantaneousCapturePointPlanner, 
           ICPAndMomentumBasedController icpAndMomentumBasedController, MomentumBasedController momentumBasedController, WalkingStatusReporter walkingStatusReporter)
   {
      
      super(variousWalkingProviders, variousWalkingManagers,rootJointAngularAccelerationControlModule, momentumBasedController, walkingControllerParameters, 
            lidarControllerInterface, dynamicGraphicObjectsListRegistry, controllerState);
     
      super.addUpdatables(icpAndMomentumBasedController.getUpdatables());

      this.variousWalkingProviders = variousWalkingProviders;
      this.variousWalkingManagers = variousWalkingManagers;
      
      setupManagers(variousWalkingManagers);
      
      FootstepProvider footstepProvider = variousWalkingProviders.getFootstepProvider();
      HashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters = variousWalkingProviders.getMapFromFootstepsToTrajectoryParameters();
      this.reinitializeControllerProvider = variousWalkingProviders.getReinitializeWalkingControllerProvider();

      toeOffKneeAngleThreashold.set(45*Math.PI/180); //52*Math.PI/180);
      useTrailingLegKneeAngleAsToeOffTrigger.set(true);
      
      
      toeOffAnklePitchThreashold.set(-40*Math.PI/180);
      useAnklePitchAngleAsToeOffTrigger.set(true);
      
            
      maximumConstantJerkFinalToeOffAngleComputer.reinitialize(maximumToeOffAngle, referenceTime);
      
      
      
      if (dynamicGraphicObjectsListRegistry == null)
      {
         VISUALIZE = false;
      }

      if (VISUALIZE)
      {
         transferToAndNextFootstepsDataVisualizer = new TransferToAndNextFootstepsDataVisualizer(registry, dynamicGraphicObjectsListRegistry);
      }
      else
      {
         transferToAndNextFootstepsDataVisualizer = null;
      }

      if (VISUALIZE)
      {
         DynamicGraphicPosition dynamicGraphicPositionECMP = new DynamicGraphicPosition("ecmpviz", ecmpViz, 0.002, YoAppearance.BlueViolet());
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("ecmpviz", dynamicGraphicPositionECMP);
         dynamicGraphicObjectsListRegistry.registerArtifact("ecmpviz", dynamicGraphicPositionECMP.createArtifact());
      }

      // Getting parameters from the icpAndMomentumBasedController
      this.icpAndMomentumBasedController = icpAndMomentumBasedController;
      
//      contactStates = momentumBasedController.getContactStates();
      upcomingSupportLeg = momentumBasedController.getUpcomingSupportLeg();
      supportLeg = icpAndMomentumBasedController.getYoSupportLeg();
      capturePoint = icpAndMomentumBasedController.getCapturePoint();
      desiredICP = icpAndMomentumBasedController.getDesiredICP();
      desiredICPVelocity = icpAndMomentumBasedController.getDesiredICPVelocity();
      bipedSupportPolygons = icpAndMomentumBasedController.getBipedSupportPolygons();
      desiredCoMHeightAcceleration = icpAndMomentumBasedController.getDesiredCoMHeightAcceleration();
      centerOfMassJacobian = momentumBasedController.getCenterOfMassJacobian();

      coMHeightTimeDerivativesSmoother = new CoMHeightTimeDerivativesSmoother(controlDT, registry);

      // this.finalDesiredICPCalculator = finalDesiredICPCalculator;
      this.centerOfMassHeightTrajectoryGenerator = centerOfMassHeightTrajectoryGenerator;
      this.swingTimeCalculationProvider = swingTimeCalculationProvider;
      this.transferTimeCalculationProvider = transferTimeCalculationProvider;
      
      this.trajectoryParametersProvider = trajectoryParametersProvider;
      this.mapFromFootstepsToTrajectoryParameters = mapFromFootstepsToTrajectoryParameters;
      this.footSwitches = footSwitches;
      this.heelPitchTouchdownProvidersManager = heelPitchTouchdownProvidersManager;
      this.icpBasedMomentumRateOfChangeControlModule = momentumRateOfChangeControlModule;

      this.instantaneousCapturePointPlanner = instantaneousCapturePointPlanner;

      this.upcomingFootstepList = new UpcomingFootstepList(footstepProvider, registry);

      this.centerOfMassHeightController = new PDController("comHeight", registry);
      centerOfMassHeightController.setProportionalGain(40.0);
      double zeta = 1.0;
      centerOfMassHeightController.setDerivativeGain(GainCalculator.computeDerivativeGain(centerOfMassHeightController.getProportionalGain(), zeta));

      String namePrefix = "walking";

      this.stateMachine = new StateMachine<WalkingState>(namePrefix + "State", namePrefix + "SwitchTime", WalkingState.class, yoTime, registry);    // this is used by name, and it is ugly.

      this.finalPositionProvider = finalPositionProvider;

      this.icpTrajectoryHasBeenInitialized = new BooleanYoVariable("icpTrajectoryHasBeenInitialized", registry);

      rememberFinalICPFromSingleSupport.set(false);    // true);
      finalDesiredICPInWorld.set(Double.NaN, Double.NaN);

      coefficientOfFriction.set(0.6);    // TODO: tune?

      setupLegJacobians(fullRobotModel);

      setupFootControlModules(footPositionTrajectoryGenerators, heelPitchTrajectoryGenerators);

      initialPelvisOrientationProvider = new SettableOrientationProvider("initialPelvis", worldFrame, registry);
      finalPelvisOrientationProvider = new SettableOrientationProvider("finalPelvis", worldFrame, registry);
      this.pelvisOrientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator("pelvis", worldFrame, swingTimeCalculationProvider,
              initialPelvisOrientationProvider, finalPelvisOrientationProvider, registry);

      setUpStateMachine();
      readyToGrabNextFootstep.set(true);

      minOrbitalEnergyForSingleSupport.set(0.007);    // 0.008
      amountToBeInsideSingleSupport.set(0.0);
      amountToBeInsideDoubleSupport.set(0.03);    // 0.02);    // TODO: necessary for stairs...
      transferTimeCalculationProvider.setTransferTime();   
      
      totalEstimatedToeOffTimeProvider.set(transferTimeCalculationProvider.getValue());
            
      stopInDoubleSupporTrajectoryTime.set(0.5);
      this.userDesiredPelvisPitch.set(desiredPelvisPitch);
      this.stayOnToes.set(stayOntoes);
      this.trailingFootPitch.set(trailingFootPitch);
      kJointPositionControl.set(100.0);
      zetaJointPositionControl.set(1.0);
      onToesTriangleAreaLimit.set(0.01);
      doToeOffIfPossible.set(walkingControllerParameters.doToeOffIfPossible());

      additionalSwingTimeForICP.set(0.1);
      minimumSwingFraction.set(0.8);

      double initialLeadingFootPitch = 0.0;    // 0.05;
      upcomingSupportLeg.set(RobotSide.RIGHT);    // TODO: stairs hack, so that the following lines use the correct leading leg
      RobotSide leadingLeg = getUpcomingSupportLeg();
      onToesInitialAngleProviders.get(leadingLeg).set(initialLeadingFootPitch);
//      onToesFinalAngleProviders.get(leadingLeg).set(initialLeadingFootPitch);

      RobotSide trailingLeg = leadingLeg.getOppositeSide();
      onToesInitialAngleProviders.get(trailingLeg).set(this.trailingFootPitch.getDoubleValue());

      controllerInitializationTime = new DoubleYoVariable("controllerInitializationTime", registry);
      alreadyBeenInDoubleSupportOnce = new BooleanYoVariable("alreadyBeenInDoubleSupportOnce", registry);

   }


   protected void setupFootControlModules(SideDependentList<PositionTrajectoryGenerator> footPositionTrajectoryGenerators,
           SideDependentList<DoubleTrajectoryGenerator> heelPitchTrajectoryGenerators)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody bipedFoot = feet.get(robotSide);
         
         //TODO: If we know the surface normal here, use it.
         FrameVector normalContactVector = null;
         momentumBasedController.setPlaneContactState(bipedFoot, bipedFoot.getContactPoints2d(), coefficientOfFriction.getDoubleValue(), normalContactVector);
         
//         ContactablePlaneBody bipedFoot = feet.get(robotSide);
//         contactStates.get(bipedFoot).set(bipedFoot.getContactPoints2d(), coefficientOfFriction.getDoubleValue());    // flat feet
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
         
 
         DoubleProvider onToesInitialPitchVelocityProvider = new ConstantDoubleProvider(0.0);
         

         DoubleTrajectoryGenerator onToesPitchTrajectoryGenerator; 

         
         
         if (replaceQuinticToeOffSplineByCubicSpline)
         {
               onToesFinalPitchProvider.set(maximumConstantJerkFinalToeOffAngleComputer.getMaximumFeasibleConstantJerkFinalToeOffAngle
                     (onToesInitialPitchProvider.getValue(), instantaneousCapturePointPlanner.getEstimatedTimeRemainingForState(yoTime.getDoubleValue())));

            
            DoubleProvider onToesTrajectoryTimeProvider = totalEstimatedToeOffTimeProvider; 
            
            onToesPitchTrajectoryGenerator = new ThirdOrderPolynomialTrajectoryGenerator(sideString + "OnToesPitch",
                  onToesInitialPitchProvider, onToesInitialPitchVelocityProvider,
                  onToesFinalPitchProvider, onToesTrajectoryTimeProvider, registry);
                
         }
         else
         {
    
            DoubleProvider onToesTrajectoryTimeProvider = transferTimeCalculationProvider;
            
            DoubleProvider onToesFinalPitchVelocityProvider = new ConstantDoubleProvider(walkingControllerParameters.getFinalToeOffPitchAngularVelocity());

            onToesPitchTrajectoryGenerator = new QuinticPolynomialTrajectoryGenerator(sideString + "OnToesPitch",
                                                                          onToesInitialPitchProvider, onToesInitialPitchVelocityProvider,
                                                                          onToesFinalPitchProvider, onToesFinalPitchVelocityProvider,
                                                                          onToesTrajectoryTimeProvider, registry);
         }

         onToesInitialAngleProviders.put(robotSide, onToesInitialPitchProvider);
         
         
         if (!useECMPinToeSupportPolygonInsteadICPTriangle)
         {
            onToesFinalAngleProviders.put(robotSide, onToesFinalPitchProvider);
         }
         

         GeometricJacobian jacobian = legJacobians.get(robotSide);
         EndEffectorControlModule endEffectorControlModule = new EndEffectorControlModule(bipedFoot, jacobian, swingPositionTrajectoryGenerator,
                                                                heelPitchTrajectoryGenerator, swingOrientationTrajectoryGenerator,
                                                                onToesPitchTrajectoryGenerator, momentumBasedController, registry);
         endEffectorControlModule.setParameters(3e-2, 500.0);
         footEndEffectorControlModules.put(bipedFoot, endEffectorControlModule);
      }
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
      for (RobotSide robotSide : RobotSide.values)
      {
         EndEffectorControlModule swingEndEffectorControlModule = footEndEffectorControlModules.get(feet.get(robotSide.getOppositeSide()));
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

         ContactablePlaneBody sameSideFoot = feet.get(robotSide);
         SingleSupportToTransferToCondition doneWithSingleSupportAndTransferToOppositeSideCondition = new SingleSupportToTransferToCondition(sameSideFoot, swingEndEffectorControlModule);
         StateTransition<WalkingState> toTransferOppositeSide = new StateTransition<WalkingState>(transferStateEnums.get(robotSide.getOppositeSide()),
               doneWithSingleSupportAndTransferToOppositeSideCondition, resetSwingTrajectoryDoneAction);
         singleSupportState.addStateTransition(toTransferOppositeSide);
      
         // Sometimes need transfer to same side when two steps are commanded on the same side. Otherwise, the feet cross over.
         ContactablePlaneBody oppositeSideFoot = feet.get(robotSide.getOppositeSide());
         SingleSupportToTransferToCondition doneWithSingleSupportAndTransferToSameSideCondition = new SingleSupportToTransferToCondition(oppositeSideFoot, swingEndEffectorControlModule);
         StateTransition<WalkingState> toTransferSameSide = new StateTransition<WalkingState>(transferStateEnums.get(robotSide),
               doneWithSingleSupportAndTransferToSameSideCondition, resetSwingTrajectoryDoneAction);
         singleSupportState.addStateTransition(toTransferSameSide);
         
         stateMachine.addState(singleSupportState);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         StateTransition<WalkingState> toTransfer = new StateTransition<WalkingState>(transferStateEnums.get(robotSide),
                                                       new DoneWithDoubleSupportCondition(robotSide));
         doubleSupportState.addStateTransition(toTransfer);
      }
   }

   private RigidBody baseForHeadOrientationControl;
   private GeometricJacobian jacobianForHeadOrientationControl;
   
   public void setupManagers(VariousWalkingManagers variousWalkingManagers)
   {
      baseForHeadOrientationControl = fullRobotModel.getElevator();
      HeadOrientationManager headOrientationManager = variousWalkingManagers.getHeadOrientationManager();
      String[] headOrientationControlJointNames = walkingControllerParameters.getDefaultHeadOrientationControlJointNames(); 

      jacobianForHeadOrientationControl = headOrientationManager.createJacobian(fullRobotModel, baseForHeadOrientationControl, headOrientationControlJointNames);
   }
  
   public void initialize()
   {
      super.initialize();
      
      momentumBasedController.setMomentumControlModuleToUse(MOMENTUM_CONTROL_MODULE_TO_USE);
      momentumBasedController.setDelayTimeBeforeTrustingContacts(DELAY_TIME_BEFORE_TRUSTING_CONTACTS);
      
      initializeContacts();

      ChestOrientationManager chestOrientationManager = variousWalkingManagers.getChestOrientationManager();
      chestOrientationManager.turnOff();

      HeadOrientationManager headOrientationManager = variousWalkingManagers.getHeadOrientationManager();
      double headKp = 40.0;
      double headZeta = 1.0;
      double headKd = GainCalculator.computeDerivativeGain(headKp, headZeta);
      headOrientationManager.setUp(baseForHeadOrientationControl, jacobianForHeadOrientationControl, headKp, headKp, headKp, headKd, headKd, headKd);
      
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

   private void initializeContacts()
   {
      momentumBasedController.clearContacts();

      for (ContactablePlaneBody contactablePlaneBody : momentumBasedController.getContactablePlaneFeet())
      {
         setFlatFootContactState(contactablePlaneBody);
      }
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
         checkForReinitialization();
         RobotSide trailingLegSide;
         RobotSide leadingLegSide;

         if (transferToSide == null)
         {
            trailingLegSide = RobotSide.LEFT;
            leadingLegSide = RobotSide.RIGHT;
         }
         else
         {
            trailingLegSide = transferToSide.getOppositeSide();
            leadingLegSide = transferToSide;
         }

         ContactablePlaneBody transferFoot = feet.get(transferToSide);

         if ((footEndEffectorControlModules.get(transferFoot) != null) && footEndEffectorControlModules.get(transferFoot).onHeel()
                 && footSwitches.get(transferToSide).hasFootHitGround())
         {
            setFlatFootContactState(transferFoot);
         }

         // note: this has to be done before the ICP trajectory generator is initialized, since it is using nextFootstep
         // TODO: Make a LOADING state and clean all of these timing hacks up.
         boolean doneFinishingSingleSupportTransfer = instantaneousCapturePointPlanner.isPerformingICPDoubleSupport();
         double estimatedTimeRemainingForState = instantaneousCapturePointPlanner.getEstimatedTimeRemainingForState(yoTime.getDoubleValue()); 
         
         if (doneFinishingSingleSupportTransfer || estimatedTimeRemainingForState < 0.02)
         {
            upcomingFootstepList.checkForFootsteps(momentumBasedController.getPointPositionGrabber(), readyToGrabNextFootstep, upcomingSupportLeg,
                  feet);
            checkForSteppingOnOrOff(transferToSide);
         }


         
         initializeICPPlannerIfNecessary();


         if (instantaneousCapturePointPlanner.isDone(yoTime.getDoubleValue()) && (transferToSide == null))
         {
            desiredICPVelocity.set(0.0, 0.0);
         }
         else
         {
            FramePoint2d desiredICPLocal = new FramePoint2d(desiredICP.getReferenceFrame());
            FrameVector2d desiredICPVelocityLocal = new FrameVector2d(desiredICPVelocity.getReferenceFrame());
            FramePoint2d ecmpLocal = new FramePoint2d(ReferenceFrame.getWorldFrame());

            FramePoint2d capturePoint2d = capturePoint.getFramePoint2dCopy();
            
            instantaneousCapturePointPlanner.getICPPositionAndVelocity(
                  desiredICPLocal, desiredICPVelocityLocal, ecmpLocal, 
                  capturePoint2d, yoTime.getDoubleValue());
            desiredICP.set(desiredICPLocal);
            desiredICPVelocity.set(desiredICPVelocityLocal);

            desiredECMP.set(ecmpLocal);

            if (VISUALIZE)
            {
               ecmpViz.set(desiredECMP.getX(), desiredECMP.getY(), 0.0);
            }
         }
         
         trailingLegKneeAngle.set(fullRobotModel.getLegJoint(trailingLegSide, LegJointName.KNEE).getQ());
         
         trailingLegAnklePitchAngle.set(fullRobotModel.getLegJoint(trailingLegSide, LegJointName.ANKLE_PITCH).getQ());
         
         
         initializeECMPbasedToeOffIfNotInitializedYet();

         // Only during the first few seconds, we will control the pelvis orientation based on midfeetZup
         if (((yoTime.getDoubleValue() - controllerInitializationTime.getDoubleValue()) < PELVIS_YAW_INITIALIZATION_TIME)
                 &&!alreadyBeenInDoubleSupportOnce.getBooleanValue())
         {
            setDesiredPelvisYawToAverageOfFeetOnStartupOnly();
         }

         // keep desired pelvis orientation as it is
         desiredPelvisAngularVelocity.set(0.0, 0.0, 0.0);
         desiredPelvisAngularAcceleration.set(0.0, 0.0, 0.0);

      }

      public void initializeICPPlannerIfNecessary()
      {
         if (!icpTrajectoryHasBeenInitialized.getBooleanValue() && instantaneousCapturePointPlanner.isDone(yoTime.getDoubleValue()))
         {
//            System.out.println("Initializing ICP planner. transferToSide = " + transferToSide);

            Pair<FramePoint2d, Double> finalDesiredICPAndTrajectoryTime = computeFinalDesiredICPAndTrajectoryTime();

            FramePoint2d finalDesiredICP = finalDesiredICPAndTrajectoryTime.first();
            double trajectoryTime = finalDesiredICPAndTrajectoryTime.second();

            finalDesiredICP.changeFrame(desiredICP.getReferenceFrame());

            if (!useECMPinToeSupportPolygonInsteadICPTriangle)
            {
               if (!stayOnToes.getBooleanValue() && (transferToSide != null))    // the only case left for determining the contact state of the trailing foot
               {
                  RobotSide trailingLeg = transferToSide.getOppositeSide();
                  ContactablePlaneBody supportFoot = feet.get(trailingLeg);
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
            }

            icpAndMomentumBasedController.updateBipedSupportPolygons(bipedSupportPolygons);    // need to always update biped support polygons after a change to the contact states
            icpTrajectoryHasBeenInitialized.set(true);
         }
      }
      
      

      public void initializeECMPbasedToeOffIfNotInitializedYet()
      {
         if (!ecmpBasedToeOffHasBeenInitialized.getBooleanValue())
         {
            if (!stayOnToes.getBooleanValue() && (transferToSide != null))    // the only case left for determining the contact state of the trailing foot
            {
               if (useECMPinToeSupportPolygonInsteadICPTriangle)
               {
                  RobotSide trailingLeg = transferToSide.getOppositeSide();

                  ContactablePlaneBody supportFoot = feet.get(trailingLeg);

                  ContactablePlaneBody trailingFoot = feet.get(trailingLeg);
                  ContactablePlaneBody leadingFoot = feet.get(transferToSide);
                  FrameConvexPolygon2d OnToesSupportPolygon = getOnToesSupportPolygon(trailingFoot, leadingFoot);
                  boolean desiredECMPOK = OnToesSupportPolygon.isPointInside(desiredECMP.getFramePoint2dCopy());
                  desiredECMPinSupportPolygon.set(desiredECMPOK);
                  toeOff.set(desiredECMPOK && doToeOffIfPossible.getBooleanValue());

                  double minimumToeOffTime = 0.0; // JOJO: might want to set a value here
                  double remainingToeOffTime = instantaneousCapturePointPlanner.getEstimatedTimeRemainingForState(yoTime.getDoubleValue()); 
                  

                  toeOffTrigger.set
                  (
                        toeOff.getBooleanValue() && (remainingToeOffTime > minimumToeOffTime) && 
                        (
                              (!useTrailingLegKneeAngleAsToeOffTrigger.getBooleanValue() && !useAnklePitchAngleAsToeOffTrigger.getBooleanValue()) || 
                              (trailingLegKneeAngle.getDoubleValue() < toeOffKneeAngleThreashold.getDoubleValue() && useTrailingLegKneeAngleAsToeOffTrigger.getBooleanValue()) || 
                              (trailingLegAnklePitchAngle.getDoubleValue() < toeOffAnklePitchThreashold.getDoubleValue() && useAnklePitchAngleAsToeOffTrigger.getBooleanValue())
                              
                              && (Math.abs(trailingLegKneeAngle.getDoubleValue()) < Math.PI - (minimumKneeAngleAfterToeOff + maximumToeOffAngle))
                        )
                        
                  );
                  
                  
                  if (toeOffTrigger.getBooleanValue())
                  {
                     onToesFinalPitchProvider.set(maximumConstantJerkFinalToeOffAngleComputer.getMaximumFeasibleConstantJerkFinalToeOffAngle
                           (onToesInitialAngleProviders.get(trailingLeg).getValue(), remainingToeOffTime));

                     
                     setOnToesContactState(supportFoot);
                     icpAndMomentumBasedController.updateBipedSupportPolygons(bipedSupportPolygons);    // need to always update biped support polygons after a change to the contact states
                     ecmpBasedToeOffHasBeenInitialized.set(true);
                     

                     totalEstimatedToeOffTimeProvider.set(instantaneousCapturePointPlanner.getEstimatedTimeRemainingForState(yoTime.getDoubleValue()));
                  }
                  else
                  {
                     setFlatFootContactState(supportFoot);
                  }
               }
            }
         }
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

         else if (rememberFinalICPFromSingleSupport.getBooleanValue() &&!finalDesiredICPInWorld.containsNaN())
         {
            FramePoint2d finalDesiredICP = finalDesiredICPInWorld.getFramePoint2dCopy();
            double trajectoryTime = transferTimeCalculationProvider.getValue();

            finalDesiredICPAndTrajectoryTime = new Pair<FramePoint2d, Double>(finalDesiredICP, trajectoryTime);
         }

         else
         {
            boolean inInitialize = false;
            TransferToAndNextFootstepsData transferToAndNextFootstepsData = createTransferToAndNextFootstepDataForDoubleSupport(transferToSide, inInitialize );

            instantaneousCapturePointPlanner.initializeDoubleSupport(transferToAndNextFootstepsData, yoTime.getDoubleValue());

            FramePoint2d finalDesiredICP = instantaneousCapturePointPlanner.getFinalDesiredICP();
            double trajectoryTime = transferTimeCalculationProvider.getValue();

            finalDesiredICPInWorld.set(finalDesiredICP.changeFrameCopy(worldFrame));
            finalDesiredICPAndTrajectoryTime = new Pair<FramePoint2d, Double>(finalDesiredICP, trajectoryTime);
         }

         return finalDesiredICPAndTrajectoryTime;
      }

     
      public TransferToAndNextFootstepsData createTransferToAndNextFootstepDataForDoubleSupport(RobotSide transferToSide, boolean inInitialize)
      {
         Footstep transferFromFootstep = createFootstepFromFootAndContactablePlaneBody(referenceFrames.getFootFrame(transferToSide.getOppositeSide()),
                                            feet.get(transferToSide.getOppositeSide()));
         Footstep transferToFootstep = createFootstepFromFootAndContactablePlaneBody(referenceFrames.getFootFrame(transferToSide),
                                          feet.get(transferToSide));

         FrameConvexPolygon2d transferToFootPolygon = computeFootPolygon(transferToSide, referenceFrames.getSoleFrame(transferToSide));

         Footstep nextFootstep, nextNextFootstep;
         
         if (inInitialize)
         {
            // Haven't popped the footstep off yet...
            nextFootstep = upcomingFootstepList.getNextNextFootstep();
            nextNextFootstep = upcomingFootstepList.getNextNextNextFootstep();
         }
         else
         {
            nextFootstep = upcomingFootstepList.getNextFootstep();
            nextNextFootstep = upcomingFootstepList.getNextNextFootstep();
         }
         
         double timeAllottedForSingleSupportForICP = swingTimeCalculationProvider.getValue() + additionalSwingTimeForICP.getDoubleValue();

         TransferToAndNextFootstepsData transferToAndNextFootstepsData = new TransferToAndNextFootstepsData();
         transferToAndNextFootstepsData.setTransferFromFootstep(transferFromFootstep);
         transferToAndNextFootstepsData.setTransferToFootstep(transferToFootstep);
         transferToAndNextFootstepsData.setTransferToFootPolygonInSoleFrame(transferToFootPolygon);
         transferToAndNextFootstepsData.setTransferToSide(transferToSide);
         transferToAndNextFootstepsData.setNextFootstep(nextFootstep);
         transferToAndNextFootstepsData.setNextNextFootstep(nextNextFootstep);
         transferToAndNextFootstepsData.setEstimatedStepTime(timeAllottedForSingleSupportForICP + transferTimeCalculationProvider.getValue());
         transferToAndNextFootstepsData.setW0(icpAndMomentumBasedController.getOmega0());
         transferToAndNextFootstepsData.setDoubleSupportDuration(transferTimeCalculationProvider.getValue());
         transferToAndNextFootstepsData.setSingleSupportDuration(timeAllottedForSingleSupportForICP);
         double doubleSupportInitialTransferDuration = 0.4;    // TODO: Magic Number
         transferToAndNextFootstepsData.setDoubleSupportInitialTransferDuration(doubleSupportInitialTransferDuration);
         boolean stopIfReachedEnd = (upcomingFootstepList.getNumberOfFootstepsToProvide() <= 3);    // TODO: Magic Number
         transferToAndNextFootstepsData.setStopIfReachedEnd(stopIfReachedEnd);

         if (VISUALIZE)
         {
            transferToAndNextFootstepsDataVisualizer.visualizeFootsteps(transferToAndNextFootstepsData);
         }

         return transferToAndNextFootstepsData;
      }

      @Override
      public void doTransitionIntoAction()
      {
         desiredECMPinSupportPolygon.set(false);
         toeOffTrigger.set(false);
         ecmpBasedToeOffHasBeenInitialized.set(false);

         icpTrajectoryHasBeenInitialized.set(false);
         icpTrajectoryHasBeenInitialized.set(false);
         if (DEBUG)
            System.out.println("WalkingHighLevelHumanoidController: enteringDoubleSupportState");
         setSupportLeg(null);    // TODO: check if necessary

         // TODO: simplify the following
         if (transferToSide != null)
         {
            RobotSide trailingLeg = transferToSide.getOppositeSide();
         }

         if (stayOnToes.getBooleanValue())
         {
            for (RobotSide robotSide : RobotSide.values)
            {
               setOnToesContactState(feet.get(robotSide));
            }
         }
         else
         {
            if (transferToSide == null)
            {
               for (RobotSide robotSide : RobotSide.values)
               {
                  setFlatFootContactState(feet.get(robotSide));
               }
            }
            else if (landOnHeels())
            {
               setOnHeelContactState(feet.get(transferToSide));
            }
            else
            {
               setFlatFootContactState(feet.get(transferToSide));

               // still need to determine contact state for trailing leg. This is done in doAction as soon as the previous ICP trajectory is done
            }
         }

         if (!instantaneousCapturePointPlanner.isDone(yoTime.getDoubleValue()) && (transferToSide != null))
         {
            Footstep transferToFootstep = createFootstepFromFootAndContactablePlaneBody(referenceFrames.getFootFrame(transferToSide),
                                             feet.get(transferToSide));
            TransferToAndNextFootstepsData transferToAndNextFootstepsData = createTransferToAndNextFootstepDataForSingleSupport(transferToFootstep,
                                                                               transferToSide);

            instantaneousCapturePointPlanner.reInitializeSingleSupport(transferToAndNextFootstepsData, yoTime.getDoubleValue());
         }

         icpAndMomentumBasedController.updateBipedSupportPolygons(bipedSupportPolygons);    // need to always update biped support polygons after a change to the contact states


         RobotSide transferToSideToUseInFootstepData = transferToSide;
         if (transferToSideToUseInFootstepData == null) transferToSideToUseInFootstepData = RobotSide.LEFT; //Arbitrary here.
         
         if (!centerOfMassHeightTrajectoryGenerator.hasBeenInitializedWithNextStep())
         {
//            System.out.println("Initializing centerOfMassHeightTrajectoryGenerator. transferToSide = " + transferToSide);

            boolean inInitialize = true;
            TransferToAndNextFootstepsData transferToAndNextFootstepsDataForDoubleSupport = createTransferToAndNextFootstepDataForDoubleSupport(transferToSideToUseInFootstepData, inInitialize);

            centerOfMassHeightTrajectoryGenerator.initialize(transferToAndNextFootstepsDataForDoubleSupport, transferToAndNextFootstepsDataForDoubleSupport.getTransferToSide(), null, getContactStatesList());
         }
      
         //         RobotSide transferToSideToUseInFootstepData = transferToSide;
//         if (transferToSideToUseInFootstepData == null)
//            transferToSideToUseInFootstepData = RobotSide.LEFT;    // Arbitrary here.
//         TransferToAndNextFootstepsData transferToAndNextFootstepsDataForDoubleSupport =
//            createTransferToAndNextFootstepDataForDoubleSupport(transferToSideToUseInFootstepData);
//
////       centerOfMassHeightTrajectoryGenerator.initialize(transferToAndNextFootstepsDataForDoubleSupport.getTransferToSide(), null, getContactStatesList());
//         centerOfMassHeightTrajectoryGenerator.initialize(transferToAndNextFootstepsDataForDoubleSupport,
//                 transferToAndNextFootstepsDataForDoubleSupport.getTransferToSide(), null, getContactStatesList());
      }

      @Override
      public void doTransitionOutOfAction()
      {
         desiredECMPinSupportPolygon.set(false);
         toeOffTrigger.set(false);
         ecmpBasedToeOffHasBeenInitialized.set(false);

         alreadyBeenInDoubleSupportOnce.set(true);

         if (DEBUG)
            System.out.println("WalkingHighLevelHumanoidController: leavingDoubleSupportState");
         desiredICPVelocity.set(0.0, 0.0);
         manipulationControlModule.prepareForLocomotion();
      }
   }


   private void setDesiredPelvisYawToAverageOfFeetOnStartupOnly()
   {
      FrameOrientation averageOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame());
      averageOrientationCalculator.computeAverageOrientation(averageOrientation, feet.get(RobotSide.LEFT).getPlaneFrame(),
              feet.get(RobotSide.RIGHT).getPlaneFrame(), ReferenceFrame.getWorldFrame());

      double[] yawPitchRoll = averageOrientation.getYawPitchRoll();

      averageOrientation.setYawPitchRoll(yawPitchRoll[0], userDesiredPelvisPitch.getDoubleValue(), 0.0);
      desiredPelvisOrientation.set(averageOrientation);
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

   private FrameConvexPolygon2d getOnToesSupportPolygon(ContactablePlaneBody trailingFoot, ContactablePlaneBody leadingFoot)
   {
      List<FramePoint> toePoints = getToePoints(trailingFoot);
      List<FramePoint> leadingFootPoints = leadingFoot.getContactPoints();

      List<FramePoint2d> allPoints = new ArrayList<FramePoint2d>();
      for (FramePoint framePoint : toePoints)
      {
         framePoint.changeFrame(ReferenceFrame.getWorldFrame());
         allPoints.add(framePoint.toFramePoint2d());
      }

      for (FramePoint framePoint : leadingFootPoints)
      {
         framePoint.changeFrame(ReferenceFrame.getWorldFrame());
         allPoints.add(framePoint.toFramePoint2d());
      }

      return new FrameConvexPolygon2d(allPoints);
   }

   private class SingleSupportState extends State<WalkingState>
   {
      private final RobotSide swingSide;
      private final FrameOrientation desiredPelvisOrientationToPack;
      private final FrameVector desiredPelvisAngularVelocityToPack;
      private final FrameVector desiredPelvisAngularAccelerationToPack;
      private final ErrorType[] singleSupportErrorToMonitor = new ErrorType[] {ErrorType.COM_Z, ErrorType.ICP_X, ErrorType.ICP_Y, ErrorType.PELVIS_ORIENTATION};

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
         checkForReinitialization();
         FramePoint2d desiredICPLocal = new FramePoint2d(desiredICP.getReferenceFrame());
         FrameVector2d desiredICPVelocityLocal = new FrameVector2d(desiredICPVelocity.getReferenceFrame());
         FramePoint2d ecmpLocal = new FramePoint2d(ReferenceFrame.getWorldFrame());

         FramePoint2d capturePoint2d = capturePoint.getFramePoint2dCopy();

         instantaneousCapturePointPlanner.getICPPositionAndVelocity(
               desiredICPLocal, desiredICPVelocityLocal, ecmpLocal, 
               capturePoint2d, yoTime.getDoubleValue());
         desiredICP.set(desiredICPLocal);
         desiredICPVelocity.set(desiredICPVelocityLocal);

         desiredECMP.set(ecmpLocal);

         if (VISUALIZE)
         {
            ecmpViz.set(desiredECMP.getX(), desiredECMP.getY(), 0.0);
         }

         pelvisOrientationTrajectoryGenerator.compute(stateMachine.timeInCurrentState());
         pelvisOrientationTrajectoryGenerator.get(desiredPelvisOrientationToPack);
         pelvisOrientationTrajectoryGenerator.packAngularVelocity(desiredPelvisAngularVelocityToPack);
         pelvisOrientationTrajectoryGenerator.packAngularAcceleration(desiredPelvisAngularAccelerationToPack);
         desiredPelvisOrientation.set(desiredPelvisOrientationToPack);
         desiredPelvisAngularVelocity.set(desiredPelvisAngularVelocityToPack);
         desiredPelvisAngularAcceleration.set(desiredPelvisAngularAccelerationToPack);

      }

      @Override
      public void doTransitionIntoAction()
      {
         footSwitches.get(swingSide).reset();
         if (footSwitches.get(swingSide) instanceof HeelSwitch)
            ((HeelSwitch) footSwitches.get(swingSide)).resetHeelSwitch();

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
            setOnToesContactState(feet.get(supportSide));
         }
         else
         {
            setFlatFootContactState(feet.get(supportSide));
         }

         finalPositionProvider.set(nextFootstep.getPositionInFrame(worldFrame));

         SideDependentList<Transform3D> footToWorldTransform = new SideDependentList<Transform3D>();
         for (RobotSide robotSide : RobotSide.values)
         {
            Transform3D transform = feet.get(robotSide).getBodyFrame().getTransformToDesiredFrame(worldFrame);
            footToWorldTransform.set(robotSide, transform);
         }

         Vector3d initialVectorPosition = new Vector3d();
         footToWorldTransform.get(supportSide.getOppositeSide()).get(initialVectorPosition);
         FramePoint initialFramePosition = new FramePoint(worldFrame, initialVectorPosition);
         FramePoint finalPosition = new FramePoint(worldFrame);
         finalPositionProvider.get(finalPosition);
         double stepDistance = initialFramePosition.distance(finalPosition);
         swingTimeCalculationProvider.setSwingTime(stepDistance);
         transferTimeCalculationProvider.setTransferTime();

         trajectoryParametersProvider.set(mapFromFootstepsToTrajectoryParameters.get(nextFootstep));
         finalFootOrientationProviders.get(swingSide).setOrientation(nextFootstep.getOrientationInFrame(worldFrame));

         initialPelvisOrientationProvider.setOrientation(desiredPelvisOrientation.getFrameOrientationCopy());

         FrameOrientation finalPelvisOrientation = nextFootstep.getOrientationInFrame(worldFrame);
         finalPelvisOrientation.setYawPitchRoll(finalPelvisOrientation.getYawPitchRoll()[0], userDesiredPelvisPitch.getDoubleValue(), 0.0);
         finalPelvisOrientationProvider.setOrientation(finalPelvisOrientation);
         pelvisOrientationTrajectoryGenerator.initialize();

         double stepPitch = nextFootstep.getOrientationInFrame(worldFrame).getYawPitchRoll()[1];
         onToesInitialAngleProviders.get(swingSide).set(stepPitch);

         FramePoint centerOfMass = new FramePoint(referenceFrames.getCenterOfMassFrame());
         centerOfMass.changeFrame(worldFrame);
         ContactablePlaneBody supportFoot = feet.get(supportSide);
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

         TransferToAndNextFootstepsData transferToAndNextFootstepsData = createTransferToAndNextFootstepDataForSingleSupport(nextFootstep, swingSide);
         FramePoint2d finalDesiredICP = getSingleSupportFinalDesiredICPForWalking(transferToAndNextFootstepsData, swingSide);

         ContactablePlaneBody swingFoot = feet.get(swingSide);
         setContactStateForSwing(swingFoot);
         setSupportLeg(supportSide);
         icpAndMomentumBasedController.updateBipedSupportPolygons(bipedSupportPolygons);

         // Shouldn't have to do this init anymore since it's done above...
         // icpTrajectoryGenerator.initialize(desiredICP.getFramePoint2dCopy(), finalDesiredICP, swingTimeCalculationProvider.getValue(), omega0,
         // amountToBeInsideSingleSupport.getDoubleValue(), getSupportLeg(), yoTime.getDoubleValue());

         centerOfMassHeightTrajectoryGenerator.initialize(transferToAndNextFootstepsData, getSupportLeg(), nextFootstep, getContactStatesList());

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
            boolean doubleSupportTimeHasPassed = stateMachine.timeInCurrentState() > transferTimeCalculationProvider.getValue();
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
            return icpTrajectoryHasBeenInitialized.getBooleanValue() && instantaneousCapturePointPlanner.isDone(yoTime.getDoubleValue());
         }
      }
   }


   private class SingleSupportToTransferToCondition extends DoneWithSingleSupportCondition
   {
      private final ContactablePlaneBody nextSwingFoot;
      
      public SingleSupportToTransferToCondition(ContactablePlaneBody nextSwingFoot, EndEffectorControlModule endEffectorControlModule)
      {
         super(endEffectorControlModule);
         
         this.nextSwingFoot = nextSwingFoot;
      }
      
      public boolean checkCondition()
      {
         Footstep nextFootstep = upcomingFootstepList.getNextNextFootstep(); 
         if (nextFootstep == null) return super.checkCondition();
         
         ContactablePlaneBody nextSwingFoot = nextFootstep.getBody();
         if (this.nextSwingFoot != nextSwingFoot ) return false;

         boolean condition = super.checkCondition();
         return condition;
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
         hasMinimumTimePassed.set(hasMinimumTimePassed());
                  
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

         if (hasMinimumTimePassed.getBooleanValue() && justFall.getBooleanValue()) return true;

         //Just switch states if icp is done. You had a little extra time and more isn't going to do any good.
         if (instantaneousCapturePointPlanner.isDone(yoTime.getDoubleValue())) return true;
         
         if (walkingControllerParameters.finishSwingWhenTrajectoryDone())
         {
            boolean trajectoryDone = instantaneousCapturePointPlanner.isDone(yoTime.getDoubleValue());    // endEffectorControlModule.isTrajectoryDone();

            return  hasMinimumTimePassed.getBooleanValue() && (trajectoryDone || footSwitchActivated);
         }
         else
         {
            return hasMinimumTimePassed.getBooleanValue() && footSwitchActivated;
         }
      }

      private boolean hasMinimumTimePassed()
      {
         double minimumSwingTime = swingTimeCalculationProvider.getValue() * minimumSwingFraction.getDoubleValue();

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
         boolean readyToStopWalking = (upcomingFootstepList.isFootstepProviderEmpty() && (nextFootstep == null))
                                      && ((getSupportLeg() == null) || super.checkCondition());

         return readyToStopWalking;
      }
   }


   public class ResetICPTrajectoryAction implements StateTransitionAction
   {
      public void doTransitionAction()
      {
         instantaneousCapturePointPlanner.reset(yoTime.getDoubleValue());
      }
   }


   private FramePoint2d getDoubleSupportFinalDesiredICPForDoubleSupportStance()
   {
      FramePoint2d ret = new FramePoint2d(worldFrame);
      double trailingFootToLeadingFootFactor = 0.5;    // 0.25;
      for (RobotSide robotSide : RobotSide.values)
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

   private FramePoint2d getSingleSupportFinalDesiredICPForWalking(TransferToAndNextFootstepsData transferToAndNextFootstepsData, RobotSide swingSide)
   {
      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();

      // FramePoint2d initialDesiredICP = desiredICP.getFramePoint2dCopy();
      // initialDesiredICP.changeFrame(referenceFrame);

      instantaneousCapturePointPlanner.initializeSingleSupport(transferToAndNextFootstepsData, yoTime.getDoubleValue());

      FramePoint2d finalDesiredICP = instantaneousCapturePointPlanner.getFinalDesiredICP();
      finalDesiredICP.changeFrame(referenceFrame);

      finalDesiredICPInWorld.set(finalDesiredICP);

      ContactablePlaneBody supportFoot = feet.get(swingSide.getOppositeSide());

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
         icpWayPoint = EquivalentConstantCoPCalculator.computeICPPositionWithConstantCMP(finalDesiredICP, desiredToeOffCoP, -transferTimeCalculationProvider.getValue(),
                 icpAndMomentumBasedController.getOmega0());
      }
      else
      {
         icpWayPoint = EquivalentConstantCoPCalculator.computeIntermediateICPWithConstantCMP(desiredICP.getFramePoint2dCopy(), finalDesiredICP,
                 swingTimeCalculationProvider.getValue() + transferTimeCalculationProvider.getValue(), swingTimeCalculationProvider.getValue(),
                 icpAndMomentumBasedController.getOmega0());
      }

      return icpWayPoint;
   }

   public TransferToAndNextFootstepsData createTransferToAndNextFootstepDataForSingleSupport(Footstep transferToFootstep, RobotSide swingSide)
   {
      Footstep transferFromFootstep = createFootstepFromFootAndContactablePlaneBody(referenceFrames.getFootFrame(swingSide.getOppositeSide()),
                                         feet.get(swingSide.getOppositeSide()));

      FrameConvexPolygon2d footPolygon;
      ContactablePlaneBody contactableBody = feet.get(swingSide);
      if (stayOnToes.getBooleanValue())
      {
         List<FramePoint> contactPoints = getContactPointsForWalkingOnEdge(contactableBody, ConstraintType.TOES);
         footPolygon = FrameConvexPolygon2d.constructByProjectionOntoXYPlane(contactPoints, referenceFrames.getSoleFrame(swingSide));
      }
      else
      {
         footPolygon = new FrameConvexPolygon2d(contactableBody.getContactPoints2d());
      }

      TransferToAndNextFootstepsData transferToAndNextFootstepsData = createTransferToAndNextFootstepDataForSingleSupport(transferToFootstep, swingSide,
                                                                         transferFromFootstep, footPolygon);

      return transferToAndNextFootstepsData;
   }

   public TransferToAndNextFootstepsData createTransferToAndNextFootstepDataForSingleSupport(Footstep transferToFootstep, RobotSide swingSide,
           Footstep transferFromFootstep, FrameConvexPolygon2d footPolygon)
   {
      TransferToAndNextFootstepsData transferToAndNextFootstepsData = new TransferToAndNextFootstepsData();

      transferToAndNextFootstepsData.setTransferFromFootstep(transferFromFootstep);
      transferToAndNextFootstepsData.setTransferToFootstep(transferToFootstep);

      double timeAllottedForSingleSupportForICP = swingTimeCalculationProvider.getValue() + additionalSwingTimeForICP.getDoubleValue();

      transferToAndNextFootstepsData.setTransferToFootPolygonInSoleFrame(footPolygon);
      transferToAndNextFootstepsData.setTransferToSide(swingSide);
      transferToAndNextFootstepsData.setNextFootstep(upcomingFootstepList.getNextNextFootstep());
      transferToAndNextFootstepsData.setNextNextFootstep(upcomingFootstepList.getNextNextNextFootstep());
      transferToAndNextFootstepsData.setEstimatedStepTime(timeAllottedForSingleSupportForICP + transferTimeCalculationProvider.getValue());
      transferToAndNextFootstepsData.setW0(icpAndMomentumBasedController.getOmega0());
      transferToAndNextFootstepsData.setDoubleSupportDuration(transferTimeCalculationProvider.getValue());
      transferToAndNextFootstepsData.setSingleSupportDuration(timeAllottedForSingleSupportForICP);
      double doubleSupportInitialTransferDuration = 0.4;    // TODO: Magic Number
      transferToAndNextFootstepsData.setDoubleSupportInitialTransferDuration(doubleSupportInitialTransferDuration);
      boolean stopIfReachedEnd = (upcomingFootstepList.getNumberOfFootstepsToProvide() <= 3);    // TODO: Magic Number
      transferToAndNextFootstepsData.setStopIfReachedEnd(stopIfReachedEnd);

      if (VISUALIZE)
      {
         transferToAndNextFootstepsDataVisualizer.visualizeFootsteps(transferToAndNextFootstepsData);
      }

      return transferToAndNextFootstepsData;
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

   public void doMotionControl()
   { 
      if (loopControllerForever.getBooleanValue())
      {
         while(true)
         {
            doMotionControlInternal();
         }
      }
      else 
      {
         doMotionControlInternal();
      }
   }
   
   // FIXME: don't override
   public void doMotionControlInternal()
   {
      momentumBasedController.doPrioritaryControl();
      super.callUpdatables();

      for (ContactablePlaneBody contactablePlaneBody : footEndEffectorControlModules.keySet())
      {
         EndEffectorControlModule endEffectorControlModule = footEndEffectorControlModules.get(contactablePlaneBody);
         FramePoint2d cop = momentumBasedController.getCoP(contactablePlaneBody);
         endEffectorControlModule.setCenterOfPressure(cop);
      }

      icpAndMomentumBasedController.computeCapturePoint();
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();

      desiredCoMHeightAcceleration.set(computeDesiredCoMHeightAcceleration(desiredICPVelocity.getFrameVector2dCopy()));

      doFootControl();
      doArmControl();
      doHeadControl();
      doLidarJointControl();
//    doCoMControl(); //TODO: Should we be doing this too?
      doChestControl();
      setICPBasedMomentumRateOfChangeControlModuleInputs();
      doPelvisControl();
      doJointPositionControl();

      setTorqueControlJointsToZeroDersiredAcceleration();

      momentumBasedController.doSecondaryControl();
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

      icpBasedMomentumRateOfChangeControlModule.startComputation();
      icpBasedMomentumRateOfChangeControlModule.waitUntilComputationIsDone();
      MomentumRateOfChangeData momentumRateOfChangeData = icpBasedMomentumRateOfChangeControlModule.getMomentumRateOfChangeOutputPort().getData();
      momentumBasedController.setDesiredRateOfChangeOfMomentum(momentumRateOfChangeData);
   }

   // Temporary objects to reduce garbage collection.
   private final CoMHeightPartialDerivativesData coMHeightPartialDerivatives = new CoMHeightPartialDerivativesData();
   private final ContactStatesAndUpcomingFootstepData centerOfMassHeightInputData = new ContactStatesAndUpcomingFootstepData();

   private double computeDesiredCoMHeightAcceleration(FrameVector2d desiredICPVelocity)
   {
      ReferenceFrame frame = worldFrame;

      centerOfMassHeightInputData.setCenterOfMassFrame(momentumBasedController.getCenterOfMassFrame());

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

      for (ContactablePlaneBody contactablePlaneBody : feet)
      {
         PlaneContactState contactState = momentumBasedController.getContactState(contactablePlaneBody);
         
//         YoPlaneContactState contactState = contactStates.get(contactablePlaneBody);
         if (contactState.inContact())
            contactStatesList.add(contactState);
      }

      return contactStatesList;
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
         footEndEffectorControlModules.get(contactableBody).doSingularityEscapeBeforeTransitionToNextState();
      }
      
      //TODO: If we know the surface normal here, use it.
      FrameVector normalContactVector = null;
      momentumBasedController.setPlaneContactState(contactableBody, contactPoints, coefficientOfFriction.getDoubleValue(), normalContactVector);
      updateEndEffectorControlModule(contactableBody, contactPoints, constraintType);

//      if (contactPoints.size() < oldNumberOfContactPoints)
//      {
//         // resetGroundReactionWrenchFilter();    // so that we don't end up with CoPs outside of the base of support
//      }
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

   private void updateEndEffectorControlModule(ContactablePlaneBody contactablePlaneBody, List<FramePoint2d> contactPoints, ConstraintType constraintType)
   {
//      List<FramePoint2d> contactPoints = contactState.getContactPoints2d();
      footEndEffectorControlModules.get(contactablePlaneBody).setContactPoints(contactPoints, constraintType);
   }

   // TODO: should probably precompute this somewhere else
   private FrameConvexPolygon2d computeFootPolygon(RobotSide robotSide, ReferenceFrame referenceFrame)
   {
//      List<FramePoint> contactPoints = contactStates.get(feet.get(robotSide)).getContactPoints();
      List<FramePoint> contactPoints = momentumBasedController.getContactPoints(feet.get(robotSide));
      FrameConvexPolygon2d footPolygon = FrameConvexPolygon2d.constructByProjectionOntoXYPlane(contactPoints, referenceFrame);

      return footPolygon;
   }

   private boolean landOnHeels()
   {
      return (heelPitchTouchdownProvidersManager != null) && (heelPitchTouchdownProvidersManager.getInitialAngle() != 0.0);
   }

   private void checkForSteppingOnOrOff(RobotSide transferToSide)
   {
      if ((transferToSide != null) && upcomingFootstepList.hasNextFootsteps())
      {
         ReferenceFrame initialSoleFrame;

         // NOTE: the foot may have moved so its ideal to get the previous footstep, rather than the current foot frame, if possible
         if (upcomingFootstepList.doesNextFootstepListHaveFewerThanTwoElements())
         {
            initialSoleFrame = feet.get(transferToSide.getOppositeSide()).getPlaneFrame();
         }
         else
         {
            initialSoleFrame = upcomingFootstepList.getFootstepTwoBackFromNextFootstepList().getSoleReferenceFrame();
         }

         Footstep nextFootstep = upcomingFootstepList.getNextFootstep();
         ReferenceFrame finalSoleFrame = nextFootstep.getSoleReferenceFrame();

         boolean isBlindWalking = variousWalkingProviders.getFootstepProvider().isBlindWalking();
         
         if (isBlindWalking)
         {
            this.stepOnOrOff.set(false);
         }

         else
         {
            this.stepOnOrOff.set(TwoWaypointTrajectoryUtils.stepOnOrOff(initialSoleFrame, finalSoleFrame));
         }
         
         if (stepOnOrOff.getBooleanValue())
         {
            TrajectoryParameters trajectoryParameters = new SimpleTwoWaypointTrajectoryParameters(TrajectoryWaypointGenerationMethod.STEP_ON_OR_OFF);
            mapFromFootstepsToTrajectoryParameters.put(nextFootstep, trajectoryParameters);
         }
      }
   }

   private static Footstep createFootstepFromFootAndContactablePlaneBody(ReferenceFrame footReferenceFrame, ContactablePlaneBody contactablePlaneBody)
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
   
   private void checkForReinitialization()
   {
      if(reinitializeControllerProvider.isReinitializeRequested() && (stateMachine.getCurrentStateEnum() == WalkingState.DOUBLE_SUPPORT))
      {
         reinitializeControllerProvider.set(false);
         initialize();
      }
   }

}
