package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.calculators.EquivalentConstantCoPCalculator;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.ChestOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.WalkOnToesManager;
import us.ihmc.commonWalkingControlModules.controlModules.WalkOnToesManager.ToeOffMotionType;
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
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumControlModuleBridge.MomentumControlModuleType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.packetConsumers.ReinitializeWalkingControllerProvider;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.sensors.HeelSwitch;
import us.ihmc.commonWalkingControlModules.sensors.ToeSwitch;
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
import us.ihmc.commonWalkingControlModules.trajectories.PoseTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.SettableOrientationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.SimpleTwoWaypointTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.trajectories.SwingTimeCalculationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.TransferTimeCalculationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointTrajectoryUtils;
import us.ihmc.commonWalkingControlModules.trajectories.WrapperForPositionAndOrientationTrajectoryGenerators;
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
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.VariableChangedListener;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.util.GainCalculator;
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
import com.yobotics.simulationconstructionset.util.trajectory.QuinticPolynomialTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.ThirdOrderPolynomialTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.TrajectoryParameters;
import com.yobotics.simulationconstructionset.util.trajectory.TrajectoryParametersProvider;
import com.yobotics.simulationconstructionset.util.trajectory.TrajectoryWaypointGenerationMethod;
import com.yobotics.simulationconstructionset.util.trajectory.YoPositionProvider;
import com.yobotics.simulationconstructionset.util.trajectory.YoVariableDoubleProvider;

public class WalkingHighLevelHumanoidController extends AbstractHighLevelHumanoidControlPattern
{
   private boolean VISUALIZE = true;

   private final static HighLevelState controllerState = HighLevelState.WALKING;
   private final static MomentumControlModuleType MOMENTUM_CONTROL_MODULE_TO_USE = MomentumControlModuleType.OPTIMIZATION;

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
   private final DoubleYoVariable dwellInSingleSupportDuration = new DoubleYoVariable("dwellInSingleSupportDuration", 
         "Amount of time to stay in single support after the ICP trajectory is done if you haven't registered a touchdown yet", registry);
   
   private final BooleanYoVariable loopControllerForever = new BooleanYoVariable("loopControllerForever", "For checking memory and profiling", registry);
   private final BooleanYoVariable justFall = new BooleanYoVariable("justFall", registry);
   
   private final BooleanYoVariable stepOnOrOff = new BooleanYoVariable("stepOnOrOff", registry);
   private final BooleanYoVariable controlPelvisHeightInsteadOfCoMHeight = new BooleanYoVariable("controlPelvisHeightInsteadOfCoMHeight", registry);
   
   private final BooleanYoVariable hasMinimumTimePassed = new BooleanYoVariable("hasMinimumTimePassed", registry);
   private final DoubleYoVariable minimumSwingFraction = new DoubleYoVariable("minimumSwingFraction", registry);
   
   private final BooleanYoVariable hasICPPlannerFinished = new BooleanYoVariable("hasICPPlannerFinished", registry);
   private final DoubleYoVariable timeThatICPPlannerFinished = new DoubleYoVariable("timeThatICPPlannerFinished", registry);
   
   // private final FinalDesiredICPCalculator finalDesiredICPCalculator;

   private final BooleanYoVariable rememberFinalICPFromSingleSupport = new BooleanYoVariable("rememberFinalICPFromSingleSupport", registry);
   private final YoFramePoint2d finalDesiredICPInWorld = new YoFramePoint2d("finalDesiredICPInWorld", "", worldFrame, registry);

   private final SideDependentList<FootSwitchInterface> footSwitches;

   private final DoubleYoVariable minOrbitalEnergyForSingleSupport = new DoubleYoVariable("minOrbitalEnergyForSingleSupport", registry);
   private final DoubleYoVariable amountToBeInsideSingleSupport = new DoubleYoVariable("amountToBeInsideSingleSupport", registry);
   private final DoubleYoVariable amountToBeInsideDoubleSupport = new DoubleYoVariable("amountToBeInsideDoubleSupport", registry);

   private final DoubleYoVariable userDesiredPelvisYaw = new DoubleYoVariable("userDesiredPelvisYaw", registry);
   private final DoubleYoVariable userDesiredPelvisPitch = new DoubleYoVariable("userDesiredPelvisPitch", registry);
   private final DoubleYoVariable userDesiredPelvisRoll = new DoubleYoVariable("userDesiredPelvisRoll", registry);
   
   private final SettableOrientationProvider initialPelvisOrientationProvider;
   private final SettableOrientationProvider finalPelvisOrientationProvider;
   private final OrientationTrajectoryGenerator pelvisOrientationTrajectoryGenerator;

   private final SwingTimeCalculationProvider swingTimeCalculationProvider;
   private final TransferTimeCalculationProvider transferTimeCalculationProvider;
   
   private final TrajectoryParametersProvider trajectoryParametersProvider;
   private final SideDependentList<YoVariableDoubleProvider> onToesInitialAngleProviders = new SideDependentList<YoVariableDoubleProvider>();
   private final SideDependentList<YoVariableDoubleProvider> onToesFinalAngleProviders = new SideDependentList<YoVariableDoubleProvider>();

   private final DoubleYoVariable additionalSwingTimeForICP = new DoubleYoVariable("additionalSwingTimeForICP", registry);
   private final DoubleYoVariable trailingFootPitch = new DoubleYoVariable("trailingFootPitch", registry);

   private final YoPositionProvider finalPositionProvider;

   private final DoubleYoVariable swingAboveSupportAnkle = new DoubleYoVariable("swingAboveSupportAnkle", registry);
   private final BooleanYoVariable readyToGrabNextFootstep = new BooleanYoVariable("readyToGrabNextFootstep", registry);

   private final HashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters;
   private final InstantaneousCapturePointPlanner instantaneousCapturePointPlanner;
   private final ReinitializeWalkingControllerProvider reinitializeControllerProvider;

   private final SideDependentList<SettableOrientationProvider> finalFootOrientationProviders = new SideDependentList<SettableOrientationProvider>();

   private final ICPBasedMomentumRateOfChangeControlModule icpBasedMomentumRateOfChangeControlModule;

   private final BooleanYoVariable icpTrajectoryHasBeenInitialized;
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
   private final YoFramePoint2d desiredECMP = new YoFramePoint2d("desiredECMP", "", worldFrame, registry);
   private final BooleanYoVariable desiredECMPinSupportPolygon = new BooleanYoVariable("desiredECMPinSupportPolygon", registry);
   private YoFramePoint ecmpViz = new YoFramePoint("ecmpViz", ReferenceFrame.getWorldFrame(), registry);
   
   private final YoVariableDoubleProvider totalEstimatedToeOffTimeProvider = new YoVariableDoubleProvider("totalEstimatedToeOffTimeProvider", registry);
   
   private final DoubleYoVariable singularityEscapeNullspaceMultiplierSwingLeg = new DoubleYoVariable("singularityEscapeNullspaceMultiplierSwingLeg", registry);
   private final DoubleYoVariable singularityEscapeNullspaceMultiplierSupportLeg = new DoubleYoVariable("singularityEscapeNullspaceMultiplierSupportLeg", registry);
   private final DoubleYoVariable singularityEscapeNullspaceMultiplierSupportLegLocking = new DoubleYoVariable("singularityEscapeNullspaceMultiplierSupportLegLocking", registry);

   private double referenceTime = 0.22; 
   private MaximumConstantJerkFinalToeOffAngleComputer maximumConstantJerkFinalToeOffAngleComputer = new MaximumConstantJerkFinalToeOffAngleComputer();  
   private YoVariableDoubleProvider onToesFinalPitchProvider = new YoVariableDoubleProvider("OnToesFinalPitch", registry);

   private final VariousWalkingProviders variousWalkingProviders;
   private final VariousWalkingManagers variousWalkingManagers;

   private final DoubleYoVariable walkingHeadOrientationKp = new DoubleYoVariable("walkingHeadOrientationKp", registry);
   private final DoubleYoVariable walkingHeadOrientationZeta = new DoubleYoVariable("walkingHeadOrientationZeta", registry);
   
   private final WalkOnToesManager walkOnToesManager;

   public WalkingHighLevelHumanoidController(VariousWalkingProviders variousWalkingProviders, VariousWalkingManagers variousWalkingManagers,
         SideDependentList<FootSwitchInterface> footSwitches,
           DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, 
           CoMHeightTrajectoryGenerator centerOfMassHeightTrajectoryGenerator,
           SideDependentList<PositionTrajectoryGenerator> footPositionTrajectoryGenerators,
           SideDependentList<DoubleTrajectoryGenerator> heelPitchTrajectoryGenerators, HeelPitchTouchdownProvidersManager heelPitchTouchdownProvidersManager,
           SwingTimeCalculationProvider swingTimeCalculationProvider, TransferTimeCalculationProvider transferTimeCalculationProvider, 
           YoPositionProvider finalPositionProvider,
           TrajectoryParametersProvider trajectoryParametersProvider, double desiredPelvisPitch, double trailingFootPitch,
           WalkingControllerParameters walkingControllerParameters, ICPBasedMomentumRateOfChangeControlModule momentumRateOfChangeControlModule,
           LidarControllerInterface lidarControllerInterface,
           InstantaneousCapturePointPlanner instantaneousCapturePointPlanner, 
           ICPAndMomentumBasedController icpAndMomentumBasedController, MomentumBasedController momentumBasedController, WalkingStatusReporter walkingStatusReporter)
   {
      
      super(variousWalkingProviders, variousWalkingManagers, momentumBasedController, walkingControllerParameters, 
            lidarControllerInterface, dynamicGraphicObjectsListRegistry, controllerState);
     
      super.addUpdatables(icpAndMomentumBasedController.getUpdatables());

      this.variousWalkingProviders = variousWalkingProviders;
      this.variousWalkingManagers = variousWalkingManagers;
      
      setupManagers(variousWalkingManagers);
      
      FootstepProvider footstepProvider = variousWalkingProviders.getFootstepProvider();
      HashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters = variousWalkingProviders.getMapFromFootstepsToTrajectoryParameters();
      this.reinitializeControllerProvider = variousWalkingProviders.getReinitializeWalkingControllerProvider();

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
      double kpCoMHeight = walkingControllerParameters.getKpCoMHeight();
      centerOfMassHeightController.setProportionalGain(kpCoMHeight); 
      double zetaCoMHeight =  walkingControllerParameters.getZetaCoMHeight(); 
      centerOfMassHeightController.setDerivativeGain(GainCalculator.computeDerivativeGain(centerOfMassHeightController.getProportionalGain(), zetaCoMHeight));

      String namePrefix = "walking";

      this.stateMachine = new StateMachine<WalkingState>(namePrefix + "State", namePrefix + "SwitchTime", WalkingState.class, yoTime, registry);    // this is used by name, and it is ugly.

      this.finalPositionProvider = finalPositionProvider;

      this.icpTrajectoryHasBeenInitialized = new BooleanYoVariable("icpTrajectoryHasBeenInitialized", registry);

      rememberFinalICPFromSingleSupport.set(false);    // true);
      finalDesiredICPInWorld.set(Double.NaN, Double.NaN);

      coefficientOfFriction.set(0.8); //0.6);    // TODO: tune?

      setupLegJacobians(fullRobotModel);

      walkOnToesManager = new WalkOnToesManager(walkingControllerParameters, feet, footEndEffectorControlModules, registry);
      this.centerOfMassHeightTrajectoryGenerator.attachWalkOnToesManager(walkOnToesManager);
      
      maximumConstantJerkFinalToeOffAngleComputer.reinitialize(walkOnToesManager.getMaximumToeOffAngle(), referenceTime);
      
      setupFootControlModules(footPositionTrajectoryGenerators, heelPitchTrajectoryGenerators);

      initialPelvisOrientationProvider = new SettableOrientationProvider("initialPelvis", worldFrame, registry);
      finalPelvisOrientationProvider = new SettableOrientationProvider("finalPelvis", worldFrame, registry);
      this.pelvisOrientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator("pelvis", worldFrame, swingTimeCalculationProvider,
              initialPelvisOrientationProvider, finalPelvisOrientationProvider, registry);

      setUpStateMachine();
      readyToGrabNextFootstep.set(true);

      dwellInSingleSupportDuration.set(0.2);
      
      minOrbitalEnergyForSingleSupport.set(0.007);    // 0.008
      amountToBeInsideSingleSupport.set(0.0);
      amountToBeInsideDoubleSupport.set(0.03);    // 0.02);    // TODO: necessary for stairs...
      transferTimeCalculationProvider.setTransferTime();   
      
      totalEstimatedToeOffTimeProvider.set(transferTimeCalculationProvider.getValue());
            
      stopInDoubleSupporTrajectoryTime.set(0.5);
      this.userDesiredPelvisPitch.set(desiredPelvisPitch);
      this.trailingFootPitch.set(trailingFootPitch);
      
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

      controlPelvisHeightInsteadOfCoMHeight.set(false);
   }


   protected void setupFootControlModules(SideDependentList<PositionTrajectoryGenerator> footPositionTrajectoryGenerators,
           SideDependentList<DoubleTrajectoryGenerator> heelPitchTrajectoryGenerators)
   {
      singularityEscapeNullspaceMultiplierSwingLeg.set(200.0);
      singularityEscapeNullspaceMultiplierSupportLeg.set(20.0);
      singularityEscapeNullspaceMultiplierSupportLegLocking.set(-0.5);
      double minJacobianDeterminantForSingularityEscape = 0.03;
      
      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody bipedFoot = feet.get(robotSide);

         //TODO: If we know the surface normal here, use it.
         momentumBasedController.setPlaneContactStateFullyConstrained(bipedFoot, coefficientOfFriction.getDoubleValue(), null);
         
         String sideString = robotSide.getCamelCaseNameForStartOfExpression();

         PositionTrajectoryGenerator swingPositionTrajectoryGenerator = footPositionTrajectoryGenerators.get(robotSide);
         DoubleTrajectoryGenerator heelPitchTrajectoryGenerator = (heelPitchTrajectoryGenerators == null) ? null : heelPitchTrajectoryGenerators.get(robotSide);

         OrientationProvider initialOrientationProvider = new CurrentOrientationProvider(worldFrame, bipedFoot.getBodyFrame());
         SettableOrientationProvider finalFootOrientationProvider = new SettableOrientationProvider(sideString + "FinalFootOrientation", worldFrame, registry);
         finalFootOrientationProviders.put(robotSide, finalFootOrientationProvider);

         OrientationTrajectoryGenerator swingOrientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator(sideString
                                                                                 + "SwingFootOrientation", worldFrame, swingTimeCalculationProvider,
                                                                                    initialOrientationProvider, finalFootOrientationProvider, registry);
         
         PoseTrajectoryGenerator swingPoseTrajectoryGenerator = new WrapperForPositionAndOrientationTrajectoryGenerators(swingPositionTrajectoryGenerator,
                                                                   swingOrientationTrajectoryGenerator);
         
         YoVariableDoubleProvider onToesInitialPitchProvider = new YoVariableDoubleProvider(sideString + "OnToesInitialPitch", registry);
         DoubleProvider onToesInitialPitchVelocityProvider = new ConstantDoubleProvider(0.0);
         DoubleProvider onToesTrajectoryTimeProvider = totalEstimatedToeOffTimeProvider; 
         
         DoubleTrajectoryGenerator onToesPitchTrajectoryGenerator = null; 
         switch(WalkOnToesManager.TOEOFF_MOTION_TYPE_USED)
         {
         case CUBIC_SPLINE:
            onToesFinalPitchProvider.set(maximumConstantJerkFinalToeOffAngleComputer.getMaximumFeasibleConstantJerkFinalToeOffAngle
                     (onToesInitialPitchProvider.getValue(), instantaneousCapturePointPlanner.getEstimatedTimeRemainingForState(yoTime.getDoubleValue())));
            
            onToesPitchTrajectoryGenerator = new ThirdOrderPolynomialTrajectoryGenerator(sideString + "OnToesPitch",
                                                                          onToesInitialPitchProvider, onToesInitialPitchVelocityProvider,
                                                                          onToesFinalPitchProvider, onToesTrajectoryTimeProvider, registry);
            break;
         case QUINTIC_SPLINE:
            DoubleProvider onToesFinalPitchVelocityProvider = new ConstantDoubleProvider(walkingControllerParameters.getFinalToeOffPitchAngularVelocity());

            onToesPitchTrajectoryGenerator = new QuinticPolynomialTrajectoryGenerator(sideString + "OnToesPitch",
                                                                          onToesInitialPitchProvider, onToesInitialPitchVelocityProvider,
                                                                          onToesFinalPitchProvider, onToesFinalPitchVelocityProvider,
                                                                          onToesTrajectoryTimeProvider, registry);
            break;
         case FREE:
            // Do nothing
            break;
         default:
            throw new RuntimeException("Should not get there");
         }
         
         onToesInitialAngleProviders.put(robotSide, onToesInitialPitchProvider);
         onToesFinalAngleProviders.put(robotSide, onToesFinalPitchProvider);
         
         GeometricJacobian jacobian = legJacobians.get(robotSide);
         OneDoFJoint kneeJoint = fullRobotModel.getLegJoint(robotSide, LegJointName.KNEE);
         
         final EndEffectorControlModule endEffectorControlModule;
         if (WalkOnToesManager.TOEOFF_MOTION_TYPE_USED != ToeOffMotionType.FREE)
         {
            endEffectorControlModule = new EndEffectorControlModule(bipedFoot, jacobian, kneeJoint, swingPoseTrajectoryGenerator,
                                          heelPitchTrajectoryGenerator, onToesPitchTrajectoryGenerator, momentumBasedController, registry);
         }
         else
         {
            // Let the toe pitch motion free. It seems to work better.
            endEffectorControlModule = new EndEffectorControlModule(bipedFoot, jacobian, kneeJoint, swingPoseTrajectoryGenerator,
                                          heelPitchTrajectoryGenerator, walkOnToesManager.getMaximumToeOffAngleProvider(), momentumBasedController, registry);
         }
         endEffectorControlModule.setParameters(minJacobianDeterminantForSingularityEscape, singularityEscapeNullspaceMultiplierSwingLeg.getDoubleValue());
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

      headOrientationManager.setUp(baseForHeadOrientationControl, jacobianForHeadOrientationControl);
      walkingHeadOrientationKp.set(walkingControllerParameters.getKpHeadOrientation()); 
      walkingHeadOrientationZeta.set(walkingControllerParameters.getZetaHeadOrientation());
      VariableChangedListener headGainsChangedListener = createHeadGainsChangedListener();
      headGainsChangedListener.variableChanged(null);
      
      FrameOrientation initialDesiredPelvisOrientation = new FrameOrientation(referenceFrames.getAnkleZUpFrame(getUpcomingSupportLeg()));
      initialDesiredPelvisOrientation.changeFrame(worldFrame);
      double yaw = initialDesiredPelvisOrientation.getYawPitchRoll()[0];
      initialDesiredPelvisOrientation.setYawPitchRoll(yaw, userDesiredPelvisPitch.getDoubleValue(), userDesiredPelvisRoll.getDoubleValue());
      desiredPelvisOrientation.set(initialDesiredPelvisOrientation);
      finalPelvisOrientationProvider.setOrientation(initialDesiredPelvisOrientation);    // yes, final. To make sure that the first swing phase has the right initial

      icpAndMomentumBasedController.computeCapturePoint();
      desiredICP.set(capturePoint.getFramePoint2dCopy());

      stateMachine.setCurrentState(WalkingState.DOUBLE_SUPPORT);

   }
   
   private VariableChangedListener createHeadGainsChangedListener()
   {
      VariableChangedListener ret = new VariableChangedListener()
      {
         public void variableChanged(YoVariable v)
         {
            double headKp = walkingHeadOrientationKp.getDoubleValue();
            double headZeta = walkingHeadOrientationZeta.getDoubleValue();
            double headKd = GainCalculator.computeDerivativeGain(headKp, headZeta);
            headOrientationManager.setControlGains(headKp, headKd); 
         }};
         
         walkingHeadOrientationKp.addVariableChangedListener(ret);
         walkingHeadOrientationZeta.addVariableChangedListener(ret);
      
      return ret;
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

         if ((footEndEffectorControlModules.get(transferFoot) != null) && (footEndEffectorControlModules.get(transferFoot).touchdownOnEdge())
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
         
         initializeECMPbasedToeOffIfNotInitializedYet();

         // Only during the first few seconds, we will control the pelvis orientation based on midfeetZup
         if (((yoTime.getDoubleValue() - controllerInitializationTime.getDoubleValue()) < PELVIS_YAW_INITIALIZATION_TIME)
                 &&!alreadyBeenInDoubleSupportOnce.getBooleanValue())
         {
            setDesiredPelvisYawToAverageOfFeetOnStartupOnly(transferToSide);
         }

         // keep desired pelvis orientation as it is
         desiredPelvisAngularVelocity.set(0.0, 0.0, 0.0);
         desiredPelvisAngularAcceleration.set(0.0, 0.0, 0.0);
      }

      public void initializeICPPlannerIfNecessary()
      {
         if (!icpTrajectoryHasBeenInitialized.getBooleanValue() && instantaneousCapturePointPlanner.isDone(yoTime.getDoubleValue()))
         {
            Pair<FramePoint2d, Double> finalDesiredICPAndTrajectoryTime = computeFinalDesiredICPAndTrajectoryTime();

            if (transferToSide != null)    // the only case left for determining the contact state of the trailing foot
            {
               FramePoint2d finalDesiredICP = finalDesiredICPAndTrajectoryTime.first();
               finalDesiredICP.changeFrame(desiredICP.getReferenceFrame());

               RobotSide trailingLeg = transferToSide.getOppositeSide();
               walkOnToesManager.updateToeOffStatusBasedOnICP(trailingLeg, desiredICP.getFramePoint2dCopy(), finalDesiredICP);

               if (walkOnToesManager.doToeOff())
               {
                  ContactablePlaneBody trailingFoot = feet.get(trailingLeg);
                  setOnToesContactState(trailingFoot);
               }
            }

            icpAndMomentumBasedController.updateBipedSupportPolygons(bipedSupportPolygons);    // need to always update biped support polygons after a change to the contact states
            icpTrajectoryHasBeenInitialized.set(true);
         }
      }

      public void initializeECMPbasedToeOffIfNotInitializedYet()
      {
         // the only case left for determining the contact state of the trailing foot
         if ((!ecmpBasedToeOffHasBeenInitialized.getBooleanValue()) && (transferToSide != null))
         {
            RobotSide trailingLeg = transferToSide.getOppositeSide();
            walkOnToesManager.updateToeOffStatusBasedOnECMP(trailingLeg, desiredECMP.getFramePoint2dCopy());

            if (walkOnToesManager.doToeOff())
            {
               double remainingToeOffTime = instantaneousCapturePointPlanner.getEstimatedTimeRemainingForState(yoTime.getDoubleValue()); 
               onToesFinalPitchProvider.set(maximumConstantJerkFinalToeOffAngleComputer.getMaximumFeasibleConstantJerkFinalToeOffAngle
                     (onToesInitialAngleProviders.get(trailingLeg).getValue(), remainingToeOffTime));

               ContactablePlaneBody trailingFoot = feet.get(trailingLeg);
               setOnToesContactState(trailingFoot);
               icpAndMomentumBasedController.updateBipedSupportPolygons(bipedSupportPolygons);    // need to always update biped support polygons after a change to the contact states
               ecmpBasedToeOffHasBeenInitialized.set(true);

               totalEstimatedToeOffTimeProvider.set(remainingToeOffTime);
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
         ecmpBasedToeOffHasBeenInitialized.set(false);

         icpTrajectoryHasBeenInitialized.set(false);
         if (DEBUG)
            System.out.println("WalkingHighLevelHumanoidController: enteringDoubleSupportState");
         setSupportLeg(null);    // TODO: check if necessary

         // TODO: simplify the following
         if (transferToSide != null)
         {
            RobotSide trailingLeg = transferToSide.getOppositeSide();
         }

         if (walkOnToesManager.stayOnToes())
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
            else if (walkOnToesManager.willLandOnToes())
            {
               setTouchdownOnToesContactState(feet.get(transferToSide));
            }
            else if (landOnHeels())
            {
               setTouchdownOnHeelContactState(feet.get(transferToSide));
            }
            else
            {
               setFlatFootContactState(feet.get(transferToSide));

               // still need to determine contact state for trailing leg. This is done in doAction as soon as the previous ICP trajectory is done
            }
         }
         
         walkOnToesManager.reset();

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
         // Before swinging a foot, relatch where all the other foot positions are. 
         // Otherwise there might be a jump.
         momentumBasedController.requestResetEstimatorPositionsToCurrent();
         
         desiredECMPinSupportPolygon.set(false);
         walkOnToesManager.reset();
         ecmpBasedToeOffHasBeenInitialized.set(false);

         alreadyBeenInDoubleSupportOnce.set(true);

         if (DEBUG)
            System.out.println("WalkingHighLevelHumanoidController: leavingDoubleSupportState");
         
         desiredICPVelocity.set(0.0, 0.0);
         manipulationControlModule.prepareForLocomotion();
      }
   }


   private void setDesiredPelvisYawToAverageOfFeetOnStartupOnly(RobotSide transferToSide)
   {
      FrameOrientation averageOrientation = new FrameOrientation(worldFrame);
      averageOrientationCalculator.computeAverageOrientation(averageOrientation, feet.get(RobotSide.LEFT).getPlaneFrame(),
              feet.get(RobotSide.RIGHT).getPlaneFrame(), worldFrame);

      double[] yawPitchRoll = averageOrientation.getYawPitchRoll();

      double yawOffset = 0.0;
      if (transferToSide != null)
         yawOffset = transferToSide.negateIfLeftSide(userDesiredPelvisYaw.getDoubleValue());
      
      averageOrientation.setYawPitchRoll(yawPitchRoll[0] + yawOffset, userDesiredPelvisPitch.getDoubleValue(), userDesiredPelvisRoll.getDoubleValue());
      desiredPelvisOrientation.set(averageOrientation);
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

         if (stateMachine.timeInCurrentState() < 0.5 * swingTimeCalculationProvider.getValue() && footEndEffectorControlModules.get(feet.get(swingSide)).isInSingularityNeighborhood())
         {
            footEndEffectorControlModules.get(feet.get(swingSide)).doSingularityEscape(true);
         }
      }

      @Override
      public void doTransitionIntoAction()
      {
         hasICPPlannerFinished.set(false);
         
         footSwitches.get(swingSide).reset();

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

         walkOnToesManager.checkAndRememberIfLandOnToes(swingSide.getOppositeSide(), nextFootstep);
         
         if (walkOnToesManager.willLandOnToes())
         {
            heelPitchTouchdownProvidersManager.setInitialAngle(walkOnToesManager.getToeTouchdownAngle());
            nextFootstep = Footstep.copyButChangePitch(nextFootstep, heelPitchTouchdownProvidersManager.getInitialAngle());
            heelPitchTouchdownProvidersManager.updateInitialAngularSpeed();
            nextFootstepHasBeenReplaced = true;
         }
         else if (landOnHeels())
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

         if (walkOnToesManager.stayOnToes())
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

         FrameOrientation orientation = new FrameOrientation(desiredPelvisOrientation.getReferenceFrame());
         desiredPelvisOrientation.get(orientation);
         initialPelvisOrientationProvider.setOrientation(orientation);

         FrameOrientation finalPelvisOrientation = nextFootstep.getOrientationInFrame(worldFrame);
         FramePoint swingFootFinalPosition = nextFootstep.getPositionInFrame(referenceFrames.getAnkleZUpFrame(swingSide.getOppositeSide()));
         FrameVector supportFootToSwingFoot = new FrameVector(swingFootFinalPosition);
         Vector3d temp = supportFootToSwingFoot.getVectorCopy();
         double desiredPelvisYawAngle = 0.0;
         if (Math.abs(temp.x) > 0.1)
         {
            desiredPelvisYawAngle = Math.atan2(temp.y, temp.x);
            desiredPelvisYawAngle -= swingSide.negateIfRightSide(Math.PI/2.0);
         }
         finalPelvisOrientation.setYawPitchRoll(finalPelvisOrientation.getYawPitchRoll()[0] + userDesiredPelvisYaw.getDoubleValue() * desiredPelvisYawAngle, userDesiredPelvisPitch.getDoubleValue(), userDesiredPelvisRoll.getDoubleValue());
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
         
         if (!hasICPPlannerFinished.getBooleanValue())
         {
            hasICPPlannerFinished.set(instantaneousCapturePointPlanner.isDone(yoTime.getDoubleValue()));
            if (hasICPPlannerFinished.getBooleanValue())
            {
               timeThatICPPlannerFinished.set(yoTime.getDoubleValue());
            }
         }
         
         FootSwitchInterface footSwitch = footSwitches.get(swingSide);

         // TODO probably make all FootSwitches in this class be HeelSwitches and get rid of instanceof
         boolean footSwitchActivated;
         
         if (walkOnToesManager.willLandOnToes())
         {
            if (!(footSwitch instanceof ToeSwitch))
            {
               throw new RuntimeException("toe touchdown should not be used if Robot is not using a ToeSwitch.");
            }
            
            ToeSwitch toeSwitch = (ToeSwitch) footSwitch;
            footSwitchActivated = toeSwitch.hasToeHitGround();
         }
         else if (landOnHeels())
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

         //Just switch states if icp is done, plus a little bit more. You had enough time and more isn't going to do any good.
         if (hasICPPlannerFinished.getBooleanValue() && (yoTime.getDoubleValue() > timeThatICPPlannerFinished.getDoubleValue() + dwellInSingleSupportDuration.getDoubleValue())) return true;
         
         if (walkingControllerParameters.finishSwingWhenTrajectoryDone())
         {
            return  hasMinimumTimePassed.getBooleanValue() && (hasICPPlannerFinished.getBooleanValue() || footSwitchActivated);
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

      RobotSide supportSide = swingSide.getOppositeSide();
      walkOnToesManager.updateOnToesTriangle(finalDesiredICP, supportSide);

      FramePoint2d icpWayPoint;
      if (walkOnToesManager.doToeOffIfPossible() && walkOnToesManager.isOnToesTriangleLargeEnough())
      {
         FramePoint toeOffPoint = new FramePoint(worldFrame);
         ContactablePlaneBody supportFoot = feet.get(supportSide);
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
      if (walkOnToesManager.stayOnToes())
      {
         List<FramePoint> contactPoints = getToePoints(contactableBody);
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
      
      momentumBasedController.doPassiveKneeControl();
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

      centerOfMassHeightInputData.setCenterOfMassAndPelvisZUpFrames(momentumBasedController.getCenterOfMassFrame(), momentumBasedController.getPelvisZUpFrame());

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
      
      if (controlPelvisHeightInsteadOfCoMHeight.getBooleanValue())
      {
         FramePoint pelvisPosition = new FramePoint(referenceFrames.getPelvisFrame());
         pelvisPosition.changeFrame(frame);
         zCurrent = pelvisPosition.getZ(); 
         
         zdCurrent = comVelocity.getZ(); // Just use com velocity for now for damping...
      }

      double zddDesired = centerOfMassHeightController.compute(zCurrent, zDesired, zdCurrent, zdDesired) + zddFeedForward;

      
      for (RobotSide robotSide : RobotSide.values)
      {
         EndEffectorControlModule endEffectorControlModule = footEndEffectorControlModules.get(feet.get(robotSide));
         
         if (endEffectorControlModule.getCurrentConstraintType() == ConstraintType.FULL && endEffectorControlModule.isInSingularityNeighborhood())
         {
            // Can't achieve a desired height acceleration
            zddDesired = 0.0;
            double zTreshold = 0.01;

            if (zDesired >= zCurrent - zTreshold)
            {
               // Can't achieve the desired height, just lock the knee
               endEffectorControlModule.doSingularityEscape(singularityEscapeNullspaceMultiplierSupportLegLocking.getDoubleValue());
            }
            else
            {
               // Do the singularity escape before trying to achieve the desired height
               endEffectorControlModule.doSingularityEscape(singularityEscapeNullspaceMultiplierSupportLeg.getDoubleValue());
            }
         }
      }
      
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
      RobotSide robotSide = RobotSide.LEFT;
      
      if (feet.get(robotSide.getOppositeSide()).equals(contactableBody))
         robotSide = RobotSide.RIGHT;

      // TODO cannot use world or elevator frames with non perfect sensors... some bug to fix obviously
      footEndEffectorControlModules.get(contactableBody).setContactState(ConstraintType.TOES, new FrameVector(referenceFrames.getAnkleZUpFrame(robotSide), 0.0, 0.0, 1.0));
   }

   private void setTouchdownOnHeelContactState(ContactablePlaneBody contactableBody)
   {
      RobotSide robotSide = RobotSide.LEFT;
      
      if (feet.get(robotSide.getOppositeSide()).equals(contactableBody))
         robotSide = RobotSide.RIGHT;

      // TODO cannot use world or elevator frames with non perfect sensors... some bug to fix obviously
      footEndEffectorControlModules.get(contactableBody).setContactState(ConstraintType.HEEL_TOUCHDOWN, new FrameVector(referenceFrames.getAnkleZUpFrame(robotSide), 0.0, 0.0, 1.0));
   }

   private void setTouchdownOnToesContactState(ContactablePlaneBody contactableBody)
   {
      RobotSide robotSide = RobotSide.LEFT;
      
      if (feet.get(robotSide.getOppositeSide()).equals(contactableBody))
         robotSide = RobotSide.RIGHT;

      // TODO cannot use world or elevator frames with non perfect sensors... some bug to fix obviously
      footEndEffectorControlModules.get(contactableBody).setContactState(ConstraintType.TOES_TOUCHDOWN, new FrameVector(referenceFrames.getAnkleZUpFrame(robotSide), 0.0, 0.0, 1.0));
   }

   private void setFlatFootContactState(ContactablePlaneBody contactableBody)
   {
      footEndEffectorControlModules.get(contactableBody).setContactState(ConstraintType.FULL);
   }

   private void setContactStateForSwing(ContactablePlaneBody contactableBody)
   {
      EndEffectorControlModule endEffectorControlModule = footEndEffectorControlModules.get(contactableBody);
      endEffectorControlModule.doSingularityEscape(true);
      endEffectorControlModule.setContactState(ConstraintType.UNCONSTRAINED);
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
      FramePoint footPosition = new FramePoint(referenceFrames.getAnkleZUpFrame(RobotSide.LEFT));
      footPosition.changeFrame(referenceFrames.getAnkleZUpFrame(RobotSide.RIGHT));
      // TODO Sylvain: extend the HeeTouchdownProvidersManager to detect if landing on heel is appropriate.
      return (Math.abs(footPosition.getX()) > 0.15 && heelPitchTouchdownProvidersManager != null) && (heelPitchTouchdownProvidersManager.getInitialAngle() != 0.0);
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
