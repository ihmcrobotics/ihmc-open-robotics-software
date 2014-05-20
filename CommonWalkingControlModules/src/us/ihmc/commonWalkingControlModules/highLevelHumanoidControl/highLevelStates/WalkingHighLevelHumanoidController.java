package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoFramePoint2dInPolygonCoordinate;
import us.ihmc.commonWalkingControlModules.calculators.EquivalentConstantCoPCalculator;
import us.ihmc.commonWalkingControlModules.captureRegion.PushRecoveryControlModule;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.ChestOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.WalkOnTheEdgesManager;
import us.ihmc.commonWalkingControlModules.controlModules.endEffector.FootControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.endEffector.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controlModules.endEffector.LegSingularityAndKneeCollapseAvoidanceControlModule;
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
import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointTrajectoryData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumControlModuleBridge.MomentumControlModuleType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.packetConsumers.ReinitializeWalkingControllerProvider;
import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.sensors.HeelSwitch;
import us.ihmc.commonWalkingControlModules.sensors.ToeSwitch;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightPartialDerivativesData;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTimeDerivativesCalculator;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTimeDerivativesData;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTimeDerivativesSmoother;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.CoMXYTimeDerivativesData;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantSwingTimeCalculator;
import us.ihmc.commonWalkingControlModules.trajectories.ContactStatesAndUpcomingFootstepData;
import us.ihmc.commonWalkingControlModules.trajectories.CurrentOrientationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.FlatThenPolynomialCoMHeightTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.MaximumConstantJerkFinalToeOffAngleComputer;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.SettableOrientationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.SimpleTwoWaypointTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.trajectories.SwingTimeCalculationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.SwingTimeCalculator;
import us.ihmc.commonWalkingControlModules.trajectories.TransferTimeCalculationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointTrajectoryUtils;
import us.ihmc.commonWalkingControlModules.trajectories.WalkOnTheEdgesProviders;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.humanoidRobot.partNames.LegJointName;
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
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.Twist;

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
import com.yobotics.simulationconstructionset.util.trajectory.CurrentLinearVelocityProvider;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleProvider;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.FrameBasedPositionSource;
import com.yobotics.simulationconstructionset.util.trajectory.PositionProvider;
import com.yobotics.simulationconstructionset.util.trajectory.TrajectoryParameters;
import com.yobotics.simulationconstructionset.util.trajectory.TrajectoryParametersProvider;
import com.yobotics.simulationconstructionset.util.trajectory.TrajectoryWaypointGenerationMethod;
import com.yobotics.simulationconstructionset.util.trajectory.VectorProvider;
import com.yobotics.simulationconstructionset.util.trajectory.YoPositionProvider;
import com.yobotics.simulationconstructionset.util.trajectory.YoVariableDoubleProvider;
import com.yobotics.simulationconstructionset.util.trajectory.YoVelocityProvider;

public class WalkingHighLevelHumanoidController extends AbstractHighLevelHumanoidControlPattern
{
   private boolean VISUALIZE = true;

   private static final boolean DO_TRANSITION_WHEN_TIME_IS_UP = false;
   private static final boolean DESIREDICP_FROM_POLYGON_COORDINATE = false;
   private static final boolean USE_WORLDFRAME_SURFACE_NORMAL_WHEN_FULLY_CONSTRAINED = true;

   private final static HighLevelState controllerState = HighLevelState.WALKING;
   private final static MomentumControlModuleType MOMENTUM_CONTROL_MODULE_TO_USE = MomentumControlModuleType.OPT_NULLSPACE;

   private final double PELVIS_YAW_INITIALIZATION_TIME = 1.5;

   private PushRecoveryControlModule pushRecoveryModule;

   private final BooleanYoVariable alreadyBeenInDoubleSupportOnce;

   private static enum WalkingState
   {
      LEFT_SUPPORT, RIGHT_SUPPORT, TRANSFER_TO_LEFT_SUPPORT, TRANSFER_TO_RIGHT_SUPPORT, DOUBLE_SUPPORT
   }

   private final static boolean DEBUG = false;

   private final StateMachine<WalkingState> stateMachine;
   private final CenterOfMassJacobian centerOfMassJacobian;

   private final CoMHeightTrajectoryGenerator centerOfMassHeightTrajectoryGenerator;
   private final CoMHeightTimeDerivativesCalculator coMHeightTimeDerivativesCalculator = new CoMHeightTimeDerivativesCalculator();
   private final CoMHeightTimeDerivativesSmoother coMHeightTimeDerivativesSmoother;
   private final DoubleYoVariable desiredCoMHeightFromTrajectory = new DoubleYoVariable("desiredCoMHeightFromTrajectory", registry);
   private final DoubleYoVariable desiredCoMHeightVelocityFromTrajectory = new DoubleYoVariable("desiredCoMHeightVelocityFromTrajectory", registry);
   private final DoubleYoVariable desiredCoMHeightAccelerationFromTrajectory = new DoubleYoVariable("desiredCoMHeightAccelerationFromTrajectory", registry);
   private final DoubleYoVariable desiredCoMHeightBeforeSmoothing = new DoubleYoVariable("desiredCoMHeightBeforeSmoothing", registry);
   private final DoubleYoVariable desiredCoMHeightVelocityBeforeSmoothing = new DoubleYoVariable("desiredCoMHeightVelocityBeforeSmoothing", registry);
   private final DoubleYoVariable desiredCoMHeightAccelerationBeforeSmoothing = new DoubleYoVariable("desiredCoMHeightAccelerationBeforeSmoothing", registry);
   private final DoubleYoVariable desiredCoMHeightCorrected = new DoubleYoVariable("desiredCoMHeightCorrected", registry);
   private final DoubleYoVariable desiredCoMHeightVelocityCorrected = new DoubleYoVariable("desiredCoMHeightVelocityCorrected", registry);
   private final DoubleYoVariable desiredCoMHeightAccelerationCorrected = new DoubleYoVariable("desiredCoMHeightAccelerationCorrected", registry);
   private final DoubleYoVariable desiredCoMHeightAfterSmoothing = new DoubleYoVariable("desiredCoMHeightAfterSmoothing", registry);
   private final DoubleYoVariable desiredCoMHeightVelocityAfterSmoothing = new DoubleYoVariable("desiredCoMHeightVelocityAfterSmoothing", registry);
   private final DoubleYoVariable desiredCoMHeightAccelerationAfterSmoothing = new DoubleYoVariable("desiredCoMHeightAccelerationAfterSmoothing", registry);

   private final PDController centerOfMassHeightController;
   private final SideDependentList<WalkingState> singleSupportStateEnums = new SideDependentList<WalkingState>(WalkingState.LEFT_SUPPORT,
         WalkingState.RIGHT_SUPPORT);

   private final SideDependentList<WalkingState> transferStateEnums = new SideDependentList<WalkingState>(WalkingState.TRANSFER_TO_LEFT_SUPPORT,
         WalkingState.TRANSFER_TO_RIGHT_SUPPORT);

   private final YoFramePoint2d transferToFootstep = new YoFramePoint2d("transferToFootstep", worldFrame, registry);

   private final BooleanYoVariable resetIntegratorsAfterSwing = new BooleanYoVariable("resetIntegratorsAfterSwing", registry);

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
   private final BooleanYoVariable initializingICPTrajectory = new BooleanYoVariable("initializingICPTrajectory", registry);

   // private final FinalDesiredICPCalculator finalDesiredICPCalculator;

   private final BooleanYoVariable rememberFinalICPFromSingleSupport = new BooleanYoVariable("rememberFinalICPFromSingleSupport", registry);
   private final YoFramePoint2d finalDesiredICPInWorld = new YoFramePoint2d("finalDesiredICPInWorld", "", worldFrame, registry);

   private final SideDependentList<FootSwitchInterface> footSwitches;
   private final DoubleYoVariable footLoadThresholdToHoldPosition = new DoubleYoVariable("footLoadThresholdToHoldPosition", registry);
   private final SideDependentList<BooleanYoVariable> requestSupportFootToHoldPosition = new SideDependentList<BooleanYoVariable>();

   private final DoubleYoVariable maxICPErrorBeforeSingleSupport = new DoubleYoVariable("maxICPErrorBeforeSingleSupport", registry);

   private final DoubleYoVariable minOrbitalEnergyForSingleSupport = new DoubleYoVariable("minOrbitalEnergyForSingleSupport", registry);
   private final DoubleYoVariable amountToBeInsideSingleSupport = new DoubleYoVariable("amountToBeInsideSingleSupport", registry);
   private final DoubleYoVariable amountToBeInsideDoubleSupport = new DoubleYoVariable("amountToBeInsideDoubleSupport", registry);

   private final DoubleYoVariable userDesiredPelvisYaw = new DoubleYoVariable("userDesiredPelvisYaw", registry);
   private final DoubleYoVariable userDesiredPelvisPitch = new DoubleYoVariable("userDesiredPelvisPitch", registry);
   private final DoubleYoVariable userDesiredPelvisRoll = new DoubleYoVariable("userDesiredPelvisRoll", registry);
   private final BooleanYoVariable userSetDesiredPelvis = new BooleanYoVariable("userSetDesiredPelvis", registry);

   private final SettableOrientationProvider initialPelvisOrientationProvider;
   private final SettableOrientationProvider finalPelvisOrientationProvider;
   private final OrientationTrajectoryGenerator pelvisOrientationTrajectoryGenerator;

   private final SwingTimeCalculationProvider swingTimeCalculationProvider;
   private final TransferTimeCalculationProvider transferTimeCalculationProvider;

   private final TrajectoryParametersProvider trajectoryParametersProvider;

   private final DoubleYoVariable additionalSwingTimeForICP = new DoubleYoVariable("additionalSwingTimeForICP", registry);

   private final YoPositionProvider swingFootFinalPositionProvider;

   private final DoubleYoVariable swingAboveSupportAnkle = new DoubleYoVariable("swingAboveSupportAnkle", registry);
   private final BooleanYoVariable readyToGrabNextFootstep = new BooleanYoVariable("readyToGrabNextFootstep", registry);

   private final EnumYoVariable<RobotSide> previousSupportSide = new EnumYoVariable<RobotSide>("previousSupportSide", registry, RobotSide.class);

   private final DoubleYoVariable moveICPAwayDuringSwingDistance = new DoubleYoVariable("moveICPAwayDuringSwingDistance", registry);
   private final DoubleYoVariable moveICPAwayAtEndOfSwingDistance = new DoubleYoVariable("moveICPAwayAtEndOfSwingDistance", registry);
   private final DoubleYoVariable singleSupportTimeLeftBeforeShift = new DoubleYoVariable("singleSupportTimeLeftBeforeShift", registry);

   private final HashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters;
   private final InstantaneousCapturePointPlanner instantaneousCapturePointPlanner;
   private final ReinitializeWalkingControllerProvider reinitializeControllerProvider;

   private final SideDependentList<SettableOrientationProvider> finalFootOrientationProviders = new SideDependentList<SettableOrientationProvider>();

   private final ICPBasedMomentumRateOfChangeControlModule icpBasedMomentumRateOfChangeControlModule;

   private final BooleanYoVariable icpTrajectoryHasBeenInitialized;

   private final UpcomingFootstepList upcomingFootstepList;

   private final AverageOrientationCalculator averageOrientationCalculator = new AverageOrientationCalculator();

   private final ICPAndMomentumBasedController icpAndMomentumBasedController;

   private final EnumYoVariable<RobotSide> upcomingSupportLeg;
   private final EnumYoVariable<RobotSide> supportLeg;
   private final BipedSupportPolygons bipedSupportPolygons;
   private final YoFramePoint capturePoint;
   private final YoFramePoint2d desiredICP;
   private final DoubleYoVariable icpStandOffsetX = new DoubleYoVariable("icpStandOffsetX", registry);
   private final DoubleYoVariable icpStandOffsetY = new DoubleYoVariable("icpStandOffsetY", registry);
   private final YoFrameVector2d desiredICPVelocity;

   private final DoubleYoVariable swingTimeRemainingForICPMoveViz = new DoubleYoVariable("swingTimeRemainingForICPMoveViz", registry);
   private final DoubleYoVariable amountToMoveICPAway = new DoubleYoVariable("amountToMoveICPAway", registry);
   private final DoubleYoVariable distanceFromLineToOriginalICP = new DoubleYoVariable("distanceFromLineToOriginalICP", registry);

   private final DoubleYoVariable controlledCoMHeightAcceleration;
   private final DoubleYoVariable controllerInitializationTime;

   private final TransferToAndNextFootstepsDataVisualizer transferToAndNextFootstepsDataVisualizer;

   private final BooleanYoVariable doneFinishingSingleSupportTransfer = new BooleanYoVariable("doneFinishingSingleSupportTransfer", registry);
   private final BooleanYoVariable stayInTransferWalkingState = new BooleanYoVariable("stayInTransferWalkingState", registry);

   private final BooleanYoVariable ecmpBasedToeOffHasBeenInitialized = new BooleanYoVariable("ecmpBasedToeOffHasBeenInitialized", registry);
   private final YoFramePoint2d desiredECMP = new YoFramePoint2d("desiredECMP", "", worldFrame, registry);
   private final BooleanYoVariable desiredECMPinSupportPolygon = new BooleanYoVariable("desiredECMPinSupportPolygon", registry);
   private YoFramePoint ecmpViz = new YoFramePoint("ecmpViz", worldFrame, registry);

   private final YoVariableDoubleProvider totalEstimatedToeOffTimeProvider = new YoVariableDoubleProvider("totalEstimatedToeOffTimeProvider", registry);

   private final DoubleYoVariable singularityEscapeNullspaceMultiplierSwingLeg = new DoubleYoVariable("singularityEscapeNullspaceMultiplierSwingLeg", registry);
   private final DoubleYoVariable singularityEscapeNullspaceMultiplierSupportLeg = new DoubleYoVariable("singularityEscapeNullspaceMultiplierSupportLeg",
         registry);
   private final DoubleYoVariable singularityEscapeNullspaceMultiplierSupportLegLocking = new DoubleYoVariable("singularityEscapeNullspaceMultiplierSupportLegLocking", registry);

   private double referenceTime = 0.22;
   private MaximumConstantJerkFinalToeOffAngleComputer maximumConstantJerkFinalToeOffAngleComputer = new MaximumConstantJerkFinalToeOffAngleComputer();

   private final VariousWalkingProviders variousWalkingProviders;
   private final VariousWalkingManagers variousWalkingManagers;

   private final DoubleYoVariable walkingHeadOrientationKp = new DoubleYoVariable("walkingHeadOrientationKp", registry);
   private final DoubleYoVariable walkingHeadOrientationZeta = new DoubleYoVariable("walkingHeadOrientationZeta", registry);

   private final DoubleYoVariable swingKpXY = new DoubleYoVariable("swingKpXY", registry);
   private final DoubleYoVariable swingKpZ = new DoubleYoVariable("swingKpZ", registry);
   private final DoubleYoVariable swingKpOrientation = new DoubleYoVariable("swingKpOrientation", registry);
   private final DoubleYoVariable swingZetaXYZ = new DoubleYoVariable("swingZetaXYZ", registry);
   private final DoubleYoVariable swingZetaOrientation = new DoubleYoVariable("swingZetaOrientation", registry);

   private final DoubleYoVariable holdKpXY = new DoubleYoVariable("holdKpXY", registry);
   private final DoubleYoVariable holdKpOrientation = new DoubleYoVariable("holdKpOrientation", registry);
   private final DoubleYoVariable holdZeta = new DoubleYoVariable("holdZeta", registry);

   private final DoubleYoVariable toeOffKpXY = new DoubleYoVariable("toeOffKpXY", registry);
   private final DoubleYoVariable toeOffKpOrientation = new DoubleYoVariable("toeOffKpOrientation", registry);
   private final DoubleYoVariable toeOffZeta = new DoubleYoVariable("toeOffZeta", registry);

   private final DoubleYoVariable swingMaxPositionAcceleration = new DoubleYoVariable("swingMaxPositionAcceleration", registry);
   private final DoubleYoVariable swingMaxPositionJerk = new DoubleYoVariable("swingMaxPositionJerk", registry);
   private final DoubleYoVariable swingMaxOrientationAcceleration = new DoubleYoVariable("swingMaxOrientationAcceleration", registry);
   private final DoubleYoVariable swingMaxOrientationJerk = new DoubleYoVariable("swingMaxOrientationJerk", registry);
   private final YoFramePoint2dInPolygonCoordinate doubleSupportDesiredICP;
   private final WalkOnTheEdgesManager walkOnTheEdgesManager;
   private final WalkOnTheEdgesProviders walkOnTheEdgesProviders;

   private final BooleanYoVariable doPrepareManipulationForLocomotion = new BooleanYoVariable("doPrepareManipulationForLocomotion", registry);

   public WalkingHighLevelHumanoidController(VariousWalkingProviders variousWalkingProviders, VariousWalkingManagers variousWalkingManagers,
         SideDependentList<FootSwitchInterface> footSwitches, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
         CoMHeightTrajectoryGenerator centerOfMassHeightTrajectoryGenerator, TransferTimeCalculationProvider transferTimeCalculationProvider,
         double desiredPelvisPitch, WalkingControllerParameters walkingControllerParameters,
         ICPBasedMomentumRateOfChangeControlModule momentumRateOfChangeControlModule, LidarControllerInterface lidarControllerInterface,
         InstantaneousCapturePointPlanner instantaneousCapturePointPlanner, ICPAndMomentumBasedController icpAndMomentumBasedController,
         MomentumBasedController momentumBasedController, WalkingStatusReporter walkingStatusReporter)
   {
      super(variousWalkingProviders, variousWalkingManagers, momentumBasedController, walkingControllerParameters, lidarControllerInterface,
            dynamicGraphicObjectsListRegistry, controllerState);

      super.addUpdatables(icpAndMomentumBasedController.getUpdatables());

      userSetDesiredPelvis.addVariableChangedListener(new VariableChangedListener()
      {
         public void variableChanged(YoVariable<?> v)
         {
            FrameOrientation frameOrientation = new FrameOrientation(referenceFrames.getPelvisFrame());
            frameOrientation.changeFrame(ReferenceFrame.getWorldFrame());

            userDesiredPelvisYaw.set(frameOrientation.getYawPitchRoll()[0]);
         }
      });

      this.variousWalkingProviders = variousWalkingProviders;
      this.variousWalkingManagers = variousWalkingManagers;

      setupManagers(variousWalkingManagers);

      doPrepareManipulationForLocomotion.set(walkingControllerParameters.doPrepareManipulationForLocomotion());

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

      //    contactStates = momentumBasedController.getContactStates();
      upcomingSupportLeg = momentumBasedController.getUpcomingSupportLeg();
      supportLeg = icpAndMomentumBasedController.getYoSupportLeg();
      capturePoint = icpAndMomentumBasedController.getCapturePoint();
      desiredICP = icpAndMomentumBasedController.getDesiredICP();
      desiredICPVelocity = icpAndMomentumBasedController.getDesiredICPVelocity();
      bipedSupportPolygons = icpAndMomentumBasedController.getBipedSupportPolygons();
      controlledCoMHeightAcceleration = icpAndMomentumBasedController.getControlledCoMHeightAcceleration();
      centerOfMassJacobian = momentumBasedController.getCenterOfMassJacobian();

      coMHeightTimeDerivativesSmoother = new CoMHeightTimeDerivativesSmoother(controlDT, registry);

      // this.finalDesiredICPCalculator = finalDesiredICPCalculator;
      this.centerOfMassHeightTrajectoryGenerator = centerOfMassHeightTrajectoryGenerator;

      // Swing Time Provider:
      double swingTime = walkingControllerParameters.getDefaultSwingTime();
      SwingTimeCalculator swingTimeCalculator = new ConstantSwingTimeCalculator(swingTime, registry);
      this.swingTimeCalculationProvider = new SwingTimeCalculationProvider("providedSwingTime", registry, swingTimeCalculator, swingTime);

      this.transferTimeCalculationProvider = transferTimeCalculationProvider;

      this.trajectoryParametersProvider = new TrajectoryParametersProvider(new SimpleTwoWaypointTrajectoryParameters());
      this.mapFromFootstepsToTrajectoryParameters = mapFromFootstepsToTrajectoryParameters;
      this.footSwitches = footSwitches;
      this.icpBasedMomentumRateOfChangeControlModule = momentumRateOfChangeControlModule;

      this.instantaneousCapturePointPlanner = instantaneousCapturePointPlanner;

      this.upcomingFootstepList = new UpcomingFootstepList(footstepProvider, registry);

      this.centerOfMassHeightController = new PDController("comHeight", registry);
      double kpCoMHeight = walkingControllerParameters.getKpCoMHeight();
      centerOfMassHeightController.setProportionalGain(kpCoMHeight);
      double zetaCoMHeight = walkingControllerParameters.getZetaCoMHeight();
      centerOfMassHeightController.setDerivativeGain(GainCalculator.computeDerivativeGain(centerOfMassHeightController.getProportionalGain(), zetaCoMHeight));

      String namePrefix = "walking";

      this.stateMachine = new StateMachine<WalkingState>(namePrefix + "State", namePrefix + "SwitchTime", WalkingState.class, yoTime, registry); // this is used by name, and it is ugly.

      this.swingFootFinalPositionProvider = new YoPositionProvider(new YoFramePoint("swingFootFinalPosition", worldFrame, registry));

      this.icpTrajectoryHasBeenInitialized = new BooleanYoVariable("icpTrajectoryHasBeenInitialized", registry);

      rememberFinalICPFromSingleSupport.set(false); // true);
      finalDesiredICPInWorld.set(Double.NaN, Double.NaN);

      coefficientOfFriction.set(0.0); // TODO Remove coefficient of friction from the abstract high level stuff and let the EndEffector controlModule deal with it

      setupLegJacobians(fullRobotModel);

      this.walkOnTheEdgesProviders = new WalkOnTheEdgesProviders(walkingControllerParameters, registry);
      walkOnTheEdgesManager = new WalkOnTheEdgesManager(walkingControllerParameters, walkOnTheEdgesProviders, feet, footEndEffectorControlModules, registry);
      this.centerOfMassHeightTrajectoryGenerator.attachWalkOnToesManager(walkOnTheEdgesManager);

      maximumConstantJerkFinalToeOffAngleComputer.reinitialize(walkOnTheEdgesProviders.getMaximumToeOffAngle(), referenceTime);

      setupFootControlModules();

      initialPelvisOrientationProvider = new SettableOrientationProvider("initialPelvis", worldFrame, registry);
      finalPelvisOrientationProvider = new SettableOrientationProvider("finalPelvis", worldFrame, registry);
      this.pelvisOrientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator("pelvis", worldFrame, swingTimeCalculationProvider,
            initialPelvisOrientationProvider, finalPelvisOrientationProvider, registry);

      pushRecoveryModule = new PushRecoveryControlModule(momentumBasedController, walkingControllerParameters, readyToGrabNextFootstep, icpAndMomentumBasedController,
            swingTimeCalculator, stateMachine, registry, dynamicGraphicObjectsListRegistry);

      setUpStateMachine();
      readyToGrabNextFootstep.set(true);

      dwellInSingleSupportDuration.set(0.2);

      maxICPErrorBeforeSingleSupport.set(0.02); // 0.03); // Don't transition to single support until ICP is within 1.5 cm of desired.

      minOrbitalEnergyForSingleSupport.set(0.007); // 0.008
      amountToBeInsideSingleSupport.set(0.0);
      amountToBeInsideDoubleSupport.set(0.03); // 0.02);    // TODO: necessary for stairs...
      transferTimeCalculationProvider.setTransferTime();

      totalEstimatedToeOffTimeProvider.set(transferTimeCalculationProvider.getValue());

      stopInDoubleSupporTrajectoryTime.set(0.5);
      this.userDesiredPelvisPitch.set(desiredPelvisPitch);

      additionalSwingTimeForICP.set(0.1);
      minimumSwingFraction.set(0.5); // 0.8);

      upcomingSupportLeg.set(RobotSide.RIGHT); // TODO: stairs hack, so that the following lines use the correct leading leg

      controllerInitializationTime = new DoubleYoVariable("controllerInitializationTime", registry);
      alreadyBeenInDoubleSupportOnce = new BooleanYoVariable("alreadyBeenInDoubleSupportOnce", registry);

      // TODO: Fix low level stuff so that we are truly controlling pelvis height and not CoM height.
      controlPelvisHeightInsteadOfCoMHeight.set(true);

      moveICPAwayDuringSwingDistance.set(0.012); // 0.03);
      moveICPAwayAtEndOfSwingDistance.set(0.04); // 0.08);
      singleSupportTimeLeftBeforeShift.set(0.26);

      footLoadThresholdToHoldPosition.set(0.2);

      if (DESIREDICP_FROM_POLYGON_COORDINATE)
      {
         doubleSupportDesiredICP = new YoFramePoint2dInPolygonCoordinate("desiredICP", registry);
      }
      else
      {
         doubleSupportDesiredICP = null;
      }

      resetIntegratorsAfterSwing.set(true);
   }

   private void setupFootControlModules()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         BooleanYoVariable requestHoldPosition = new BooleanYoVariable(robotSide.getCamelCaseNameForStartOfExpression() + "RequestSupportFootToHoldPosition",
               registry);
         requestSupportFootToHoldPosition.put(robotSide, requestHoldPosition);
      }

      // TODO: Pull these up to a higher level.
      // TODO: Move these to DRCRobotWalkingControlParameters:

      singularityEscapeNullspaceMultiplierSwingLeg.set(50.0);
      singularityEscapeNullspaceMultiplierSupportLeg.set(30.0);
      singularityEscapeNullspaceMultiplierSupportLegLocking.set(0.0); // -0.5);
      double minJacobianDeterminantForSingularityEscape = 0.035;

      swingKpXY.set(walkingControllerParameters.getSwingKpXY());
      swingKpZ.set(walkingControllerParameters.getSwingKpZ());
      swingKpOrientation.set(walkingControllerParameters.getSwingKpOrientation());
      swingZetaXYZ.set(walkingControllerParameters.getSwingZetaXYZ());
      swingZetaOrientation.set(walkingControllerParameters.getSwingZetaOrientation());

      holdKpXY.set(walkingControllerParameters.getHoldKpXY());
      holdKpOrientation.set(walkingControllerParameters.getHoldKpOrientation());
      holdZeta.set(walkingControllerParameters.getHoldZeta());

      toeOffKpXY.set(walkingControllerParameters.getToeOffKpXY());
      toeOffKpOrientation.set(walkingControllerParameters.getToeOffKpOrientation());
      toeOffZeta.set(walkingControllerParameters.getToeOffZeta());

      swingMaxPositionAcceleration.set(walkingControllerParameters.getSwingMaxPositionAcceleration());
      swingMaxPositionJerk.set(walkingControllerParameters.getSwingMaxPositionJerk());
      swingMaxOrientationAcceleration.set(walkingControllerParameters.getSwingMaxOrientationAcceleration());
      swingMaxOrientationJerk.set(walkingControllerParameters.getswingMaxOrientationJerk());

      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody bipedFoot = feet.get(robotSide);

         // TODO: If we know the surface normal here, use it.
         momentumBasedController.setPlaneContactStateFullyConstrained(bipedFoot);

         String sideString = robotSide.getCamelCaseNameForStartOfExpression();

         OrientationProvider initialOrientationProvider = new CurrentOrientationProvider(worldFrame, bipedFoot.getBodyFrame());
         SettableOrientationProvider finalFootOrientationProvider = new SettableOrientationProvider(sideString + "FinalFootOrientation", worldFrame, registry);
         finalFootOrientationProviders.put(robotSide, finalFootOrientationProvider);

         DoubleTrajectoryGenerator footTouchdownPitchTrajectoryGenerator = walkOnTheEdgesProviders.getFootTouchdownPitchTrajectoryGenerator(robotSide);

         int jacobianId = legJacobianIds.get(robotSide);

         FootControlModule endEffectorControlModule;

         BooleanYoVariable requestHoldPosition = requestSupportFootToHoldPosition.get(robotSide);

         DoubleProvider maximumToeOffAngleProvider = walkOnTheEdgesProviders.getMaximumToeOffAngleProvider();
         YoVelocityProvider finalDesiredVelocityProvider =
               new YoVelocityProvider(robotSide + "TouchdownVelocity", ReferenceFrame.getWorldFrame(), registry);
         finalDesiredVelocityProvider.set(new Vector3d(0.0, 0.0, walkingControllerParameters.getDesiredTouchdownVelocity()));
         PositionProvider initialPositionProvider = new FrameBasedPositionSource(referenceFrames.getFootFrame(robotSide));
         VectorProvider initialVelocityProvider = new CurrentLinearVelocityProvider(referenceFrames.getFootFrame(robotSide),
               fullRobotModel.getFoot(robotSide), twistCalculator);
         
         endEffectorControlModule = new FootControlModule(controlDT, bipedFoot, jacobianId, robotSide,
                 footTouchdownPitchTrajectoryGenerator, maximumToeOffAngleProvider, requestHoldPosition, walkingControllerParameters,
                 swingTimeCalculationProvider, initialPositionProvider, initialVelocityProvider, swingFootFinalPositionProvider,
                 initialOrientationProvider, finalDesiredVelocityProvider, finalFootOrientationProvider, trajectoryParametersProvider,
                 dynamicGraphicObjectsListRegistry, momentumBasedController, registry);


         VariableChangedListener swingGainsChangedListener = createEndEffectorGainsChangedListener(endEffectorControlModule);
         swingGainsChangedListener.variableChanged(null);

         endEffectorControlModule.setParameters(minJacobianDeterminantForSingularityEscape, singularityEscapeNullspaceMultiplierSwingLeg.getDoubleValue());
         footEndEffectorControlModules.put(robotSide, endEffectorControlModule);
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
         FootControlModule swingEndEffectorControlModule = footEndEffectorControlModules.get(robotSide.getOppositeSide());
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
               new DoneWithTransferCondition(robotSide));
         transferState.addStateTransition(toSingleSupport);
         stateMachine.addState(transferState);

         State<WalkingState> singleSupportState = new SingleSupportState(robotSide);
         StateTransition<WalkingState> toDoubleSupport2 = new StateTransition<WalkingState>(doubleSupportState.getStateEnum(), stopWalkingCondition,
               stopWalkingStateTransitionActions);
         singleSupportState.addStateTransition(toDoubleSupport2);

         ContactablePlaneBody sameSideFoot = feet.get(robotSide);
         SingleSupportToTransferToCondition doneWithSingleSupportAndTransferToOppositeSideCondition = new SingleSupportToTransferToCondition(sameSideFoot,
               swingEndEffectorControlModule);
         StateTransition<WalkingState> toTransferOppositeSide = new StateTransition<WalkingState>(transferStateEnums.get(robotSide.getOppositeSide()),
               doneWithSingleSupportAndTransferToOppositeSideCondition, resetSwingTrajectoryDoneAction);
         singleSupportState.addStateTransition(toTransferOppositeSide);

         // Sometimes need transfer to same side when two steps are commanded on the same side. Otherwise, the feet cross over.
         ContactablePlaneBody oppositeSideFoot = feet.get(robotSide.getOppositeSide());
         SingleSupportToTransferToCondition doneWithSingleSupportAndTransferToSameSideCondition = new SingleSupportToTransferToCondition(oppositeSideFoot,
               swingEndEffectorControlModule);
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

//      if (pushRecoveryModule.isEnabled())
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            StateTransition<WalkingState> toFalling = new StateTransition<WalkingState>(singleSupportStateEnums.get(robotSide),
                  pushRecoveryModule.new IsFallingFromDoubleSupport(robotSide));
            doubleSupportState.addStateTransition(toFalling);
         }
      }
   }

   private RigidBody baseForHeadOrientationControl;
   private int jacobianIdForHeadOrientationControl;

   public void setupManagers(VariousWalkingManagers variousWalkingManagers)
   {
      baseForHeadOrientationControl = fullRobotModel.getElevator();
      HeadOrientationManager headOrientationManager = variousWalkingManagers.getHeadOrientationManager();
      String[] headOrientationControlJointNames = walkingControllerParameters.getDefaultHeadOrientationControlJointNames();

      if (headOrientationManager != null)
      {
         jacobianIdForHeadOrientationControl = headOrientationManager.createJacobian(fullRobotModel, headOrientationControlJointNames);
         headOrientationManager.setUp(baseForHeadOrientationControl, jacobianIdForHeadOrientationControl);
      }
   }

   public void initialize()
   {
      super.initialize();

      momentumBasedController.setMomentumControlModuleToUse(MOMENTUM_CONTROL_MODULE_TO_USE);

      initializeContacts();

      ChestOrientationManager chestOrientationManager = variousWalkingManagers.getChestOrientationManager();
      if (chestOrientationManager != null)
         chestOrientationManager.turnOff();

      HeadOrientationManager headOrientationManager = variousWalkingManagers.getHeadOrientationManager();
      if (headOrientationManager != null)
      {
         headOrientationManager.setUp(baseForHeadOrientationControl, jacobianIdForHeadOrientationControl);
         headOrientationManager.setMaxAccelerationAndJerk(walkingControllerParameters.getMaxAccelerationUpperBody(), walkingControllerParameters.getMaxJerkUpperBody());
         walkingHeadOrientationKp.set(walkingControllerParameters.getKpHeadOrientation());
         walkingHeadOrientationZeta.set(walkingControllerParameters.getZetaHeadOrientation());
         VariableChangedListener headGainsChangedListener = createHeadGainsChangedListener();
         headGainsChangedListener.variableChanged(null);
      }

      FrameOrientation initialDesiredPelvisOrientation = new FrameOrientation(referenceFrames.getAnkleZUpFrame(getUpcomingSupportLeg()));
      initialDesiredPelvisOrientation.changeFrame(worldFrame);
      double yaw = initialDesiredPelvisOrientation.getYawPitchRoll()[0];
      initialDesiredPelvisOrientation.setYawPitchRoll(yaw, userDesiredPelvisPitch.getDoubleValue(), userDesiredPelvisRoll.getDoubleValue());
      desiredPelvisOrientation.set(initialDesiredPelvisOrientation);
      finalPelvisOrientationProvider.setOrientation(initialDesiredPelvisOrientation); // yes, final. To make sure that the first swing phase has the right initial

      icpAndMomentumBasedController.computeCapturePoint();
      desiredICP.set(capturePoint.getFramePoint2dCopy());

      stateMachine.setCurrentState(WalkingState.DOUBLE_SUPPORT);

   }

   private VariableChangedListener createHeadGainsChangedListener()
   {
      VariableChangedListener ret = new VariableChangedListener()
      {
         public void variableChanged(YoVariable<?> v)
         {
            double headKp = walkingHeadOrientationKp.getDoubleValue();
            double headZeta = walkingHeadOrientationZeta.getDoubleValue();
            double headKd = GainCalculator.computeDerivativeGain(headKp, headZeta);
            headOrientationManager.setControlGains(headKp, headKd);
         }
      };

      walkingHeadOrientationKp.addVariableChangedListener(ret);
      walkingHeadOrientationZeta.addVariableChangedListener(ret);

      return ret;
   }

   private VariableChangedListener createEndEffectorGainsChangedListener(final FootControlModule endEffectorControlModule)
   {
      VariableChangedListener ret = new VariableChangedListener()
      {
         public void variableChanged(YoVariable<?> v)
         {
            endEffectorControlModule.setHoldGains(holdKpXY.getDoubleValue(), holdKpOrientation.getDoubleValue(), holdZeta.getDoubleValue());
            endEffectorControlModule.setSwingGains(swingKpXY.getDoubleValue(), swingKpZ.getDoubleValue(), swingKpOrientation.getDoubleValue(),
                  swingZetaXYZ.getDoubleValue(), swingZetaOrientation.getDoubleValue());
            endEffectorControlModule.setToeOffGains(toeOffKpXY.getDoubleValue(), toeOffKpOrientation.getDoubleValue(), toeOffZeta.getDoubleValue());
            endEffectorControlModule.setMaxAccelerationAndJerk(swingMaxPositionAcceleration.getDoubleValue(), swingMaxPositionJerk.getDoubleValue(),
                  swingMaxOrientationAcceleration.getDoubleValue(), swingMaxOrientationJerk.getDoubleValue());
         }
      };

      swingKpXY.addVariableChangedListener(ret);
      swingKpZ.addVariableChangedListener(ret);
      swingKpOrientation.addVariableChangedListener(ret);
      swingZetaXYZ.addVariableChangedListener(ret);
      swingZetaOrientation.addVariableChangedListener(ret);

      swingMaxPositionAcceleration.addVariableChangedListener(ret);
      swingMaxPositionJerk.addVariableChangedListener(ret);
      swingMaxOrientationAcceleration.addVariableChangedListener(ret);
      swingMaxOrientationJerk.addVariableChangedListener(ret);

      holdKpXY.addVariableChangedListener(ret);
      holdKpOrientation.addVariableChangedListener(ret);
      holdZeta.addVariableChangedListener(ret);

      toeOffKpXY.addVariableChangedListener(ret);
      toeOffKpOrientation.addVariableChangedListener(ret);
      toeOffZeta.addVariableChangedListener(ret);

      return ret;
   }

   private void initializeContacts()
   {
      momentumBasedController.clearContacts();

      for (RobotSide robotSide : RobotSide.values)
      {
         setFlatFootContactState(robotSide);
      }
   }

   private EnumYoVariable<RobotSide> trailingLeg = new EnumYoVariable<RobotSide>("trailingLeg", "", registry, RobotSide.class, true);

   private class DoubleSupportState extends State<WalkingState>
   {
      private final RobotSide transferToSide;
      private final FramePoint2d desiredICPLocal = new FramePoint2d();
      private final FrameVector2d desiredICPVelocityLocal = new FrameVector2d();
      private final FramePoint2d ecmpLocal = new FramePoint2d();
      private final FramePoint2d capturePoint2d = new FramePoint2d();

      public DoubleSupportState(RobotSide transferToSide)
      {
         super((transferToSide == null) ? WalkingState.DOUBLE_SUPPORT : transferStateEnums.get(transferToSide));
         this.transferToSide = transferToSide;
      }

      @Override
      public void doAction()
      {
         doNotIntegrateAnkleAccelerations();

         checkForReinitialization();

         if (transferToSide == null)
         {
            for (RobotSide robotSide : RobotSide.values)
            {
               FootControlModule footEndEffectorControlModule = footEndEffectorControlModules.get(robotSide);
               if ((footEndEffectorControlModule.isInEdgeTouchdownState() && walkOnTheEdgesManager.isEdgeTouchDownDone(robotSide))
                     || (footEndEffectorControlModule.getCurrentConstraintType() == ConstraintType.TOES))
                  setFlatFootContactState(robotSide);
            }
         }
         else
         {
            FootControlModule footEndEffectorControlModule = footEndEffectorControlModules.get(transferToSide);
            if ((footEndEffectorControlModule.isInEdgeTouchdownState() && walkOnTheEdgesManager.isEdgeTouchDownDone(transferToSide))
                  || (footEndEffectorControlModule.getCurrentConstraintType() == ConstraintType.TOES))
               setFlatFootContactState(transferToSide);
         }

         // note: this has to be done before the ICP trajectory generator is initialized, since it is using nextFootstep
         // TODO: Make a LOADING state and clean all of these timing hacks up.
         doneFinishingSingleSupportTransfer.set(instantaneousCapturePointPlanner.isPerformingICPDoubleSupport());
         double estimatedTimeRemainingForState = instantaneousCapturePointPlanner.getEstimatedTimeRemainingForState(yoTime.getDoubleValue());

         if (doneFinishingSingleSupportTransfer.getBooleanValue() || (estimatedTimeRemainingForState < 0.02))
         {
            upcomingFootstepList.checkForFootsteps(readyToGrabNextFootstep, upcomingSupportLeg, feet);
            checkForSteppingOnOrOff(transferToSide);
         }

         initializeICPPlannerIfNecessary();

         desiredICPLocal.setToZero(desiredICP.getReferenceFrame());
         desiredICPVelocityLocal.setToZero(desiredICPVelocity.getReferenceFrame());
         ecmpLocal.setToZero(worldFrame);
         capturePoint.getFrameTuple2dIncludingFrame(capturePoint2d);

         instantaneousCapturePointPlanner.getICPPositionAndVelocity(desiredICPLocal, desiredICPVelocityLocal, ecmpLocal, capturePoint2d,
               yoTime.getDoubleValue());

         if (transferToSide != null)
         {
            FramePoint2d stanceFootLocation = new FramePoint2d(referenceFrames.getAnkleZUpFrame(transferToSide));
            moveICPToInsideOfFootAtEndOfSwing(transferToSide.getOppositeSide(), stanceFootLocation, swingTimeCalculationProvider.getValue(), 0.0,
                  desiredICPLocal);
         }
         else
         {
            RobotSide previousSupport = previousSupportSide.getEnumValue();
            if (previousSupport != null)
            {
               FramePoint2d stanceFootLocation = new FramePoint2d(referenceFrames.getAnkleZUpFrame(previousSupport.getOppositeSide()));
               moveICPToInsideOfFootAtEndOfSwing(previousSupport, stanceFootLocation, swingTimeCalculationProvider.getValue(), 0.0, desiredICPLocal);
            }

            desiredICPLocal.changeFrame(referenceFrames.getMidFeetZUpFrame());
            desiredICPLocal.setX(desiredICPLocal.getX() + icpStandOffsetX.getDoubleValue());
            desiredICPLocal.setY(desiredICPLocal.getY() + icpStandOffsetY.getDoubleValue());

            FrameConvexPolygon2d supportPolygonInMidFeetZUp = bipedSupportPolygons.getSupportPolygonInMidFeetZUp();
            supportPolygonInMidFeetZUp.orthogonalProjection(desiredICPLocal);
            supportPolygonInMidFeetZUp.pullPointTowardsCentroid(desiredICPLocal, 0.10);

            desiredICPLocal.changeFrame(worldFrame);

            //          limitICPToMiddleOfFootOrInside(RobotSide.LEFT, desiredICPLocal);
            //          limitICPToMiddleOfFootOrInside(RobotSide.RIGHT, desiredICPLocal);
         }

         desiredICP.set(desiredICPLocal);
         desiredICPVelocity.set(desiredICPVelocityLocal);

         desiredECMP.set(ecmpLocal);

         if (VISUALIZE)
         {
            ecmpViz.set(desiredECMP.getX(), desiredECMP.getY(), 0.0);
         }

         initializeECMPbasedToeOffIfNotInitializedYet();

         // Only during the first few seconds, we will control the pelvis orientation based on midfeetZup
         if (((yoTime.getDoubleValue() - controllerInitializationTime.getDoubleValue()) < PELVIS_YAW_INITIALIZATION_TIME)
               && !alreadyBeenInDoubleSupportOnce.getBooleanValue())
         {
            setDesiredPelvisYawToAverageOfFeetOnStartupOnly(transferToSide);
         }

         if (userSetDesiredPelvis.getBooleanValue())
         {
            desiredPelvisOrientation.set(userDesiredPelvisYaw.getDoubleValue(), userDesiredPelvisPitch.getDoubleValue(), userDesiredPelvisRoll.getDoubleValue());
         }

         // keep desired pelvis orientation as it is
         desiredPelvisAngularVelocity.set(0.0, 0.0, 0.0);
         desiredPelvisAngularAcceleration.set(0.0, 0.0, 0.0);
      }

      boolean initializedAtStart = false;

      public void initializeICPPlannerIfNecessary()
      {
         if (!icpTrajectoryHasBeenInitialized.getBooleanValue() && instantaneousCapturePointPlanner.isDone(yoTime.getDoubleValue()))
         {
            initializingICPTrajectory.set(true);

            Pair<FramePoint2d, Double> finalDesiredICPAndTrajectoryTime = computeFinalDesiredICPAndTrajectoryTime();

            if (transferToSide != null) // the only case left for determining the contact state of the trailing foot
            {
               FramePoint2d finalDesiredICP = finalDesiredICPAndTrajectoryTime.first();
               finalDesiredICP.changeFrame(desiredICP.getReferenceFrame());

               RobotSide trailingLeg = transferToSide.getOppositeSide();
               walkOnTheEdgesManager.updateToeOffStatusBasedOnICP(trailingLeg, desiredICP.getFramePoint2dCopy(), finalDesiredICP);

               if (walkOnTheEdgesManager.doToeOff())
               {
                  setOnToesContactState(trailingLeg);
               }
            }
            else if (!initializedAtStart)
            {
               FramePoint2d finalDesiredICP = finalDesiredICPAndTrajectoryTime.first();
               finalDesiredICP.changeFrame(desiredICP.getReferenceFrame());

               desiredICP.set(finalDesiredICP);

               TransferToAndNextFootstepsData transferToAndNextFootstepsData = createTransferToAndNextFootstepDataForDoubleSupport(RobotSide.LEFT, true);
               instantaneousCapturePointPlanner.initializeDoubleSupport(transferToAndNextFootstepsData, 0.1);

               initializedAtStart = true;
            }

            icpAndMomentumBasedController.updateBipedSupportPolygons(bipedSupportPolygons); // need to always update biped support polygons after a change to the contact states
            icpTrajectoryHasBeenInitialized.set(true);
         }
         else
         {
            initializingICPTrajectory.set(false);
         }
      }

      private final FramePoint2d desiredCMP = new FramePoint2d();

      public void initializeECMPbasedToeOffIfNotInitializedYet()
      {
         // the only case left for determining the contact state of the trailing foot
         if ((!ecmpBasedToeOffHasBeenInitialized.getBooleanValue()) && (transferToSide != null))
         {
            RobotSide trailingLeg = transferToSide.getOppositeSide();
            icpBasedMomentumRateOfChangeControlModule.getDesiredCMP(desiredCMP);
            walkOnTheEdgesManager.updateToeOffStatusBasedOnECMP(trailingLeg, desiredCMP, desiredICP.getFramePoint2dCopy(), capturePoint2d);

            if (walkOnTheEdgesManager.doToeOff())
            {
               double remainingToeOffTime = instantaneousCapturePointPlanner.getEstimatedTimeRemainingForState(yoTime.getDoubleValue());
               walkOnTheEdgesProviders.setToeOffFinalAngle(maximumConstantJerkFinalToeOffAngleComputer.getMaximumFeasibleConstantJerkFinalToeOffAngle(
                     walkOnTheEdgesProviders.getToeOffInitialAngle(trailingLeg), remainingToeOffTime));

               setOnToesContactState(trailingLeg);
               icpAndMomentumBasedController.updateBipedSupportPolygons(bipedSupportPolygons); // need to always update biped support polygons after a change to the contact states
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

            //          finalDesiredICPInWorld.set(finalDesiredICP);

            finalDesiredICPAndTrajectoryTime = new Pair<FramePoint2d, Double>(finalDesiredICP, trajectoryTime);
         }

         else if (rememberFinalICPFromSingleSupport.getBooleanValue() && !finalDesiredICPInWorld.containsNaN())
         {
            FramePoint2d finalDesiredICP = finalDesiredICPInWorld.getFramePoint2dCopy();
            double trajectoryTime = transferTimeCalculationProvider.getValue();

            finalDesiredICPAndTrajectoryTime = new Pair<FramePoint2d, Double>(finalDesiredICP, trajectoryTime);
         }

         else
         {
            boolean inInitialize = false;
            TransferToAndNextFootstepsData transferToAndNextFootstepsData = createTransferToAndNextFootstepDataForDoubleSupport(transferToSide, inInitialize);

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
         Footstep transferToFootstep = createFootstepFromFootAndContactablePlaneBody(referenceFrames.getFootFrame(transferToSide), feet.get(transferToSide));

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
         double doubleSupportInitialTransferDuration = 0.4; // TODO: Magic Number
         transferToAndNextFootstepsData.setDoubleSupportInitialTransferDuration(doubleSupportInitialTransferDuration);
         boolean stopIfReachedEnd = (upcomingFootstepList.getNumberOfFootstepsToProvide() <= 3); // TODO: Magic Number
         transferToAndNextFootstepsData.setStopIfReachedEnd(stopIfReachedEnd);
         transferToAndNextFootstepsData.setCurrentDesiredICP(desiredICP.getFramePoint2dCopy(), desiredICPVelocity.getFrameVector2dCopy());

         if (VISUALIZE)
         {
            transferToAndNextFootstepsDataVisualizer.visualizeFootsteps(transferToAndNextFootstepsData);
         }

         return transferToAndNextFootstepsData;
      }

      @Override
      public void doTransitionIntoAction()
      {
         icpStandOffsetX.set(0.0);
         icpStandOffsetY.set(0.0);

         desiredECMPinSupportPolygon.set(false);
         ecmpBasedToeOffHasBeenInitialized.set(false);
         trailingLeg.set(transferToSide);

         icpTrajectoryHasBeenInitialized.set(false);
         if (DEBUG)
            System.out.println("WalkingHighLevelHumanoidController: enteringDoubleSupportState");
         setSupportLeg(null); // TODO: check if necessary

         if (walkOnTheEdgesManager.stayOnToes())
         {
            setOnToesContactStates();
         }
         else if (transferToSide == null)
         {
            for (RobotSide robotSide : RobotSide.values)
            {
               if (!footEndEffectorControlModules.get(robotSide).isInEdgeTouchdownState())
                  setFlatFootContactState(robotSide);
            }
         }
         else if (walkOnTheEdgesManager.willLandOnToes())
         {
            setTouchdownOnToesContactState(transferToSide);
         }
         else if (walkOnTheEdgesManager.willLandOnHeel())
         {
            setTouchdownOnHeelContactState(transferToSide);
         }
         else
         {
            if (footEndEffectorControlModules.get(transferToSide.getOppositeSide()).getCurrentConstraintType() == ConstraintType.SWING) // That case happens when doing 2 steps on same side
               setFlatFootContactState(transferToSide.getOppositeSide());
            setFlatFootContactState(transferToSide); // still need to determine contact state for trailing leg. This is done in doAction as soon as the previous ICP trajectory is done
         }

         walkOnTheEdgesManager.reset();

         if (transferToSide != null)
         {
            if (!instantaneousCapturePointPlanner.isDone(yoTime.getDoubleValue()))
            {
               Footstep transferToFootstep = createFootstepFromFootAndContactablePlaneBody(referenceFrames.getFootFrame(transferToSide),
                     feet.get(transferToSide));
               TransferToAndNextFootstepsData transferToAndNextFootstepsData = createTransferToAndNextFootstepDataForSingleSupport(transferToFootstep,
                     transferToSide);

               instantaneousCapturePointPlanner.reInitializeSingleSupport(transferToAndNextFootstepsData, yoTime.getDoubleValue());
            }
         }
         else
         {
            //          Do something smart here when going to DoubleSupport state.

            //            instantaneousCapturePointPlanner.initializeForStoppingInDoubleSupport(yoTime.getDoubleValue());

         }

         icpAndMomentumBasedController.updateBipedSupportPolygons(bipedSupportPolygons); // need to always update biped support polygons after a change to the contact states

         if (DESIREDICP_FROM_POLYGON_COORDINATE)
         {
            doubleSupportDesiredICP.updatePointAndPolygon(bipedSupportPolygons.getSupportPolygonInMidFeetZUp(), desiredICP.getFramePoint2dCopy());
         }

         RobotSide transferToSideToUseInFootstepData = transferToSide;
         if (transferToSideToUseInFootstepData == null)
            transferToSideToUseInFootstepData = RobotSide.LEFT; // Arbitrary here.

         if (!centerOfMassHeightTrajectoryGenerator.hasBeenInitializedWithNextStep())
         {
            //          System.out.println("Initializing centerOfMassHeightTrajectoryGenerator. transferToSide = " + transferToSide);

            boolean inInitialize = true;
            TransferToAndNextFootstepsData transferToAndNextFootstepsDataForDoubleSupport = createTransferToAndNextFootstepDataForDoubleSupport(
                  transferToSideToUseInFootstepData, inInitialize);

            centerOfMassHeightTrajectoryGenerator.initialize(transferToAndNextFootstepsDataForDoubleSupport,
                  transferToAndNextFootstepsDataForDoubleSupport.getTransferToSide(), null, getContactStatesList());
         }

         if (pushRecoveryModule.isEnabled())
         {
            pushRecoveryModule.setRecoverFromDoubleSupportFootStep(null);
            pushRecoveryModule.setRecoveringFromDoubleSupportState(false);
         }
      }

      @Override
      public void doTransitionOutOfAction()
      {
         icpStandOffsetX.set(0.0);
         icpStandOffsetY.set(0.0);

         desiredECMPinSupportPolygon.set(false);
         walkOnTheEdgesManager.reset();
         ecmpBasedToeOffHasBeenInitialized.set(false);

         alreadyBeenInDoubleSupportOnce.set(true);

         if (DEBUG)
            System.out.println("WalkingHighLevelHumanoidController: leavingDoubleSupportState");

         desiredICPVelocity.set(0.0, 0.0);

         if (manipulationControlModule != null && doPrepareManipulationForLocomotion.getBooleanValue())
            manipulationControlModule.prepareForLocomotion();
      }
   }

   private void setDesiredPelvisYawToAverageOfFeetOnStartupOnly(RobotSide transferToSide)
   {
      FrameOrientation averageOrientation = new FrameOrientation(worldFrame);
      averageOrientationCalculator.computeAverageOrientation(averageOrientation, feet.get(RobotSide.LEFT).getPlaneFrame(), feet.get(RobotSide.RIGHT).getPlaneFrame(), worldFrame);

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
      private final ErrorType[] singleSupportErrorToMonitor = new ErrorType[] { ErrorType.COM_Z, ErrorType.ICP_X, ErrorType.ICP_Y, ErrorType.PELVIS_ORIENTATION };

      private final FramePoint2d desiredICPLocal = new FramePoint2d();
      private final FrameVector2d desiredICPVelocityLocal = new FrameVector2d();
      private final FramePoint2d ecmpLocal = new FramePoint2d();
      private final FramePoint2d capturePoint2d = new FramePoint2d();

      private Footstep nextFootstep;
      private final FramePoint swingFootPosition = new FramePoint();
      private double captureTime;

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
         integrateAnkleAccelerationsOnSwingLeg(swingSide);

         checkForReinitialization();
         desiredICPLocal.setToZero(desiredICP.getReferenceFrame());
         desiredICPVelocityLocal.setToZero(desiredICPVelocity.getReferenceFrame());
         ecmpLocal.setToZero(worldFrame);

         capturePoint.getFrameTuple2dIncludingFrame(capturePoint2d);

         instantaneousCapturePointPlanner.getICPPositionAndVelocity(desiredICPLocal, desiredICPVelocityLocal, ecmpLocal, capturePoint2d,
               yoTime.getDoubleValue());

         RobotSide supportSide = swingSide.getOppositeSide();
         double swingTimeRemaining = swingTimeCalculationProvider.getValue() - (stateMachine.timeInCurrentState() - captureTime);
         FramePoint2d transferToFootstepLocation = transferToFootstep.getFramePoint2dCopy();
         FrameConvexPolygon2d footPolygon = computeFootPolygon(supportSide, referenceFrames.getAnkleZUpFrame(supportSide));
         double omega0 = icpAndMomentumBasedController.getOmega0();
         
         ReferenceFrame swingFootFrame = referenceFrames.getAnkleZUpFrame(swingSide);
         swingFootPosition.setToZero(swingFootFrame);
         swingFootPosition.changeFrame(nextFootstep.getPosition().getReferenceFrame());

         if (pushRecoveryModule.isEnabled())
         {
            boolean footstepHasBeenAdjusted = pushRecoveryModule.checkAndUpdateFootstep(swingSide, swingTimeRemaining, capturePoint2d, nextFootstep, omega0,
                  footPolygon);

            if (footstepHasBeenAdjusted)
            {
               captureTime = stateMachine.timeInCurrentState();
               updateFootstepParameters();
//               double oldSwingTime = swingTimeCalculationProvider.getValue();

               double distance = 2.0 * swingFootPosition.distance(nextFootstep.getPosition());
               swingTimeCalculationProvider.setSwingTime(Math.max(0.2, distance));//oldSwingTime - captureTime);

               footEndEffectorControlModules.get(swingSide).replanTrajectory();

               TransferToAndNextFootstepsData transferToAndNextFootstepsData = createTransferToAndNextFootstepDataForSingleSupport(nextFootstep, swingSide);
               instantaneousCapturePointPlanner.initializeSingleSupport(transferToAndNextFootstepsData, yoTime.getDoubleValue());
            }
         }

         moveICPToInsideOfFootAtEndOfSwing(supportSide, transferToFootstepLocation, swingTimeCalculationProvider.getValue(), swingTimeRemaining,
               desiredICPLocal);

         desiredICP.set(desiredICPLocal);
         desiredICPVelocity.set(desiredICPVelocityLocal);

         desiredECMP.set(ecmpLocal);

         if (VISUALIZE)
         {
            ecmpViz.set(desiredECMP.getX(), desiredECMP.getY(), 0.0);
         }

         pelvisOrientationTrajectoryGenerator.compute(stateMachine.timeInCurrentState() - captureTime);
         pelvisOrientationTrajectoryGenerator.get(desiredPelvisOrientationToPack);
         pelvisOrientationTrajectoryGenerator.packAngularVelocity(desiredPelvisAngularVelocityToPack);
         pelvisOrientationTrajectoryGenerator.packAngularAcceleration(desiredPelvisAngularAccelerationToPack);
         desiredPelvisOrientation.set(desiredPelvisOrientationToPack);
         desiredPelvisAngularVelocity.set(desiredPelvisAngularVelocityToPack);
         desiredPelvisAngularAcceleration.set(desiredPelvisAngularAccelerationToPack);

         if ((stateMachine.timeInCurrentState() - captureTime < 0.5 * swingTimeCalculationProvider.getValue())
               && footEndEffectorControlModules.get(swingSide).isInSingularityNeighborhood())
         {
            footEndEffectorControlModules.get(swingSide).doSingularityEscape(true);
         }
      }

      @Override
      public void doTransitionIntoAction()
      {
         captureTime = 0.0;
         hasICPPlannerFinished.set(false);
         trailingLeg.set(null);

         footSwitches.get(swingSide).reset();

         if (pushRecoveryModule.isEnabled() && pushRecoveryModule.getIsRecoveringFromDoubleSupportFall())
         {
            nextFootstep = pushRecoveryModule.getRecoverFromDoubleSupportFootStep();
         }
         else
         {
            nextFootstep = upcomingFootstepList.getNextFootstep();
         }

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

         walkOnTheEdgesManager.updateEdgeTouchdownStatus(swingSide.getOppositeSide(), nextFootstep);

         if (walkOnTheEdgesManager.willLandOnEdge())
         {
            nextFootstep = walkOnTheEdgesManager.createFootstepForEdgeTouchdown(nextFootstep);
            walkOnTheEdgesManager.updateTouchdownInitialAngularVelocity();
            nextFootstepHasBeenReplaced = true;
         }

         if (nextFootstepHasBeenReplaced)
            switchTrajectoryParametersMapping(oldNextFootstep, nextFootstep);

         if (DEBUG)
            System.out.println("WalkingHighLevelHumanoidController: enteringSingleSupportState");
         RobotSide supportSide = swingSide.getOppositeSide();

         setSupportLeg(supportSide);

         if (walkOnTheEdgesManager.stayOnToes())
         {
            setOnToesContactState(supportSide);
         }
         else
         {
            setFlatFootContactState(supportSide);
         }

         updateFootstepParameters();

         double stepPitch = nextFootstep.getOrientationInFrame(worldFrame).getYawPitchRoll()[1];
         walkOnTheEdgesProviders.setToeOffInitialAngle(swingSide, stepPitch);

         TransferToAndNextFootstepsData transferToAndNextFootstepsData = createTransferToAndNextFootstepDataForSingleSupport(nextFootstep, swingSide);
         instantaneousCapturePointPlanner.initializeSingleSupport(transferToAndNextFootstepsData, yoTime.getDoubleValue());

         if (walkingControllerParameters.resetDesiredICPToCurrentAtStartOfSwing())
         {
            desiredICP.set(capturePoint.getFramePoint2dCopy()); // TODO: currently necessary for stairs because of the omega0 jump, but should get rid of this
         }
      }

      private void updateFootstepParameters()
      {
         transferToFootstep.set(nextFootstep.getPosition2dCopy());
         RobotSide supportSide = swingSide.getOppositeSide();

         swingFootFinalPositionProvider.set(nextFootstep.getPositionInFrame(worldFrame));

         SideDependentList<Transform3D> footToWorldTransform = new SideDependentList<Transform3D>();
         for (RobotSide robotSide : RobotSide.values)
         {
            Transform3D transform = feet.get(robotSide).getBodyFrame().getTransformToDesiredFrame(worldFrame);
            footToWorldTransform.set(robotSide, transform);
         }

         Vector3d initialVectorPosition = new Vector3d();
         footToWorldTransform.get(supportSide.getOppositeSide()).get(initialVectorPosition);
         FramePoint initialFramePosition = new FramePoint(worldFrame, initialVectorPosition);
         FramePoint footFinalPosition = new FramePoint(worldFrame);
         swingFootFinalPositionProvider.get(footFinalPosition);
         double stepDistance = initialFramePosition.distance(footFinalPosition);
         swingTimeCalculationProvider.setSwingTimeByDistance(stepDistance);
         transferTimeCalculationProvider.setTransferTime();

         trajectoryParametersProvider.set(mapFromFootstepsToTrajectoryParameters.get(nextFootstep));
         finalFootOrientationProviders.get(swingSide).setOrientation(nextFootstep.getOrientationInFrame(worldFrame));

         FrameOrientation orientation = new FrameOrientation(desiredPelvisOrientation.getReferenceFrame());
         desiredPelvisOrientation.get(orientation);
         initialPelvisOrientationProvider.setOrientation(orientation);

         FrameOrientation finalPelvisOrientation = nextFootstep.getOrientationInFrame(worldFrame);
         FrameOrientation tempOrientation = new FrameOrientation();
         tempOrientation.interpolate(orientation, finalPelvisOrientation, 0.5);
         finalPelvisOrientation.setYawPitchRoll(tempOrientation.getYaw(), 0.0, 0.0);
         FramePoint swingFootFinalPosition = nextFootstep.getPositionInFrame(referenceFrames.getAnkleZUpFrame(swingSide.getOppositeSide()));
         FrameVector supportFootToSwingFoot = new FrameVector(swingFootFinalPosition);
         Vector3d temp = supportFootToSwingFoot.getVectorCopy();
         double desiredPelvisYawAngle = 0.0;
         if (Math.abs(temp.x) > 0.1)
         {
            desiredPelvisYawAngle = Math.atan2(temp.y, temp.x);
            desiredPelvisYawAngle -= swingSide.negateIfRightSide(Math.PI / 2.0);
         }

         finalPelvisOrientation.setYawPitchRoll(finalPelvisOrientation.getYaw() + userDesiredPelvisYaw.getDoubleValue() * desiredPelvisYawAngle,
               userDesiredPelvisPitch.getDoubleValue(), userDesiredPelvisRoll.getDoubleValue());
         finalPelvisOrientationProvider.setOrientation(finalPelvisOrientation);
         pelvisOrientationTrajectoryGenerator.initialize();

         FramePoint centerOfMass = new FramePoint(referenceFrames.getCenterOfMassFrame());
         centerOfMass.changeFrame(worldFrame);
         //ContactablePlaneBody supportFoot = feet.get(supportSide);
         //Transform3D supportFootToWorldTransform = footToWorldTransform.get(supportSide);
         //double footHeight = DesiredFootstepCalculatorTools.computeMinZPointInFrame(supportFootToWorldTransform, supportFoot, worldFrame).getZ();
         //double comHeight = centerOfMass.getZ() - footHeight;
         icpAndMomentumBasedController.computeCapturePoint();

         TransferToAndNextFootstepsData transferToAndNextFootstepsData = createTransferToAndNextFootstepDataForSingleSupport(nextFootstep, swingSide);
         //FramePoint2d finalDesiredICP = getSingleSupportFinalDesiredICPForWalking(transferToAndNextFootstepsData, swingSide);

         setContactStateForSwing(swingSide);
         setSupportLeg(supportSide);
         icpAndMomentumBasedController.updateBipedSupportPolygons(bipedSupportPolygons);

         // Shouldn't have to do this init anymore since it's done above...
         // icpTrajectoryGenerator.initialize(desiredICP.getFramePoint2dCopy(), finalDesiredICP, swingTimeCalculationProvider.getValue(), omega0,
         // amountToBeInsideSingleSupport.getDoubleValue(), getSupportLeg(), yoTime.getDoubleValue());

         centerOfMassHeightTrajectoryGenerator.initialize(transferToAndNextFootstepsData, getSupportLeg(), nextFootstep, getContactStatesList());

         if (DEBUG)
            System.out.println("WalkingHighLevelHumanoidController: nextFootstep will change now!");
         readyToGrabNextFootstep.set(true);

         //       instantaneousCapturePointPlanner.setDoHeelToToeTransfer(walkOnTheEdgesManager.willDoToeOff(transferToAndNextFootstepsData));
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

         if (pushRecoveryModule.isEnabled())
         {
            if (!pushRecoveryModule.getIsRecoveringFromDoubleSupportFall())
            {
               previousSupportSide.set(swingSide.getOppositeSide());
            }

            // ContactableBody swingFoot = contactablePlaneBodies.get(swingSide);
            // Footstep desiredFootstep = desiredFootstepCalculator.updateAndGetDesiredFootstep(swingSide.getOppositeSide());
            // contactStates.get(swingFoot).setContactPoints(desiredFootstep.getExpectedContactPoints());
            // updateFootStateMachines(swingFoot);

            pushRecoveryModule.reset();
         }

         resetLoadedLegIntegrators(swingSide);
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
      private final RobotSide robotSide;
      private final FramePoint2d capturePoint2d = new FramePoint2d();
      private final FramePoint2d desiredICP2d = new FramePoint2d();

      public DoneWithTransferCondition(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }

      public boolean checkCondition()
      {
         if (stayInTransferWalkingState.getBooleanValue())
            return false;

         if (!walkOnTheEdgesManager.isEdgeTouchDownDone(robotSide))
            return false;

         if (walkingControllerParameters.checkOrbitalEnergyCondition())
         {
            // TODO: not really nice, but it'll do:
            FlatThenPolynomialCoMHeightTrajectoryGenerator flatThenPolynomialCoMHeightTrajectoryGenerator = (FlatThenPolynomialCoMHeightTrajectoryGenerator) centerOfMassHeightTrajectoryGenerator;
            double orbitalEnergy = flatThenPolynomialCoMHeightTrajectoryGenerator.computeOrbitalEnergyIfInitializedNow(getUpcomingSupportLeg());

            // return transferICPTrajectoryDone.getBooleanValue() && orbitalEnergy > minOrbitalEnergy;
            return icpTrajectoryHasBeenInitialized.getBooleanValue() && (orbitalEnergy > minOrbitalEnergyForSingleSupport.getDoubleValue());
         }
         else
         {
            boolean icpTrajectoryIsDone = icpTrajectoryHasBeenInitialized.getBooleanValue() && instantaneousCapturePointPlanner.isDone(yoTime.getDoubleValue());
            if (!icpTrajectoryIsDone)
               return false;

            capturePoint.getFrameTuple2dIncludingFrame(capturePoint2d);
            desiredICP.getFrameTuple2dIncludingFrame(desiredICP2d);

            double distanceFromDesiredToActual = capturePoint2d.distance(desiredICP2d);
            boolean closeEnough = distanceFromDesiredToActual < maxICPErrorBeforeSingleSupport.getDoubleValue();

            return closeEnough;
         }
      }
   }

   private class SingleSupportToTransferToCondition extends DoneWithSingleSupportCondition
   {
      private final ContactablePlaneBody nextSwingFoot;

      public SingleSupportToTransferToCondition(ContactablePlaneBody nextSwingFoot, FootControlModule endEffectorControlModule)
      {
         super(endEffectorControlModule);

         this.nextSwingFoot = nextSwingFoot;
      }

      public boolean checkCondition()
      {
         Footstep nextFootstep = upcomingFootstepList.getNextNextFootstep();
         if (nextFootstep == null)
            return super.checkCondition();

         ContactablePlaneBody nextSwingFoot = nextFootstep.getBody();
         if (this.nextSwingFoot != nextSwingFoot)
            return false;

         boolean condition = super.checkCondition();

         return condition;
      }

   }

   private class DoneWithSingleSupportCondition implements StateTransitionCondition
   {
      public DoneWithSingleSupportCondition(FootControlModule endEffectorControlModule)
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

         if (walkOnTheEdgesManager.willLandOnToes())
         {
            if (!(footSwitch instanceof ToeSwitch))
            {
               throw new RuntimeException("toe touchdown should not be used if Robot is not using a ToeSwitch.");
            }

            ToeSwitch toeSwitch = (ToeSwitch) footSwitch;
            footSwitchActivated = toeSwitch.hasToeHitGround();
         }
         else if (walkOnTheEdgesManager.willLandOnHeel())
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

         if (hasMinimumTimePassed.getBooleanValue() && justFall.getBooleanValue())
            return true;

         // Just switch states if icp is done, plus a little bit more. You had enough time and more isn't going to do any good.
         
         if (pushRecoveryModule.isEnabled() && pushRecoveryModule.getIsRecoveringFromDoubleSupportFall())
         {
            return stateMachine.timeInCurrentState() > pushRecoveryModule.getTrustTimeToConsiderSwingFinished();
         }

         if (DO_TRANSITION_WHEN_TIME_IS_UP)
         {
            if (hasICPPlannerFinished.getBooleanValue()
                  && (yoTime.getDoubleValue() > timeThatICPPlannerFinished.getDoubleValue() + dwellInSingleSupportDuration.getDoubleValue()))
               return true;
         }

         if (walkingControllerParameters.finishSwingWhenTrajectoryDone())
         {
            return hasMinimumTimePassed.getBooleanValue() && (hasICPPlannerFinished.getBooleanValue() || footSwitchActivated);
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
      private FootControlModule endEffectorControlModule;

      public ResetSwingTrajectoryDoneAction(FootControlModule endEffectorControlModule)
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
      public StopWalkingCondition(FootControlModule endEffectorControlModule)
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
      double trailingFootToLeadingFootFactor = 0.5; // 0.25;
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
      ReferenceFrame referenceFrame = worldFrame;

      // FramePoint2d initialDesiredICP = desiredICP.getFramePoint2dCopy();
      // initialDesiredICP.changeFrame(referenceFrame);

      instantaneousCapturePointPlanner.initializeSingleSupport(transferToAndNextFootstepsData, yoTime.getDoubleValue());

      FramePoint2d finalDesiredICP = instantaneousCapturePointPlanner.getFinalDesiredICP();
      finalDesiredICP.changeFrame(referenceFrame);

      finalDesiredICPInWorld.set(finalDesiredICP);

      RobotSide supportSide = swingSide.getOppositeSide();
      walkOnTheEdgesManager.updateOnToesTriangle(finalDesiredICP, supportSide);

      FramePoint2d icpWayPoint;
      if (walkOnTheEdgesManager.doToeOffIfPossible() && walkOnTheEdgesManager.isOnToesTriangleLargeEnough())
      {
         FramePoint toeOffPoint = new FramePoint(worldFrame);
         ContactablePlaneBody supportFoot = feet.get(supportSide);
         List<FramePoint> toePoints = getToePoints(supportFoot);
         toeOffPoint.interpolate(toePoints.get(0), toePoints.get(1), 0.5);
         FramePoint2d toeOffPoint2d = toeOffPoint.toFramePoint2d();
         double toeOffPointToFinalDesiredFactor = 0.2; // TODO: magic number
         FramePoint2d desiredToeOffCoP = new FramePoint2d(worldFrame);
         desiredToeOffCoP.interpolate(toeOffPoint2d, finalDesiredICP, toeOffPointToFinalDesiredFactor);
         icpWayPoint = EquivalentConstantCoPCalculator.computeICPPositionWithConstantCMP(finalDesiredICP, desiredToeOffCoP,
               -transferTimeCalculationProvider.getValue(), icpAndMomentumBasedController.getOmega0());
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

      final FrameConvexPolygon2d footPolygon = new FrameConvexPolygon2d();
      ContactablePlaneBody contactableBody = feet.get(swingSide);
      if (walkOnTheEdgesManager.stayOnToes())
      {
         List<FramePoint> contactPoints = getToePoints(contactableBody);
         footPolygon.setIncludingFrameByProjectionOntoXYPlaneAndUpdate(referenceFrames.getSoleFrame(swingSide), contactPoints);
      }
      else
      {
         footPolygon.setIncludingFrameAndUpdate(contactableBody.getContactPoints2d());
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
      double doubleSupportInitialTransferDuration = 0.4; // TODO: Magic Number
      transferToAndNextFootstepsData.setDoubleSupportInitialTransferDuration(doubleSupportInitialTransferDuration);
      boolean stopIfReachedEnd = (upcomingFootstepList.getNumberOfFootstepsToProvide() <= 3); // TODO: Magic Number
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
      List<FramePoint> toePoints = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(supportFoot.getContactPointsCopy(), forward, nToePoints);
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
         while (true)
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
      for (RobotSide robotSide : RobotSide.values)
      {
         requestSupportFootToHoldPosition.get(robotSide).set(footSwitches.get(robotSide).computeFootLoadPercentage() < footLoadThresholdToHoldPosition.getDoubleValue());
      }

      momentumBasedController.doPrioritaryControl();
      super.callUpdatables();

      icpAndMomentumBasedController.computeCapturePoint();
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();

      controlledCoMHeightAcceleration.set(computeDesiredCoMHeightAcceleration(desiredICPVelocity.getFrameVector2dCopy()));

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
      momentumBasedController.doProportionalControlOnCoP();
   }

   // TODO: connect ports instead
   private void setICPBasedMomentumRateOfChangeControlModuleInputs()
   {
      icpBasedMomentumRateOfChangeControlModule.getBipedSupportPolygonsInputPort().setData(bipedSupportPolygons);

      CapturePointData capturePointData = new CapturePointData();
      capturePointData.set(capturePoint.getFramePoint2dCopy(), icpAndMomentumBasedController.getOmega0());
      icpBasedMomentumRateOfChangeControlModule.getCapturePointInputPort().setData(capturePointData);

      CapturePointTrajectoryData capturePointTrajectoryData = new CapturePointTrajectoryData();
      capturePointTrajectoryData.set(finalDesiredICPInWorld.getFramePoint2dCopy(), desiredICP.getFramePoint2dCopy(), desiredICPVelocity.getFrameVector2dCopy());
      icpBasedMomentumRateOfChangeControlModule.getDesiredCapturePointTrajectoryInputPort().setData(capturePointTrajectoryData);

      icpBasedMomentumRateOfChangeControlModule.getSupportLegInputPort().setData(getSupportLeg());

      icpBasedMomentumRateOfChangeControlModule.getDesiredCenterOfMassHeightAccelerationInputPort().setData(controlledCoMHeightAcceleration.getDoubleValue());

      icpBasedMomentumRateOfChangeControlModule.startComputation();
      icpBasedMomentumRateOfChangeControlModule.waitUntilComputationIsDone();
      MomentumRateOfChangeData momentumRateOfChangeData = icpBasedMomentumRateOfChangeControlModule.getMomentumRateOfChangeOutputPort().getData();
      momentumBasedController.setDesiredRateOfChangeOfMomentum(momentumRateOfChangeData);
   }

   // Temporary objects to reduce garbage collection.
   private final CoMHeightPartialDerivativesData coMHeightPartialDerivatives = new CoMHeightPartialDerivativesData();
   private final ContactStatesAndUpcomingFootstepData centerOfMassHeightInputData = new ContactStatesAndUpcomingFootstepData();
   private final FramePoint comPosition = new FramePoint();
   private final FrameVector comVelocity = new FrameVector(worldFrame);
   private final FrameVector2d comXYVelocity = new FrameVector2d();
   private final FrameVector2d comXYAcceleration = new FrameVector2d();
   private final CoMHeightTimeDerivativesData comHeightDataBeforeSmoothing = new CoMHeightTimeDerivativesData();
   private final CoMHeightTimeDerivativesData comHeightDataAfterSmoothing = new CoMHeightTimeDerivativesData();
   private final CoMXYTimeDerivativesData comXYTimeDerivatives = new CoMXYTimeDerivativesData();
   private final FramePoint desiredCenterOfMassHeightPoint = new FramePoint(worldFrame);

   private double computeDesiredCoMHeightAcceleration(FrameVector2d desiredICPVelocity)
   {
      double zCurrent = comPosition.getZ();
      double zdCurrent = comVelocity.getZ();
      Footstep nextFootstep;

      if (controlPelvisHeightInsteadOfCoMHeight.getBooleanValue())
      {
         FramePoint pelvisPosition = new FramePoint(referenceFrames.getPelvisFrame());
         pelvisPosition.changeFrame(worldFrame);
         zCurrent = pelvisPosition.getZ();
         Twist pelvisTwist = new Twist();
         twistCalculator.packTwistOfBody(pelvisTwist, fullRobotModel.getPelvis());
         pelvisTwist.changeFrame(worldFrame);
         zdCurrent = comVelocity.getZ(); // Just use com velocity for now for damping...
      }

      centerOfMassHeightInputData.setCenterOfMassAndPelvisZUpFrames(momentumBasedController.getCenterOfMassFrame(), momentumBasedController.getPelvisZUpFrame());

      List<? extends PlaneContactState> contactStatesList = getContactStatesList();

      centerOfMassHeightInputData.setContactStates(contactStatesList);

      centerOfMassHeightInputData.setSupportLeg(getSupportLeg());

      if (pushRecoveryModule.isEnabled() && pushRecoveryModule.getIsRecoveringFromDoubleSupportFall())
      {
         nextFootstep = pushRecoveryModule.getRecoverFromDoubleSupportFootStep();
      }
      else
      {
         nextFootstep = upcomingFootstepList.getNextFootstep();
      }
      
      centerOfMassHeightInputData.setUpcomingFootstep(nextFootstep);

      centerOfMassHeightTrajectoryGenerator.solve(coMHeightPartialDerivatives, centerOfMassHeightInputData);

      comPosition.setToZero(referenceFrames.getCenterOfMassFrame());
      centerOfMassJacobian.packCenterOfMassVelocity(comVelocity);
      comPosition.changeFrame(worldFrame);
      comVelocity.changeFrame(worldFrame);

      // TODO: use current omega0 instead of previous
      comXYVelocity.setIncludingFrame(comVelocity.getReferenceFrame(), comVelocity.getX(), comVelocity.getY());
      comXYAcceleration.setIncludingFrame(desiredICPVelocity);
      comXYAcceleration.sub(comXYVelocity);
      comXYAcceleration.scale(icpAndMomentumBasedController.getOmega0()); // MathTools.square(omega0.getDoubleValue()) * (com.getX() - copX);

      // FrameVector2d comd2dSquared = new FrameVector2d(comXYVelocity.getReferenceFrame(), comXYVelocity.getX() * comXYVelocity.getX(), comXYVelocity.getY() * comXYVelocity.getY());

      comXYTimeDerivatives.setCoMXYPosition(comPosition.toFramePoint2d());
      comXYTimeDerivatives.setCoMXYVelocity(comXYVelocity);
      comXYTimeDerivatives.setCoMXYAcceleration(comXYAcceleration);

      coMHeightTimeDerivativesCalculator.computeCoMHeightTimeDerivatives(comHeightDataBeforeSmoothing, comXYTimeDerivatives, coMHeightPartialDerivatives);

      comHeightDataBeforeSmoothing.getComHeight(desiredCenterOfMassHeightPoint);
      desiredCoMHeightFromTrajectory.set(desiredCenterOfMassHeightPoint.getZ());
      desiredCoMHeightVelocityFromTrajectory.set(comHeightDataBeforeSmoothing.getComHeightVelocity());
      desiredCoMHeightAccelerationFromTrajectory.set(comHeightDataBeforeSmoothing.getComHeightAcceleration());

      //    correctCoMHeight(desiredICPVelocity, zCurrent, comHeightDataBeforeSmoothing, false, false);

      //    comHeightDataBeforeSmoothing.getComHeight(desiredCenterOfMassHeightPoint);
      //    desiredCoMHeightBeforeSmoothing.set(desiredCenterOfMassHeightPoint.getZ());
      //    desiredCoMHeightVelocityBeforeSmoothing.set(comHeightDataBeforeSmoothing.getComHeightVelocity());
      //    desiredCoMHeightAccelerationBeforeSmoothing.set(comHeightDataBeforeSmoothing.getComHeightAcceleration());

      coMHeightTimeDerivativesSmoother.smooth(comHeightDataAfterSmoothing, comHeightDataBeforeSmoothing);

      comHeightDataAfterSmoothing.getComHeight(desiredCenterOfMassHeightPoint);
      desiredCoMHeightAfterSmoothing.set(desiredCenterOfMassHeightPoint.getZ());
      desiredCoMHeightVelocityAfterSmoothing.set(comHeightDataAfterSmoothing.getComHeightVelocity());
      desiredCoMHeightAccelerationAfterSmoothing.set(comHeightDataAfterSmoothing.getComHeightAcceleration());

      correctCoMHeight(desiredICPVelocity, zCurrent, comHeightDataAfterSmoothing, true, true);

      comHeightDataAfterSmoothing.getComHeight(desiredCenterOfMassHeightPoint);
      desiredCoMHeightCorrected.set(desiredCenterOfMassHeightPoint.getZ());
      desiredCoMHeightVelocityCorrected.set(comHeightDataAfterSmoothing.getComHeightVelocity());
      desiredCoMHeightAccelerationCorrected.set(comHeightDataAfterSmoothing.getComHeightAcceleration());

      comHeightDataAfterSmoothing.getComHeight(desiredCenterOfMassHeightPoint);

      double zDesired = desiredCenterOfMassHeightPoint.getZ();
      double zdDesired = comHeightDataAfterSmoothing.getComHeightVelocity();
      double zddFeedForward = comHeightDataAfterSmoothing.getComHeightAcceleration();

      double zddDesired = centerOfMassHeightController.compute(zCurrent, zDesired, zdCurrent, zdDesired) + zddFeedForward;

      for (RobotSide robotSide : RobotSide.values)
      {
         FootControlModule endEffectorControlModule = footEndEffectorControlModules.get(robotSide);

         if (endEffectorControlModule.isInFlatSupportState() && endEffectorControlModule.isInSingularityNeighborhood())
         {
            // Ignore the desired height acceleration only if EndEffectorControlModule is not taking care of singularity during support
            if (!LegSingularityAndKneeCollapseAvoidanceControlModule.USE_SINGULARITY_AVOIDANCE_SUPPORT)
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

   private final SideDependentList<Double> legLengths = new SideDependentList<Double>();

   private void correctCoMHeight(FrameVector2d desiredICPVelocity, double zCurrent, CoMHeightTimeDerivativesData comHeightData, boolean checkForKneeCollapsing,
         boolean checkForStraightKnee)
   {
      ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();

      RobotSide[] leadingLegFirst;
      if (trailingLeg.getEnumValue() != null)
         leadingLegFirst = new RobotSide[] { trailingLeg.getEnumValue().getOppositeSide(), trailingLeg.getEnumValue() };
      else
         leadingLegFirst = RobotSide.values;

      for (RobotSide robotSide : RobotSide.values)
      {
         legLengths.put(robotSide, footEndEffectorControlModules.get(robotSide).updateAndGetLegLength());
      }

      // Correct, if necessary, the CoM height trajectory to avoid the knee to collapse
      if (checkForKneeCollapsing)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            footEndEffectorControlModules.get(robotSide).correctCoMHeightTrajectoryForCollapseAvoidance(desiredICPVelocity, comHeightData, zCurrent,
                  pelvisZUpFrame, footSwitches.get(robotSide).computeFootLoadPercentage());
         }
      }

      // Correct, if necessary, the CoM height trajectory to avoid straight knee
      if (checkForStraightKnee)
      {
         for (RobotSide robotSide : leadingLegFirst)
         {
            FootControlModule footEndEffectorControlModule = footEndEffectorControlModules.get(robotSide);
            footEndEffectorControlModule.correctCoMHeightTrajectoryForSingularityAvoidance(desiredICPVelocity, comHeightData, zCurrent, pelvisZUpFrame);
         }
      }

      // Do that after to make sure the swing foot will land
      for (RobotSide robotSide : RobotSide.values)
      {
         footEndEffectorControlModules.get(robotSide).correctCoMHeightTrajectoryForUnreachableFootStep(comHeightData);
      }
   }

   private List<PlaneContactState> getContactStatesList()
   {
      List<PlaneContactState> contactStatesList = new ArrayList<PlaneContactState>();

      for (ContactablePlaneBody contactablePlaneBody : feet)
      {
         PlaneContactState contactState = momentumBasedController.getContactState(contactablePlaneBody);

         //       YoPlaneContactState contactState = contactStates.get(contactablePlaneBody);
         if (contactState.inContact())
            contactStatesList.add(contactState);
      }

      return contactStatesList;
   }

   private void setOnToesContactStates()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         setOnToesContactState(robotSide);
      }
   }

   private final FrameVector footNormalContactVector = new FrameVector(worldFrame, 0.0, 0.0, 1.0);

   private void setOnToesContactState(RobotSide robotSide)
   {
      FootControlModule footEndEffectorControlModule = footEndEffectorControlModules.get(robotSide);
      if (footEndEffectorControlModule.isInFlatSupportState())
      {
         footNormalContactVector.setIncludingFrame(feet.get(robotSide).getPlaneFrame(), 0.0, 0.0, 1.0);
         footNormalContactVector.changeFrame(worldFrame);
      }
      else
      {
         footNormalContactVector.setIncludingFrame(worldFrame, 0.0, 0.0, 1.0);
      }

      footEndEffectorControlModule.setContactState(ConstraintType.TOES, footNormalContactVector);
   }

   private void setTouchdownOnHeelContactState(RobotSide robotSide)
   {
      footNormalContactVector.setIncludingFrame(worldFrame, 0.0, 0.0, 1.0);
      footEndEffectorControlModules.get(robotSide).setContactState(ConstraintType.HEEL_TOUCHDOWN, footNormalContactVector);
   }

   private void setTouchdownOnToesContactState(RobotSide robotSide)
   {
      footNormalContactVector.setIncludingFrame(worldFrame, 0.0, 0.0, 1.0);
      footEndEffectorControlModules.get(robotSide).setContactState(ConstraintType.TOES_TOUCHDOWN, footNormalContactVector);
   }

   private void setFlatFootContactState(RobotSide robotSide)
   {
      if (USE_WORLDFRAME_SURFACE_NORMAL_WHEN_FULLY_CONSTRAINED)
         footNormalContactVector.setIncludingFrame(worldFrame, 0.0, 0.0, 1.0);
      else
         footNormalContactVector.setIncludingFrame(feet.get(robotSide).getPlaneFrame(), 0.0, 0.0, 1.0);
      footEndEffectorControlModules.get(robotSide).setContactState(ConstraintType.FULL, footNormalContactVector);
   }

   private void setContactStateForSwing(RobotSide robotSide)
   {
      FootControlModule endEffectorControlModule = footEndEffectorControlModules.get(robotSide);
      endEffectorControlModule.doSingularityEscape(true);
      endEffectorControlModule.setContactState(ConstraintType.SWING);
   }

   private final List<FramePoint> tempContactPoints = new ArrayList<FramePoint>();
   private final FrameConvexPolygon2d tempFootPolygon = new FrameConvexPolygon2d(worldFrame);

   // TODO: should probably precompute this somewhere else
   private FrameConvexPolygon2d computeFootPolygon(RobotSide robotSide, ReferenceFrame referenceFrame)
   {
      momentumBasedController.getContactPoints(feet.get(robotSide), tempContactPoints);
      tempFootPolygon.setIncludingFrameByProjectionOntoXYPlaneAndUpdate(referenceFrame, tempContactPoints);

      return tempFootPolygon;
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
      framePose.changeFrame(worldFrame);

      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("poseReferenceFrame", framePose);

      ReferenceFrame soleReferenceFrame = FootstepUtils.createSoleFrame(poseReferenceFrame, contactablePlaneBody);
      List<FramePoint> expectedContactPoints = FootstepUtils.getContactPointsInFrame(contactablePlaneBody, soleReferenceFrame);
      boolean trustHeight = true;

      Footstep footstep = new Footstep(contactablePlaneBody, poseReferenceFrame, soleReferenceFrame, expectedContactPoints, trustHeight);

      return footstep;
   }

   private void checkForReinitialization()
   {
      if (reinitializeControllerProvider == null)
         return;

      if (reinitializeControllerProvider.isReinitializeRequested() && (stateMachine.getCurrentStateEnum() == WalkingState.DOUBLE_SUPPORT))
      {
         reinitializeControllerProvider.set(false);
         initialize();
      }
   }

   public void integrateAnkleAccelerationsOnSwingLeg(RobotSide swingSide)
   {
      fullRobotModel.getLegJoint(swingSide, LegJointName.ANKLE_PITCH).setIntegrateDesiredAccelerations(true);
      fullRobotModel.getLegJoint(swingSide, LegJointName.ANKLE_ROLL).setIntegrateDesiredAccelerations(true);
      fullRobotModel.getLegJoint(swingSide.getOppositeSide(), LegJointName.ANKLE_PITCH).setIntegrateDesiredAccelerations(false);
      fullRobotModel.getLegJoint(swingSide.getOppositeSide(), LegJointName.ANKLE_ROLL).setIntegrateDesiredAccelerations(false);
   }

   private void doNotIntegrateAnkleAccelerations()
   {
      fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.ANKLE_PITCH).setIntegrateDesiredAccelerations(false);
      fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.ANKLE_ROLL).setIntegrateDesiredAccelerations(false);
      fullRobotModel.getLegJoint(RobotSide.RIGHT, LegJointName.ANKLE_PITCH).setIntegrateDesiredAccelerations(false);
      fullRobotModel.getLegJoint(RobotSide.RIGHT, LegJointName.ANKLE_ROLL).setIntegrateDesiredAccelerations(false);
   }

   private void resetLoadedLegIntegrators(RobotSide robotSide)
   {
      if (resetIntegratorsAfterSwing.getBooleanValue())
      {
         fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_PITCH).resetDesiredAccelerationIntegrator();
         fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_ROLL).resetDesiredAccelerationIntegrator();
         fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_YAW).resetDesiredAccelerationIntegrator();
         fullRobotModel.getLegJoint(robotSide, LegJointName.KNEE).resetDesiredAccelerationIntegrator();
         fullRobotModel.getLegJoint(robotSide, LegJointName.ANKLE_PITCH).resetDesiredAccelerationIntegrator();
         fullRobotModel.getLegJoint(robotSide, LegJointName.ANKLE_ROLL).resetDesiredAccelerationIntegrator();
      }
   }

   private final FrameVector2d stanceAnkleToOriginalICPVector = new FrameVector2d();

   private final FramePoint2d stanceToSwingPoint = new FramePoint2d();
   private final FrameVector2d stanceToSwingVector = new FrameVector2d();

   private void moveICPToInsideOfFootAtEndOfSwing(RobotSide supportSide, FramePoint2d upcomingFootstepLocation, double swingTime, double swingTimeRemaining,
         FramePoint2d desiredICPToMove)
   {
      swingTimeRemainingForICPMoveViz.set(swingTimeRemaining);

      ReferenceFrame supportAnkleFrame = referenceFrames.getAnkleZUpFrame(supportSide);
      desiredICPToMove.changeFrame(supportAnkleFrame);
      stanceAnkleToOriginalICPVector.setIncludingFrame(desiredICPToMove);

      stanceToSwingPoint.setIncludingFrame(upcomingFootstepLocation);
      stanceToSwingPoint.changeFrame(supportAnkleFrame);

      stanceToSwingVector.setIncludingFrame(stanceToSwingPoint);
      double stanceToSwingDistance = stanceToSwingVector.length();
      if (stanceToSwingDistance < 0.001)
      {
         desiredICPToMove.changeFrame(desiredICP.getReferenceFrame());

         return;
      }

      stanceToSwingVector.normalize();

      distanceFromLineToOriginalICP.set(stanceToSwingVector.dot(stanceAnkleToOriginalICPVector));
      double timeToUseBeforeShift = singleSupportTimeLeftBeforeShift.getDoubleValue();
      if (timeToUseBeforeShift < 0.01)
         timeToUseBeforeShift = 0.01;

      double deltaTime = swingTime - timeToUseBeforeShift;
      double percent;
      if (deltaTime <= 1e-7)
         percent = 1.0;
      else
      {
         percent = (swingTime - swingTimeRemaining) / (deltaTime);
      }

      percent = MathTools.clipToMinMax(percent, 0.0, 1.0);

      double maxDistanceToMove = moveICPAwayDuringSwingDistance.getDoubleValue();
      maxDistanceToMove = MathTools.clipToMinMax(maxDistanceToMove, 0.0, stanceToSwingDistance / 2.0);
      double duringSwingDistance = (percent * maxDistanceToMove);

      if (swingTimeRemaining > timeToUseBeforeShift)
      {
         amountToMoveICPAway.set(duringSwingDistance);
      }
      else
      {
         percent = (1.0 - swingTimeRemaining / timeToUseBeforeShift);
         percent = MathTools.clipToMinMax(percent, 0.0, 1.0);

         maxDistanceToMove = moveICPAwayAtEndOfSwingDistance.getDoubleValue();
         maxDistanceToMove = MathTools.clipToMinMax(maxDistanceToMove, 0.0, stanceToSwingDistance / 2.0);

         maxDistanceToMove -= moveICPAwayDuringSwingDistance.getDoubleValue();
         if (maxDistanceToMove < 0.0)
            maxDistanceToMove = 0.0;

         amountToMoveICPAway.set(duringSwingDistance + (percent * maxDistanceToMove));
      }

      if (distanceFromLineToOriginalICP.getDoubleValue() > amountToMoveICPAway.getDoubleValue())
      {
         desiredICPToMove.changeFrame(desiredICP.getReferenceFrame());

         return;
      }

      double additionalDistance = amountToMoveICPAway.getDoubleValue() - distanceFromLineToOriginalICP.getDoubleValue();

      stanceToSwingVector.scale(additionalDistance);
      desiredICPToMove.add(stanceToSwingVector);

      desiredICPToMove.changeFrame(desiredICP.getReferenceFrame());
   }

   private void limitICPToMiddleOfFootOrInside(RobotSide supportSide, FramePoint2d desiredICPLocal)
   {
      desiredICPLocal.changeFrame(referenceFrames.getAnkleZUpFrame(supportSide));
      double minimumInside = 0.005;

      if (supportSide.negateIfLeftSide(desiredICPLocal.getY()) < minimumInside)
      {
         desiredICPLocal.setY(supportSide.negateIfLeftSide(minimumInside));
      }

      desiredICPLocal.changeFrame(desiredICP.getReferenceFrame());
   }

}
