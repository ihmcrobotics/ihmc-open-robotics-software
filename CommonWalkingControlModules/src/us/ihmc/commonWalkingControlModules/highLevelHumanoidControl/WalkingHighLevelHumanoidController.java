package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;

import javax.media.j3d.Transform3D;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.calculators.EquivalentConstantCoPCalculator;
import us.ihmc.commonWalkingControlModules.calculators.GainCalculator;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.ChestOrientationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.DegenerateOrientationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.EndEffectorControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.GroundReactionWrenchDistributor;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.head.DesiredHeadOrientationProvider;
import us.ihmc.commonWalkingControlModules.controlModules.head.HeadOrientationControlModule;
import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorTools;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepProvider;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepUtils;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointTrajectoryData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.ICPBasedMomentumRateOfChangeControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.OrientationTrajectoryData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.RootJointAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.commonWalkingControlModules.outputs.ProcessedOutputsInterface;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LimbName;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
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
import us.ihmc.commonWalkingControlModules.trajectories.InstantaneousCapturePointTrajectory;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.SettableOrientationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.YoVariableDoubleProvider;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
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
import com.yobotics.simulationconstructionset.util.PDController;
import com.yobotics.simulationconstructionset.util.PIDController;
import com.yobotics.simulationconstructionset.util.graphics.BagOfBalls;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameOrientation;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePose;
import com.yobotics.simulationconstructionset.util.statemachines.State;
import com.yobotics.simulationconstructionset.util.statemachines.StateMachine;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransition;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransitionAction;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransitionCondition;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleProvider;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.PositionTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.YoPositionProvider;

public class WalkingHighLevelHumanoidController extends ICPAndMomentumBasedController
{
   private static final long serialVersionUID = 7436158470125024491L;

   private static enum WalkingState {LEFT_SUPPORT, RIGHT_SUPPORT, TRANSFER_TO_LEFT_SUPPORT, TRANSFER_TO_RIGHT_SUPPORT, DOUBLE_SUPPORT}

   private final static boolean DEBUG = false;
   private final StateMachine<WalkingState> stateMachine;
   private final CenterOfMassJacobian centerOfMassJacobian;

   private final CoMHeightTrajectoryGenerator centerOfMassHeightTrajectoryGenerator;
   private final CoMHeightTimeDerivativesCalculator coMHeightTimeDerivativesCalculator = new CoMHeightTimeDerivativesCalculator();
   private final CoMHeightTimeDerivativesSmoother coMHeightTimeDerivativesSmoother = new CoMHeightTimeDerivativesSmoother(controlDT, registry);

   private final PDController centerOfMassHeightController;
   private final SideDependentList<WalkingState> singleSupportStateEnums = new SideDependentList<WalkingState>(WalkingState.LEFT_SUPPORT,
                                                                              WalkingState.RIGHT_SUPPORT);

   private final SideDependentList<WalkingState> transferStateEnums = new SideDependentList<WalkingState>(WalkingState.TRANSFER_TO_LEFT_SUPPORT,
                                                                         WalkingState.TRANSFER_TO_RIGHT_SUPPORT);

   private final YoVariableDoubleProvider transferTimeProvider = new YoVariableDoubleProvider("transferTime", registry);
   private final DoubleYoVariable doubleSupportTime = new DoubleYoVariable("doubleSupportTime", registry);

   private final FinalDesiredICPCalculator finalDesiredICPCalculator;

   protected final SideDependentList<FootSwitchInterface> footSwitches;

   // FIXME: reimplement and improve com trajectory visualization
   private final BagOfBalls comTrajectoryBagOfBalls;
   private int comTrajectoryCounter = 0;

   // private final BooleanYoVariable transferICPTrajectoryDone = new BooleanYoVariable("transferICPTrajectoryDone", registry);
   private final DoubleYoVariable minOrbitalEnergyForSingleSupport = new DoubleYoVariable("minOrbitalEnergyForSingleSupport", registry);
   private final DoubleYoVariable amountToBeInsideSingleSupport = new DoubleYoVariable("amountToBeInsideSingleSupport", registry);
   private final DoubleYoVariable amountToBeInsideDoubleSupport = new DoubleYoVariable("amountToBeInsideDoubleSupport", registry);

   private final DegenerateOrientationControlModule chestOrientationControlModule;

   private final DoubleYoVariable userDesiredPelvisPitch = new DoubleYoVariable("userDesiredPelvisPitch", registry);
   private final SettableOrientationProvider initialPelvisOrientationProvider;
   private final SettableOrientationProvider finalPelvisOrientationProvider;
   private final OrientationTrajectoryGenerator pelvisOrientationTrajectoryGenerator;

   private final HashMap<ContactablePlaneBody, EndEffectorControlModule> endEffectorControlModules = new HashMap<ContactablePlaneBody,
                                                                                                        EndEffectorControlModule>();
   private final SideDependentList<RigidBodySpatialAccelerationControlModule> handControlModules =
      new SideDependentList<RigidBodySpatialAccelerationControlModule>();

   private final SideDependentList<PositionTrajectoryGenerator> footPositionTrajectoryGenerators;
   private final DoubleProvider swingTimeProvider;
   private final SideDependentList<YoVariableDoubleProvider> onEdgeInitialAngleProviders = new SideDependentList<YoVariableDoubleProvider>();
   private final SideDependentList<YoVariableDoubleProvider> onEdgeFinalAngleProviders = new SideDependentList<YoVariableDoubleProvider>();
   private final BooleanYoVariable stayOnToes = new BooleanYoVariable("stayOnToes", registry);
   private final DoubleYoVariable trailingFootPitch = new DoubleYoVariable("trailingFootPitch", registry);
   private final SideDependentList<OneDoFJoint[]> armJoints = new SideDependentList<OneDoFJoint[]>();
   private final SideDependentList<OneDoFJoint[]> handJoints = new SideDependentList<OneDoFJoint[]>();

   private final DoubleYoVariable kUpperBody = new DoubleYoVariable("kUpperBody", registry);
   private final DoubleYoVariable zetaUpperBody = new DoubleYoVariable("zetaUpperBody", registry);
   private final YoPositionProvider finalPositionProvider;

   private final DoubleYoVariable swingAboveSupportAnkle = new DoubleYoVariable("swingAboveSupportAnkle", registry);

   private Footstep nextFootstep = null;
   private final YoFramePose nextFootstepPose = new YoFramePose("nextFootstep", "", worldFrame, registry);
   private Footstep nextNextFootstep = null;

   private final FootstepProvider footstepProvider;
   private final InstantaneousCapturePointTrajectory icpTrajectoryGenerator;
   private final SideDependentList<SettableOrientationProvider> finalFootOrientationProviders = new SideDependentList<SettableOrientationProvider>();

   private final SideDependentList<FramePose> desiredHandPoses = new SideDependentList<FramePose>();

   private final OneDoFJoint[] positionControlJoints;

   private final OneDoFJoint jointForExtendedNeckPitchRange;
   private final HeadOrientationControlModule headOrientationControlModule;
   private final DesiredHeadOrientationProvider desiredHeadOrientationProvider;

   private final boolean checkOrbitalCondition;
   private final SideDependentList<EnumMap<LimbName, GeometricJacobian>> jacobians = SideDependentList.createListOfEnumMaps(LimbName.class);
   private final ControlFlowInputPort<OrientationTrajectoryData> desiredPelvisOrientationTrajectoryInputPort;
   private final ICPBasedMomentumRateOfChangeControlModule icpBasedMomentumRateOfChangeControlModule;
   private final YoFrameOrientation desiredPelvisOrientation = new YoFrameOrientation("desiredPelvis", worldFrame, registry);

   private final DoubleYoVariable coefficientOfFriction = new DoubleYoVariable("coefficientOfFriction", registry);
   private final OneDoFJoint lidarJoint;
   private final PIDController lidarJointVelocityController = new PIDController("lidar", registry);
   private final DoubleYoVariable desiredLidarVelocity = new DoubleYoVariable("desiredLidarVelocity", registry);
   private final BooleanYoVariable toeOff = new BooleanYoVariable("toeOff", registry);
   private final boolean resetDesiredICPToCurrentAtStartOfSwing;
   private final BooleanYoVariable icpTrajectoryHasBeenInitialized;
   private final BooleanYoVariable doToeOffIfPossible = new BooleanYoVariable("doToeOffIfPossible", registry);

   public WalkingHighLevelHumanoidController(FullRobotModel fullRobotModel, CommonWalkingReferenceFrames referenceFrames, TwistCalculator twistCalculator,
           CenterOfMassJacobian centerOfMassJacobian, SideDependentList<? extends ContactablePlaneBody> bipedFeet, BipedSupportPolygons bipedSupportPolygons,
           SideDependentList<FootSwitchInterface> footSwitches, double gravityZ, DoubleYoVariable yoTime, double controlDT,
           DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, FootstepProvider footstepProvider,
           DesiredHeadOrientationProvider desiredHeadOrientationProvider, CoMHeightTrajectoryGenerator centerOfMassHeightTrajectoryGenerator,
           GroundReactionWrenchDistributor groundReactionWrenchDistributor, SideDependentList<PositionTrajectoryGenerator> footPositionTrajectoryGenerators,
           DoubleProvider swingTimeProvider, YoPositionProvider finalPositionProvider, boolean stayOntoes, double desiredPelvisPitch, double trailingFootPitch,
           ArrayList<Updatable> updatables, ProcessedOutputsInterface processedOutputs, WalkingControllerParameters walkingControllerParameters,
           ICPBasedMomentumRateOfChangeControlModule momentumRateOfChangeControlModule, RootJointAccelerationControlModule rootJointAccelerationControlModule,
           ControlFlowInputPort<OrientationTrajectoryData> desiredPelvisOrientationTrajectoryInputPort, OneDoFJoint lidarJoint,
           FinalDesiredICPCalculator finalDesiredICPCalculator)
   {
      super(fullRobotModel, centerOfMassJacobian, referenceFrames, yoTime, gravityZ, twistCalculator, bipedFeet, bipedSupportPolygons, controlDT,
            processedOutputs, groundReactionWrenchDistributor, updatables, momentumRateOfChangeControlModule, rootJointAccelerationControlModule,
            walkingControllerParameters.getGroundReactionWrenchBreakFrequencyHertz(), dynamicGraphicObjectsListRegistry);

      this.finalDesiredICPCalculator = finalDesiredICPCalculator;
      this.icpBasedMomentumRateOfChangeControlModule = momentumRateOfChangeControlModule;
      this.desiredPelvisOrientationTrajectoryInputPort = desiredPelvisOrientationTrajectoryInputPort;
      this.centerOfMassJacobian = centerOfMassJacobian;
      this.centerOfMassHeightTrajectoryGenerator = centerOfMassHeightTrajectoryGenerator;
      this.swingTimeProvider = swingTimeProvider;
      this.footstepProvider = footstepProvider;
      this.footSwitches = footSwitches;

      this.centerOfMassHeightController = new PDController("comHeight", registry);
      centerOfMassHeightController.setProportionalGain(40.0);
      double zeta = 1.0;
      centerOfMassHeightController.setDerivativeGain(GainCalculator.computeDerivativeGain(centerOfMassHeightController.getProportionalGain(), zeta));

      String namePrefix = "walking";
      icpTrajectoryGenerator = new ConstantCoPInstantaneousCapturePointTrajectory(namePrefix, bipedSupportPolygons, controlDT, registry);

      this.stateMachine = new StateMachine<WalkingState>(namePrefix + "State", namePrefix + "SwitchTime", WalkingState.class, yoTime, registry);    // this is used by name, and it is ugly.

      this.finalPositionProvider = finalPositionProvider;

      this.footPositionTrajectoryGenerators = footPositionTrajectoryGenerators;
      this.icpTrajectoryHasBeenInitialized = new BooleanYoVariable("icpTrajectoryHasBeenInitialized", registry);


      coefficientOfFriction.set(0.6);    // TODO: Make DRCFlatGroundWalkingTest work with this at 0.7
      
      setupLimbJacobians(fullRobotModel);

      for (RobotSide robotSide : RobotSide.values())
      {
         ContactablePlaneBody bipedFoot = bipedFeet.get(robotSide);
         contactStates.get(bipedFoot).set(bipedFoot.getContactPoints2d(), coefficientOfFriction.getDoubleValue());    // flat feet
         String sideString = robotSide.getCamelCaseNameForStartOfExpression();

         PositionTrajectoryGenerator swingPositionTrajectoryGenerator = footPositionTrajectoryGenerators.get(robotSide);

         OrientationProvider initialOrientationProvider = new CurrentOrientationProvider(worldFrame, bipedFoot.getBodyFrame());
         SettableOrientationProvider finalFootOrientationProvider = new SettableOrientationProvider(sideString + "FinalFootOrientation", worldFrame, registry);
         finalFootOrientationProviders.put(robotSide, finalFootOrientationProvider);

         OrientationTrajectoryGenerator swingOrientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator(sideString
                                                                                 + "SwingFootOrientation", worldFrame, swingTimeProvider,
                                                                                    initialOrientationProvider, finalFootOrientationProvider, registry);

         YoVariableDoubleProvider onEdgeInitialPitchProvider = new YoVariableDoubleProvider(sideString + "OnToesInitialPitch", registry);
         YoVariableDoubleProvider onEdgeFinalPitchProvider = new YoVariableDoubleProvider(sideString + "OnToesFinalPitch", registry);
         DoubleProvider onToesTrajectoryTimeProvider = transferTimeProvider;
         DoubleTrajectoryGenerator onToesPitchTrajectoryGenerator = new CubicPolynomialTrajectoryGenerator(sideString + "OnToesPitch",
                                                                       onEdgeInitialPitchProvider, onEdgeFinalPitchProvider, onToesTrajectoryTimeProvider,
                                                                       registry);

         onEdgeInitialAngleProviders.put(robotSide, onEdgeInitialPitchProvider);
         onEdgeFinalAngleProviders.put(robotSide, onEdgeFinalPitchProvider);

         GeometricJacobian jacobian = jacobians.get(robotSide).get(LimbName.LEG);
         EndEffectorControlModule endEffectorControlModule = new EndEffectorControlModule(bipedFoot, jacobian, swingPositionTrajectoryGenerator,
                                                                swingOrientationTrajectoryGenerator, onToesPitchTrajectoryGenerator, yoTime, twistCalculator,
                                                                registry);
         endEffectorControlModule.setParameters(3e-2, 500.0);
         endEffectorControlModules.put(bipedFoot, endEffectorControlModule);

      }

      initialPelvisOrientationProvider = new SettableOrientationProvider("initialPelvis", worldFrame, registry);
      finalPelvisOrientationProvider = new SettableOrientationProvider("finalPelvis", worldFrame, registry);
      this.pelvisOrientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator("pelvis", worldFrame, swingTimeProvider,
              initialPelvisOrientationProvider, finalPelvisOrientationProvider, registry);

      comTrajectoryBagOfBalls = new BagOfBalls(500, 0.01, "comBagOfBalls", YoAppearance.Red(), registry, dynamicGraphicObjectsListRegistry);

      setUpStateMachine(swingTimeProvider);

      minOrbitalEnergyForSingleSupport.set(0.007);    // 0.008
      amountToBeInsideSingleSupport.set(0.0);
      amountToBeInsideDoubleSupport.set(0.03);    // 0.02);    // TODO: necessary for stairs...
      transferTimeProvider.set(0.2);    // 0.5);    // 0.2);    // 0.6;    // 0.3
      doubleSupportTime.set(0.5);
      this.userDesiredPelvisPitch.set(desiredPelvisPitch);
      this.stayOnToes.set(stayOntoes);
      this.trailingFootPitch.set(trailingFootPitch);
      kUpperBody.set(100.0);
      zetaUpperBody.set(1.0);
      lidarJointVelocityController.setIntegralGain(0.01);
      lidarJointVelocityController.setProportionalGain(0.01);    // proportional gain corresponds to velocity, derivative corresponds to acceleration
      desiredLidarVelocity.set(10.0);
      this.resetDesiredICPToCurrentAtStartOfSwing = walkingControllerParameters.resetDesiredICPToCurrentAtStartOfSwing();


      double initialLeadingFootPitch = 0.05;
      upcomingSupportLeg.set(RobotSide.RIGHT);    // TODO: stairs hack, so that the following lines use the correct leading leg
      RobotSide leadingLeg = getUpcomingSupportLeg();
      onEdgeInitialAngleProviders.get(leadingLeg).set(initialLeadingFootPitch);
      onEdgeFinalAngleProviders.get(leadingLeg).set(initialLeadingFootPitch);

      RobotSide trailingLeg = leadingLeg.getOppositeSide();
      onEdgeInitialAngleProviders.get(trailingLeg).set(this.trailingFootPitch.getDoubleValue());

      this.checkOrbitalCondition = walkingControllerParameters.checkOrbitalCondition();

      for (RobotSide robotSide : RobotSide.values())
      {
         final RigidBody hand = fullRobotModel.getHand(robotSide);
         armJoints.put(robotSide, createOneDoFJointPath(fullRobotModel.getChest(), hand));

         final RigidBodySpatialAccelerationControlModule rigidBodySpatialAccelerationControlModule =
            new RigidBodySpatialAccelerationControlModule(robotSide.getCamelCaseNameForStartOfExpression() + "HandControlModule", twistCalculator, hand,
               hand.getParentJoint().getFrameAfterJoint(), registry);
         rigidBodySpatialAccelerationControlModule.setPositionProportionalGains(100.0, 100.0, 100.0);
         rigidBodySpatialAccelerationControlModule.setPositionDerivativeGains(20.0, 20.0, 20.0);
         rigidBodySpatialAccelerationControlModule.setOrientationProportionalGains(100.0, 100.0, 100.0);
         rigidBodySpatialAccelerationControlModule.setOrientationDerivativeGains(20.0, 20.0, 20.0);
         handControlModules.put(robotSide, rigidBodySpatialAccelerationControlModule);

         final ReferenceFrame chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
         FramePose desiredHandPosition = new FramePose(chestFrame, walkingControllerParameters.getDesiredHandPosesWithRespectToChestFrame().get(robotSide));

         this.desiredHandPoses.put(robotSide, desiredHandPosition);

         HashSet<OneDoFJoint> allSideHandJoints = new HashSet<OneDoFJoint>();

         for (InverseDynamicsJoint handChildJoint : hand.getChildrenJoints())
         {
            allSideHandJoints.add((OneDoFJoint) handChildJoint);
            createHandJoints(robotSide, handChildJoint, allSideHandJoints);
         }

         handJoints.put(robotSide, allSideHandJoints.toArray(new OneDoFJoint[allSideHandJoints.size()]));
      }


      this.desiredHeadOrientationProvider = desiredHeadOrientationProvider;

      String[] headOrientationControlJointNames = walkingControllerParameters.getHeadOrientationControlJointNames();
      String[] chestOrientationControlJointNames = walkingControllerParameters.getChestOrientationControlJointNames();

      RigidBody elevator = fullRobotModel.getElevator();
      InverseDynamicsJoint[] allJoints = ScrewTools.computeJointsInOrder(elevator);
      InverseDynamicsJoint[] headOrientationControlJoints = ScrewTools.findJointsWithNames(allJoints, headOrientationControlJointNames);
      InverseDynamicsJoint[] chestOrientationControlJoints = ScrewTools.findJointsWithNames(allJoints, chestOrientationControlJointNames);

      RigidBody pelvis = fullRobotModel.getPelvis();
      if (headOrientationControlJoints.length > 0)
      {
         final RigidBody head = fullRobotModel.getHead();
         GeometricJacobian neckJacobian = new GeometricJacobian(headOrientationControlJoints, head.getBodyFixedFrame());
         ReferenceFrame pelvisFrame = pelvis.getBodyFixedFrame();
         ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
         ReferenceFrame[] availableHeadOrientationControlFrames = new ReferenceFrame[] {pelvisZUpFrame, pelvisFrame, ReferenceFrame.getWorldFrame()};
         headOrientationControlModule = new HeadOrientationControlModule(neckJacobian, pelvis, elevator, twistCalculator,
                 availableHeadOrientationControlFrames, walkingControllerParameters, registry, dynamicGraphicObjectsListRegistry);
         headOrientationControlModule.setOrientationToTrack(new FrameOrientation(pelvisZUpFrame), pelvis);
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
      }

      this.lidarJoint = lidarJoint;

      if (lidarJoint != null)
      {
         unconstrainedJoints.remove(lidarJoint);
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

   private void createHandJoints(RobotSide side, InverseDynamicsJoint parentJoint, HashSet<OneDoFJoint> allSideHandJoints)
   {
      for (InverseDynamicsJoint childJoint : parentJoint.getSuccessor().getChildrenJoints())
      {
         allSideHandJoints.add((OneDoFJoint) childJoint);
         createHandJoints(side, childJoint, allSideHandJoints);
      }
   }

   private OneDoFJoint[] createOneDoFJointPath(RigidBody base, RigidBody endEffector)
   {
      InverseDynamicsJoint[] joints = ScrewTools.createJointPath(base, endEffector);
      OneDoFJoint[] oneDoFJoints = new OneDoFJoint[ScrewTools.computeNumberOfJointsOfType(OneDoFJoint.class, joints)];
      ScrewTools.filterJoints(joints, oneDoFJoints, OneDoFJoint.class);

      return oneDoFJoints;
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

   private void setUpStateMachine(DoubleProvider stepTimeProvider)
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
      super.initialize();
      FrameOrientation initialDesiredPelvisOrientation = new FrameOrientation(referenceFrames.getAnkleZUpFrame(getUpcomingSupportLeg()));
      initialDesiredPelvisOrientation.changeFrame(worldFrame);
      double yaw = initialDesiredPelvisOrientation.getYawPitchRoll()[0];
      initialDesiredPelvisOrientation.setYawPitchRoll(yaw, userDesiredPelvisPitch.getDoubleValue(), 0.0);
      desiredPelvisOrientation.set(initialDesiredPelvisOrientation);
      finalPelvisOrientationProvider.setOrientation(initialDesiredPelvisOrientation);    // yes, final. To make sure that the first swing phase has the right initial

      computeCapturePoint();
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
         // note: this has to be done before the ICP trajectory generator is initialized, since it is using nextFootstep
         checkForFootsteps();

         if (icpTrajectoryGenerator.isDone())
         {
            if (!icpTrajectoryHasBeenInitialized.getBooleanValue())
            {
               FramePoint2d finalDesiredICP;
               double trajectoryTime;
               if (transferToSide == null)
               {
                  finalDesiredICP = getDoubleSupportFinalDesiredICPForDoubleSupportStance();
                  trajectoryTime = doubleSupportTime.getDoubleValue();
               }
               else
               {
                  FramePose currentFramePose = new FramePose(referenceFrames.getFootFrame(transferToSide));
                  currentFramePose.changeFrame(worldFrame);

                  FrameConvexPolygon2d footPolygon = computeFootPolygon(transferToSide, referenceFrames.getSoleFrame(transferToSide));
                  PoseReferenceFrame currentPoseReferenceFrame = new PoseReferenceFrame("currentPose", currentFramePose);

                  ContactablePlaneBody contactablePlaneBody = bipedFeet.get(transferToSide);
                  ReferenceFrame soleReferenceFrame = FootstepUtils.createSoleFrame(currentPoseReferenceFrame, contactablePlaneBody);
                  List<FramePoint> expectedContactPoints = FootstepUtils.getContactPointsInFrame(contactablePlaneBody, soleReferenceFrame);
                  boolean trustHeight = true;

                  Footstep transferToFootstep = new Footstep(contactablePlaneBody, currentPoseReferenceFrame, soleReferenceFrame, expectedContactPoints,
                                                   trustHeight);


                  TransferToAndNextFootstepsData transferToAndNextFootstepsData = new TransferToAndNextFootstepsData();
                  transferToAndNextFootstepsData.setTransferToFootstep(transferToFootstep);
                  transferToAndNextFootstepsData.setTransferToFootPolygonInSoleFrame(footPolygon);
                  transferToAndNextFootstepsData.setTransferToSide(transferToSide);
                  transferToAndNextFootstepsData.setNextFootstep(nextFootstep);
                  transferToAndNextFootstepsData.setNextNextFootstep(nextNextFootstep);
                  transferToAndNextFootstepsData.setEstimatedStepTime(swingTimeProvider.getValue() + transferTimeProvider.getValue());
                  transferToAndNextFootstepsData.setW0(getOmega0());

                  finalDesiredICP = finalDesiredICPCalculator.getFinalDesiredICPForWalking(transferToAndNextFootstepsData);
                  trajectoryTime = transferTimeProvider.getValue();
               }

               finalDesiredICP.changeFrame(desiredICP.getReferenceFrame());

               if (!stayOnToes.getBooleanValue() && (transferToSide != null))    // the only case left for determining the contact state of the trailing foot
               {
                  RobotSide trailingLeg = transferToSide.getOppositeSide();
                  ContactablePlaneBody supportFoot = bipedFeet.get(trailingLeg);
                  List<FramePoint> toePoints = getToePoints(supportFoot);
                  Collection<FramePoint2d> points = new ArrayList<FramePoint2d>();
                  for (FramePoint toePoint : toePoints)
                  {
                     points.add(toePoint.toFramePoint2d());
                  }

                  points.add(finalDesiredICP);
                  FrameConvexPolygon2d onToesTriangle = new FrameConvexPolygon2d(points);
                  boolean desiredICPOK = onToesTriangle.isPointInside(desiredICP.getFramePoint2dCopy());
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

               updateBipedSupportPolygons(bipedSupportPolygons);    // need to always update biped support polygons after a change to the contact states

               icpTrajectoryGenerator.initialize(desiredICP.getFramePoint2dCopy(), finalDesiredICP, trajectoryTime, getOmega0(),
                                                 amountToBeInsideDoubleSupport.getDoubleValue(), getSupportLeg());

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
            icpTrajectoryGenerator.pack(desiredICPLocal, desiredICPVelocityLocal, getOmega0());
            desiredICP.set(desiredICPLocal);
            desiredICPVelocity.set(desiredICPVelocityLocal);
         }

         doFootcontrol();
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
            onEdgeFinalAngleProviders.get(trailingLeg).set(trailingFootPitch.getDoubleValue());
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
            else
            {
               setFlatFootContactState(bipedFeet.get(transferToSide));

               // still need to determine contact state for trailing leg. This is done in doAction as soon as the previous ICP trajectory is done
            }
         }

         updateBipedSupportPolygons(bipedSupportPolygons);    // need to always update biped support polygons after a change to the contact states

         centerOfMassHeightTrajectoryGenerator.initialize(getSupportLeg(), null, getContactStatesList());
      }

      @Override
      public void doTransitionOutOfAction()
      {
         if (DEBUG)
            System.out.println("WalkingHighLevelHumanoidController: leavingDoubleSupportState");
         desiredICPVelocity.set(0.0, 0.0);
      }

      private void checkForFootsteps()
      {
         if (nextFootstep == null)
         {
            nextFootstep = footstepProvider.poll();

            if (nextFootstep != null)
            {
               upcomingSupportLeg.set(getRobotSide(nextFootstep.getBody(), bipedFeet).getOppositeSide());
               nextFootstepPose.set(nextFootstep.getPoseCopy());
            }
         }
         else if (nextNextFootstep == null)
         {
            nextNextFootstep = footstepProvider.peek();
         }
      }
   }


   private class SingleSupportState extends State<WalkingState>
   {
      private final RobotSide swingSide;
      private final FrameOrientation desiredPelvisOrientationToPack;

      public SingleSupportState(RobotSide robotSide)
      {
         super(singleSupportStateEnums.get(robotSide));
         this.swingSide = robotSide.getOppositeSide();
         this.desiredPelvisOrientationToPack = new FrameOrientation(worldFrame);
      }

      @Override
      public void doAction()
      {
         if (!icpTrajectoryGenerator.isDone())
         {
            FramePoint2d desiredICPLocal = new FramePoint2d(desiredICP.getReferenceFrame());
            FrameVector2d desiredICPVelocityLocal = new FrameVector2d(desiredICPVelocity.getReferenceFrame());
            icpTrajectoryGenerator.pack(desiredICPLocal, desiredICPVelocityLocal, getOmega0());
            desiredICP.set(desiredICPLocal);
            desiredICPVelocity.set(desiredICPVelocityLocal);
         }
         else
         {
            // don't change desiredICP
            desiredICPVelocity.set(0.0, 0.0);
         }

         pelvisOrientationTrajectoryGenerator.compute(stateMachine.timeInCurrentState());
         pelvisOrientationTrajectoryGenerator.get(desiredPelvisOrientationToPack);
         desiredPelvisOrientation.set(desiredPelvisOrientationToPack);

         doFootcontrol();
      }

      @Override
      public void doTransitionIntoAction()
      {
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
         }

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


         initializeTrajectory(swingSide, null);
         if (DEBUG)
            System.out.println("WalkingHighLevelHumanoidController: nextFootstep will change now!");
         nextFootstep = null;
         nextNextFootstep = null;
      }

      @Override
      public void doTransitionOutOfAction()
      {
         if (DEBUG)
            System.out.println("WalkingHighLevelController: leavingDoubleSupportState");

         footstepProvider.notifyComplete();

//       ContactableBody swingFoot = contactablePlaneBodies.get(swingSide);
//       Footstep desiredFootstep = desiredFootstepCalculator.updateAndGetDesiredFootstep(swingSide.getOppositeSide());
//       contactStates.get(swingFoot).setContactPoints(desiredFootstep.getExpectedContactPoints());
//       updateFootStateMachines(swingFoot);
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
         if (nextFootstep == null)
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
         if (checkOrbitalCondition)
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
      private EndEffectorControlModule endEffectorControlModule;

      public DoneWithSingleSupportCondition(EndEffectorControlModule endEffectorControlModule)
      {
         this.endEffectorControlModule = endEffectorControlModule;
      }

      public boolean checkCondition()
      {
         RobotSide swingSide = getSupportLeg().getOppositeSide();
         double minimumSwingFraction = 0.5;
         double minimumSwingTime = swingTimeProvider.getValue() * minimumSwingFraction;
         boolean footHitGround = (stateMachine.timeInCurrentState() > minimumSwingTime) && footSwitches.get(swingSide).hasFootHitGround();

         // transferring out of single support once the ICP trajectory is done guarantees that we never reach the zero velocity desired
         boolean trajectoryDone = icpTrajectoryGenerator.isDone(); // endEffectorControlModule.isTrajectoryDone();

         return trajectoryDone || footHitGround;
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
         boolean readyToStopWalking = (footstepProvider.isEmpty() && (nextFootstep == null)) && ((getSupportLeg() == null) || super.checkCondition());

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


   private FramePoint2d getSingleSupportFinalDesiredICPForWalking(Footstep desiredFootstep, Footstep footstepAfterThisOne, RobotSide swingSide)
   {
      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
      FramePoint2d initialDesiredICP = desiredICP.getFramePoint2dCopy();
      initialDesiredICP.changeFrame(referenceFrame);

      FrameConvexPolygon2d footPolygon;
      ContactablePlaneBody contactableBody = bipedFeet.get(swingSide);
      if (stayOnToes.getBooleanValue())
      {
         List<FramePoint> contactPoints = getContactPointsForWalkingOnToes(contactableBody);
         contactPoints = DesiredFootstepCalculatorTools.fixTwoPointsAndCopy(contactPoints);    // TODO: terrible
         footPolygon = FrameConvexPolygon2d.constructByProjectionOntoXYPlane(contactPoints, referenceFrames.getSoleFrame(swingSide));
      }
      else
      {
         footPolygon = new FrameConvexPolygon2d(contactableBody.getContactPoints2d());
      }

      TransferToAndNextFootstepsData transferToAndNextFootstepsData = new TransferToAndNextFootstepsData();

      transferToAndNextFootstepsData.setTransferToFootstep(desiredFootstep);

      transferToAndNextFootstepsData.setTransferToFootPolygonInSoleFrame(footPolygon);
      transferToAndNextFootstepsData.setTransferToSide(swingSide);
      transferToAndNextFootstepsData.setNextFootstep(footstepAfterThisOne);
      transferToAndNextFootstepsData.setNextNextFootstep(null);
      transferToAndNextFootstepsData.setEstimatedStepTime(swingTimeProvider.getValue() + transferTimeProvider.getValue());
      transferToAndNextFootstepsData.setW0(getOmega0());

      FramePoint2d finalDesiredICP = finalDesiredICPCalculator.getFinalDesiredICPForWalking(transferToAndNextFootstepsData);

      finalDesiredICP.changeFrame(referenceFrame);

      ContactablePlaneBody supportFoot = bipedFeet.get(swingSide.getOppositeSide());

      FramePoint2d icpWayPoint;
      if (doToeOffIfPossible.getBooleanValue())
      {
         FramePoint2d desiredToeOffCoP = new FramePoint2d(worldFrame);
         FramePoint toeOffPoint = new FramePoint(worldFrame);
         List<FramePoint> toePoints = getToePoints(supportFoot);
         toeOffPoint.interpolate(toePoints.get(0), toePoints.get(1), 0.5);
         FramePoint2d toeOffPoint2d = toeOffPoint.toFramePoint2d();
         double toeOffPointToFinalDesiredFactor = 0.2;
         desiredToeOffCoP.interpolate(toeOffPoint2d, finalDesiredICP, toeOffPointToFinalDesiredFactor);
         icpWayPoint = EquivalentConstantCoPCalculator.computeICPMotionWithConstantCMP(finalDesiredICP, desiredToeOffCoP, -transferTimeProvider.getValue(),
                 getOmega0());
      }
      else
      {
         double glideFactor = 0.9;
         icpWayPoint = new FramePoint2d(worldFrame);
         icpWayPoint.interpolate(desiredICP.getFramePoint2dCopy(), finalDesiredICP, glideFactor);
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

   public void initializeTrajectory(RobotSide swingSide, SpatialAccelerationVector taskSpaceAcceleration)
   {
      RobotSide supportSide = swingSide.getOppositeSide();
      ReferenceFrame trajectoryGeneratorFrame = ReferenceFrame.getWorldFrame();

//    ReferenceFrame swingAnkleZUpFrame = referenceFrames.getAnkleZUpFrame(swingSide);
//    ReferenceFrame swingFootFrame = referenceFrames.getFootFrame(swingSide);
//    FramePoint initialPosition = new FramePoint(swingAnkleZUpFrame);
//    initialPosition.changeFrame(trajectoryGeneratorFrame);
//
//    Twist footTwist = new Twist();
//    twistCalculator.packTwistOfBody(footTwist, fullRobotModel.getFoot(swingSide));

//    SpatialAccelerationVector taskSpaceAccelerationWithRespectToWorld = new SpatialAccelerationVector();
//    spatialAccelerationCalculator.compute();
//    spatialAccelerationCalculator.packAccelerationOfBody(taskSpaceAccelerationWithRespectToWorld, fullRobotModel.getPelvis());
//
//    Twist pelvisTwist = new Twist();
//    twistCalculator.packTwistOfBody(pelvisTwist, fullRobotModel.getPelvis());
//    taskSpaceAccelerationWithRespectToWorld.changeFrame(taskSpaceAccelerationWithRespectToWorld.getBaseFrame(), pelvisTwist, pelvisTwist);
//
//    Twist footPelvisTwist = new Twist();
//    twistCalculator.packRelativeTwist(footPelvisTwist, fullRobotModel.getPelvis(), fullRobotModel.getFoot(swingSide));
//
//    taskSpaceAcceleration.changeFrame(taskSpaceAccelerationWithRespectToWorld.getExpressedInFrame(), footTwist, footPelvisTwist);
//    taskSpaceAccelerationWithRespectToWorld.add(taskSpaceAcceleration);
//    FramePoint swingAnkle = new FramePoint(swingFootFrame);
//    swingAnkle.changeFrame(taskSpaceAccelerationWithRespectToWorld.getBaseFrame());
//    footTwist.changeFrame(taskSpaceAccelerationWithRespectToWorld.getExpressedInFrame());

//      footTwist.changeFrame(swingFootFrame);
//      FrameVector initialVelocity = new FrameVector(trajectoryGeneratorFrame);
//      footTwist.packLinearPart(initialVelocity);
//      initialVelocity.changeFrame(trajectoryGeneratorFrame);

//    footTwist.changeFrame(taskSpaceAccelerationWithRespectToWorld.getExpressedInFrame());
//      FrameVector initialAcceleration = new FrameVector(worldFrame);

//    taskSpaceAccelerationWithRespectToWorld.packAccelerationOfPointFixedInBodyFrame(footTwist, swingAnkle, initialAcceleration);
//    initialAcceleration.changeFrame(trajectoryGeneratorFrame);

//      initialAcceleration.setToZero(trajectoryGeneratorFrame);    // TODO

      finalPositionProvider.set(nextFootstep.getPositionInFrame(worldFrame));
      finalFootOrientationProviders.get(swingSide).setOrientation(nextFootstep.getOrientationInFrame(worldFrame));

      FrameOrientation initialPelvisOrientation = new FrameOrientation(worldFrame);
      finalPelvisOrientationProvider.get(initialPelvisOrientation);
      initialPelvisOrientation.changeFrame(worldFrame);
      initialPelvisOrientationProvider.setOrientation(initialPelvisOrientation);

      FrameOrientation finalPelvisOrientation = nextFootstep.getOrientationInFrame(worldFrame);
      finalPelvisOrientation.setYawPitchRoll(finalPelvisOrientation.getYawPitchRoll()[0], userDesiredPelvisPitch.getDoubleValue(), 0.0);
      finalPelvisOrientationProvider.setOrientation(finalPelvisOrientation);
      pelvisOrientationTrajectoryGenerator.initialize();

      FramePoint finalDesiredStepLocation = nextFootstep.getPositionInFrame(trajectoryGeneratorFrame);
      finalDesiredStepLocation.setZ(finalDesiredStepLocation.getZ());

      double stepPitch = nextFootstep.getOrientationInFrame(worldFrame).getYawPitchRoll()[1];
      onEdgeInitialAngleProviders.get(swingSide).set(stepPitch);
      onEdgeFinalAngleProviders.get(swingSide).set(stepPitch);

      FramePoint centerOfMass = new FramePoint(referenceFrames.getCenterOfMassFrame());
      centerOfMass.changeFrame(worldFrame);
      ContactablePlaneBody supportFoot = bipedFeet.get(supportSide);
      Transform3D footToWorldTransform = supportFoot.getBodyFrame().getTransformToDesiredFrame(worldFrame);
      double footHeight = DesiredFootstepCalculatorTools.computeMinZPointInFrame(footToWorldTransform, supportFoot, worldFrame).getZ();
      double comHeight = centerOfMass.getZ() - footHeight;
      double omega0 = CapturePointCalculator.computeOmega0ConstantHeight(gravity, comHeight);
      this.omega0.set(omega0);
      computeCapturePoint();

      if (resetDesiredICPToCurrentAtStartOfSwing)
      {
         desiredICP.set(capturePoint.getFramePoint2dCopy());    // TODO: currently necessary for stairs because of the omega0 jump, but should get rid of this
      }

      FramePoint2d finalDesiredICP = getSingleSupportFinalDesiredICPForWalking(nextFootstep, nextNextFootstep, swingSide);

      ContactablePlaneBody swingFoot = bipedFeet.get(swingSide);
      setContactStateForSwing(swingFoot);
      setSupportLeg(supportSide);
      updateBipedSupportPolygons(bipedSupportPolygons);

      icpTrajectoryGenerator.initialize(desiredICP.getFramePoint2dCopy(), finalDesiredICP, swingTimeProvider.getValue(), omega0,
                                        amountToBeInsideSingleSupport.getDoubleValue(), getSupportLeg());

      centerOfMassHeightTrajectoryGenerator.initialize(getSupportLeg(), nextFootstep, getContactStatesList());
   }

   public void doMotionControl()
   {
      for (ContactablePlaneBody contactablePlaneBody : endEffectorControlModules.keySet())
      {
         EndEffectorControlModule endEffectorControlModule = endEffectorControlModules.get(contactablePlaneBody);
         FramePoint2d cop = getCoP(contactablePlaneBody);
         endEffectorControlModule.setCenterOfPressure(cop);
      }

      computeCapturePoint();
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
      desiredCoMHeightAcceleration.set(computeDesiredCoMHeightAcceleration(desiredICPVelocity.getFrameVector2dCopy()));

      double kUpperBody = this.kUpperBody.getDoubleValue();
      double dUpperBody = GainCalculator.computeDerivativeGain(kUpperBody, zetaUpperBody.getDoubleValue());
      for (OneDoFJoint joint : positionControlJoints)
      {
         doPDControl(joint, kUpperBody, dUpperBody, 0.0, 0.0);
      }

      if (chestOrientationControlModule != null)
         doChestControl();

      if (headOrientationControlModule != null)
         doHeadControl();

      doArmControl();

      setICPBasedMomentumRateOfChangeControlModuleInputs();

      if (lidarJoint != null)
         setOneDoFJointAcceleration(lidarJoint, 0.0);
   }

   // TODO: connect ports instead
   private void setICPBasedMomentumRateOfChangeControlModuleInputs()
   {
      icpBasedMomentumRateOfChangeControlModule.getBipedSupportPolygonsInputPort().setData(bipedSupportPolygons);

      CapturePointData capturePointData = new CapturePointData();
      capturePointData.set(capturePoint.getFramePoint2dCopy(), omega0.getDoubleValue());
      icpBasedMomentumRateOfChangeControlModule.getCapturePointInputPort().setData(capturePointData);

      CapturePointTrajectoryData capturePointTrajectoryData = new CapturePointTrajectoryData();
      capturePointTrajectoryData.set(desiredICP.getFramePoint2dCopy(), desiredICPVelocity.getFrameVector2dCopy());
      icpBasedMomentumRateOfChangeControlModule.getDesiredCapturePointTrajectoryInputPort().setData(capturePointTrajectoryData);

      icpBasedMomentumRateOfChangeControlModule.getSupportLegInputPort().setData(getSupportLeg());

      icpBasedMomentumRateOfChangeControlModule.getDesiredCenterOfMassHeightAccelerationInputPort().setData(desiredCoMHeightAcceleration.getDoubleValue());

      // FIXME: use angular velocity, angular acceleration
      OrientationTrajectoryData desiredPelvisOrientationTrajectoryData = new OrientationTrajectoryData();
      desiredPelvisOrientationTrajectoryData.set(desiredPelvisOrientation.getFrameOrientationCopy(), new FrameVector(worldFrame), new FrameVector(worldFrame));
      desiredPelvisOrientationTrajectoryInputPort.setData(desiredPelvisOrientationTrajectoryData);

   }

   private void doChestControl()
   {
      chestOrientationControlModule.compute();
      solver.setDesiredSpatialAcceleration(chestOrientationControlModule.getJacobian(), chestOrientationControlModule.getTaskspaceConstraintData());
   }

   public void doHeadControl()
   {
      headOrientationControlModule.compute();
      solver.setDesiredSpatialAcceleration(headOrientationControlModule.getJacobian(), headOrientationControlModule.getTaskspaceConstraintData());


      if (jointForExtendedNeckPitchRange != null)
      {
         double kUpperBody = this.kUpperBody.getDoubleValue();
         double dUpperBody = GainCalculator.computeDerivativeGain(kUpperBody, zetaUpperBody.getDoubleValue());
         double angle = 0.0;
         if (desiredHeadOrientationProvider != null)
         {
            angle = desiredHeadOrientationProvider.getDesiredExtendedNeckPitchJointAngle();
         }

         doPDControl(jointForExtendedNeckPitchRange, kUpperBody, dUpperBody, angle, 0.0);
      }
   }

   public void doArmControl()
   {
      RigidBody base = fullRobotModel.getChest();
      final ReferenceFrame chestFrame = base.getBodyFixedFrame();
      for (RobotSide robotSide : RobotSide.values())
      {
         FramePose desiredHandPose = desiredHandPoses.get(robotSide);

         FrameVector desiredLinearVelocityOfOrigin = new FrameVector(chestFrame);
         FrameVector desiredAngularVelocity = new FrameVector(chestFrame);
         FrameVector desiredLinearAccelerationOfOrigin = new FrameVector(chestFrame);
         FrameVector desiredAngularAcceleration = new FrameVector(chestFrame);
         handControlModules.get(robotSide).doPositionControl(desiredHandPose.getPostionCopy(), desiredHandPose.getOrientationCopy(),
                                desiredLinearVelocityOfOrigin, desiredAngularVelocity, desiredLinearAccelerationOfOrigin, desiredAngularAcceleration, base);

         SpatialAccelerationVector handAcceleration = new SpatialAccelerationVector();
         handControlModules.get(robotSide).packAcceleration(handAcceleration);

         DenseMatrix64F nullspaceMultipliers = new DenseMatrix64F(0, 1);
         GeometricJacobian jacobian = jacobians.get(robotSide).get(LimbName.ARM);

         TaskspaceConstraintData taskspaceConstraintData = new TaskspaceConstraintData();
         taskspaceConstraintData.set(handAcceleration, nullspaceMultipliers);
         solver.setDesiredSpatialAcceleration(jacobian, taskspaceConstraintData);
      }
   }

   // Temporary objects to reduce garbage collection.
   private final CoMHeightPartialDerivativesData coMHeightPartialDerivatives = new CoMHeightPartialDerivativesData();
   private final ContactStatesAndUpcomingFootstepData centerOfMassHeightInputData = new ContactStatesAndUpcomingFootstepData();

   private double computeDesiredCoMHeightAcceleration(FrameVector2d desiredICPVelocity)
   {
      ReferenceFrame frame = worldFrame;

      centerOfMassHeightInputData.setCenterOfMassFrame(centerOfMassFrame);

      List<? extends PlaneContactState> contactStatesList = getContactStatesList();

      centerOfMassHeightInputData.setContactStates(contactStatesList);

      centerOfMassHeightInputData.setSupportLeg(getSupportLeg());
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
      comXYAcceleration.scale(getOmega0());    // MathTools.square(omega0.getDoubleValue()) * (com.getX() - copX);

//    FrameVector2d comd2dSquared = new FrameVector2d(comXYVelocity.getReferenceFrame(), comXYVelocity.getX() * comXYVelocity.getX(), comXYVelocity.getY() * comXYVelocity.getY());


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
         solver.setDesiredSpatialAcceleration(jacobian, taskspaceConstraintData);
      }
   }

   private void setOnToesContactState(ContactablePlaneBody contactableBody)
   {
      YoPlaneContactState contactState = contactStates.get(contactableBody);
      List<FramePoint> contactPoints = getContactPointsForWalkingOnToes(contactableBody);
      List<FramePoint2d> contactPoints2d = new ArrayList<FramePoint2d>(contactPoints.size());
      for (FramePoint contactPoint : contactPoints)
      {
         contactPoint.changeFrame(contactableBody.getPlaneFrame());
         contactPoints2d.add(contactPoint.toFramePoint2d());
      }

      contactState.set(contactPoints2d, coefficientOfFriction.getDoubleValue());
      updateEndEffectorControlModule(contactableBody, contactState);
      resetCoPFilter(contactableBody);
   }

   private void setFlatFootContactState(ContactablePlaneBody contactableBody)
   {
      YoPlaneContactState contactState = contactStates.get(contactableBody);
      contactState.set(contactableBody.getContactPoints2d(), coefficientOfFriction.getDoubleValue());
      updateEndEffectorControlModule(contactableBody, contactState);
   }

   private void setContactStateForSwing(ContactablePlaneBody contactableBody)
   {
      YoPlaneContactState contactState = contactStates.get(contactableBody);
      contactState.set(new ArrayList<FramePoint2d>(), coefficientOfFriction.getDoubleValue());
      endEffectorControlModules.get(contactableBody).doSingularityEscapeBeforeTransitionToNextState();
      updateEndEffectorControlModule(contactableBody, contactState);
   }

   private List<FramePoint> getContactPointsForWalkingOnToes(ContactablePlaneBody contactableBody)
   {
      FrameVector forward = new FrameVector(contactableBody.getBodyFrame(), 1.0, 0.0, 0.0);
      List<FramePoint> contactPoints = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(contactableBody.getContactPoints(), forward, 2);

      return contactPoints;
   }

   private void updateEndEffectorControlModule(ContactablePlaneBody contactablePlaneBody, PlaneContactState contactState)
   {
      List<FramePoint2d> contactPoints = contactState.getContactPoints2d();
      endEffectorControlModules.get(contactablePlaneBody).setContactPoints(contactPoints);
   }

// TODO: should probably precompute this somewhere else
   private FrameConvexPolygon2d computeFootPolygon(RobotSide robotSide, ReferenceFrame referenceFrame)
   {
      List<FramePoint> contactPoints = contactStates.get(bipedFeet.get(robotSide)).getContactPoints();
      contactPoints = DesiredFootstepCalculatorTools.fixTwoPointsAndCopy(contactPoints);    // TODO: terrible
      FrameConvexPolygon2d footPolygon = FrameConvexPolygon2d.constructByProjectionOntoXYPlane(contactPoints, referenceFrame);

      return footPolygon;
   }

   protected void doAdditionalTorqueControl()
   {
      if (lidarJoint != null)
      {
         double lidarJointTau = lidarJointVelocityController.compute(lidarJoint.getQd(), desiredLidarVelocity.getDoubleValue(), 0.0, 0.0, controlDT);
         lidarJoint.setTau(lidarJoint.getTau() + lidarJointTau);
      }
   }

   private static RobotSide getRobotSide(ContactablePlaneBody body, SideDependentList<? extends ContactablePlaneBody> bipedFeet)
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         if (body == bipedFeet.get(robotSide))
            return robotSide;
      }

      throw new RuntimeException("ContactablePlaneBody: " + body + " not found.");
   }
}
