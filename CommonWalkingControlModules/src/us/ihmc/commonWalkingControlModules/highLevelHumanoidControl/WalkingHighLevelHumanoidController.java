package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import javax.media.j3d.Transform3D;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.calculators.GainCalculator;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodyOrientationControlModule;
import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorTools;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointCalculator;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LimbName;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.trajectories.CenterOfMassHeightTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantCoPInstantaneousCapturePointTrajectory;
import us.ihmc.commonWalkingControlModules.trajectories.CubicPolynomialTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.CurrentOrientationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.FlatThenPolynomialCoMHeightTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.InstantaneousCapturePointTrajectory;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.SettableOrientationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.YoVariableDoubleProvider;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.PDController;
import com.yobotics.simulationconstructionset.util.graphics.BagOfBalls;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.statemachines.State;
import com.yobotics.simulationconstructionset.util.statemachines.StateMachine;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransition;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransitionAction;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransitionCondition;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleProvider;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.PositionTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.YoPositionProvider;

public class WalkingHighLevelHumanoidController extends AbstractHighLevelHumanoidController
{
   private static enum WalkingState {LEFT_SUPPORT, RIGHT_SUPPORT, TRANSFER_TO_LEFT_SUPPORT, TRANSFER_TO_RIGHT_SUPPORT, DOUBLE_SUPPORT}

   private final StateMachine stateMachine;
   private final CenterOfMassJacobian centerOfMassJacobian;

   private final CenterOfMassHeightTrajectoryGenerator centerOfMassHeightTrajectoryGenerator;
   private final PDController centerOfMassHeightController;
   private final SideDependentList<WalkingState> singleSupportStateEnums = new SideDependentList<WalkingState>(WalkingState.LEFT_SUPPORT,
                                                                              WalkingState.RIGHT_SUPPORT);

   private final SideDependentList<WalkingState> transferStateEnums = new SideDependentList<WalkingState>(WalkingState.TRANSFER_TO_LEFT_SUPPORT,
                                                                         WalkingState.TRANSFER_TO_RIGHT_SUPPORT);

   private final YoVariableDoubleProvider doubleSupportTimeProvider = new YoVariableDoubleProvider("doubleSupportTime", registry);

   private final DoubleYoVariable singleSupportICPGlideScaleFactor = new DoubleYoVariable("singleSupportICPGlideScaleFactor", registry);
   private final BooleanYoVariable walk = new BooleanYoVariable("walk", registry);


   // FIXME: reimplement nullspace stuff:
   private final double swingNullspaceMultiplier = 500.0;    // needs to be pretty high to fight the limit stops...
   
   // FIXME: reimplement and improve com trajectory visualization
   private final BagOfBalls comTrajectoryBagOfBalls;
   private int comTrajectoryCounter = 0;

   // private final BooleanYoVariable transferICPTrajectoryDone = new BooleanYoVariable("transferICPTrajectoryDone", registry);
   private final DoubleYoVariable minOrbitalEnergyForSingleSupport = new DoubleYoVariable("minOrbitalEnergyForSingleSupport", registry);
   private final DoubleYoVariable amountToBeInsideSingleSupport = new DoubleYoVariable("amountToBeInsideSingleSupport", registry);
   private final DoubleYoVariable amountToBeInsideDoubleSupport = new DoubleYoVariable("amountToBeInsideDoubleSupport", registry);

   private final RigidBodyOrientationControlModule chestOrientationControlModule;

   private final DoubleYoVariable userDesiredPelvisPitch = new DoubleYoVariable("userDesiredPelvisPitch", registry);
   private final SettableOrientationProvider initialPelvisOrientationProvider;
   private final SettableOrientationProvider finalPelvisOrientationProvider;
   private final OrientationTrajectoryGenerator pelvisOrientationTrajectoryGenerator;

   private final HashMap<RigidBody, EndEffectorControlModule> footStateMachines = new HashMap<RigidBody, EndEffectorControlModule>();
   private final SideDependentList<PositionTrajectoryGenerator> footPositionTrajectoryGenerators;
   private final DoubleProvider swingTimeProvider;
   private final SideDependentList<YoVariableDoubleProvider> onEdgeInitialAngleProviders = new SideDependentList<YoVariableDoubleProvider>();
   private final SideDependentList<YoVariableDoubleProvider> onEdgeFinalAngleProviders = new SideDependentList<YoVariableDoubleProvider>();
   private final BooleanYoVariable stayOnToes = new BooleanYoVariable("stayOnToes", registry);
   private final DoubleYoVariable trailingFootPitch = new DoubleYoVariable("trailingFootPitch", registry);
   private final OneDoFJoint[] upperBodyJoints;

   private final DoubleYoVariable kUpperBody = new DoubleYoVariable("kUpperBody", registry);
   private final DoubleYoVariable zetaUpperBody = new DoubleYoVariable("zetaUpperBody", registry);
   private final YoPositionProvider finalPositionProvider;

   private final InstantaneousCapturePointTrajectory icpTrajectoryGenerator;

   public WalkingHighLevelHumanoidController(FullRobotModel fullRobotModel, CommonWalkingReferenceFrames referenceFrames, TwistCalculator twistCalculator,
           CenterOfMassJacobian centerOfMassJacobian, SideDependentList<? extends ContactablePlaneBody> bipedFeet, BipedSupportPolygons bipedSupportPolygons,
           SideDependentList<FootSwitchInterface> footSwitches, double gravityZ, DoubleYoVariable yoTime, double controlDT, YoVariableRegistry registry,
           DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, DesiredFootstepCalculator desiredFootstepCalculator,
           CenterOfMassHeightTrajectoryGenerator centerOfMassHeightTrajectoryGenerator,
           SideDependentList<PositionTrajectoryGenerator> footPositionTrajectoryGenerators, DoubleProvider swingTimeProvider,
           YoPositionProvider finalPositionProvider, boolean stayOntoes, double desiredPelvisPitch, double trailingFootPitch, ArrayList<Updatable> updatables)
   {
      super(fullRobotModel, centerOfMassJacobian, referenceFrames, yoTime, gravityZ, twistCalculator, bipedFeet, bipedSupportPolygons, controlDT, footSwitches,
            updatables, registry, dynamicGraphicObjectsListRegistry);

      this.desiredFootstepCalculator = desiredFootstepCalculator;

      this.centerOfMassJacobian = centerOfMassJacobian;
      this.centerOfMassHeightTrajectoryGenerator = centerOfMassHeightTrajectoryGenerator;
      this.swingTimeProvider = swingTimeProvider;

      this.centerOfMassHeightController = new PDController("comHeight", registry);
      centerOfMassHeightController.setProportionalGain(40.0);    // about 4000 / totalMass
      double zeta = 1.0;
      centerOfMassHeightController.setDerivativeGain(2.0 * zeta * Math.sqrt(centerOfMassHeightController.getProportionalGain()));

      String namePrefix = "walking";
      icpTrajectoryGenerator = new ConstantCoPInstantaneousCapturePointTrajectory(bipedSupportPolygons, gravity, controlDT, registry);

//    doubleSupportICPTrajectoryGenerator = new ParabolicVelocityInstantaneousCapturePointTrajectory(namePrefix,
//          bipedSupportPolygons, controlDT, registry);

      this.stateMachine = new StateMachine(namePrefix + "State", namePrefix + "SwitchTime", WalkingState.class, yoTime, registry);
      upcomingSupportLeg.set(RobotSide.RIGHT);

      singleSupportICPGlideScaleFactor.set(0.9);

      this.finalPositionProvider = finalPositionProvider;

      this.chestOrientationControlModule = new RigidBodyOrientationControlModule("chest", fullRobotModel.getPelvis(), fullRobotModel.getChest(),
              twistCalculator, registry);
      chestOrientationControlModule.setProportionalGains(100.0, 100.0, 100.0);
      chestOrientationControlModule.setDerivativeGains(20.0, 20.0, 20.0);

      this.footPositionTrajectoryGenerators = footPositionTrajectoryGenerators;

      for (RobotSide robotSide : RobotSide.values())
      {
         contactStates.get(fullRobotModel.getFoot(robotSide)).setContactPoints(bipedFeet.get(robotSide).getContactPoints2d());    // flat feet
         String sideString = robotSide.getCamelCaseNameForStartOfExpression();
         RigidBody footBody = fullRobotModel.getFoot(robotSide);
         ContactablePlaneBody bipedFoot = bipedFeet.get(robotSide);

         PositionTrajectoryGenerator swingPositionTrajectoryGenerator = footPositionTrajectoryGenerators.get(robotSide);

         OrientationProvider initialOrientationProvider = new CurrentOrientationProvider(worldFrame, bipedFoot.getBodyFrame());
         OrientationProvider finalOrientationProvider = new DesiredFootstepBasedOrientationProvider(robotSide);

         OrientationTrajectoryGenerator swingOrientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator(sideString
                                                                                 + "SwingFootOrientation", worldFrame, swingTimeProvider,
                                                                                    initialOrientationProvider, finalOrientationProvider, registry);

         YoVariableDoubleProvider onEdgeInitialPitchProvider = new YoVariableDoubleProvider(sideString + "OnToesInitialPitch", registry);
         YoVariableDoubleProvider onEdgeFinalPitchProvider = new YoVariableDoubleProvider(sideString + "OnToesFinalPitch", registry);
         DoubleProvider onToesTrajectoryTimeProvider = doubleSupportTimeProvider;
         DoubleTrajectoryGenerator onToesPitchTrajectoryGenerator = new CubicPolynomialTrajectoryGenerator(sideString + "OnToesPitch",
                                                                       onEdgeInitialPitchProvider, onEdgeFinalPitchProvider, onToesTrajectoryTimeProvider,
                                                                       registry);

         onEdgeInitialAngleProviders.put(robotSide, onEdgeInitialPitchProvider);
         onEdgeFinalAngleProviders.put(robotSide, onEdgeFinalPitchProvider);

         EndEffectorControlModule footStateMachine = new EndEffectorControlModule(bipedFoot, swingPositionTrajectoryGenerator,
                                                        swingOrientationTrajectoryGenerator, onToesPitchTrajectoryGenerator, yoTime, twistCalculator, registry);
         footStateMachines.put(footBody, footStateMachine);
      }

      initialPelvisOrientationProvider = new SettableOrientationProvider("initialPelvis", worldFrame, registry);
      finalPelvisOrientationProvider = new SettableOrientationProvider("finalPelvis", worldFrame, registry);
      this.pelvisOrientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator("pelvis", worldFrame, swingTimeProvider,
              initialPelvisOrientationProvider, finalPelvisOrientationProvider, registry);

      comTrajectoryBagOfBalls = new BagOfBalls(500, 0.01, "comBagOfBalls", YoAppearance.Red(), registry, dynamicGraphicObjectsListRegistry);

      setUpStateMachine(swingTimeProvider);

      walk.set(false);
      minOrbitalEnergyForSingleSupport.set(0.007);    // 0.008
      amountToBeInsideSingleSupport.set(0.0);
      amountToBeInsideDoubleSupport.set(0.03);    // 0.02); // TODO: possibly link to limit in SacrificeDeltaCMP control module
      doubleSupportTimeProvider.set(0.2);    // 0.5);    // 0.2);    // 0.6;    // 0.3
      this.userDesiredPelvisPitch.set(desiredPelvisPitch);
      this.stayOnToes.set(stayOntoes);
      this.trailingFootPitch.set(trailingFootPitch);
      kUpperBody.set(100.0);
      zetaUpperBody.set(1.0);

      double initialLeadingFootPitch = 0.05;

      RobotSide leadingLeg = getUpcomingSupportLeg();
      onEdgeInitialAngleProviders.get(leadingLeg).set(initialLeadingFootPitch);
      onEdgeFinalAngleProviders.get(leadingLeg).set(initialLeadingFootPitch);

      RobotSide trailingLeg = leadingLeg.getOppositeSide();
      onEdgeInitialAngleProviders.get(trailingLeg).set(this.trailingFootPitch.getDoubleValue());

      InverseDynamicsJoint[] upperBodyJoints = ScrewTools.computeJointsInOrder(fullRobotModel.getChest());
      int nRevoluteJoints = ScrewTools.computeNumberOfJointsOfType(OneDoFJoint.class, upperBodyJoints);
      this.upperBodyJoints = new OneDoFJoint[nRevoluteJoints];
      ScrewTools.filterJoints(upperBodyJoints, this.upperBodyJoints, OneDoFJoint.class);
   }

   private void setUpStateMachine(DoubleProvider stepTimeProvider)
   {
      DoubleSupportState doubleSupportState = new DoubleSupportState(null);

      stateMachine.addState(doubleSupportState);

      for (RobotSide robotSide : RobotSide.values())
      {
         State transferState = new DoubleSupportState(robotSide);
         StateTransition toDoubleSupport = new StateTransition(doubleSupportState.getStateEnum(), new StopWalkingCondition(), new ResetICPTrajectoryAction());
         transferState.addStateTransition(toDoubleSupport);
         StateTransition toSingleSupport = new StateTransition(singleSupportStateEnums.get(robotSide), new DoneWithTransferCondition());
         transferState.addStateTransition(toSingleSupport);
         stateMachine.addState(transferState);

         State singleSupportState = new SingleSupportState(robotSide);
         StateTransition toDoubleSupport2 = new StateTransition(doubleSupportState.getStateEnum(), new StopWalkingCondition(), new ResetICPTrajectoryAction());
         singleSupportState.addStateTransition(toDoubleSupport2);
         StateTransition toTransfer = new StateTransition(transferStateEnums.get(robotSide.getOppositeSide()), new DoneWithSingleSupportCondition());
         singleSupportState.addStateTransition(toTransfer);
         stateMachine.addState(singleSupportState);
      }

      for (RobotSide robotSide : RobotSide.values())
      {
         StateTransition toTransfer = new StateTransition(transferStateEnums.get(robotSide), new DoneWithDoubleSupportCondition(robotSide));
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

      stateMachine.setCurrentState(WalkingState.DOUBLE_SUPPORT);
   }

   private class DoubleSupportState extends State
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
         evaluateCoMTrajectory();

         if (icpTrajectoryGenerator.isDone() && (transferToSide == null))
         {
            // keep desiredICP the same
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
         setSupportLeg(null);

         RobotSide trailingLeg = getUpcomingSupportLeg().getOppositeSide();
         onEdgeFinalAngleProviders.get(trailingLeg).set(trailingFootPitch.getDoubleValue());

         if (stayOnToes.getBooleanValue())
         {
            for (RobotSide robotSide : RobotSide.values())
            {
               setOnToesContactState(bipedFeet.get(robotSide));
            }
         }
         else
         {
//          if (transferToSide != null)
//             setOnToesContactState(bipedFeet.get(trailingLeg));
//          else
//             setFlatFootContactState(bipedFeet.get(trailingLeg));
            setFlatFootContactState(bipedFeet.get(trailingLeg));
            setFlatFootContactState(bipedFeet.get(getUpcomingSupportLeg()));
         }


         updateBipedSupportPolygons();

         FramePoint2d finalDesiredICP;

         desiredICP.checkReferenceFrameMatch(capturePoint.getReferenceFrame());
         desiredICP.set(capturePoint.getX(), capturePoint.getY());

         if (transferToSide == null)
         {
            finalDesiredICP = getDoubleSupportFinalDesiredICPForDoubleSupportStance();
         }
         else
         {
            FramePose currentFramePose = new FramePose(referenceFrames.getFootFrame(transferToSide));
            FrameConvexPolygon2d footPolygon = computeFootPolygon(transferToSide, referenceFrames.getSoleFrame(transferToSide));
            finalDesiredICP = getDoubleSupportFinalDesiredICPForWalking(currentFramePose, footPolygon, transferToSide);
         }

         finalDesiredICP.changeFrame(desiredICP.getReferenceFrame());
         icpTrajectoryGenerator.initialize(desiredICP.getFramePoint2dCopy(), finalDesiredICP, doubleSupportTimeProvider.getValue(), getOmega0(),
                                           amountToBeInsideDoubleSupport.getDoubleValue());

         if (transferToSide == null)
            centerOfMassHeightTrajectoryGenerator.initialize(getSupportLeg(), getUpcomingSupportLeg());
         else
         {
            desiredFootstepCalculator.initializeDesiredFootstep(transferToSide);
            centerOfMassHeightTrajectoryGenerator.initialize(getSupportLeg(), upcomingSupportLeg.getEnumValue());
         }
      }

      @Override
      public void doTransitionOutOfAction()
      {
         desiredICPVelocity.set(0.0, 0.0);
      }
   }


   private class SingleSupportState extends State
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
         evaluateCoMTrajectory();

         PositionTrajectoryGenerator positionTrajectoryGenerator = footPositionTrajectoryGenerators.get(swingSide);
         if (!positionTrajectoryGenerator.isDone())
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
         RobotSide supportSide = swingSide.getOppositeSide();
         initializeTrajectory(swingSide, null);
         setSupportLeg(supportSide);

         if (stayOnToes.getBooleanValue())
         {
            setOnToesContactState(bipedFeet.get(supportSide));
         }
         else
         {
            setFlatFootContactState(bipedFeet.get(supportSide));
         }

         setContactStateForSwing(bipedFeet.get(swingSide));
         updateBipedSupportPolygons();

         centerOfMassHeightTrajectoryGenerator.initialize(getSupportLeg(), upcomingSupportLeg.getEnumValue());
      }

      @Override
      public void doTransitionOutOfAction()
      {
         upcomingSupportLeg.set(upcomingSupportLeg.getEnumValue().getOppositeSide());

//       ContactableBody swingFoot = contactablePlaneBodies.get(swingSide);
//       Footstep desiredFootstep = desiredFootstepCalculator.updateAndGetDesiredFootstep(swingSide.getOppositeSide());
//       contactStates.get(swingFoot).setContactPoints(desiredFootstep.getExpectedContactPoints());
//       updateFootStateMachines(swingFoot);
      }
   }


   public class DoneWithDoubleSupportCondition implements StateTransitionCondition
   {
      private final RobotSide robotSide;

      public DoneWithDoubleSupportCondition(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }

      public boolean checkCondition()
      {
         boolean doubleSupportTimeHasPassed = stateMachine.timeInCurrentState() > doubleSupportTimeProvider.getValue();
         boolean transferringToThisRobotSide = robotSide == upcomingSupportLeg.getEnumValue();

         return walk.getBooleanValue() && transferringToThisRobotSide && doubleSupportTimeHasPassed;
      }
   }


   public class DoneWithTransferCondition implements StateTransitionCondition
   {
      public boolean checkCondition()
      {
         // TODO: not really nice, but it'll do:
         if (centerOfMassHeightTrajectoryGenerator instanceof FlatThenPolynomialCoMHeightTrajectoryGenerator)
         {
            FlatThenPolynomialCoMHeightTrajectoryGenerator flatThenPolynomialCoMHeightTrajectoryGenerator =
               (FlatThenPolynomialCoMHeightTrajectoryGenerator) centerOfMassHeightTrajectoryGenerator;
            double orbitalEnergy = flatThenPolynomialCoMHeightTrajectoryGenerator.computeOrbitalEnergyIfInitializedNow(getUpcomingSupportLeg());

            // return transferICPTrajectoryDone.getBooleanValue() && orbitalEnergy > minOrbitalEnergy;
            return orbitalEnergy > minOrbitalEnergyForSingleSupport.getDoubleValue();
         }
         else
         {
            return icpTrajectoryGenerator.isDone();
         }
      }
   }


   public class DoneWithSingleSupportCondition implements StateTransitionCondition
   {
      public boolean checkCondition()
      {
         RobotSide swingSide = getSupportLeg().getOppositeSide();
         double minimumSwingFraction = 0.5;
         double minimumSwingTime = swingTimeProvider.getValue() * minimumSwingFraction;
         boolean footHitGround = (stateMachine.timeInCurrentState() > minimumSwingTime) && footSwitches.get(swingSide).hasFootHitGround();

         return (footPositionTrajectoryGenerators.get(swingSide).isDone() || footHitGround);
      }
   }


   public class StopWalkingCondition extends DoneWithSingleSupportCondition
   {
      public boolean checkCondition()
      {
         return !walk.getBooleanValue() && ((getSupportLeg() == null) || super.checkCondition());
      }
   }


   public class ResetICPTrajectoryAction implements StateTransitionAction
   {
      public void doTransitionAction()
      {
         icpTrajectoryGenerator.reset();
      }
   }


   public void setPreviousCoP(FramePoint2d previousCoP)
   {
      previousCoP.changeFrame(this.previousCoP.getReferenceFrame());
      this.previousCoP.set(previousCoP);
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

   private FramePoint2d getDoubleSupportFinalDesiredICPForWalking(FramePose footstepPose, FrameConvexPolygon2d footPolygonInSoleFrame, RobotSide transferToSide)
   {
      if (footPolygonInSoleFrame.getReferenceFrame() != referenceFrames.getSoleFrame(transferToSide))
      {
         throw new RuntimeException("not in sole frame");
      }

      ReferenceFrame frame = referenceFrames.getAnkleZUpFrame(transferToSide);
      footstepPose.changeFrame(frame);

      Transform3D footstepTransform = new Transform3D();
      footstepPose.getTransform3D(footstepTransform);

      FramePoint2d centroid2d = footPolygonInSoleFrame.getCentroidCopy();
      FramePoint centroid = centroid2d.toFramePoint();
      centroid.changeFrame(referenceFrames.getFootFrame(transferToSide));
      centroid.changeFrameUsingTransform(frame, footstepTransform);
      FramePoint2d ret = centroid.toFramePoint2d();

      double extraX = 0.0;    // 0.02
      double extraY = transferToSide.negateIfLeftSide(0.04);
      FrameVector2d offset = new FrameVector2d(frame, extraX, extraY);
      offset.changeFrame(ret.getReferenceFrame());
      ret.add(offset);

      return ret;
   }

   private FramePoint2d getSingleSupportFinalDesiredICPForWalking(Footstep desiredFootstep, RobotSide swingSide)
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

      FramePoint2d finalDesiredICP = getDoubleSupportFinalDesiredICPForWalking(desiredFootstep.getPoseCopy(), footPolygon, swingSide);    // yes, swing side
      finalDesiredICP.changeFrame(referenceFrame);

      FrameLineSegment2d initialToFinal = new FrameLineSegment2d(initialDesiredICP, finalDesiredICP);

      return initialToFinal.pointBetweenEndPointsGivenParameter(singleSupportICPGlideScaleFactor.getDoubleValue());
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

      Footstep desiredFootstep = desiredFootstepCalculator.updateAndGetDesiredFootstep(supportSide);
      finalPositionProvider.set(desiredFootstep.getPositionInFrame(worldFrame));

      FrameOrientation initialPelvisOrientation = new FrameOrientation(worldFrame);
      finalPelvisOrientationProvider.get(initialPelvisOrientation);
      initialPelvisOrientation.changeFrame(worldFrame);
      initialPelvisOrientationProvider.setOrientation(initialPelvisOrientation);

      FrameOrientation finalPelvisOrientation = desiredFootstep.getOrientationInFrame(worldFrame);
      finalPelvisOrientation.setYawPitchRoll(finalPelvisOrientation.getYawPitchRoll()[0], userDesiredPelvisPitch.getDoubleValue(), 0.0);
      finalPelvisOrientationProvider.setOrientation(finalPelvisOrientation);
      pelvisOrientationTrajectoryGenerator.initialize();

      FramePoint finalDesiredStepLocation = desiredFootstep.getPositionInFrame(trajectoryGeneratorFrame);
      finalDesiredStepLocation.setZ(finalDesiredStepLocation.getZ());

      double stepPitch = desiredFootstep.getOrientationInFrame(worldFrame).getYawPitchRoll()[1];
      onEdgeInitialAngleProviders.get(swingSide).set(stepPitch);
      onEdgeFinalAngleProviders.get(swingSide).set(stepPitch);

      FramePoint centerOfMass = new FramePoint(referenceFrames.getCenterOfMassFrame());
      centerOfMass.changeFrame(worldFrame);
      ContactablePlaneBody supportFoot = bipedFeet.get(supportSide);
      Transform3D footToWorldTransform = supportFoot.getBodyFrame().getTransformToDesiredFrame(worldFrame);
      double footHeight = DesiredFootstepCalculatorTools.computeMinZPointInFrame(footToWorldTransform, supportFoot, worldFrame).getZ();
      double comHeight = centerOfMass.getZ() - footHeight;
      double omega0 = CapturePointCalculator.computeOmega0ConstantHeight(gravity, comHeight);
      setOmega0(omega0);
      computeCapturePoint();
      desiredICP.set(capturePoint.getFramePoint2dCopy());    // TODO: is this what we want?

      FramePoint2d finalDesiredICP = getSingleSupportFinalDesiredICPForWalking(desiredFootstep, swingSide);

      ContactablePlaneBody swingFoot = bipedFeet.get(swingSide);
      setContactStateForSwing(swingFoot);
      setSupportLeg(supportSide);
      updateBipedSupportPolygons();

      icpTrajectoryGenerator.initialize(desiredICP.getFramePoint2dCopy(), finalDesiredICP, swingTimeProvider.getValue(), omega0,
                                        amountToBeInsideSingleSupport.getDoubleValue());

      stateMachine.doAction();    // computes trajectory and stores results in YoFrameVectors and Points.
   }

   private void evaluateCoMTrajectory()
   {
      centerOfMassHeightTrajectoryGenerator.compute();
//      comTrajectoryCounter++;
//
//      if (comTrajectoryCounter >= 5)
//      {
//         FramePoint desiredCoM = new FramePoint(referenceFrames.getCenterOfMassFrame());
//         desiredCoM.changeFrame(worldFrame);
//         desiredCoM.setY(0.0);
//         desiredCoM.setZ(centerOfMassHeightTrajectoryGenerator.getDesiredCenterOfMassHeight());
//         comTrajectoryBagOfBalls.setBallLoop(desiredCoM);
//         comTrajectoryCounter = 0;
//      }
   }

   public void doControl()
   {
      computeCapturePoint();
      stateMachine.checkTransitionConditionsThoroughly();
      stateMachine.doAction();
      desiredCoMHeightAcceleration.set(computeDesiredCoMHeightAcceleration(desiredICPVelocity.getFrameVector2dCopy()));
      doChestControl();
      doUpperBodyControl();
   }

   private void doChestControl()
   {
      ReferenceFrame pelvisFrame = fullRobotModel.getPelvis().getBodyFixedFrame();
      FrameOrientation desiredOrientation = new FrameOrientation(pelvisFrame);
      FrameVector desiredAngularVelocity = new FrameVector(pelvisFrame);
      FrameVector desiredAngularAcceleration = new FrameVector(pelvisFrame);

      FrameVector outputToPack = new FrameVector(fullRobotModel.getChest().getBodyFixedFrame());
      chestOrientationControlModule.controlSpine(outputToPack, desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
      chestAngularAccelerationWithRespectToPelvis.set(outputToPack);
      chestAngularAccelerationcalculator.compute(getChestAngularAcceleration());

      // TODO: have chestAngularAccelerationcalculator not set the result in the joints
      for (InverseDynamicsJoint joint : spineJoints)
      {
         DenseMatrix64F jointAcceleration = jointAccelerations.get(joint);
         if (jointAcceleration == null)
         {
            jointAcceleration = new DenseMatrix64F(joint.getDegreesOfFreedom(), 1);
            jointAccelerations.put(joint, jointAcceleration);
         }

         joint.packDesiredAccelerationMatrix(jointAcceleration, 0);
      }
   }

   private void doUpperBodyControl()
   {
      double kUpperBody = this.kUpperBody.getDoubleValue();
      double dUpperBody = GainCalculator.computeDerivativeGain(kUpperBody, zetaUpperBody.getDoubleValue());

      doPDControl(upperBodyJoints, kUpperBody, dUpperBody);
   }

   private double computeDesiredCoMHeightAcceleration(FrameVector2d desiredICPVelocity)
   {
      double zDesired = centerOfMassHeightTrajectoryGenerator.getDesiredCenterOfMassHeight();
      double dzdxDesired = centerOfMassHeightTrajectoryGenerator.getDesiredCenterOfMassHeightSlope();
      double d2zdx2Desired = centerOfMassHeightTrajectoryGenerator.getDesiredCenterOfMassHeightSecondDerivative();

      ReferenceFrame stairDirectionFrame = worldFrame;

      desiredICPVelocity.changeFrame(stairDirectionFrame);

      FramePoint com = new FramePoint(referenceFrames.getCenterOfMassFrame());
      com.changeFrame(stairDirectionFrame);

      FrameVector comd = new FrameVector(stairDirectionFrame);
      centerOfMassJacobian.packCenterOfMassVelocity(comd);
      comd.changeFrame(stairDirectionFrame);

      // TODO: use current omega0 instead of previous
      double xd = comd.getX();
      double icpdx = desiredICPVelocity.getX();
      double xdd = getOmega0() * (icpdx - xd);    // MathTools.square(omega0.getDoubleValue()) * (com.getX() - copX);

      double zdDesired = dzdxDesired * xd;
      double zddFeedForward = d2zdx2Desired * MathTools.square(xd) + dzdxDesired * xdd;

      double zCurrent = com.getZ();
      double zdCurrent = comd.getZ();

      double zddDesired = centerOfMassHeightController.compute(zCurrent, zDesired, zdCurrent, zdDesired) + zddFeedForward;
      double epsilon = 1e-12;
      zddDesired = MathTools.clipToMinMax(zddDesired, -gravity + epsilon, Double.POSITIVE_INFINITY);

      return zddDesired;
   }

   private void doFootcontrol()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         SpatialAccelerationVector footAcceleration = new SpatialAccelerationVector();
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         footStateMachines.get(foot).packDesiredFootAcceleration(footAcceleration);
         ReferenceFrame bodyFixedFrame = fullRobotModel.getFoot(robotSide).getBodyFixedFrame();
         footAcceleration.changeBodyFrameNoRelativeAcceleration(bodyFixedFrame);
         footAcceleration.changeFrameNoRelativeMotion(bodyFixedFrame);
         DenseMatrix64F nullspaceMultipliers = new DenseMatrix64F(1, 0);

         Pair<SpatialAccelerationVector, DenseMatrix64F> pair = new Pair<SpatialAccelerationVector, DenseMatrix64F>(footAcceleration, nullspaceMultipliers);
         setEndEffectorSpatialAcceleration(robotSide, LimbName.LEG, pair);
      }
   }

   private void setOnToesContactState(ContactablePlaneBody contactableBody)
   {
      RigidBody rigidBody = contactableBody.getRigidBody();
      YoPlaneContactState contactState = contactStates.get(rigidBody);
      List<FramePoint> contactPoints = getContactPointsForWalkingOnToes(contactableBody);
      List<FramePoint2d> contactPoints2d = new ArrayList<FramePoint2d>(contactPoints.size());
      for (FramePoint contactPoint : contactPoints)
      {
         contactPoint.changeFrame(contactableBody.getPlaneFrame());
         contactPoints2d.add(contactPoint.toFramePoint2d());
      }

      contactState.setContactPoints(contactPoints2d);
      updateFootStateMachine(rigidBody, contactState);
   }

   private void setFlatFootContactState(ContactablePlaneBody contactableBody)
   {
      RigidBody rigidBody = contactableBody.getRigidBody();
      YoPlaneContactState contactState = contactStates.get(rigidBody);
      contactState.setContactPoints(contactableBody.getContactPoints2d());
      updateFootStateMachine(rigidBody, contactState);
   }

   private void setContactStateForSwing(ContactablePlaneBody contactableBody)
   {
      RigidBody rigidBody = contactableBody.getRigidBody();
      YoPlaneContactState contactState = contactStates.get(rigidBody);
      contactState.setContactPoints(new ArrayList<FramePoint2d>());
      updateFootStateMachine(rigidBody, contactState);
   }

   private List<FramePoint> getContactPointsForWalkingOnToes(ContactablePlaneBody contactableBody)
   {
      FrameVector forward = new FrameVector(contactableBody.getBodyFrame(), 1.0, 0.0, 0.0);
      List<FramePoint> contactPoints = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(contactableBody.getContactPoints(), forward, 2);

      return contactPoints;
   }

   private void updateFootStateMachine(RigidBody rigidBody, PlaneContactState contactState)
   {
      List<FramePoint2d> contactPoints = contactState.getContactPoints2d();
      footStateMachines.get(rigidBody).setContactPoints(contactPoints);
   }

   private void updateBipedSupportPolygons()
   {
      SideDependentList<List<FramePoint>> contactPoints = new SideDependentList<List<FramePoint>>();
      for (RobotSide robotSide : RobotSide.values)
      {
         contactPoints.put(robotSide, contactStates.get(fullRobotModel.getFoot(robotSide)).getContactPoints());
      }

      bipedSupportPolygons.update(contactPoints);
   }

// TODO: should probably precompute this somewhere else
   private FrameConvexPolygon2d computeFootPolygon(RobotSide robotSide, ReferenceFrame referenceFrame)
   {
      List<FramePoint> contactPoints = contactStates.get(fullRobotModel.getFoot(robotSide)).getContactPoints();
      contactPoints = DesiredFootstepCalculatorTools.fixTwoPointsAndCopy(contactPoints);    // TODO: terrible
      FrameConvexPolygon2d footPolygon = FrameConvexPolygon2d.constructByProjectionOntoXYPlane(contactPoints, referenceFrame);

      return footPolygon;
   }

   private class DesiredFootstepBasedOrientationProvider implements OrientationProvider
   {
      private RobotSide swingSide;

      public DesiredFootstepBasedOrientationProvider(RobotSide swingSide)
      {
         this.swingSide = swingSide;
      }

      public void get(FrameOrientation orientationToPack)
      {
         Footstep footstep = desiredFootstepCalculator.updateAndGetDesiredFootstep(swingSide.getOppositeSide());
         orientationToPack.set(footstep.getOrientationInFrame(worldFrame));
      }
   }
}
