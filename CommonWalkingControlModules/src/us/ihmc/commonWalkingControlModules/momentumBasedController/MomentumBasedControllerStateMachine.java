package us.ihmc.commonWalkingControlModules.momentumBasedController;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedFootInterface;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.FootPolygonEnum;
import us.ihmc.commonWalkingControlModules.calculators.MaximumICPVelocityCalculator;
import us.ihmc.commonWalkingControlModules.captureRegion.CapturePointCalculatorInterface;
import us.ihmc.commonWalkingControlModules.controlModules.FixedAxisOfRotationControlModule;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.desiredFootStep.SimpleWorldDesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.SimpleDesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.commonWalkingControlModules.trajectories.CartesianTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.CenterOfMassHeightTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantCoPInstantaneousCapturePointTrajectory;
import us.ihmc.commonWalkingControlModules.trajectories.FifthOrderWaypointCartesianTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.LinearFootstepCalculatorBasedCoMHeightTrajectoryGenerator;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.Orientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.BagOfBalls;
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

public class MomentumBasedControllerStateMachine extends StateMachine
{
   private static enum MomentumBasedControllerState {LEFT_SUPPORT, RIGHT_SUPPORT, TRANSFER_TO_LEFT_SUPPORT, TRANSFER_TO_RIGHT_SUPPORT, DOUBLE_SUPPORT}

   private static final String name = "momentumSM";
   private static final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final SideDependentList<MomentumBasedControllerState> singleSupportStateEnums =
      new SideDependentList<MomentumBasedControllerStateMachine.MomentumBasedControllerState>(MomentumBasedControllerState.LEFT_SUPPORT,
                            MomentumBasedControllerState.RIGHT_SUPPORT);

   private final SideDependentList<MomentumBasedControllerState> transferStateEnums =
      new SideDependentList<MomentumBasedControllerStateMachine.MomentumBasedControllerState>(MomentumBasedControllerState.TRANSFER_TO_LEFT_SUPPORT,
                            MomentumBasedControllerState.TRANSFER_TO_RIGHT_SUPPORT);

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final FullRobotModel fullRobotModel;
   private final CommonWalkingReferenceFrames referenceFrames;
   private final TwistCalculator twistCalculator;
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;
   private final SideDependentList<BipedFootInterface> bipedFeet;
   private final BipedSupportPolygons bipedSupportPolygons;
   private final SideDependentList<CartesianTrajectoryGenerator> cartesianTrajectoryGenerators = new SideDependentList<CartesianTrajectoryGenerator>();
   private final double controlDT;
   private final DesiredHeadingControlModule desiredHeadingControlModule;
   private final DesiredFootstepCalculator desiredFootstepCalculator;
   private final CenterOfMassHeightTrajectoryGenerator centerOfMassHeightTrajectoryGenerator;
   private final SideDependentList<FootSwitchInterface> footSwitches;
   private final SideDependentList<ConstantCoPInstantaneousCapturePointTrajectory> icpTrajectoryGenerators =
      new SideDependentList<ConstantCoPInstantaneousCapturePointTrajectory>();


   private final YoFramePoint2d desiredICP;
   private final YoFrameVector2d desiredICPVelocity;

   private final EnumYoVariable<RobotSide> supportLeg = EnumYoVariable.create("supportLeg", RobotSide.class, registry);
   private final EnumYoVariable<RobotSide> plantedLeg = EnumYoVariable.create("plantedLeg", RobotSide.class, registry);

   private final EnumYoVariable<RobotSide> upcomingSupportLeg = EnumYoVariable.create("upcomingSupportLeg", RobotSide.class, registry);

   private final SideDependentList<YoFramePoint> desiredFootPositions = new SideDependentList<YoFramePoint>();
   private final SideDependentList<YoFrameVector> desiredFootVelocities = new SideDependentList<YoFrameVector>();
   private final SideDependentList<YoFrameVector> desiredFootAccelerations = new SideDependentList<YoFrameVector>();

   private final SideDependentList<YoFrameOrientation> desiredFootOrientations = new SideDependentList<YoFrameOrientation>();
   private final SideDependentList<YoFrameVector> desiredFootAngularVelocities = new SideDependentList<YoFrameVector>();
   private final SideDependentList<YoFrameVector> desiredFootAngularAccelerations = new SideDependentList<YoFrameVector>();
   private final SideDependentList<DoubleYoVariable> nullspaceMultipliers = new SideDependentList<DoubleYoVariable>();

   private final DoubleYoVariable desiredCoMHeight = new DoubleYoVariable("desiredCoMHeight", registry);
   private final DoubleYoVariable desiredCoMHeightVelocity = new DoubleYoVariable("desiredCoMHeightVelocity", registry);
   private final DoubleYoVariable desiredCoMHeightAcceleration = new DoubleYoVariable("desiredCoMHeightAcceleration", registry);

   private final CapturePointCalculatorInterface capturePointCalculator;
   private final FramePoint2d previousCoP;
   private final double doubleSupportTime = 0.6; // 0.6;    // 0.3
   private final double stepTime = 0.75; // 0.55;
   private final double waypointHeight = 0.07; // -0.02;    // 0.05; // 0.15;

   private final DoubleYoVariable singleSupportICPGlideScaleFactor = new DoubleYoVariable("singleSupportICPGlideScaleFactor", registry);
   private final BooleanYoVariable walk = new BooleanYoVariable("walk", registry);
   private final BooleanYoVariable liftUpHeels = new BooleanYoVariable("liftUpHeels", registry);
   private final DoubleYoVariable heelsUpFootPitch = new DoubleYoVariable("heelsUpFootPitch", registry);

   private double comHeight;
   private final double gravity;
   private final double swingNullspaceMultiplier = 100.0;    // needs to be pretty high to fight the limit stops...
   private final SideDependentList<BooleanYoVariable> trajectoryInitialized = new SideDependentList<BooleanYoVariable>();

   private final BagOfBalls bagOfBalls;

   private final SideDependentList<FixedAxisOfRotationControlModule> fixedAxisOfRotationControlModules =
      new SideDependentList<FixedAxisOfRotationControlModule>();
   private final BooleanYoVariable transferICPTrajectoryDone = new BooleanYoVariable("transferICPTrajectoryDone", registry);

   public MomentumBasedControllerStateMachine(FullRobotModel fullRobotModel, CommonWalkingReferenceFrames referenceFrames, TwistCalculator twistCalculator,
           SideDependentList<BipedFootInterface> bipedFeet, BipedSupportPolygons bipedSupportPolygons, SideDependentList<FootSwitchInterface> footSwitches,
           ProcessedSensorsInterface processedSensors, CapturePointCalculatorInterface capturePointCalculator, DoubleYoVariable t, double controlDT, double footHeight, YoVariableRegistry parentRegistry,
           DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      super(name + "State", name + "SwitchTime", MomentumBasedControllerState.class, t, registry);
      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = referenceFrames;
      this.twistCalculator = twistCalculator;
      this.bipedFeet = bipedFeet;
      this.bipedSupportPolygons = bipedSupportPolygons;
      this.controlDT = controlDT;
      this.desiredHeadingControlModule = new SimpleDesiredHeadingControlModule(0.0, controlDT, registry);
      this.footSwitches = footSwitches;
      this.gravity = -processedSensors.getGravityInWorldFrame().getZ();
      this.spatialAccelerationCalculator = new SpatialAccelerationCalculator(fullRobotModel.getElevator(), twistCalculator, gravity, true);
      this.capturePointCalculator = capturePointCalculator;

      for (RobotSide supportSide : RobotSide.values())
      {
         icpTrajectoryGenerators.put(supportSide,
                                     new ConstantCoPInstantaneousCapturePointTrajectory(supportSide, bipedSupportPolygons, gravity, controlDT, registry));
      }

//    SimpleDesiredFootstepCalculator simpleDesiredFootstepCalculator = new SimpleDesiredFootstepCalculator(referenceFrames.getAnkleZUpReferenceFrames(),
//          desiredHeadingControlModule, registry); // TODO: pass in

      SimpleWorldDesiredFootstepCalculator simpleDesiredFootstepCalculator =
         new SimpleWorldDesiredFootstepCalculator(referenceFrames.getAnkleZUpReferenceFrames(), desiredHeadingControlModule, registry, footHeight);
      simpleDesiredFootstepCalculator.setupParametersForR2InverseDynamics();
      this.desiredFootstepCalculator = simpleDesiredFootstepCalculator;
      upcomingSupportLeg.set(RobotSide.LEFT);

//    BoxDesiredFootstepCalculator boxDesiredFootstepCalculator = new BoxDesiredFootstepCalculator(referenceFrames.getAnkleZUpReferenceFrames(),
//          desiredHeadingControlModule, registry);
//    boxDesiredFootstepCalculator.setupParametersForR2();
//    this.desiredFootstepCalculator = boxDesiredFootstepCalculator;
//    upcomingSupportLeg.set(RobotSide.RIGHT);

//      this.centerOfMassHeightTrajectoryGenerator = new LinearCenterOfMassHeightTrajectoryGenerator(processedSensors, referenceFrames,
//              desiredHeadingControlModule.getDesiredHeadingFrame(), footHeight, parentRegistry);

//    this.centerOfMassHeightTrajectoryGenerator = new OrbitalEnergyCubicTrajectoryGenerator(processedSensors, referenceFrames,
//          desiredHeadingControlModule.getDesiredHeadingFrame(), footHeight, parentRegistry);

      this.centerOfMassHeightTrajectoryGenerator = new LinearFootstepCalculatorBasedCoMHeightTrajectoryGenerator(processedSensors, desiredFootstepCalculator, footHeight, desiredHeadingControlModule.getDesiredHeadingFrame(), registry);

      desiredICP = new YoFramePoint2d("desiredICP", "", ReferenceFrame.getWorldFrame(), registry);
      desiredICPVelocity = new YoFrameVector2d("desiredICPVelocity", "", ReferenceFrame.getWorldFrame(), registry);

      singleSupportICPGlideScaleFactor.set(0.8); // 1.0);
      FramePoint2d finalDesiredICPForDoubleSupportStance = getDoubleSupportFinalDesiredICPForDoubleSupportStance();
      finalDesiredICPForDoubleSupportStance.changeFrame(desiredICP.getReferenceFrame());

      for (RobotSide robotSide : RobotSide.values())
      {
         YoFramePoint desiredSwingFootPosition = new YoFramePoint("desired" + robotSide.getCamelCaseNameForMiddleOfExpression() + "SwingFootPosition", "",
                                                    worldFrame, registry);
         desiredFootPositions.put(robotSide, desiredSwingFootPosition);

         YoFrameVector desiredSwingFootVelocity = new YoFrameVector("desired" + robotSide.getCamelCaseNameForMiddleOfExpression() + "SwingFootVelocity", "",
                                                     worldFrame, registry);
         desiredFootVelocities.put(robotSide, desiredSwingFootVelocity);

         YoFrameVector desiredSwingFootAcceleration = new YoFrameVector("desired" + robotSide.getCamelCaseNameForMiddleOfExpression()
                                                         + "SwingFootAcceleration", "", worldFrame, registry);
         desiredFootAccelerations.put(robotSide, desiredSwingFootAcceleration);

         cartesianTrajectoryGenerators.put(robotSide,
                                           new FifthOrderWaypointCartesianTrajectoryGenerator(robotSide.getCamelCaseNameForStartOfExpression(), worldFrame,
                                              stepTime, waypointHeight, registry));

//       cartesianTrajectoryGenerators.put(robotSide,
//                                         new StraightUpThenParabolicCartesianTrajectoryGenerator(robotSide.getCamelCaseNameForStartOfExpression()
//                                            + "CartesianTrajectory", supportAnkleZUpFrame, straightUpAverageVelocity, parabolicTime, initialGroundClearance,
//                                               registry));

//        cartesianTrajectoryGenerators.put(robotSide,
//        new ParabolicCartesianTrajectoryGenerator(robotSide.getCamelCaseNameForStartOfExpression() + "CartesianTrajectory",
//        worldFrame, stepTime, 0.15, registry));

         desiredFootOrientations.put(robotSide, new YoFrameOrientation("desired" + robotSide.getCamelCaseNameForStartOfExpression() + "FootOrientation", "", ReferenceFrame.getWorldFrame(), registry));
         desiredFootAngularVelocities.put(robotSide,
                                          new YoFrameVector("desired" + robotSide.getCamelCaseNameForStartOfExpression() + "FootOmega",
                                             ReferenceFrame.getWorldFrame(), registry));
         desiredFootAngularAccelerations.put(robotSide,
                 new YoFrameVector("desired" + robotSide.getCamelCaseNameForStartOfExpression() + "FootOmegad", ReferenceFrame.getWorldFrame(), registry));
         nullspaceMultipliers.put(robotSide, new DoubleYoVariable(robotSide.getCamelCaseNameForStartOfExpression() + "NullspaceMultiplier", registry));

         trajectoryInitialized.put(robotSide, new BooleanYoVariable(robotSide.getCamelCaseNameForStartOfExpression() + "TrajectoryInitialized", registry));

         fixedAxisOfRotationControlModules.put(robotSide,
                 new FixedAxisOfRotationControlModule(robotSide.getCamelCaseNameForStartOfExpression() + "FootOrientation",
                    referenceFrames.getFootFrame(robotSide), worldFrame));
      }

      this.previousCoP = new FramePoint2d(ReferenceFrame.getWorldFrame());

      bagOfBalls = new BagOfBalls(100, registry, dynamicGraphicObjectsListRegistry);

      setUpStateMachine();
      parentRegistry.addChild(registry);

//    walk.set(true);
      liftUpHeels.set(true);
      heelsUpFootPitch.set(0.2);
   }

   private void setUpStateMachine()
   {
      DoubleSupportState doubleSupportState = new DoubleSupportState(null);

      for (RobotSide robotSide : RobotSide.values())
      {
         StateTransition toTransfer = new StateTransition(transferStateEnums.get(robotSide), new DoneWithDoubleSupportCondition(robotSide));
         doubleSupportState.addStateTransition(toTransfer);
      }

      addState(doubleSupportState);

      for (RobotSide robotSide : RobotSide.values())
      {
         State transferState = new DoubleSupportState(robotSide);
         StateTransition toDoubleSupport = new StateTransition(doubleSupportState.getStateEnum(), new StopWalkingCondition(), new ResetICPTrajectoryAction());
         transferState.addStateTransition(toDoubleSupport);
         StateTransition toSingleSupport = new StateTransition(singleSupportStateEnums.get(robotSide), new DoneWithTransferCondition(robotSide));
         transferState.addStateTransition(toSingleSupport);
         addState(transferState);

         State singleSupportState = new SingleSupportState(robotSide);
         StateTransition toTransfer = new StateTransition(transferStateEnums.get(robotSide.getOppositeSide()), new DoneWithSingleSupportCondition());
         singleSupportState.addStateTransition(toTransfer);
         addState(singleSupportState);
      }

      setCurrentState(doubleSupportState.getStateEnum());
   }

   public void packDesiredICP(FramePoint2d desiredICPToPack)
   {
      desiredICP.getFramePoint2dAndChangeFrame(desiredICPToPack);
   }

   public void packDesiredICPVelocity(FrameVector2d desiredICPVelocityToPack)
   {
      desiredICPVelocityToPack.set(desiredICPVelocity.getReferenceFrame(), 0.0, 0.0);
      desiredICPVelocity.getFrameVector2d(desiredICPVelocityToPack);
   }

   public RobotSide getSupportLeg()
   {
      return supportLeg.getEnumValue();
   }

   public RobotSide getPlantedLeg()
   {
      return plantedLeg.getEnumValue();
   }
   
   public RobotSide getUpcomingSupportLeg()
   {
      return upcomingSupportLeg.getEnumValue();
   }

   public FramePose getDesiredFootPose(RobotSide robotSide)
   {
      ReferenceFrame footFrame = referenceFrames.getFootFrame(robotSide);
      FramePoint footPosition = desiredFootPositions.get(robotSide).getFramePointCopy();
      footPosition.changeFrame(footFrame);

      Orientation footOrientation = desiredFootOrientations.get(robotSide).getFrameOrientationCopy();
      footOrientation.changeFrame(footFrame);

      FramePose ret = new FramePose(footPosition, footOrientation);

      return ret;
   }

   public Twist getDesiredFootTwist(RobotSide robotSide)
   {
      ReferenceFrame footFrame = referenceFrames.getFootFrame(robotSide);
      ReferenceFrame elevatorFrame = fullRobotModel.getElevatorFrame();
      FrameVector angularVelocity = desiredFootAngularVelocities.get(robotSide).getFrameVectorCopy();
      angularVelocity.changeFrame(footFrame);
      FrameVector linearVelocity = desiredFootVelocities.get(robotSide).getFrameVectorCopy();
      linearVelocity.changeFrame(footFrame);
      Twist ret = new Twist(footFrame, elevatorFrame, footFrame, linearVelocity.getVector(), angularVelocity.getVector());

      return ret;
   }

   public SpatialAccelerationVector getDesiredFootAcceleration(RobotSide robotSide)
   {
      ReferenceFrame footFrame = referenceFrames.getFootFrame(robotSide);
      ReferenceFrame elevatorFrame = fullRobotModel.getElevatorFrame();
      FrameVector angularAcceleration = desiredFootAngularAccelerations.get(robotSide).getFrameVectorCopy();
      angularAcceleration.changeFrame(footFrame);

      FrameVector originAcceleration = desiredFootAccelerations.get(robotSide).getFrameVectorCopy();
      originAcceleration.changeFrame(footFrame);
      Twist twistOfFootWithRespectToElevator = new Twist();
      twistCalculator.packTwistOfBody(twistOfFootWithRespectToElevator, fullRobotModel.getFoot(robotSide));
      twistOfFootWithRespectToElevator.changeBodyFrameNoRelativeTwist(footFrame);
      twistOfFootWithRespectToElevator.changeBaseFrameNoRelativeTwist(elevatorFrame);
      SpatialAccelerationVector ret = new SpatialAccelerationVector(footFrame, elevatorFrame, footFrame);
      ret.setBasedOnOriginAcceleration(angularAcceleration, originAcceleration, twistOfFootWithRespectToElevator);

      return ret;
   }

   public double getNullspaceMultiplier(RobotSide robotSide)
   {
      return nullspaceMultipliers.get(robotSide).getDoubleValue();
   }

   public double getDesiredCoMHeight()
   {
      return desiredCoMHeight.getDoubleValue();
   }

   public double getDesiredCoMHeightVelocity()
   {
      return desiredCoMHeightVelocity.getDoubleValue();
   }

   public double getDesiredCoMHeightAcceleration()
   {
      return desiredCoMHeightAcceleration.getDoubleValue();
   }

   private class DoubleSupportState extends State
   {
      private final RobotSide transferToSide;
      private final YoFramePoint2d initialDesiredICP;

      public DoubleSupportState(RobotSide transferToSide)
      {
         super((transferToSide == null) ? MomentumBasedControllerState.DOUBLE_SUPPORT : transferStateEnums.get(transferToSide));
         this.transferToSide = transferToSide;
         String prefix = (transferToSide == null) ? "doubleSupport" : transferToSide.getCamelCaseNameForStartOfExpression() + "Transfer";
         ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame(); // (transferToSide == null) ? referenceFrames.getMidFeetZUpFrame() : referenceFrames.getAnkleZUpFrame(transferToSide);
         initialDesiredICP = new YoFramePoint2d(prefix + "InitialDesiredICP", "", referenceFrame, registry);
      }

      @Override
      public void doAction()
      {
         evaluateCoMHeightTrajectory();

         RobotSide orientationControlSide = upcomingSupportLeg.getEnumValue().getOppositeSide();

         for (RobotSide robotSide : RobotSide.values())
         {
            BipedFootInterface bipedFoot = bipedFeet.get(robotSide);
//            if (liftUpHeels.getBooleanValue() && orientationControlSide == robotSide)
            if (liftUpHeels.getBooleanValue())
            {
               bipedFoot.setFootPolygonInUse(FootPolygonEnum.ONTOES);
               bipedFoot.setShift(1.0);

//               double t = timeInCurrentState();
//               double amplitude = 0.2;
//               double offset = 0.05;
//               double omega = Math.PI;
//               double q = amplitude * (1.0 - Math.cos(omega * t)) + offset;
//               double qd = amplitude * omega * Math.sin(omega * t);
//               double qdd = amplitude * omega * omega * Math.cos(omega * t);
//
//               double q = 0.2; // 2;
//               double qd = 0.0;
//               double qdd = 0.0;
//
//               fixedAxisOfRotationControlModules.get(robotSide).compute(q, qd, qdd);
//
//               FramePose desiredPose = new FramePose(worldFrame);
//               Twist desiredTwist = new Twist();
//               SpatialAccelerationVector feedForward = new SpatialAccelerationVector();
//
//               fixedAxisOfRotationControlModules.get(robotSide).pack(desiredPose, desiredTwist, feedForward);
//               
//               desiredFootPositions.get(robotSide).set(desiredPose.getPositionInFrame(desiredFootPositions.get(robotSide).getReferenceFrame()));
//               desiredFootVelocities.get(robotSide).set(desiredTwist.getBodyOriginLinearVelocityInBaseFrame());
//               desiredFootAccelerations.get(robotSide).set(0.0, 0.0, 0.0); // TODO
//
//               desiredFootOrientations.get(robotSide).set(desiredPose.getOrientation());
//               desiredFootAngularVelocities.get(robotSide).set(desiredTwist.getAngularVelocityInBaseFrame());
//               desiredFootAngularAccelerations.get(robotSide).set(0.0, 0.0, 0.0); // TODO

               setDesiredFootPosVelAccForSupportSide(robotSide, heelsUpFootPitch.getDoubleValue());
            }
            else
            {
               bipedFoot.setFootPolygonInUse(FootPolygonEnum.FLAT);
               setDesiredFootPosVelAccForSupportSide(robotSide, 0.0);
            }
         }
         
         if ((transferToSide != null) &&!icpTrajectoryGenerators.get(transferToSide.getOppositeSide()).isDone())
         {
            FramePoint2d desiredICPLocal = new FramePoint2d(desiredICP.getReferenceFrame());
            FrameVector2d desiredICPVelocityLocal = new FrameVector2d(desiredICPVelocity.getReferenceFrame());
            icpTrajectoryGenerators.get(transferToSide.getOppositeSide()).pack(desiredICPLocal, desiredICPVelocityLocal, comHeight);
            desiredICP.set(desiredICPLocal);
            desiredICPVelocity.set(desiredICPVelocityLocal);
         }
         else
         {
            FramePoint2d finalDesiredICP;

            if (transferToSide == null)
               finalDesiredICP = getDoubleSupportFinalDesiredICPForDoubleSupportStance();
            else
               finalDesiredICP = getDoubleSupportFinalDesiredICPForWalking(transferToSide);
            finalDesiredICP.changeFrame(initialDesiredICP.getReferenceFrame());
            FrameVector2d icpMove = new FrameVector2d(finalDesiredICP);
            icpMove.sub(initialDesiredICP.getFramePoint2dCopy());
            double desiredICPVelocityMagnitude = icpMove.length() / doubleSupportTime;

            double epsilon = desiredICPVelocityMagnitude * controlDT;
            finalDesiredICP.changeFrame(desiredICP.getReferenceFrame());

            if (desiredICP.distance(finalDesiredICP) > epsilon)
            {
               linearlyMoveDesiredICP(desiredICPVelocityMagnitude, finalDesiredICP);
               transferICPTrajectoryDone.set(false);
            }
            else
            {
               FramePoint2d desiredCapturePoint = finalDesiredICP;
               desiredCapturePoint.changeFrame(desiredICP.getReferenceFrame());
               desiredICP.set(desiredCapturePoint);
               desiredICPVelocity.set(0.0, 0.0);
               transferICPTrajectoryDone.set(true);
            }
         }
      }

      @Override
      public void doTransitionIntoAction()
      {
         if (liftUpHeels.getBooleanValue())
         {
            for (RobotSide robotSide : RobotSide.values())
            {
               BipedFootInterface bipedFoot = bipedFeet.get(robotSide);
               bipedFoot.setFootPolygonInUse(FootPolygonEnum.ONTOES);
               bipedFoot.setShift(1.0);
            }
         }

         setSupportLeg(null);
         transferICPTrajectoryDone.set(false);

//         for (RobotSide robotSide : RobotSide.values())
//         {
//            FrameVector axisOfRotation = new FrameVector(referenceFrames.getFootFrame(robotSide), 0.0, 1.0, 0.0);
//            FramePoint offset = bipedFeet.get(robotSide).getToePointsCopy()[0];
//            fixedAxisOfRotationControlModules.get(robotSide).initialize(axisOfRotation, offset);
//         }

         if (transferToSide == null)
         {
            capturePointCalculator.computeCapturePoint(getSupportLeg());
            FramePoint2d actualICP = capturePointCalculator.getCapturePoint2dInFrame(initialDesiredICP.getReferenceFrame());
            desiredICP.set(actualICP);
         }

         FramePoint2d desiredICPLocal = desiredICP.getFramePoint2dCopy();
         desiredICPLocal.changeFrame(initialDesiredICP.getReferenceFrame());
         initialDesiredICP.set(desiredICPLocal);    

         if (transferToSide == null)
            centerOfMassHeightTrajectoryGenerator.initialize(null);
         else
         {
            desiredFootstepCalculator.initializeDesiredFootstep(transferToSide);
            centerOfMassHeightTrajectoryGenerator.initialize(upcomingSupportLeg.getEnumValue());
         }

         for (RobotSide robotSide : RobotSide.values())
         {
            nullspaceMultipliers.get(robotSide).set(0.0);
         }

//         if (transferToSide != null)
//         {
//            icpTrajectoryGenerators.get(transferToSide.getOppositeSide()).changeSideToUseForReferenceFrame(transferToSide);
//         }
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
      private final ReferenceFrame supportAnkleZUpFrame;

      private final FramePoint positionToPack;
      private final FrameVector velocityToPack;
      private final FrameVector accelerationToPack;

      private int counter = 0;

      public SingleSupportState(RobotSide robotSide)
      {
         super(singleSupportStateEnums.get(robotSide));
         this.swingSide = robotSide.getOppositeSide();
         supportAnkleZUpFrame = referenceFrames.getAnkleZUpFrame(swingSide.getOppositeSide());
         this.positionToPack = new FramePoint(supportAnkleZUpFrame);
         this.velocityToPack = new FrameVector(supportAnkleZUpFrame);
         this.accelerationToPack = new FrameVector(supportAnkleZUpFrame);
      }

      @Override
      public void doAction()
      {
         evaluateCoMHeightTrajectory();

         RobotSide supportSide = swingSide.getOppositeSide();
//         setDesiredFootPosVelAccForSupportSide(supportSide);

         CartesianTrajectoryGenerator cartesianTrajectoryGenerator = cartesianTrajectoryGenerators.get(swingSide);
         if (!cartesianTrajectoryGenerator.isDone() && trajectoryInitialized.get(swingSide).getBooleanValue())
         {
            cartesianTrajectoryGenerator.computeNextTick(positionToPack, velocityToPack, accelerationToPack, controlDT);
            FramePoint2d desiredICPLocal = new FramePoint2d(desiredICP.getReferenceFrame());
            FrameVector2d desiredICPVelocityLocal = new FrameVector2d(desiredICPVelocity.getReferenceFrame());
            icpTrajectoryGenerators.get(supportSide).pack(desiredICPLocal, desiredICPVelocityLocal, comHeight);
            desiredICP.set(desiredICPLocal);
            desiredICPVelocity.set(desiredICPVelocityLocal);
         }
         else
         {
            positionToPack.setToZero(referenceFrames.getFootFrame(swingSide));
            velocityToPack.setToZero(referenceFrames.getFootFrame(swingSide));
            accelerationToPack.setToZero(referenceFrames.getFootFrame(swingSide));

            // don't change desiredICP
            desiredICPVelocity.set(0.0, 0.0);
         }

         positionToPack.changeFrame(desiredFootPositions.get(swingSide).getReferenceFrame());
         velocityToPack.changeFrame(desiredFootVelocities.get(swingSide).getReferenceFrame());
         accelerationToPack.changeFrame(desiredFootAccelerations.get(swingSide).getReferenceFrame());

         desiredFootPositions.get(swingSide).set(positionToPack);
         desiredFootVelocities.get(swingSide).set(velocityToPack);
         desiredFootAccelerations.get(swingSide).set(accelerationToPack);

//         desiredFootOrientations.get(swingSide).setYawPitchRoll(0.0, 0.0, 0.0);    // TODO
//         desiredFootAngularVelocities.get(swingSide).set(0.0, 0.0, 0.0);
//         desiredFootAngularAccelerations.get(swingSide).set(0.0, 0.0, 0.0);

         counter++;

         if (counter >= 100)
         {
            positionToPack.changeFrame(worldFrame);
            bagOfBalls.setBallLoop(positionToPack);
            counter = 0;
         }
      }

      @Override
      public void doTransitionIntoAction()
      {
         RobotSide supportSide = swingSide.getOppositeSide();
         setSupportLeg(supportSide);

         centerOfMassHeightTrajectoryGenerator.initialize(upcomingSupportLeg.getEnumValue());
         nullspaceMultipliers.get(swingSide).set(swingNullspaceMultiplier);
         nullspaceMultipliers.get(supportSide).set(0.0);
         trajectoryInitialized.get(swingSide).set(false);

         setDesiredFootPosVelAccForSupportSide(supportSide, heelsUpFootPitch.getDoubleValue());
      }

      @Override
      public void doTransitionOutOfAction()
      {
         upcomingSupportLeg.set(upcomingSupportLeg.getEnumValue().getOppositeSide());
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
         boolean doubleSupportTimeHasPassed = timeInCurrentState() > doubleSupportTime;
         boolean transferringToThisRobotSide = robotSide == upcomingSupportLeg.getEnumValue();

         return walk.getBooleanValue() && transferringToThisRobotSide && doubleSupportTimeHasPassed;
      }
   }


   public class DoneWithTransferCondition implements StateTransitionCondition
   {
      private final RobotSide robotSide;

      public DoneWithTransferCondition(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }

      public boolean checkCondition()
      {
      // FIXME: terrible condition
         FrameConvexPolygon2d footPolygon = bipedFeet.get(robotSide).getFootPolygonInUseInAnkleZUp();
//         boolean capturePointInsideFootPolygon = footPolygon.isPointInside(capturePoint.changeFrameCopy(footPolygon.getReferenceFrame()));
//         boolean copInsideFootPolygon = footPolygon.isPointInside(previousCoP.changeFrameCopy(footPolygon.getReferenceFrame()));
//
         capturePointCalculator.computeCapturePoint(getSupportLeg());
         FramePoint2d capturePointLocal = capturePointCalculator.getCapturePoint2dInFrame(footPolygon.getReferenceFrame());
         boolean inFrontOfFoot = true;
         for (FramePoint2d point : footPolygon.getClockwiseOrderedListOfFramePoints())
         {
            if (point.getX() > capturePointLocal.getX())
               inFrontOfFoot = false;
         }
         
         return (transferICPTrajectoryDone.getBooleanValue() || !footSwitches.get(robotSide.getOppositeSide()).hasFootHitGround()) && inFrontOfFoot;
      }
   }


   public class DoneWithSingleSupportCondition implements StateTransitionCondition
   {
      public boolean checkCondition()
      {
         RobotSide swingSide = supportLeg.getEnumValue().getOppositeSide();
         double minimumSwingFraction = 0.5;
         double minimumSwingTime = cartesianTrajectoryGenerators.get(swingSide).getFinalTime() * minimumSwingFraction;
         boolean footHitGround = (timeInCurrentState() > minimumSwingTime) && footSwitches.get(swingSide).hasFootHitGround();

         return trajectoryInitialized.get(swingSide).getBooleanValue() && (cartesianTrajectoryGenerators.get(swingSide).isDone() || footHitGround);
      }
   }


   public class StopWalkingCondition implements StateTransitionCondition
   {
      public boolean checkCondition()
      {
         return !walk.getBooleanValue();
      }
   }


   public class ResetICPTrajectoryAction implements StateTransitionAction
   {
      public void doTransitionAction()
      {
         for (RobotSide robotSide : RobotSide.values())
         {
            icpTrajectoryGenerators.get(robotSide).reset();
         }
      }
   }

   public void setPreviousCoP(FramePoint2d previousCoP)
   {
      previousCoP.changeFrame(this.previousCoP.getReferenceFrame());
      this.previousCoP.set(previousCoP);
   }

   public void setCoMHeight(double comHeight)
   {
      this.comHeight = comHeight;
   }

   private void linearlyMoveDesiredICP(double desiredICPVelocityMagnitude, FramePoint2d finalDesiredICP)
   {
      double omega0 = Math.sqrt(gravity / comHeight);

      finalDesiredICP.changeFrame(desiredICPVelocity.getReferenceFrame());
      desiredICPVelocity.set(finalDesiredICP);
      desiredICPVelocity.sub(desiredICP.getFramePoint2dCopy());
      desiredICPVelocity.normalize();
      FramePoint2d capturePoint = capturePointCalculator.getCapturePoint2dInFrame(desiredICPVelocity.getReferenceFrame());
      double maxDesiredICPVelocityMagnitude = MaximumICPVelocityCalculator.computeMaximumICPVelocity(bipedSupportPolygons.getSupportPolygonInMidFeetZUp(),
                                                 capturePoint, desiredICPVelocity.getFrameVector2dCopy(), omega0);
      desiredICPVelocityMagnitude = MathTools.clipToMinMax(desiredICPVelocityMagnitude, Double.NEGATIVE_INFINITY, maxDesiredICPVelocityMagnitude);
      desiredICPVelocity.scale(desiredICPVelocityMagnitude);
      integrateDesiredICPVelocity();
   }

   private void integrateDesiredICPVelocity()
   {
      FrameVector2d desiredICPDisplacement = desiredICPVelocity.getFrameVector2dCopy();
      desiredICPDisplacement.scale(controlDT);
      desiredICP.add(desiredICPDisplacement);
   }

   private FramePoint2d getDoubleSupportFinalDesiredICPForDoubleSupportStance()
   {
    return bipedSupportPolygons.getSupportPolygonInMidFeetZUp().getCentroidCopy();
//      FramePoint2d ret = new FramePoint2d(referenceFrames.getMidFeetZUpFrame());
//      double extraX = 0.2;
//      FrameVector2d offset = new FrameVector2d(desiredHeadingControlModule.getDesiredHeadingFrame(), extraX, 0.0);
//      offset.changeFrame(ret.getReferenceFrame());
//      ret.add(offset);
//      return ret;
   }

   private FramePoint2d getDoubleSupportFinalDesiredICPForWalking(RobotSide supportSide)
   {
      FramePoint2d ret = bipedFeet.get(supportSide).getFootPolygonInUseInAnkleZUp().getCentroidCopy();
      double extraX = 0.005;
      double extraY = supportSide.negateIfLeftSide(0.01);
      FrameVector2d offset = new FrameVector2d(desiredHeadingControlModule.getDesiredHeadingFrame(), extraX, extraY);
      offset.changeFrame(ret.getReferenceFrame());
      ret.add(offset);
      
//      FramePoint2d ret = bipedFeet.get(supportSide).getFlatFootPolygonInAnkleZUp().getCentroidCopy();
//      FrameVector2d offset = new FrameVector2d(ret.getReferenceFrame(), 0.0, 0.0);
//      ret.add(offset);

      return ret;
   }

   private FramePoint2d getSingleSupportFinalDesiredICPForWalking(FramePoint finalDesiredStepLocation, RobotSide swingSide)
   {
      ReferenceFrame referenceFrame = desiredHeadingControlModule.getDesiredHeadingFrame(); // referenceFrames.getAnkleZUpFrame(swingSide.getOppositeSide());
      FramePoint2d desiredICPLocal = desiredICP.getFramePoint2dCopy();
      desiredICPLocal.changeFrame(referenceFrame);

      FramePoint2d finalDesiredStepLocation2d = finalDesiredStepLocation.toFramePoint2d();
      finalDesiredStepLocation2d.changeFrame(referenceFrame);
      FrameVector2d stepOffset = new FrameVector2d(finalDesiredStepLocation2d);
      FrameVector2d finalDesiredICPOffset = new FrameVector2d(getDoubleSupportFinalDesiredICPForWalking(swingSide));    // TODO: do something about expected orientation of swing foot
      finalDesiredICPOffset.changeFrame(referenceFrame);
      finalDesiredICPOffset.add(stepOffset);
      finalDesiredICPOffset.sub(desiredICPLocal);

      finalDesiredICPOffset.scale(singleSupportICPGlideScaleFactor.getDoubleValue());
      FramePoint2d finalDesiredICP = new FramePoint2d(desiredICPLocal);
      finalDesiredICP.add(finalDesiredICPOffset);

      return finalDesiredICP;
   }

   private void evaluateCoMHeightTrajectory()
   {
      centerOfMassHeightTrajectoryGenerator.compute();
      desiredCoMHeight.set(centerOfMassHeightTrajectoryGenerator.getDesiredCenterOfMassHeight());
      desiredCoMHeightVelocity.set(centerOfMassHeightTrajectoryGenerator.getDesiredCenterOfMassHeightVelocity());
      desiredCoMHeightAcceleration.set(centerOfMassHeightTrajectoryGenerator.getDesiredCenterOfMassHeightAcceleration());
   }

   private void setDesiredFootPosVelAccForSupportSide(RobotSide supportSide, double footPitch)
   {
      FramePoint desiredFootPosition = new FramePoint(referenceFrames.getFootFrame(supportSide));
      desiredFootPosition.changeFrame(desiredFootPositions.get(supportSide).getReferenceFrame());
      desiredFootPositions.get(supportSide).set(desiredFootPosition);
      desiredFootVelocities.get(supportSide).set(0.0, 0.0, 0.0);
      desiredFootAccelerations.get(supportSide).set(0.0, 0.0, 0.0);

      desiredFootOrientations.get(supportSide).setYawPitchRoll(0.0, footPitch, 0.0);
      desiredFootAngularVelocities.get(supportSide).set(0.0, 0.0, 0.0);
      desiredFootAngularAccelerations.get(supportSide).set(0.0, 0.0, 0.0);
   }

   public boolean isSwingFoot(RobotSide robotSide)
   {
      RobotSide supportLeg = getSupportLeg();
      if (supportLeg == null)
         return false;
      else
         return robotSide != supportLeg;
   }

   private void setSupportLeg(RobotSide supportLeg)
   {
      this.supportLeg.set(supportLeg);

      for (RobotSide robotSide : RobotSide.values())
      {
         boolean isSupportingFoot = (supportLeg == robotSide) || (supportLeg == null);
         bipedFeet.get(robotSide).setIsSupportingFoot(isSupportingFoot);
      }

      bipedSupportPolygons.update(bipedFeet.get(RobotSide.LEFT), bipedFeet.get(RobotSide.RIGHT));
   }

   public void initializeTrajectory(RobotSide swingSide, SpatialAccelerationVector taskSpaceAcceleration)
   {
      RobotSide supportSide = swingSide.getOppositeSide();
      ReferenceFrame swingAnkleZUpFrame = referenceFrames.getAnkleZUpFrame(swingSide);
      ReferenceFrame trajectoryGeneratorFrame = cartesianTrajectoryGenerators.get(swingSide).getReferenceFrame();
      ReferenceFrame swingFootFrame = referenceFrames.getFootFrame(swingSide);
      FramePoint initialPosition = new FramePoint(swingAnkleZUpFrame);
      initialPosition.changeFrame(trajectoryGeneratorFrame);

      Twist footTwist = new Twist();
      twistCalculator.packTwistOfBody(footTwist, fullRobotModel.getFoot(swingSide));

      SpatialAccelerationVector taskSpaceAccelerationWithRespectToWorld = new SpatialAccelerationVector();
      spatialAccelerationCalculator.compute();
      spatialAccelerationCalculator.packAccelerationOfBody(taskSpaceAccelerationWithRespectToWorld, fullRobotModel.getPelvis());

      Twist pelvisTwist = new Twist();
      twistCalculator.packTwistOfBody(pelvisTwist, fullRobotModel.getPelvis());
      taskSpaceAccelerationWithRespectToWorld.changeFrame(taskSpaceAccelerationWithRespectToWorld.getBaseFrame(), pelvisTwist, pelvisTwist);

      Twist footPelvisTwist = new Twist(pelvisTwist);
      footPelvisTwist.invert();
      footPelvisTwist.changeFrame(footTwist.getExpressedInFrame());
      footPelvisTwist.add(footTwist);

      taskSpaceAcceleration.changeFrame(taskSpaceAccelerationWithRespectToWorld.getExpressedInFrame(), footTwist, footPelvisTwist);
      taskSpaceAccelerationWithRespectToWorld.add(taskSpaceAcceleration);
      FramePoint swingAnkle = new FramePoint(swingFootFrame);
      swingAnkle.changeFrame(taskSpaceAccelerationWithRespectToWorld.getBaseFrame());
      footTwist.changeFrame(taskSpaceAccelerationWithRespectToWorld.getExpressedInFrame());

      FrameVector initialAcceleration = new FrameVector(worldFrame);
      taskSpaceAccelerationWithRespectToWorld.packAccelerationOfPointFixedInBodyFrame(footTwist, swingAnkle, initialAcceleration);
      initialAcceleration.changeFrame(trajectoryGeneratorFrame);

      initialAcceleration.setToZero(trajectoryGeneratorFrame);    // TODO

      footTwist.changeFrame(swingFootFrame);
      FrameVector initialVelocity = new FrameVector(trajectoryGeneratorFrame);
      footTwist.packLinearPart(initialVelocity);
      initialVelocity.changeFrame(trajectoryGeneratorFrame);

      Footstep desiredFootstep = desiredFootstepCalculator.updateAndGetDesiredFootstep(supportSide);
      FramePoint finalDesiredStepLocation = desiredFootstep.getFootstepPositionInFrame(trajectoryGeneratorFrame);
      finalDesiredStepLocation.setZ(finalDesiredStepLocation.getZ() + 5e-3); // FIXME: hack!!!

      FrameVector finalDesiredVelocity = new FrameVector(trajectoryGeneratorFrame);

      CartesianTrajectoryGenerator cartesianTrajectoryGenerator = cartesianTrajectoryGenerators.get(swingSide);
      cartesianTrajectoryGenerator.initialize(initialPosition, initialVelocity, initialAcceleration, finalDesiredStepLocation, finalDesiredVelocity);

      FramePoint2d finalDesiredICP = getSingleSupportFinalDesiredICPForWalking(finalDesiredStepLocation, swingSide);
      icpTrajectoryGenerators.get(supportSide).initialize(desiredICP.getFramePoint2dCopy(), finalDesiredICP,
                                  cartesianTrajectoryGenerators.get(swingSide).getFinalTime(), comHeight);

      nullspaceMultipliers.get(swingSide).set(0.0);
      trajectoryInitialized.get(swingSide).set(true);

      // TODO: orientation stuff

      doAction();    // computes trajectory and stores results in YoFrameVectors and Points.
   }

   public boolean trajectoryInitialized(RobotSide robotSide)
   {
      return trajectoryInitialized.get(robotSide).getBooleanValue();
   }
}
