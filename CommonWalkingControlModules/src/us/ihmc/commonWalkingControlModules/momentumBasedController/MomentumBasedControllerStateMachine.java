package us.ihmc.commonWalkingControlModules.momentumBasedController;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedFootInterface;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.calculators.MaximumICPVelocityCalculator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.desiredFootStep.SimpleDesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.SimpleDesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.commonWalkingControlModules.trajectories.CartesianTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.CenterOfMassHeightTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantCoPInstantaneousCapturePointTrajectory;
import us.ihmc.commonWalkingControlModules.trajectories.LinearCenterOfMassHeightTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.StraightUpThenParabolicCartesianTrajectoryGenerator;
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
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
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

   private final FullRobotModel fullRobotModel;
   private final CommonWalkingReferenceFrames referenceFrames;
   private final TwistCalculator twistCalculator;
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
   private final EnumYoVariable<RobotSide> upcomingSupportLeg = EnumYoVariable.create("upcomingSupportLeg", RobotSide.class, registry);

   private final SideDependentList<YoFramePoint> desiredFootPositions = new SideDependentList<YoFramePoint>();
   private final SideDependentList<YoFrameVector> desiredFootVelocities = new SideDependentList<YoFrameVector>();
   private final SideDependentList<YoFrameVector> desiredFootAccelerations = new SideDependentList<YoFrameVector>();

   private final SideDependentList<Orientation> desiredFootOrientations = new SideDependentList<Orientation>();
   private final SideDependentList<YoFrameVector> desiredFootAngularVelocities = new SideDependentList<YoFrameVector>();
   private final SideDependentList<YoFrameVector> desiredFootAngularAccelerations = new SideDependentList<YoFrameVector>();

   private final DoubleYoVariable desiredCoMHeight = new DoubleYoVariable("desiredCoMHeight", registry);
   private final DoubleYoVariable desiredCoMHeightVelocity = new DoubleYoVariable("desiredCoMHeightVelocity", registry);
   private final DoubleYoVariable desiredCoMHeightAcceleration = new DoubleYoVariable("desiredCoMHeightAcceleration", registry);

   private final FramePoint2d capturePoint;
   private final FramePoint2d previousCoP;
   private final double doubleSupportTime = 0.9;
   private final double straightUpAverageVelocity = 3.5;
   private final double parabolicTime = 0.75;
   private final double initialGroundClearance = 0.2; // 0.15;

   private final DoubleYoVariable singleSupportICPGlideScaleFactor = new DoubleYoVariable("singleSupportICPGlideScaleFactor", registry);
   private final BooleanYoVariable walk = new BooleanYoVariable("walk", registry);

   private double comHeight;
   private double gravity;

   public MomentumBasedControllerStateMachine(FullRobotModel fullRobotModel, CommonWalkingReferenceFrames referenceFrames, TwistCalculator twistCalculator,
           SideDependentList<BipedFootInterface> bipedFeet, BipedSupportPolygons bipedSupportPolygons, SideDependentList<FootSwitchInterface> footSwitches,
           ProcessedSensorsInterface processedSensors, DoubleYoVariable t, double controlDT, double footHeight, YoVariableRegistry parentRegistry)
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

      for (RobotSide supportSide : RobotSide.values())
      {
         icpTrajectoryGenerators.put(supportSide, new ConstantCoPInstantaneousCapturePointTrajectory(supportSide, bipedSupportPolygons, gravity, controlDT));
      }

      SimpleDesiredFootstepCalculator simpleDesiredFootstepCalculator = new SimpleDesiredFootstepCalculator(referenceFrames.getAnkleZUpReferenceFrames(),
                                                                           desiredHeadingControlModule, registry);    // TODO: pass in
      simpleDesiredFootstepCalculator.setupParametersForR2InverseDynamics();
      this.desiredFootstepCalculator = simpleDesiredFootstepCalculator;

//    BoxDesiredFootstepCalculator boxDesiredFootstepCalculator = new BoxDesiredFootstepCalculator(referenceFrames.getAnkleZUpReferenceFrames(),
//                                                                   desiredHeadingControlModule, registry);
//    boxDesiredFootstepCalculator.setupParametersForR2();
//    this.desiredFootstepCalculator = boxDesiredFootstepCalculator;

      this.centerOfMassHeightTrajectoryGenerator = new LinearCenterOfMassHeightTrajectoryGenerator(processedSensors, referenceFrames,
              desiredHeadingControlModule.getDesiredHeadingFrame(), footHeight, parentRegistry);

//    this.centerOfMassHeightTrajectoryGenerator = new OrbitalEnergyCubicTrajectoryGenerator(processedSensors, referenceFrames,
//          desiredHeadingControlModule.getDesiredHeadingFrame(), footHeight, parentRegistry);

      desiredICP = new YoFramePoint2d("desiredICP", "", ReferenceFrame.getWorldFrame(), registry);
      desiredICPVelocity = new YoFrameVector2d("desiredICPVelocity", "", ReferenceFrame.getWorldFrame(), registry);

      singleSupportICPGlideScaleFactor.set(1.0);
      FramePoint2d finalDesiredICPForDoubleSupportStance = getDoubleSupportFinalDesiredICPForDoubleSupportStance();
      finalDesiredICPForDoubleSupportStance.changeFrame(desiredICP.getReferenceFrame());
      desiredICP.set(finalDesiredICPForDoubleSupportStance);

      for (RobotSide robotSide : RobotSide.values())
      {
         ReferenceFrame supportAnkleZUpFrame = referenceFrames.getAnkleZUpFrame(robotSide.getOppositeSide());
         YoFramePoint desiredSwingFootPosition = new YoFramePoint("desired" + robotSide.getCamelCaseNameForMiddleOfExpression() + "SwingFootPosition", "",
                                                    supportAnkleZUpFrame, registry);
         desiredFootPositions.put(robotSide, desiredSwingFootPosition);

         YoFrameVector desiredSwingFootVelocity = new YoFrameVector("desired" + robotSide.getCamelCaseNameForMiddleOfExpression() + "SwingFootVelocity", "",
                                                     supportAnkleZUpFrame, registry);
         desiredFootVelocities.put(robotSide, desiredSwingFootVelocity);

         YoFrameVector desiredSwingFootAcceleration = new YoFrameVector("desired" + robotSide.getCamelCaseNameForMiddleOfExpression()
                                                         + "SwingFootAcceleration", "", supportAnkleZUpFrame, registry);
         desiredFootAccelerations.put(robotSide, desiredSwingFootAcceleration);

         cartesianTrajectoryGenerators.put(robotSide,
                                           new StraightUpThenParabolicCartesianTrajectoryGenerator(robotSide.getCamelCaseNameForStartOfExpression()
                                              + "CartesianTrajectory", supportAnkleZUpFrame, straightUpAverageVelocity, parabolicTime, initialGroundClearance,
                                                 registry));

         // cartesianTrajectoryGenerators.put(robotSide,
         // new ParabolicCartesianTrajectoryGenerator(robotSide.getCamelCaseNameForStartOfExpression() + "CartesianTrajectory",
         // supportAnkleZUpFrame, parabolicTime, initialGroundClearance, registry));

         desiredFootOrientations.put(robotSide, new Orientation(ReferenceFrame.getWorldFrame()));
         desiredFootAngularVelocities.put(robotSide,
                                          new YoFrameVector("desired" + robotSide.getCamelCaseNameForStartOfExpression() + "FootOmega",
                                             ReferenceFrame.getWorldFrame(), registry));
         desiredFootAngularAccelerations.put(robotSide,
                 new YoFrameVector("desired" + robotSide.getCamelCaseNameForStartOfExpression() + "FootOmegad", ReferenceFrame.getWorldFrame(), registry));
      }

      this.capturePoint = new FramePoint2d(ReferenceFrame.getWorldFrame());
      this.previousCoP = new FramePoint2d(ReferenceFrame.getWorldFrame());
      upcomingSupportLeg.set(RobotSide.LEFT);
      walk.set(true);

      setUpStateMachine();
      parentRegistry.addChild(registry);
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

   public RobotSide getUpcomingSupportLeg()
   {
      return upcomingSupportLeg.getEnumValue();
   }

   public FramePose getDesiredFootPose(RobotSide robotSide)
   {
      ReferenceFrame footFrame = referenceFrames.getFootFrame(robotSide);
      FramePoint footPosition = desiredFootPositions.get(robotSide).getFramePointCopy();
      footPosition.changeFrame(footFrame);

      Orientation footOrientation = desiredFootOrientations.get(robotSide).changeFrameCopy(footFrame);

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
         ReferenceFrame referenceFrame = (transferToSide == null) ? referenceFrames.getMidFeetZUpFrame() : referenceFrames.getAnkleZUpFrame(transferToSide);
         initialDesiredICP = new YoFramePoint2d(prefix + "InitialDesiredICP", "", referenceFrame, registry);
      }

      @Override
      public void doAction()
      {
         evaluateCoMHeightTrajectory();

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
               linearlyMoveDesiredICP(desiredICPVelocityMagnitude, finalDesiredICP);
            else
            {
               FramePoint2d desiredCapturePoint = finalDesiredICP;
               desiredCapturePoint.changeFrame(desiredICP.getReferenceFrame());
               desiredICP.set(desiredCapturePoint);
               desiredICPVelocity.set(0.0, 0.0);
            }
         }

         for (RobotSide robotSide : RobotSide.values())
         {
            desiredFootOrientations.get(robotSide).setYawPitchRoll(0.0, 0.0, 0.0);
            desiredFootAngularVelocities.get(robotSide).set(0.0, 0.0, 0.0);
            desiredFootAngularAccelerations.get(robotSide).set(0.0, 0.0, 0.0);
         }
      }

      @Override
      public void doTransitionIntoAction()
      {
         setSupportLeg(null);

         for (RobotSide robotSide : RobotSide.values())
         {
            setDesiredFootPosVelAccForSupportSide(robotSide);
         }

         FramePoint2d desiredICPLocal = desiredICP.getFramePoint2dCopy();
         desiredICPLocal.changeFrame(initialDesiredICP.getReferenceFrame());
         initialDesiredICP.set(desiredICPLocal);
         if (transferToSide == null)
            centerOfMassHeightTrajectoryGenerator.initialize(null);
         else
            centerOfMassHeightTrajectoryGenerator.initialize(upcomingSupportLeg.getEnumValue());
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
      private final ReferenceFrame swingAnkleZUpFrame;
      private final ReferenceFrame supportAnkleZUpFrame;

      private final FramePoint positionToPack;
      private final FrameVector velocityToPack;
      private final FrameVector accelerationToPack;

      public SingleSupportState(RobotSide robotSide)
      {
         super(singleSupportStateEnums.get(robotSide));
         this.swingSide = robotSide.getOppositeSide();
         swingAnkleZUpFrame = referenceFrames.getAnkleZUpFrame(swingSide);
         supportAnkleZUpFrame = referenceFrames.getAnkleZUpFrame(swingSide.getOppositeSide());
         this.positionToPack = new FramePoint(supportAnkleZUpFrame);
         this.velocityToPack = new FrameVector(supportAnkleZUpFrame);
         this.accelerationToPack = new FrameVector(supportAnkleZUpFrame);
      }

      @Override
      public void doAction()
      {
         evaluateCoMHeightTrajectory();

         CartesianTrajectoryGenerator cartesianTrajectoryGenerator = cartesianTrajectoryGenerators.get(swingSide);
         if (!cartesianTrajectoryGenerator.isDone())
         {
            cartesianTrajectoryGenerator.computeNextTick(positionToPack, velocityToPack, accelerationToPack, controlDT);
            desiredFootPositions.get(swingSide).set(positionToPack);
            desiredFootVelocities.get(swingSide).set(velocityToPack);
            desiredFootAccelerations.get(swingSide).set(accelerationToPack);
         }

         FramePoint2d desiredICPLocal = new FramePoint2d(desiredICP.getReferenceFrame());
         FrameVector2d desiredICPVelocityLocal = new FrameVector2d(desiredICPVelocity.getReferenceFrame());
         RobotSide supportSide = swingSide.getOppositeSide();
         icpTrajectoryGenerators.get(supportSide).pack(desiredICPLocal, desiredICPVelocityLocal, comHeight);
         desiredICP.set(desiredICPLocal);
         desiredICPVelocity.set(desiredICPVelocityLocal);

         desiredFootOrientations.get(swingSide).setYawPitchRoll(0.0, 0.0, 0.0);    // TODO
         desiredFootAngularVelocities.get(swingSide).set(0.0, 0.0, 0.0);
         desiredFootAngularAccelerations.get(swingSide).set(0.0, 0.0, 0.0);
      }

      @Override
      public void doTransitionIntoAction()
      {
         RobotSide supportSide = swingSide.getOppositeSide();
         setSupportLeg(supportSide);

         setDesiredFootPosVelAccForSupportSide(supportSide);

         FramePoint initialPosition = new FramePoint(swingAnkleZUpFrame);
         initialPosition.changeFrame(supportAnkleZUpFrame);

         FrameVector initialVelocity = new FrameVector(supportAnkleZUpFrame);
         Twist footTwist = new Twist();
         twistCalculator.packTwistOfBody(footTwist, fullRobotModel.getFoot(swingSide));
         footTwist.changeFrame(swingAnkleZUpFrame);
         footTwist.packLinearPart(initialVelocity);
         initialVelocity.changeFrame(supportAnkleZUpFrame);

         desiredFootstepCalculator.initializeDesiredFootstep(supportSide);
         Footstep desiredFootstep = desiredFootstepCalculator.updateAndGetDesiredFootstep(supportSide);
         FramePoint finalDesiredStepLocation = desiredFootstep.getFootstepPositionInFrame(supportAnkleZUpFrame);

         CartesianTrajectoryGenerator cartesianTrajectoryGenerator = cartesianTrajectoryGenerators.get(swingSide);
         cartesianTrajectoryGenerator.initialize(initialPosition, initialVelocity, finalDesiredStepLocation);

         FramePoint2d finalDesiredICP = getSingleSupportFinalDesiredICPForWalking(finalDesiredStepLocation, swingSide);
         icpTrajectoryGenerators.get(supportSide).initialize(desiredICP.getFramePoint2dCopy(), finalDesiredICP, cartesianTrajectoryGenerator.getFinalTime(),
                                     comHeight);

         centerOfMassHeightTrajectoryGenerator.initialize(upcomingSupportLeg.getEnumValue());
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
         FrameConvexPolygon2d footPolygon = bipedSupportPolygons.getFootPolygonInAnkleZUp(robotSide);
         boolean capturePointInsideFootPolygon = footPolygon.isPointInside(capturePoint.changeFrameCopy(footPolygon.getReferenceFrame()));
         boolean copInsideFootPolygon = footPolygon.isPointInside(previousCoP.changeFrameCopy(footPolygon.getReferenceFrame()));

         return capturePointInsideFootPolygon && copInsideFootPolygon;
      }
   }


   public class DoneWithSingleSupportCondition implements StateTransitionCondition
   {
      public boolean checkCondition()
      {
         RobotSide swingSide = supportLeg.getEnumValue().getOppositeSide();
         double minimumSwingFraction = 0.5;
         double minimumSwingTime = cartesianTrajectoryGenerators.get(swingSide).getFinalTime() * minimumSwingFraction;
         boolean footHitGroundEarly = (timeInCurrentState() > minimumSwingTime) && footSwitches.get(swingSide).hasFootHitGround();

         return cartesianTrajectoryGenerators.get(swingSide).isDone() || footHitGroundEarly;
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


   public void setCapturePoint(FramePoint2d capturePoint)
   {
      capturePoint.changeFrame(this.capturePoint.getReferenceFrame());
      this.capturePoint.set(capturePoint);
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
   }

   private FramePoint2d getDoubleSupportFinalDesiredICPForWalking(RobotSide supportSide)
   {
      FramePoint2d ret = bipedSupportPolygons.getSweetSpotCopy(supportSide);
      FrameVector2d offset = new FrameVector2d(ret.getReferenceFrame(), 0.0, 0.0);
      ret.add(offset);

      return ret;
   }

   private FramePoint2d getSingleSupportFinalDesiredICPForWalking(FramePoint finalDesiredStepLocation, RobotSide swingSide)
   {
      ReferenceFrame supportAnkleZUpFrame = referenceFrames.getAnkleZUpFrame(swingSide.getOppositeSide());
      FramePoint2d desiredICPLocal = desiredICP.getFramePoint2dCopy();
      desiredICPLocal.changeFrame(finalDesiredStepLocation.getReferenceFrame());

      FramePoint2d finalDesiredStepLocation2d = finalDesiredStepLocation.toFramePoint2d();
      finalDesiredStepLocation2d.changeFrame(supportAnkleZUpFrame);
      FrameVector2d stepOffset = new FrameVector2d(finalDesiredStepLocation2d);
      FrameVector2d finalDesiredICPOffset = new FrameVector2d(getDoubleSupportFinalDesiredICPForWalking(swingSide));    // TODO: do something about expected orientation of swing foot
      finalDesiredICPOffset.changeFrame(supportAnkleZUpFrame);
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

   private void setDesiredFootPosVelAccForSupportSide(RobotSide supportSide)
   {
      FramePoint desiredFootPosition = new FramePoint(referenceFrames.getFootFrame(supportSide));
      desiredFootPosition.changeFrame(desiredFootPositions.get(supportSide).getReferenceFrame());
      desiredFootPositions.get(supportSide).set(desiredFootPosition);
      desiredFootVelocities.get(supportSide).set(0.0, 0.0, 0.0);
      desiredFootAccelerations.get(supportSide).set(0.0, 0.0, 0.0);

      desiredFootOrientations.get(supportSide).setYawPitchRoll(0.0, 0.0, 0.0);
      desiredFootAngularVelocities.get(supportSide).set(0.0, 0.0, 0.0);
      desiredFootAngularAccelerations.get(supportSide).set(0.0, 0.0, 0.0);
   }

   public boolean isSwingFoot(RobotSide robotSide)
   {
      if (getSupportLeg() == null)
         return false;
      else
         return getSupportLeg().getOppositeSide() == robotSide;
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
}
