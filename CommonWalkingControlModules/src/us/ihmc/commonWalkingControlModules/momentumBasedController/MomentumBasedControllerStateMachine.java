package us.ihmc.commonWalkingControlModules.momentumBasedController;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.calculators.EquivalentConstantCoPCalculator;
import us.ihmc.commonWalkingControlModules.calculators.MaximumICPVelocityCalculator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.desiredFootStep.SimpleDesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.SimpleDesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.commonWalkingControlModules.trajectories.CartesianTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.CenterOfMassHeightTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.LinearCenterOfMassHeightTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.StraightUpThenParabolicCartesianTrajectoryGenerator;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameLine2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.GeometryTools;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;

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
   private final BipedSupportPolygons bipedSupportPolygons;
   private final SideDependentList<CartesianTrajectoryGenerator> cartesianTrajectoryGenerators = new SideDependentList<CartesianTrajectoryGenerator>();
   private final double controlDT;
   private final DesiredHeadingControlModule desiredHeadingControlModule;
   private final DesiredFootstepCalculator desiredFootstepCalculator;
   private final CenterOfMassHeightTrajectoryGenerator centerOfMassHeightTrajectoryGenerator;


   private final YoFramePoint2d desiredICP;
   private final YoFrameVector2d desiredICPVelocity;

   private final EnumYoVariable<RobotSide> supportLeg = EnumYoVariable.create("supportLeg", RobotSide.class, registry);
   private final EnumYoVariable<RobotSide> upcomingSupportLeg = EnumYoVariable.create("upcomingSupportLeg", RobotSide.class, registry);
   private final SideDependentList<YoFramePoint> desiredSwingFootPositions = new SideDependentList<YoFramePoint>();
   private final SideDependentList<YoFrameVector> desiredSwingFootVelocities = new SideDependentList<YoFrameVector>();
   private final SideDependentList<YoFrameVector> desiredSwingFootAccelerations = new SideDependentList<YoFrameVector>();
   private final DoubleYoVariable desiredCoMHeight = new DoubleYoVariable("desiredCoMHeight", registry);
   private final DoubleYoVariable desiredCoMHeightVelocity = new DoubleYoVariable("desiredCoMHeightVelocity", registry);
   private final DoubleYoVariable desiredCoMHeightAcceleration = new DoubleYoVariable("desiredCoMHeightAcceleration", registry);

   private final FramePoint2d capturePoint;
   private final SideDependentList<Double> previousLegStrengths = new SideDependentList<Double>();
   private final double doubleSupportTime = 0.9;
   private final double straightUpAverageVelocity = 3.5;
   private final double parabolicTime = 0.5;    // 0.75 for stairs
   private final double initialGroundClearance = 0.18;    // 0.15 for stairs

   private final DoubleYoVariable singleSupportICPGlideScaleFactor = new DoubleYoVariable("singleSupportICPGlideScaleFactor", registry);

   private double comHeight;
   private double gravity;


   // TODO: add desired swing foot orientation and angular velocity

   public MomentumBasedControllerStateMachine(FullRobotModel fullRobotModel, CommonWalkingReferenceFrames referenceFrames, TwistCalculator twistCalculator,
           BipedSupportPolygons bipedSupportPolygons, ProcessedSensorsInterface processedSensors, DoubleYoVariable t, double controlDT, double footHeight,
           YoVariableRegistry parentRegistry)
   {
      super(name + "State", name + "SwitchTime", MomentumBasedControllerState.class, t, registry);
      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = referenceFrames;
      this.twistCalculator = twistCalculator;
      this.bipedSupportPolygons = bipedSupportPolygons;
      this.controlDT = controlDT;
      this.desiredHeadingControlModule = new SimpleDesiredHeadingControlModule(0.0, controlDT, registry);
      SimpleDesiredFootstepCalculator simpleDesiredFootstepCalculator = new SimpleDesiredFootstepCalculator(referenceFrames.getAnkleZUpReferenceFrames(),
                                                                           desiredHeadingControlModule, parentRegistry);    // TODO: pass in
      simpleDesiredFootstepCalculator.setupParametersForR2InverseDynamics();
      this.desiredFootstepCalculator = simpleDesiredFootstepCalculator;
      this.centerOfMassHeightTrajectoryGenerator = new LinearCenterOfMassHeightTrajectoryGenerator(processedSensors, referenceFrames,
              desiredHeadingControlModule.getDesiredHeadingFrame(), footHeight, parentRegistry);
      
//      this.centerOfMassHeightTrajectoryGenerator = new OrbitalEnergyCubicTrajectoryGenerator(processedSensors, referenceFrames,
//            desiredHeadingControlModule.getDesiredHeadingFrame(), footHeight, parentRegistry);

      desiredICP = new YoFramePoint2d("desiredICP", "", ReferenceFrame.getWorldFrame(), registry);
      desiredICPVelocity = new YoFrameVector2d("desiredICPVelocity", "", ReferenceFrame.getWorldFrame(), registry);

      singleSupportICPGlideScaleFactor.set(1.0);
      FramePoint2d finalDesiredICPForDoubleSupportStance = getFinalDesiredICPForDoubleSupportStance();
      finalDesiredICPForDoubleSupportStance.changeFrame(desiredICP.getReferenceFrame());
      desiredICP.set(finalDesiredICPForDoubleSupportStance);

      for (RobotSide robotSide : RobotSide.values())
      {
         ReferenceFrame supportAnkleZUpFrame = referenceFrames.getAnkleZUpFrame(robotSide.getOppositeSide());
         YoFramePoint desiredSwingFootPosition = new YoFramePoint("desired" + robotSide.getCamelCaseNameForMiddleOfExpression() + "SwingFootPosition", "",
                                                    supportAnkleZUpFrame, registry);
         desiredSwingFootPositions.put(robotSide, desiredSwingFootPosition);

         YoFrameVector desiredSwingFootVelocity = new YoFrameVector("desired" + robotSide.getCamelCaseNameForMiddleOfExpression() + "SwingFootVelocity", "",
                                                     supportAnkleZUpFrame, registry);
         desiredSwingFootVelocities.put(robotSide, desiredSwingFootVelocity);

         YoFrameVector desiredSwingFootAcceleration = new YoFrameVector("desired" + robotSide.getCamelCaseNameForMiddleOfExpression()
                                                         + "SwingFootAcceleration", "", supportAnkleZUpFrame, registry);
         desiredSwingFootAccelerations.put(robotSide, desiredSwingFootAcceleration);

         cartesianTrajectoryGenerators.put(robotSide,
                                           new StraightUpThenParabolicCartesianTrajectoryGenerator(robotSide.getCamelCaseNameForStartOfExpression()
                                              + "CartesianTrajectory", supportAnkleZUpFrame, straightUpAverageVelocity, parabolicTime, initialGroundClearance,
                                                 registry));
      }

      this.capturePoint = new FramePoint2d(ReferenceFrame.getWorldFrame());
      upcomingSupportLeg.set(RobotSide.LEFT);

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
         StateTransition toSingleSupport = new StateTransition(singleSupportStateEnums.get(robotSide), new DoneWithTransferCondition(robotSide));
         transferState.addStateTransition(toSingleSupport);
         addState(transferState);

         State singleSupportState = new SingleSupportState(robotSide);
         StateTransition toDoubleSupport = new StateTransition(transferStateEnums.get(robotSide.getOppositeSide()), new DoneWithSingleSupportCondition());
         singleSupportState.addStateTransition(toDoubleSupport);
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

   public void packDesiredSwingFootPosition(FramePoint desiredSwingFootPositionToPack)
   {
      if (supportLeg.getEnumValue() == null)
         throw new RuntimeException("In double support.");
      desiredSwingFootPositions.get(supportLeg.getEnumValue().getOppositeSide()).getFramePointAndChangeFrameOfPackedPoint(desiredSwingFootPositionToPack);
   }

   public void packDesiredSwingFootVelocity(FrameVector desiredSwingFootVelocityToPack)
   {
      if (supportLeg.getEnumValue() == null)
         throw new RuntimeException("In double support.");
      desiredSwingFootVelocities.get(supportLeg.getEnumValue().getOppositeSide()).getFrameVectorAndChangeFrameOfPackedVector(desiredSwingFootVelocityToPack);
   }

   public void packDesiredSwingFootAcceleration(FrameVector desiredSwingFootAccelerationToPack)
   {
      if (supportLeg.getEnumValue() == null)
         throw new RuntimeException("In double support.");
      desiredSwingFootAccelerations.get(supportLeg.getEnumValue().getOppositeSide()).getFrameVectorAndChangeFrameOfPackedVector(
          desiredSwingFootAccelerationToPack);
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

         FramePoint2d finalDesiredICP;

         if (transferToSide == null)
            finalDesiredICP = getFinalDesiredICPForDoubleSupportStance();
         else
            finalDesiredICP = getFinalDesiredICPForWalking(transferToSide);
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

      @Override
      public void doTransitionIntoAction()
      {
         supportLeg.set(null);
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

      private final YoFramePoint2d initialDesiredICP;
      private final YoFramePoint2d finalDesiredICP;


      public SingleSupportState(RobotSide robotSide)
      {
         super(singleSupportStateEnums.get(robotSide));
         this.swingSide = robotSide.getOppositeSide();
         swingAnkleZUpFrame = referenceFrames.getAnkleZUpFrame(swingSide);
         supportAnkleZUpFrame = referenceFrames.getAnkleZUpFrame(swingSide.getOppositeSide());
         this.positionToPack = new FramePoint(supportAnkleZUpFrame);
         this.velocityToPack = new FrameVector(supportAnkleZUpFrame);
         this.accelerationToPack = new FrameVector(supportAnkleZUpFrame);

         initialDesiredICP = new YoFramePoint2d(robotSide.getCamelCaseNameForStartOfExpression() + "SwingInitialDesiredICP", "",
                 referenceFrames.getAnkleZUpFrame(robotSide), registry);
         finalDesiredICP = new YoFramePoint2d(robotSide.getCamelCaseNameForStartOfExpression() + "SwingFinalDesiredICP", "",
                 referenceFrames.getAnkleZUpFrame(robotSide), registry);
      }

      @Override
      public void doAction()
      {
         evaluateCoMHeightTrajectory();

         CartesianTrajectoryGenerator cartesianTrajectoryGenerator = cartesianTrajectoryGenerators.get(swingSide);
         if (!cartesianTrajectoryGenerator.isDone())
         {
            cartesianTrajectoryGenerator.computeNextTick(positionToPack, velocityToPack, accelerationToPack, controlDT);
            desiredSwingFootPositions.get(swingSide).set(positionToPack);
            desiredSwingFootVelocities.get(swingSide).set(velocityToPack);
            desiredSwingFootAccelerations.get(swingSide).set(accelerationToPack);
         }

         double omega0 = Math.sqrt(gravity / comHeight);

         double expT = Math.exp(omega0 * timeInCurrentState());
         double expTf = Math.exp(omega0 * cartesianTrajectoryGenerator.getFinalTime());
         double parameter = (expT - 1.0) / (expTf - 1.0);
         double parameterd = omega0 * expT / (expTf - 1.0);

         FrameVector2d initialToFinal = new FrameVector2d(finalDesiredICP.getFramePoint2dCopy());
         initialToFinal.sub(initialDesiredICP.getFramePoint2dCopy());

         FrameVector2d offset = new FrameVector2d(initialToFinal);
         offset.scale(parameter);
         FramePoint2d desiredICPLocal = initialDesiredICP.getFramePoint2dCopy();
         desiredICPLocal.add(offset);
         desiredICPLocal.changeFrame(desiredICP.getReferenceFrame());
         desiredICP.set(desiredICPLocal);

         FrameVector2d desiredICPVelocityLocal = new FrameVector2d(initialToFinal);
         desiredICPVelocityLocal.scale(parameterd);
         desiredICPVelocityLocal.changeFrame(desiredICPVelocity.getReferenceFrame());
         desiredICPVelocity.set(desiredICPVelocityLocal);
      }

      @Override
      public void doTransitionIntoAction()
      {
         supportLeg.set(swingSide.getOppositeSide());


         FramePoint initialPosition = new FramePoint(swingAnkleZUpFrame);
         initialPosition.changeFrame(supportAnkleZUpFrame);

         FrameVector initialVelocity = new FrameVector(supportAnkleZUpFrame);
         Twist footTwist = new Twist();
         twistCalculator.packTwistOfBody(footTwist, fullRobotModel.getFoot(swingSide));
         footTwist.changeFrame(swingAnkleZUpFrame);
         footTwist.packLinearPart(initialVelocity);
         initialVelocity.changeFrame(supportAnkleZUpFrame);

         desiredFootstepCalculator.initializeDesiredFootstep(swingSide.getOppositeSide());
         Footstep desiredFootstep = desiredFootstepCalculator.updateAndGetDesiredFootstep(swingSide.getOppositeSide());
         FramePoint finalDesiredStepLocation = desiredFootstep.getFootstepPositionInFrame(supportAnkleZUpFrame);

         CartesianTrajectoryGenerator cartesianTrajectoryGenerator = cartesianTrajectoryGenerators.get(swingSide);
         cartesianTrajectoryGenerator.initialize(initialPosition, initialVelocity, finalDesiredStepLocation);


         FramePoint2d desiredICPLocal = desiredICP.getFramePoint2dCopy();
         desiredICPLocal.changeFrame(finalDesiredStepLocation.getReferenceFrame());
         initialDesiredICP.set(desiredICPLocal);

         FramePoint2d finalDesiredStepLocation2d = finalDesiredStepLocation.toFramePoint2d();
         finalDesiredStepLocation2d.changeFrame(supportAnkleZUpFrame);
         FrameVector2d stepOffset = new FrameVector2d(finalDesiredStepLocation2d);
         FrameVector2d finalDesiredICPOffset = new FrameVector2d(getFinalDesiredICPForWalking(swingSide));    // TODO: do something about expected orientation of swing foot
         finalDesiredICPOffset.changeFrame(supportAnkleZUpFrame);
         finalDesiredICPOffset.add(stepOffset);
         finalDesiredICPOffset.sub(desiredICPLocal);

         finalDesiredICPOffset.scale(singleSupportICPGlideScaleFactor.getDoubleValue());
         FramePoint2d finalDesiredICP = new FramePoint2d(desiredICPLocal);
         finalDesiredICP.add(finalDesiredICPOffset);

         // make sure it is feasible:
         FramePoint2d equivalentConstantCoP = EquivalentConstantCoPCalculator.computeEquivalentConstantCoP(desiredICPLocal, finalDesiredICP,
                                                 cartesianTrajectoryGenerator.getFinalTime(), comHeight, gravity);
         if (!desiredICPLocal.epsilonEquals(finalDesiredICP, 0.0))
         {
            FrameLine2d line = new FrameLine2d(desiredICPLocal, finalDesiredICP);
            GeometryTools.movePointInsidePolygonAlongLine(equivalentConstantCoP, bipedSupportPolygons.getFootPolygonInAnkleZUp(supportLeg.getEnumValue()),
                    line);
            finalDesiredICP = EquivalentConstantCoPCalculator.computePredictedICP(desiredICPLocal, equivalentConstantCoP,
                    cartesianTrajectoryGenerator.getFinalTime(), comHeight, gravity);
         }

         this.finalDesiredICP.set(finalDesiredICP);

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
         return (robotSide == upcomingSupportLeg.getEnumValue()) && (timeInCurrentState() > doubleSupportTime);
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
         boolean legStrengthHighEnough = previousLegStrengths.get(robotSide) > 0.99;

         return capturePointInsideFootPolygon && legStrengthHighEnough;
      }
   }


   public class DoneWithSingleSupportCondition implements StateTransitionCondition
   {
      public boolean checkCondition()
      {
         RobotSide swingSide = supportLeg.getEnumValue().getOppositeSide();

         return cartesianTrajectoryGenerators.get(swingSide).isDone();
      }
   }


   public void setCapturePoint(FramePoint2d capturePoint)
   {
      capturePoint.changeFrame(this.capturePoint.getReferenceFrame());
      this.capturePoint.set(capturePoint);
   }

   public void setPreviousLegStrengths(SideDependentList<Double> legStrengths)
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         previousLegStrengths.put(robotSide, legStrengths.get(robotSide));
      }
   }

   public void setGravity(double gravity)
   {
      this.gravity = gravity;
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

   private FramePoint2d getFinalDesiredICPForDoubleSupportStance()
   {
      return bipedSupportPolygons.getSupportPolygonInMidFeetZUp().getCentroidCopy();
   }

   private FramePoint2d getFinalDesiredICPForWalking(RobotSide supportSide)
   {
      FramePoint2d ret = bipedSupportPolygons.getSweetSpotCopy(supportSide);
      FrameVector2d offset = new FrameVector2d(ret.getReferenceFrame(), 0.0, 0.0);
      ret.add(offset);
      return ret;
   }

   private void evaluateCoMHeightTrajectory()
   {
      centerOfMassHeightTrajectoryGenerator.compute();
      desiredCoMHeight.set(centerOfMassHeightTrajectoryGenerator.getDesiredCenterOfMassHeight());
      desiredCoMHeightVelocity.set(centerOfMassHeightTrajectoryGenerator.getDesiredCenterOfMassHeightVelocity());
      desiredCoMHeightAcceleration.set(centerOfMassHeightTrajectoryGenerator.getDesiredCenterOfMassHeightAcceleration());
   }
}
