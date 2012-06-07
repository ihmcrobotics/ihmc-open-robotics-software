package us.ihmc.commonWalkingControlModules.momentumBasedController;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.calculators.MaximumICPVelocityCalculator;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.trajectories.CartesianTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.ParabolicCartesianTrajectoryGenerator;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
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


   private final YoFramePoint2d desiredICP;
   private final YoFrameVector2d desiredICPVelocity;
   private final YoFramePoint2d initialDesiredICP;

   private final EnumYoVariable<RobotSide> supportLeg = EnumYoVariable.create("supportLeg", RobotSide.class, registry);
   private final EnumYoVariable<RobotSide> upcomingSupportLeg = EnumYoVariable.create("upcomingSupportLeg", RobotSide.class, registry);
   private final SideDependentList<YoFramePoint> desiredSwingFootPositions = new SideDependentList<YoFramePoint>();
   private final SideDependentList<YoFrameVector> desiredSwingFootVelocities = new SideDependentList<YoFrameVector>();
   private final SideDependentList<YoFrameVector> desiredSwingFootAccelerations = new SideDependentList<YoFrameVector>();
   private final DoubleYoVariable desiredCoMHeight = new DoubleYoVariable("desiredCoMHeight", registry);

   private final FramePoint2d capturePoint;
   private final SideDependentList<Double> previousLegStrengths = new SideDependentList<Double>();
   private final DoubleYoVariable stepLength = new DoubleYoVariable("stepLength", registry);
   private final DoubleYoVariable stepWidth = new DoubleYoVariable("stepWidth", registry);
   private final double doubleSupportTime = 0.5;
   private final double stepTime = 1.0;

   private double comHeight;
   private double gravity;


   // TODO: add desired swing foot orientation and angular velocity

   public MomentumBasedControllerStateMachine(FullRobotModel fullRobotModel, CommonWalkingReferenceFrames referenceFrames, TwistCalculator twistCalculator,
           BipedSupportPolygons bipedSupportPolygons, DoubleYoVariable t, double controlDT, YoVariableRegistry parentRegistry)
   {
      super(name + "State", name + "SwitchTime", MomentumBasedControllerState.class, t, registry);
      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = referenceFrames;
      this.twistCalculator = twistCalculator;
      this.bipedSupportPolygons = bipedSupportPolygons;
      this.controlDT = controlDT;

      desiredICP = new YoFramePoint2d("desiredICP", "", ReferenceFrame.getWorldFrame(), registry);
      desiredICPVelocity = new YoFrameVector2d("desiredICPVelocity", "", ReferenceFrame.getWorldFrame(), registry);
      initialDesiredICP = new YoFramePoint2d("initialDesiredICP", "", ReferenceFrame.getWorldFrame(), registry);

      double groundClearance = 0.2;
      desiredCoMHeight.set(1.2);
      stepLength.set(0.3);
      stepWidth.set(0.25);

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
                                           new ParabolicCartesianTrajectoryGenerator(robotSide.getCamelCaseNameForStartOfExpression() + "CartesianTrajectory",
                                              supportAnkleZUpFrame, stepTime, groundClearance, registry));
      }

      this.capturePoint = new FramePoint2d(ReferenceFrame.getWorldFrame());
      upcomingSupportLeg.set(RobotSide.LEFT);

      setUpStateMachine();
      parentRegistry.addChild(registry);
   }

   private void setUpStateMachine()
   {
      DoubleSupportState doubleSupportState = new DoubleSupportState();
      doubleSupportState.setDesiredICPToFinalDesired();

      for (RobotSide robotSide : RobotSide.values())
      {
         StateTransition toTransfer = new StateTransition(transferStateEnums.get(robotSide), new DoneWithDoubleSupportCondition(robotSide));
         doubleSupportState.addStateTransition(toTransfer);
      }

      addState(doubleSupportState);

      for (RobotSide robotSide : RobotSide.values())
      {
         State transferState = new TransferState(robotSide);
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
      desiredICP.getFramePoint2d(desiredICPToPack);
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

   private class DoubleSupportState extends State
   {
      public DoubleSupportState()
      {
         super(MomentumBasedControllerState.DOUBLE_SUPPORT);
      }

      public void setDesiredICPToFinalDesired()
      {
         FramePoint2d desiredCapturePoint = getFinalDesiredICP();
         desiredCapturePoint.changeFrame(desiredICP.getReferenceFrame());
         desiredICP.set(desiredCapturePoint);
      }

      private FramePoint2d getFinalDesiredICP()
      {
         return bipedSupportPolygons.getSupportPolygonInMidFeetZUp().getCentroidCopy();
      }

      @Override
      public void doAction()
      {
         FramePoint2d finalDesiredICP = getFinalDesiredICP();
         finalDesiredICP.changeFrame(initialDesiredICP.getReferenceFrame());
         FrameVector2d icpMove = new FrameVector2d(finalDesiredICP);
         icpMove.sub(initialDesiredICP.getFramePoint2dCopy());
         double desiredICPVelocityMagnitude = icpMove.length() / doubleSupportTime;

         double epsilon = 1e-4;
         if ((timeInCurrentState() < doubleSupportTime) && (desiredICP.distance(finalDesiredICP) > epsilon))
            linearlyMoveDesiredICP(desiredICPVelocityMagnitude, getFinalDesiredICP());
         else
         {
            setDesiredICPToFinalDesired();
            desiredICPVelocity.set(0.0, 0.0);
         }
      }

      @Override
      public void doTransitionIntoAction()
      {
         supportLeg.set(null);
         initialDesiredICP.set(desiredICP);
      }

      @Override
      public void doTransitionOutOfAction()
      {
         desiredICPVelocity.set(0.0, 0.0);
      }
   }


   private class TransferState extends State
   {
      private final RobotSide robotSide;
      private final double transferTime = 0.75;

      public TransferState(RobotSide robotSide)
      {
         super(transferStateEnums.get(robotSide));
         this.robotSide = robotSide;
      }

      private FramePoint2d getFinalDesiredICP()
      {
         return bipedSupportPolygons.getSweetSpotCopy(robotSide);
      }

      @Override
      public void doAction()
      {
         FramePoint2d finalDesiredICP = getFinalDesiredICP();
         finalDesiredICP.changeFrame(initialDesiredICP.getReferenceFrame());
         FrameVector2d icpMove = new FrameVector2d(finalDesiredICP);
         icpMove.sub(initialDesiredICP.getFramePoint2dCopy());
         double desiredICPVelocityMagnitude = icpMove.length() / transferTime;

         double epsilon = 1e-4;
         if (desiredICP.distance(finalDesiredICP) > epsilon)
            linearlyMoveDesiredICP(desiredICPVelocityMagnitude, getFinalDesiredICP());
         else
         {
            FramePoint2d desiredCapturePoint = getFinalDesiredICP();
            desiredCapturePoint.changeFrame(desiredICP.getReferenceFrame());
            desiredICP.set(desiredCapturePoint);
            desiredICPVelocity.set(0.0, 0.0);
         }
      }

      @Override
      public void doTransitionIntoAction()
      {
         supportLeg.set(null);
         initialDesiredICP.set(desiredICP);
      }

      @Override
      public void doTransitionOutOfAction()
      {
         upcomingSupportLeg.set(upcomingSupportLeg.getEnumValue().getOppositeSide());
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
         FramePoint2d desiredCapturePoint = bipedSupportPolygons.getSweetSpotCopy(swingSide.getOppositeSide());
         desiredCapturePoint.changeFrame(desiredICP.getReferenceFrame());
         desiredICP.set(desiredCapturePoint);

         CartesianTrajectoryGenerator cartesianTrajectoryGenerator = cartesianTrajectoryGenerators.get(swingSide);
         if (!cartesianTrajectoryGenerator.isDone())
         {
            cartesianTrajectoryGenerator.computeNextTick(positionToPack, velocityToPack, accelerationToPack, controlDT);
            desiredSwingFootPositions.get(swingSide).set(positionToPack);
            desiredSwingFootVelocities.get(swingSide).set(velocityToPack);
            desiredSwingFootAccelerations.get(swingSide).set(accelerationToPack);
         }
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

         FramePoint finalDesiredStepLocation = new FramePoint(supportAnkleZUpFrame);
         finalDesiredStepLocation.setX(finalDesiredStepLocation.getX() + stepLength.getDoubleValue());
         finalDesiredStepLocation.setY(swingSide.negateIfRightSide(stepWidth.getDoubleValue()));

         cartesianTrajectoryGenerators.get(swingSide).initialize(initialPosition, initialVelocity, finalDesiredStepLocation);


         FramePoint2d finalCapturePoint = finalDesiredStepLocation.toFramePoint2d();
         finalCapturePoint.scale(0.5);    // half way to the finalDesiredStepLocation; this is allowed because it's already in supportAnkleZUpFrame.
//         FramePoint2d equivalentConstantCoP = EquivalentConstantCoPCalculator.computeEquivalentConstantCoP(capturePoint, finalCapturePoint, stepTime,
//                                                 comHeight, gravity);
         // TODO
      }

      @Override
      public void doTransitionOutOfAction()
      {
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

      FrameVector2d desiredICPDisplacement = desiredICPVelocity.getFrameVector2dCopy();
      desiredICPDisplacement.scale(controlDT);
      desiredICP.add(desiredICPDisplacement);
   }
}
