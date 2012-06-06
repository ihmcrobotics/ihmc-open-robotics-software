package us.ihmc.commonWalkingControlModules.momentumBasedController;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.trajectories.CartesianTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.ParabolicCartesianTrajectoryGenerator;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
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
   private final EnumYoVariable<RobotSide> supportLeg = EnumYoVariable.create("supportLeg", RobotSide.class, registry);
   private final EnumYoVariable<RobotSide> upcomingSupportLeg = EnumYoVariable.create("upcomingSupportLeg", RobotSide.class, registry);
   private final SideDependentList<YoFramePoint> desiredSwingFootPositions = new SideDependentList<YoFramePoint>();
   private final SideDependentList<YoFrameVector> desiredSwingFootVelocities = new SideDependentList<YoFrameVector>();
   private final SideDependentList<YoFrameVector> desiredSwingFootAccelerations = new SideDependentList<YoFrameVector>();

   private final DoubleYoVariable desiredCoMHeight = new DoubleYoVariable("desiredCoMHeight", registry);

   private final FramePoint2d capturePoint;
   private final SideDependentList<Double> previousLegStrengths = new SideDependentList<Double>();


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

      double stepTime = 1.0;
      double groundClearance = 0.2;
      desiredCoMHeight.set(1.2);

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
      State doubleSupportState = new DoubleSupportState();
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
         StateTransition toDoubleSupport = new StateTransition(MomentumBasedControllerState.DOUBLE_SUPPORT, new DoneWithSingleSupportCondition());
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

      @Override
      public void doAction()
      {
         FramePoint2d desiredCapturePoint = bipedSupportPolygons.getSupportPolygonInMidFeetZUp().getCentroidCopy();
         desiredCapturePoint.changeFrame(desiredICP.getReferenceFrame());
         desiredICP.set(desiredCapturePoint);
      }

      @Override
      public void doTransitionIntoAction()
      {
         supportLeg.set(null);
      }

      @Override
      public void doTransitionOutOfAction()
      {
         // TODO Auto-generated method stub

      }
   }


   private class TransferState extends State
   {
      private final RobotSide robotSide;

      public TransferState(RobotSide robotSide)
      {
         super(transferStateEnums.get(robotSide));
         this.robotSide = robotSide;
      }

      @Override
      public void doAction()
      {
         FramePoint2d desiredCapturePoint = bipedSupportPolygons.getSweetSpotCopy(robotSide);
         desiredCapturePoint.changeFrame(desiredICP.getReferenceFrame());
         desiredICP.set(desiredCapturePoint);
      }

      @Override
      public void doTransitionIntoAction()
      {
      }

      @Override
      public void doTransitionOutOfAction()
      {
         upcomingSupportLeg.set(upcomingSupportLeg.getEnumValue().getOppositeSide());
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
         
//       desiredSwingFootPositions.get(swingSide).set(0.0, swingSide.negateIfRightSide(0.38), 0.03);
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

         FramePoint finalDesiredPosition = new FramePoint(supportAnkleZUpFrame);
         finalDesiredPosition.setX(finalDesiredPosition.getX() + 0.2);
         finalDesiredPosition.setY(swingSide.negateIfRightSide(0.3));
         finalDesiredPosition.setZ(-3e-3);

         cartesianTrajectoryGenerators.get(swingSide).initialize(initialPosition, initialVelocity, finalDesiredPosition);
      }

      @Override
      public void doTransitionOutOfAction()
      {
         // TODO Auto-generated method stub

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
         return (robotSide == upcomingSupportLeg.getEnumValue()) && (timeInCurrentState() > 1.0);    // FIXME
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
}
