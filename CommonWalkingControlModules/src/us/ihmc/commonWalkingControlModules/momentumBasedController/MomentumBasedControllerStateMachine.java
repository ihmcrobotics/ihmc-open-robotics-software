package us.ihmc.commonWalkingControlModules.momentumBasedController;

import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.trajectories.CartesianTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.ParabolicCartesianTrajectoryGenerator;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint2d;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.statemachines.State;
import com.yobotics.simulationconstructionset.util.statemachines.StateMachine;

public class MomentumBasedControllerStateMachine extends StateMachine
{
   private static enum MomentumBasedControllerState {LEFT_SUPPORT, RIGHT_SUPPORT, TRANSFER_TO_LEFT, TRANSFER_TO_RIGHT, DOUBLE_SUPPORT}

   private static final String name = "momentumSM";
   private static final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final SideDependentList<MomentumBasedControllerState> singleSupportStateEnums =
      new SideDependentList<MomentumBasedControllerStateMachine.MomentumBasedControllerState>(MomentumBasedControllerState.LEFT_SUPPORT,
                            MomentumBasedControllerState.RIGHT_SUPPORT);

   private final SideDependentList<MomentumBasedControllerState> transferStateEnums =
      new SideDependentList<MomentumBasedControllerStateMachine.MomentumBasedControllerState>(MomentumBasedControllerState.TRANSFER_TO_LEFT,
                            MomentumBasedControllerState.TRANSFER_TO_RIGHT);

   private final SideDependentList<CartesianTrajectoryGenerator> cartesianTrajectoryGenerators = new SideDependentList<CartesianTrajectoryGenerator>();


   private final YoFramePoint2d desiredICP;
   private final EnumYoVariable<RobotSide> supportLeg = EnumYoVariable.create("supportLeg", RobotSide.class, registry);
   private final SideDependentList<YoFramePoint> desiredSwingFootPositions = new SideDependentList<YoFramePoint>();
   private final SideDependentList<YoFrameVector> desiredSwingFootVelocities = new SideDependentList<YoFrameVector>();

   public MomentumBasedControllerStateMachine(CommonWalkingReferenceFrames referenceFrames, DoubleYoVariable t, YoVariableRegistry parentRegistry)
   {
      super(name, name + "SwitchTime", MomentumBasedControllerState.class, t, registry);
      desiredICP = new YoFramePoint2d("desiredICP", "", referenceFrames.getMidFeetZUpFrame(), registry);

      double stepTime = 1.0;
      double groundClearance = 0.1;

      for (RobotSide robotSide : RobotSide.values())
      {
         ReferenceFrame supportAnkleZUpFrame = referenceFrames.getAnkleZUpFrame(robotSide.getOppositeSide());
         YoFramePoint desiredSwingFootPosition = new YoFramePoint("desired" + robotSide.getCamelCaseNameForMiddleOfExpression() + "SwingFootPosition", "",
                                                    supportAnkleZUpFrame, registry);
         desiredSwingFootPositions.put(robotSide, desiredSwingFootPosition);

         YoFrameVector desiredSwingFootVelocity = new YoFrameVector("desired" + robotSide.getCamelCaseNameForMiddleOfExpression() + "SwingFootVelocity", "",
                                                     supportAnkleZUpFrame, registry);
         desiredSwingFootVelocities.put(robotSide, desiredSwingFootVelocity);

         cartesianTrajectoryGenerators.put(robotSide,
                                           new ParabolicCartesianTrajectoryGenerator(robotSide.getCamelCaseNameForStartOfExpression() + "CartesianTrajectory",
                                              supportAnkleZUpFrame, stepTime, groundClearance, registry));

         desiredSwingFootPosition.set(0.0, robotSide.negateIfRightSide(0.38), 0.03);    // TODO: remove
      }

      supportLeg.set(null);    // TODO: remove

      setUpStateMachine();
      parentRegistry.addChild(registry);
   }

   private void setUpStateMachine()
   {
   }

   public void packDesiredICP(FramePoint2d desiredICPToPack)
   {
      desiredICP.getFramePoint2d(desiredICPToPack);
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

   private class DoubleSupportState extends State
   {
      public DoubleSupportState()
      {
         super(MomentumBasedControllerState.DOUBLE_SUPPORT);
      }

      @Override
      public void doAction()
      {
         // TODO Auto-generated method stub

      }

      @Override
      public void doTransitionIntoAction()
      {
         // TODO Auto-generated method stub

      }

      @Override
      public void doTransitionOutOfAction()
      {
         // TODO Auto-generated method stub

      }
   }


   private class SingleSupportState extends State
   {
      private final RobotSide robotSide;

      public SingleSupportState(RobotSide robotSide)
      {
         super(singleSupportStateEnums.get(robotSide));
         this.robotSide = robotSide;
      }

      @Override
      public void doAction()
      {
         // TODO Auto-generated method stub

      }

      @Override
      public void doTransitionIntoAction()
      {
         // TODO Auto-generated method stub

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
         // TODO Auto-generated method stub

      }

      @Override
      public void doTransitionIntoAction()
      {
         // TODO Auto-generated method stub

      }

      @Override
      public void doTransitionOutOfAction()
      {
         // TODO Auto-generated method stub

      }
   }
}
