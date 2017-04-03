package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.GenericStateMachine;

public class KneeControlModule
{
   private static final double STRAIGHT_KNEE_ANGLE = 0.2;
   public enum KneeControlType
   {
      STRAIGHTENING, STRAIGHT, BENT, CONTROLLABLE
   }

   private final YoVariableRegistry registry;

   private final EnumYoVariable<KneeControlType> requestedState;
   private final GenericStateMachine<KneeControlType, AbstractKneeControlState> stateMachine;

   private final DoubleYoVariable desiredAngle;
   private final DoubleYoVariable desiredAngleWhenStraight;

   public KneeControlModule(RobotSide robotSide, HighLevelHumanoidControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String namePrefix = sidePrefix + "Knee";
      registry = new YoVariableRegistry(sidePrefix + getClass().getSimpleName());

      desiredAngle = new DoubleYoVariable(namePrefix + "DesiredAngle", registry);
      desiredAngleWhenStraight = new DoubleYoVariable(namePrefix + "DesiredAngleWhenStraight", registry);
      desiredAngle.set(STRAIGHT_KNEE_ANGLE);
      desiredAngleWhenStraight.set(STRAIGHT_KNEE_ANGLE);

      // set up states and state machine
      DoubleYoVariable time = controllerToolbox.getYoTime();
      stateMachine = new GenericStateMachine<>(namePrefix + "State", namePrefix + "SwitchTime", KneeControlType.class, time, registry);
      requestedState = EnumYoVariable.create(namePrefix + "RequestedState", "", KneeControlType.class, registry, true);
      requestedState.set(null);

      OneDoFJoint kneeJoint = controllerToolbox.getFullRobotModel().getLegJoint(robotSide, LegJointName.KNEE_PITCH);

      AbstractKneeControlState straighteningState = new StraighteningKneeControlState(kneeJoint);
      AbstractKneeControlState straightState = new StraightKneeControlState(kneeJoint);
      AbstractKneeControlState bentState = new BentKneeControlState(kneeJoint);
      AbstractKneeControlState controlledState = new ControllableKneeControlState(kneeJoint);

      stateMachine.addState(straighteningState);
      stateMachine.addState(straightState);
      stateMachine.addState(bentState);

      //// TODO: 4/2/17  setup states 

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
   }

   public void doControl()
   {
      stateMachine.getCurrentState().doAction();
   }

   private abstract class AbstractKneeControlState extends FinishableState<KneeControlType>
   {
      protected final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();

      public AbstractKneeControlState(KneeControlType stateEnum)
      {
         super(stateEnum);
      }

      public PrivilegedConfigurationCommand getPrivilegedConfigurationCommand()
      {
         return privilegedConfigurationCommand;
      }
   }

   private class StraighteningKneeControlState extends AbstractKneeControlState
   {
      private static final double straighteningDuration = 1.0;
      private final OneDoFJoint kneeJoint;

      private double startingPosition;

      public StraighteningKneeControlState(OneDoFJoint kneeJoint)
      {
         super(KneeControlType.STRAIGHTENING);

         this.kneeJoint = kneeJoint;
      }

      @Override
      public boolean isDone()
      {
         return getTimeInCurrentState() > straighteningDuration;
      }

      @Override
      public void doAction()
      {
         double phase = (straighteningDuration - getTimeInCurrentState()) / straighteningDuration;
         double desiredPrivilegedPosition = phase * startingPosition + (1.0 - phase) * desiredAngleWhenStraight.getDoubleValue();

         privilegedConfigurationCommand.clear();
         privilegedConfigurationCommand.addJoint(kneeJoint, desiredPrivilegedPosition);
      }

      @Override
      public void doTransitionIntoAction()
      {
         startingPosition = kneeJoint.getQ();
      }

      @Override
      public void doTransitionOutOfAction()
      {
      }
   }

   private class StraightKneeControlState extends AbstractKneeControlState
   {
      private final OneDoFJoint kneeJoint;

      public StraightKneeControlState(OneDoFJoint kneeJoint)
      {
         super(KneeControlType.STRAIGHT);

         this.kneeJoint = kneeJoint;
      }

      @Override
      public boolean isDone()
      {
         return false;
      }

      @Override
      public void doAction()
      {
         privilegedConfigurationCommand.clear();
         privilegedConfigurationCommand.addJoint(kneeJoint, desiredAngleWhenStraight.getDoubleValue());
      }

      @Override
      public void doTransitionIntoAction()
      {
      }

      @Override
      public void doTransitionOutOfAction()
      {
      }
   }

   private class BentKneeControlState extends  AbstractKneeControlState
   {
      public BentKneeControlState(OneDoFJoint kneeJoint)
      {
         super(KneeControlType.BENT);

         privilegedConfigurationCommand.addJoint(kneeJoint, PrivilegedConfigurationOption.AT_MID_RANGE);
      }

      @Override
      public boolean isDone()
      {
         return false;
      }

      @Override
      public void doAction()
      {
      }

      @Override
      public void doTransitionIntoAction()
      {
      }

      @Override
      public void doTransitionOutOfAction()
      {
      }
   }

   private class ControllableKneeControlState extends AbstractKneeControlState
   {
      private final OneDoFJoint kneeJoint;

      public ControllableKneeControlState(OneDoFJoint kneeJoint)
      {
         super(KneeControlType.CONTROLLABLE);

         this.kneeJoint = kneeJoint;
      }

      @Override
      public boolean isDone()
      {
         return false;
      }

      @Override
      public void doAction()
      {
         privilegedConfigurationCommand.clear();
         privilegedConfigurationCommand.addJoint(kneeJoint, desiredAngle.getDoubleValue());
      }

      @Override
      public void doTransitionIntoAction()
      {
         desiredAngle.set(desiredAngleWhenStraight.getDoubleValue());
      }

      @Override
      public void doTransitionOutOfAction()
      {
      }
   }
}
