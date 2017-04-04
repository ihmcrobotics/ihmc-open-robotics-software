package us.ihmc.commonWalkingControlModules.controlModules.kneeAngle;

import us.ihmc.commonWalkingControlModules.configurations.StraightLegWalkingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommand;
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
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachineTools;

import java.util.ArrayList;
import java.util.List;

public class KneeControlModule
{
   public enum KneeControlType
   {
      STRAIGHTEN_TO_STRAIGHT, STRAIGHT, STRAIGHTEN_TO_CONTROLLABLE, BENT, CONTROLLABLE
   }

   private final YoVariableRegistry registry;

   private final EnumYoVariable<KneeControlType> requestedState;
   private final GenericStateMachine<KneeControlType, AbstractKneeControlState> stateMachine;

   private final DoubleYoVariable desiredAngle;
   private final DoubleYoVariable desiredAngleWhenStraight;

   public KneeControlModule(RobotSide robotSide, HighLevelHumanoidControllerToolbox controllerToolbox, StraightLegWalkingParameters straightLegWalkingParameters,
         YoVariableRegistry parentRegistry)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String namePrefix = sidePrefix + "Knee";
      registry = new YoVariableRegistry(sidePrefix + getClass().getSimpleName());

      desiredAngle = new DoubleYoVariable(namePrefix + "DesiredAngle", registry);
      desiredAngle.set(straightLegWalkingParameters.getStraightKneeAngle());

      desiredAngleWhenStraight = new DoubleYoVariable(namePrefix + "DesiredAngleWhenStraight", registry);
      desiredAngleWhenStraight.set(straightLegWalkingParameters.getStraightKneeAngle());

      DoubleYoVariable durationForStraightening = new DoubleYoVariable(namePrefix + "DurationForStraightening", registry);
      durationForStraightening.set(straightLegWalkingParameters.getDurationForStanceLegStraightening());

      // set up states and state machine
      DoubleYoVariable time = controllerToolbox.getYoTime();
      stateMachine = new GenericStateMachine<>(namePrefix + "State", namePrefix + "SwitchTime", KneeControlType.class, time, registry);
      requestedState = EnumYoVariable.create(namePrefix + "RequestedState", "", KneeControlType.class, registry, true);
      requestedState.set(null);

      OneDoFJoint kneeJoint = controllerToolbox.getFullRobotModel().getLegJoint(robotSide, LegJointName.KNEE_PITCH);

      List<AbstractKneeControlState> states = new ArrayList<>();

      AbstractKneeControlState straighteningToStraightState = new StraightenToStraightControlState(kneeJoint, durationForStraightening);
      states.add(straighteningToStraightState);
      AbstractKneeControlState straightState = new StraightKneeControlState(kneeJoint);
      states.add(straightState);
      AbstractKneeControlState bentState = new BentKneeControlState(kneeJoint);
      states.add(bentState);
      AbstractKneeControlState straighteningToControlState = new StraightenToControllableControlState(kneeJoint, durationForStraightening);
      states.add(straighteningToControlState);
      AbstractKneeControlState controlledState = new ControllableKneeControlState(kneeJoint);
      states.add(controlledState);

      straighteningToStraightState.setDefaultNextState(KneeControlType.STRAIGHT);
      straighteningToControlState.setDefaultNextState(KneeControlType.CONTROLLABLE);

      setupStateMachine(states, straightLegWalkingParameters.attemptToStraightenLegs());

      parentRegistry.addChild(registry);
   }

   private void setupStateMachine(List<AbstractKneeControlState> states, boolean attemptToStraightenLegs)
   {
      for (AbstractKneeControlState fromState : states)
      {
         for (AbstractKneeControlState toState : states)
         {
            StateMachineTools.addRequestedStateTransition(requestedState, false, fromState, toState);
         }
      }

      for (AbstractKneeControlState state : states)
      {
         stateMachine.addState(state);
      }

      if (attemptToStraightenLegs)
         stateMachine.setCurrentState(KneeControlType.STRAIGHT);
      else
         stateMachine.setCurrentState(KneeControlType.BENT);

   }

   public void initialize()
   {
   }

   public void doControl()
   {
      stateMachine.checkTransitionConditions();
      stateMachine.getCurrentState().doAction();
   }

   public void setKneeAngleState(KneeControlType controlType)
   {
      requestedState.set(controlType);
   }

   public KneeControlType getCurrentKneeControlState()
   {
      return stateMachine.getCurrentStateEnum();
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return stateMachine.getCurrentState().getPrivilegedConfigurationCommand();
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

   private class StraightenToStraightControlState extends StraighteningKneeControlState
   {
      public StraightenToStraightControlState(OneDoFJoint kneeJoint, DoubleYoVariable durationForStraightening)
      {
         super(KneeControlType.STRAIGHTEN_TO_STRAIGHT, kneeJoint, durationForStraightening);
      }
   }

   private class StraightenToControllableControlState extends StraighteningKneeControlState
   {
      public StraightenToControllableControlState(OneDoFJoint kneeJoint, DoubleYoVariable durationForStraightening)
      {
         super(KneeControlType.STRAIGHTEN_TO_CONTROLLABLE, kneeJoint, durationForStraightening);
      }
   }

   private class StraighteningKneeControlState extends AbstractKneeControlState
   {
      private final OneDoFJoint kneeJoint;
      private final DoubleYoVariable durationForStraightening;

      private double startingPosition;
      private double straighteningDuration;

      public StraighteningKneeControlState(KneeControlType stateEnum, OneDoFJoint kneeJoint, DoubleYoVariable durationForStraightening)
      {
         super(stateEnum);

         this.kneeJoint = kneeJoint;
         this.durationForStraightening = durationForStraightening;
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

         if (isDone())
            transitionToDefaultNextState();
      }

      @Override
      public void doTransitionIntoAction()
      {
         startingPosition = kneeJoint.getQ();
         straighteningDuration = durationForStraightening.getDoubleValue();
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

   private class BentKneeControlState extends AbstractKneeControlState
   {
      private final OneDoFJoint kneeJoint;
      public BentKneeControlState(OneDoFJoint kneeJoint)
      {
         super(KneeControlType.BENT);

         this.kneeJoint = kneeJoint;
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
         privilegedConfigurationCommand.clear();
         privilegedConfigurationCommand.addJoint(kneeJoint, PrivilegedConfigurationOption.AT_MID_RANGE);
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
