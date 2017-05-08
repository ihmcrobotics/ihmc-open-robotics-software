package us.ihmc.commonWalkingControlModules.controlModules.kneeAngle;

import us.ihmc.commonWalkingControlModules.configurations.StraightLegWalkingParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.robotics.InterpolationTools;
import us.ihmc.robotics.controllers.YoPDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
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
      STRAIGHTEN_TO_STRAIGHT, STRAIGHT, STRAIGHTEN_TO_CONTROLLABLE, COLLAPSE, BENT, CONTROLLABLE
   }

   private static final boolean ONLY_MOVE_PRIV_POS_IF_NOT_BENDING = true;
   private static final boolean SCALE_STRAIGHT_GAIN_WITH_ERROR = false;

   private final YoVariableRegistry registry;

   private final EnumYoVariable<KneeControlType> requestedState;
   private final GenericStateMachine<KneeControlType, AbstractKneeControlState> stateMachine;

   private final DoubleYoVariable legPitchPrivilegedWeight;
   private final DoubleYoVariable legPitchPrivilegedPositionGain;
   private final DoubleYoVariable legPitchPrivilegedVelocityGain;

   private final DoubleYoVariable kneeStraightPrivilegedWeight;
   private final DoubleYoVariable kneeStraightPrivilegedPositionGain;
   private final DoubleYoVariable kneeStraightPrivilegedVelocityGain;

   private final DoubleYoVariable kneeBentPrivilegedWeight;
   private final DoubleYoVariable kneeBentPrivilegedPositionGain;
   private final DoubleYoVariable kneeBentPrivilegedVelocityGain;

   private final DoubleYoVariable privilegedMaxAcceleration;

   private final DoubleYoVariable desiredAngle;
   private final DoubleYoVariable desiredAngleWhenStraight;

   private final DoubleYoVariable straighteningSpeed;
   private final DoubleYoVariable collapsingDuration;

   private final BooleanYoVariable activelyControl;

   private final YoPDGains jointspaceGains;
   private final DoubleYoVariable jointspaceWeight;

   public KneeControlModule(RobotSide robotSide, HighLevelHumanoidControllerToolbox controllerToolbox, StraightLegWalkingParameters straightLegWalkingParameters,
         YoVariableRegistry parentRegistry)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String namePrefix = sidePrefix + "Knee";
      registry = new YoVariableRegistry(sidePrefix + getClass().getSimpleName());

      activelyControl = new BooleanYoVariable(namePrefix + "ActivelyControl", registry);
      activelyControl.set(false);

      jointspaceWeight = new DoubleYoVariable(namePrefix + "JointspaceWeight", registry);
      jointspaceWeight.set(1.0);

      jointspaceGains = new YoPDGains(namePrefix, registry);
      jointspaceGains.setKp(40.0);
      jointspaceGains.setKd(6.0);

      legPitchPrivilegedWeight = new DoubleYoVariable(sidePrefix + "LegPitchPrivilegedWeight", registry);
      legPitchPrivilegedPositionGain = new DoubleYoVariable(sidePrefix + "LegPitchPrivilegedKp", registry);
      legPitchPrivilegedVelocityGain = new DoubleYoVariable(sidePrefix + "LegPitchPrivilegedKv", registry);

      kneeStraightPrivilegedWeight = new DoubleYoVariable(sidePrefix + "KneeStraightPrivilegedWeight", registry);
      kneeStraightPrivilegedPositionGain = new DoubleYoVariable(sidePrefix + "KneeStraightPrivilegedKp", registry);
      kneeStraightPrivilegedVelocityGain = new DoubleYoVariable(sidePrefix + "KneeStraightPrivilegedKv", registry);

      kneeBentPrivilegedWeight = new DoubleYoVariable(sidePrefix + "KneeBentPrivilegedWeight", registry);
      kneeBentPrivilegedPositionGain = new DoubleYoVariable(sidePrefix + "KneeBentPrivilegedKp", registry);
      kneeBentPrivilegedVelocityGain = new DoubleYoVariable(sidePrefix + "KneeBentPrivilegedKv", registry);

      privilegedMaxAcceleration = new DoubleYoVariable(namePrefix + "PrivilegedMaxAcceleration", registry);

      legPitchPrivilegedWeight.set(straightLegWalkingParameters.getLegPitchPrivilegedWeight());
      legPitchPrivilegedPositionGain.set(straightLegWalkingParameters.getLegPitchPrivilegedConfigurationGain());
      legPitchPrivilegedVelocityGain.set(straightLegWalkingParameters.getLegPitchPrivilegedVelocityGain());

      kneeStraightPrivilegedWeight.set(straightLegWalkingParameters.getKneeStraightLegPrivilegedWeight());
      kneeStraightPrivilegedPositionGain.set(straightLegWalkingParameters.getKneeStraightLegPrivilegedConfigurationGain());
      kneeStraightPrivilegedVelocityGain.set(straightLegWalkingParameters.getKneeStraightLegPrivilegedVelocityGain());

      kneeBentPrivilegedWeight.set(straightLegWalkingParameters.getKneeBentLegPrivilegedWeight());
      kneeBentPrivilegedPositionGain.set(straightLegWalkingParameters.getKneeBentLegPrivilegedConfigurationGain());
      kneeBentPrivilegedVelocityGain.set(straightLegWalkingParameters.getKneeBentLegPrivilegedVelocityGain());

      privilegedMaxAcceleration.set(straightLegWalkingParameters.getPrivilegedMaxAcceleration());

      desiredAngle = new DoubleYoVariable(namePrefix + "DesiredAngle", registry);
      desiredAngle.set(straightLegWalkingParameters.getStraightKneeAngle());

      desiredAngleWhenStraight = new DoubleYoVariable(namePrefix + "DesiredAngleWhenStraight", registry);
      desiredAngleWhenStraight.set(straightLegWalkingParameters.getStraightKneeAngle());

      straighteningSpeed = new DoubleYoVariable(namePrefix + "SupportKneeStraighteningSpeed", registry);
      straighteningSpeed.set(straightLegWalkingParameters.getSpeedForSupportKneeStraightening());

      collapsingDuration = new DoubleYoVariable(namePrefix + "SupportKneeCollapsingDuration", registry);
      collapsingDuration.set(straightLegWalkingParameters.getSupportKneeCollapsingDuration());

      // set up states and state machine
      DoubleYoVariable time = controllerToolbox.getYoTime();
      stateMachine = new GenericStateMachine<>(namePrefix + "State", namePrefix + "SwitchTime", KneeControlType.class, time, registry);
      requestedState = EnumYoVariable.create(namePrefix + "RequestedState", "", KneeControlType.class, registry, true);
      requestedState.set(null);

      setupStateMachine(controllerToolbox, robotSide);

      if (straightLegWalkingParameters.attemptToStraightenLegs())
         stateMachine.setCurrentState(KneeControlType.STRAIGHT);
      else
         stateMachine.setCurrentState(KneeControlType.BENT);

      parentRegistry.addChild(registry);
   }

   private void setupStateMachine(HighLevelHumanoidControllerToolbox controllerToolbox, RobotSide robotSide)
   {
      OneDoFJoint hipPitchJoint = controllerToolbox.getFullRobotModel().getLegJoint(robotSide, LegJointName.HIP_PITCH);
      OneDoFJoint kneePitchJoint = controllerToolbox.getFullRobotModel().getLegJoint(robotSide, LegJointName.KNEE_PITCH);
      OneDoFJoint anklePitchJoint = controllerToolbox.getFullRobotModel().getLegJoint(robotSide, LegJointName.ANKLE_PITCH);

      List<AbstractKneeControlState> states = new ArrayList<>();

      AbstractKneeControlState straighteningToStraightState = new StraightenToStraightControlState(hipPitchJoint, kneePitchJoint, anklePitchJoint, straighteningSpeed);
      AbstractKneeControlState straightState = new StraightKneeControlState(hipPitchJoint, kneePitchJoint, anklePitchJoint);
      AbstractKneeControlState bentState = new BentKneeControlState(hipPitchJoint, kneePitchJoint, anklePitchJoint);
      AbstractKneeControlState collapseState = new CollapseKneeControlState(hipPitchJoint, kneePitchJoint, anklePitchJoint);
      AbstractKneeControlState straighteningToControlState = new StraightenToControllableControlState(hipPitchJoint, kneePitchJoint, anklePitchJoint, straighteningSpeed);
      AbstractKneeControlState controlledState = new ControllableKneeControlState(hipPitchJoint, kneePitchJoint, anklePitchJoint);
      states.add(straighteningToStraightState);
      states.add(straightState);
      states.add(bentState);
      states.add(collapseState);
      states.add(straighteningToControlState);
      states.add(controlledState);

      straighteningToStraightState.setDefaultNextState(KneeControlType.STRAIGHT);
      straighteningToControlState.setDefaultNextState(KneeControlType.CONTROLLABLE);
      collapseState.setDefaultNextState(KneeControlType.BENT);

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

   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return stateMachine.getCurrentState().getJointspaceFeedbackControlCommand();
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

      public JointspaceFeedbackControlCommand getJointspaceFeedbackControlCommand()
      {
         return null;
      }

      public PrivilegedConfigurationCommand getPrivilegedConfigurationCommand()
      {
         return privilegedConfigurationCommand;
      }
   }

   private class StraightenToStraightControlState extends StraighteningKneeControlState
   {
      public StraightenToStraightControlState(OneDoFJoint hipPitchJoint, OneDoFJoint kneePitchJoint, OneDoFJoint anklePitchJoint,
            DoubleYoVariable straighteningSpeed)
      {
         super(KneeControlType.STRAIGHTEN_TO_STRAIGHT, hipPitchJoint, kneePitchJoint, anklePitchJoint, straighteningSpeed);
      }
   }

   private class StraightenToControllableControlState extends StraighteningKneeControlState
   {
      public StraightenToControllableControlState(OneDoFJoint hipPitchJoint, OneDoFJoint kneePitchJoint, OneDoFJoint anklePitchJoint,
            DoubleYoVariable straighteningSpeed)
      {
         super(KneeControlType.STRAIGHTEN_TO_CONTROLLABLE, hipPitchJoint, kneePitchJoint, anklePitchJoint, straighteningSpeed);
      }
   }

   private class StraighteningKneeControlState extends AbstractKneeControlState
   {
      private static final int hipPitchJointIndex = 0;
      private static final int kneePitchJointIndex = 1;
      private static final int anklePitchJointIndex = 2;

      private final OneDoFJoint kneePitchJoint;

      private final DoubleYoVariable yoStraighteningSpeed;

      private double startingPosition;

      private double timeUntilStraight;
      private double straighteningSpeed;

      private double dwellTime;
      private double desiredPrivilegedPosition;

      private double previousTime;

      public StraighteningKneeControlState(KneeControlType stateEnum, OneDoFJoint hipPitchJoint, OneDoFJoint kneePitchJoint, OneDoFJoint anklePitchJoint,
            DoubleYoVariable straighteningSpeed)
      {
         super(stateEnum);

         this.kneePitchJoint = kneePitchJoint;

         this.yoStraighteningSpeed = straighteningSpeed;

         privilegedConfigurationCommand.addJoint(hipPitchJoint, PrivilegedConfigurationOption.AT_ZERO);
         privilegedConfigurationCommand.addJoint(kneePitchJoint, Double.NaN);
         privilegedConfigurationCommand.addJoint(anklePitchJoint, PrivilegedConfigurationOption.AT_ZERO);
      }

      @Override
      public boolean isDone()
      {
         return getTimeInCurrentState() > (timeUntilStraight + dwellTime);
      }

      @Override
      public void doAction()
      {
         double estimatedDT = estimateDT();
         double currentPosition = kneePitchJoint.getQ();

         if (ONLY_MOVE_PRIV_POS_IF_NOT_BENDING)
         {
            if (currentPosition > startingPosition) // the knee is bending
               dwellTime += estimatedDT;
            else
               desiredPrivilegedPosition -= estimatedDT * straighteningSpeed;
         }
         else
         {
            desiredPrivilegedPosition -= estimatedDT * straighteningSpeed;
         }

         double gainModifier = 1.0;
         if (SCALE_STRAIGHT_GAIN_WITH_ERROR)
         {
            double absoluteError = Math.abs(kneePitchJoint.getQ() - desiredAngleWhenStraight.getDoubleValue()) / (2.0 * Math.PI);
            gainModifier = 1.0 / (1.0 + absoluteError);
         }

         privilegedConfigurationCommand.setWeight(hipPitchJointIndex, legPitchPrivilegedWeight.getDoubleValue());
         privilegedConfigurationCommand.setConfigurationGain(hipPitchJointIndex, legPitchPrivilegedPositionGain.getDoubleValue());
         privilegedConfigurationCommand.setVelocityGain(hipPitchJointIndex, legPitchPrivilegedVelocityGain.getDoubleValue());
         privilegedConfigurationCommand.setMaxAcceleration(hipPitchJointIndex, privilegedMaxAcceleration.getDoubleValue());

         privilegedConfigurationCommand.setOneDoFJoint(kneePitchJointIndex, desiredPrivilegedPosition);
         privilegedConfigurationCommand.setWeight(kneePitchJointIndex, kneeStraightPrivilegedWeight.getDoubleValue());
         privilegedConfigurationCommand.setConfigurationGain(kneePitchJointIndex, gainModifier * kneeStraightPrivilegedPositionGain.getDoubleValue());
         privilegedConfigurationCommand.setVelocityGain(kneePitchJointIndex, kneeStraightPrivilegedVelocityGain.getDoubleValue());
         privilegedConfigurationCommand.setMaxAcceleration(kneePitchJointIndex, privilegedMaxAcceleration.getDoubleValue());

         privilegedConfigurationCommand.setWeight(anklePitchJointIndex, legPitchPrivilegedWeight.getDoubleValue());
         privilegedConfigurationCommand.setConfigurationGain(anklePitchJointIndex, legPitchPrivilegedPositionGain.getDoubleValue());
         privilegedConfigurationCommand.setVelocityGain(anklePitchJointIndex, legPitchPrivilegedVelocityGain.getDoubleValue());
         privilegedConfigurationCommand.setMaxAcceleration(anklePitchJointIndex, privilegedMaxAcceleration.getDoubleValue());

         if (isDone())
            transitionToDefaultNextState();
      }

      @Override
      public void doTransitionIntoAction()
      {
         startingPosition = kneePitchJoint.getQ();

         straighteningSpeed = yoStraighteningSpeed.getDoubleValue();
         timeUntilStraight = (startingPosition - desiredAngleWhenStraight.getDoubleValue()) / straighteningSpeed;
         timeUntilStraight = Math.max(timeUntilStraight, 0.0);

         desiredPrivilegedPosition = startingPosition;

         previousTime = 0.0;
         dwellTime = 0.0;
      }

      @Override
      public void doTransitionOutOfAction()
      {
      }

      private double estimateDT()
      {
         double currentTime = getTimeInCurrentState();

         double estimatedDT = currentTime - previousTime;
         previousTime = currentTime;

         return estimatedDT;
      }
   }

   private class StraightKneeControlState extends AbstractKneeControlState
   {
      private final JointspaceFeedbackControlCommand jointspaceFeedbackControlCommand = new JointspaceFeedbackControlCommand();

      private static final int hipPitchJointIndex = 0;
      private static final int kneePitchJointIndex = 1;
      private static final int anklePitchJointIndex = 2;

      private final OneDoFJoint kneePitchJoint;

      public StraightKneeControlState(OneDoFJoint hipPitchJoint, OneDoFJoint kneePitchJoint, OneDoFJoint anklePitchJoint)
      {
         super(KneeControlType.STRAIGHT);

         this.kneePitchJoint = kneePitchJoint;

         jointspaceFeedbackControlCommand.addJoint(kneePitchJoint, Double.NaN, Double.NaN, Double.NaN);

         privilegedConfigurationCommand.addJoint(hipPitchJoint, PrivilegedConfigurationOption.AT_ZERO);
         privilegedConfigurationCommand.addJoint(kneePitchJoint, Double.NaN);
         privilegedConfigurationCommand.addJoint(anklePitchJoint, PrivilegedConfigurationOption.AT_ZERO);
      }

      @Override
      public boolean isDone()
      {
         return false;
      }

      @Override
      public void doAction()
      {
         double gainModifier = 1.0;
         if (SCALE_STRAIGHT_GAIN_WITH_ERROR)
         {
            double absoluteError = Math.abs(kneePitchJoint.getQ() - desiredAngleWhenStraight.getDoubleValue()) / (2.0 * Math.PI);
            gainModifier = 1.0 / (1.0 + absoluteError);
         }

         privilegedConfigurationCommand.setWeight(hipPitchJointIndex, legPitchPrivilegedWeight.getDoubleValue());
         privilegedConfigurationCommand.setConfigurationGain(hipPitchJointIndex, legPitchPrivilegedPositionGain.getDoubleValue());
         privilegedConfigurationCommand.setVelocityGain(hipPitchJointIndex, legPitchPrivilegedVelocityGain.getDoubleValue());
         privilegedConfigurationCommand.setMaxAcceleration(hipPitchJointIndex, privilegedMaxAcceleration.getDoubleValue());

         privilegedConfigurationCommand.setOneDoFJoint(kneePitchJointIndex, desiredAngleWhenStraight.getDoubleValue());
         privilegedConfigurationCommand.setWeight(kneePitchJointIndex, kneeStraightPrivilegedWeight.getDoubleValue());
         privilegedConfigurationCommand.setConfigurationGain(kneePitchJointIndex, gainModifier * kneeStraightPrivilegedPositionGain.getDoubleValue());
         privilegedConfigurationCommand.setVelocityGain(kneePitchJointIndex, kneeStraightPrivilegedVelocityGain.getDoubleValue());
         privilegedConfigurationCommand.setMaxAcceleration(kneePitchJointIndex, privilegedMaxAcceleration.getDoubleValue());

         privilegedConfigurationCommand.setWeight(anklePitchJointIndex, legPitchPrivilegedWeight.getDoubleValue());
         privilegedConfigurationCommand.setConfigurationGain(anklePitchJointIndex, legPitchPrivilegedPositionGain.getDoubleValue());
         privilegedConfigurationCommand.setVelocityGain(anklePitchJointIndex, legPitchPrivilegedVelocityGain.getDoubleValue());
         privilegedConfigurationCommand.setMaxAcceleration(anklePitchJointIndex, privilegedMaxAcceleration.getDoubleValue());

         jointspaceFeedbackControlCommand.setOneDoFJoint(0, desiredAngleWhenStraight.getDoubleValue(), 0.0, 0.0);
         jointspaceFeedbackControlCommand.setGains(jointspaceGains);
         jointspaceFeedbackControlCommand.setWeightForSolver(jointspaceWeight.getDoubleValue());
      }

      @Override
      public void doTransitionIntoAction()
      {
      }

      @Override
      public void doTransitionOutOfAction()
      {
      }

      @Override
      public JointspaceFeedbackControlCommand getJointspaceFeedbackControlCommand()
      {
         if (activelyControl.getBooleanValue())
            return jointspaceFeedbackControlCommand;
         else
            return null;
      }
   }

   private class BentKneeControlState extends AbstractKneeControlState
   {
      private static final int hipPitchJointIndex = 0;
      private static final int kneePitchJointIndex = 1;
      private static final int anklePitchJointIndex = 2;

      public BentKneeControlState(OneDoFJoint hipPitchJoint, OneDoFJoint kneePitchJoint, OneDoFJoint anklePitchJoint)
      {
         super(KneeControlType.BENT);

         privilegedConfigurationCommand.addJoint(hipPitchJoint, PrivilegedConfigurationOption.AT_ZERO);
         privilegedConfigurationCommand.addJoint(kneePitchJoint, PrivilegedConfigurationOption.AT_MID_RANGE);
         privilegedConfigurationCommand.addJoint(anklePitchJoint, PrivilegedConfigurationOption.AT_ZERO);
      }

      @Override
      public boolean isDone()
      {
         return false;
      }

      @Override
      public void doAction()
      {
         privilegedConfigurationCommand.setWeight(hipPitchJointIndex, legPitchPrivilegedWeight.getDoubleValue());
         privilegedConfigurationCommand.setConfigurationGain(hipPitchJointIndex, legPitchPrivilegedPositionGain.getDoubleValue());
         privilegedConfigurationCommand.setVelocityGain(hipPitchJointIndex, legPitchPrivilegedVelocityGain.getDoubleValue());
         privilegedConfigurationCommand.setMaxAcceleration(hipPitchJointIndex, privilegedMaxAcceleration.getDoubleValue());

         privilegedConfigurationCommand.setWeight(kneePitchJointIndex, kneeBentPrivilegedWeight.getDoubleValue());
         privilegedConfigurationCommand.setConfigurationGain(kneePitchJointIndex, kneeBentPrivilegedPositionGain.getDoubleValue());
         privilegedConfigurationCommand.setVelocityGain(kneePitchJointIndex, kneeBentPrivilegedVelocityGain.getDoubleValue());
         privilegedConfigurationCommand.setMaxAcceleration(kneePitchJointIndex, privilegedMaxAcceleration.getDoubleValue());

         privilegedConfigurationCommand.setWeight(anklePitchJointIndex, legPitchPrivilegedWeight.getDoubleValue());
         privilegedConfigurationCommand.setConfigurationGain(anklePitchJointIndex, legPitchPrivilegedPositionGain.getDoubleValue());
         privilegedConfigurationCommand.setVelocityGain(anklePitchJointIndex, legPitchPrivilegedVelocityGain.getDoubleValue());
         privilegedConfigurationCommand.setMaxAcceleration(anklePitchJointIndex, privilegedMaxAcceleration.getDoubleValue());
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

   private class CollapseKneeControlState extends AbstractKneeControlState
   {
      private static final int hipPitchJointIndex = 0;
      private static final int kneePitchJointIndex = 1;
      private static final int anklePitchJointIndex = 2;

      private final double kneePitchMidrange;

      public CollapseKneeControlState(OneDoFJoint hipPitchJoint, OneDoFJoint kneePitchJoint, OneDoFJoint anklePitchJoint)
      {
         super(KneeControlType.COLLAPSE);

         privilegedConfigurationCommand.addJoint(hipPitchJoint, PrivilegedConfigurationOption.AT_ZERO);
         privilegedConfigurationCommand.addJoint(kneePitchJoint, Double.NaN);
         privilegedConfigurationCommand.addJoint(anklePitchJoint, PrivilegedConfigurationOption.AT_ZERO);

         double kneePitchLimitUpper = kneePitchJoint.getJointLimitUpper();
         if (Double.isNaN(kneePitchLimitUpper) || Double.isInfinite(kneePitchLimitUpper))
            kneePitchLimitUpper = Math.PI;
         double kneePitchLimitLower = kneePitchJoint.getJointLimitLower();
         if (Double.isNaN(kneePitchLimitLower) || Double.isInfinite(kneePitchLimitLower))
            kneePitchLimitLower = -Math.PI;
         kneePitchMidrange = 0.5 * (kneePitchLimitUpper - kneePitchLimitLower);
      }

      @Override
      public boolean isDone()
      {
         return getTimeInCurrentState() > collapsingDuration.getDoubleValue();
      }

      @Override
      public void doAction()
      {
         privilegedConfigurationCommand.setWeight(hipPitchJointIndex, legPitchPrivilegedWeight.getDoubleValue());
         privilegedConfigurationCommand.setConfigurationGain(hipPitchJointIndex, legPitchPrivilegedPositionGain.getDoubleValue());
         privilegedConfigurationCommand.setVelocityGain(hipPitchJointIndex, legPitchPrivilegedVelocityGain.getDoubleValue());
         privilegedConfigurationCommand.setMaxAcceleration(hipPitchJointIndex, privilegedMaxAcceleration.getDoubleValue());

         double desiredKneePosition = InterpolationTools.linearInterpolate(desiredAngleWhenStraight.getDoubleValue(), kneePitchMidrange,
               getTimeInCurrentState() / collapsingDuration.getDoubleValue());

         privilegedConfigurationCommand.setOneDoFJoint(kneePitchJointIndex, desiredKneePosition);
         privilegedConfigurationCommand.setWeight(kneePitchJointIndex, kneeBentPrivilegedWeight.getDoubleValue());
         privilegedConfigurationCommand.setConfigurationGain(kneePitchJointIndex, kneeBentPrivilegedPositionGain.getDoubleValue());
         privilegedConfigurationCommand.setVelocityGain(kneePitchJointIndex, kneeBentPrivilegedVelocityGain.getDoubleValue());
         privilegedConfigurationCommand.setMaxAcceleration(kneePitchJointIndex, privilegedMaxAcceleration.getDoubleValue());

         privilegedConfigurationCommand.setWeight(anklePitchJointIndex, legPitchPrivilegedWeight.getDoubleValue());
         privilegedConfigurationCommand.setConfigurationGain(anklePitchJointIndex, legPitchPrivilegedPositionGain.getDoubleValue());
         privilegedConfigurationCommand.setVelocityGain(anklePitchJointIndex, legPitchPrivilegedVelocityGain.getDoubleValue());
         privilegedConfigurationCommand.setMaxAcceleration(anklePitchJointIndex, privilegedMaxAcceleration.getDoubleValue());

         if (isDone())
            transitionToDefaultNextState();
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
      private static final int hipPitchJointIndex = 0;
      private static final int kneePitchJointIndex = 1;
      private static final int anklePitchJointIndex = 2;

      public ControllableKneeControlState(OneDoFJoint hipPitchJoint, OneDoFJoint kneePitchJoint, OneDoFJoint anklePitchJoint)
      {
         super(KneeControlType.CONTROLLABLE);

         privilegedConfigurationCommand.addJoint(hipPitchJoint, PrivilegedConfigurationOption.AT_ZERO);
         privilegedConfigurationCommand.addJoint(kneePitchJoint, Double.NaN);
         privilegedConfigurationCommand.addJoint(anklePitchJoint, PrivilegedConfigurationOption.AT_ZERO);
      }

      @Override
      public boolean isDone()
      {
         return false;
      }

      @Override
      public void doAction()
      {
         privilegedConfigurationCommand.setWeight(hipPitchJointIndex, legPitchPrivilegedWeight.getDoubleValue());
         privilegedConfigurationCommand.setConfigurationGain(hipPitchJointIndex, legPitchPrivilegedPositionGain.getDoubleValue());
         privilegedConfigurationCommand.setVelocityGain(hipPitchJointIndex, legPitchPrivilegedVelocityGain.getDoubleValue());
         privilegedConfigurationCommand.setMaxAcceleration(hipPitchJointIndex, privilegedMaxAcceleration.getDoubleValue());

         privilegedConfigurationCommand.setOneDoFJoint(kneePitchJointIndex, desiredAngle.getDoubleValue());
         privilegedConfigurationCommand.setWeight(kneePitchJointIndex, kneeBentPrivilegedWeight.getDoubleValue());
         privilegedConfigurationCommand.setConfigurationGain(kneePitchJointIndex, kneeBentPrivilegedPositionGain.getDoubleValue());
         privilegedConfigurationCommand.setVelocityGain(kneePitchJointIndex, kneeBentPrivilegedVelocityGain.getDoubleValue());
         privilegedConfigurationCommand.setMaxAcceleration(kneePitchJointIndex, privilegedMaxAcceleration.getDoubleValue());

         privilegedConfigurationCommand.setWeight(anklePitchJointIndex, legPitchPrivilegedWeight.getDoubleValue());
         privilegedConfigurationCommand.setConfigurationGain(anklePitchJointIndex, legPitchPrivilegedPositionGain.getDoubleValue());
         privilegedConfigurationCommand.setVelocityGain(anklePitchJointIndex, legPitchPrivilegedVelocityGain.getDoubleValue());
         privilegedConfigurationCommand.setMaxAcceleration(anklePitchJointIndex, privilegedMaxAcceleration.getDoubleValue());
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
