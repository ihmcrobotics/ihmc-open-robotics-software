package us.ihmc.commonWalkingControlModules.controlModules.legConfiguration;

import us.ihmc.commonWalkingControlModules.configurations.StraightLegWalkingParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.robotics.InterpolationTools;
import us.ihmc.robotics.MathTools;
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

public class LegConfigurationControlModule
{
   public enum LegConfigurationType
   {
      STRAIGHTEN_TO_STRAIGHT, STRAIGHT, STRAIGHTEN_TO_CONTROLLABLE, COLLAPSE, BENT, CONTROLLABLE
   }

   private static final boolean ONLY_MOVE_PRIV_POS_IF_NOT_BENDING = true;
   private static final boolean SCALE_STRAIGHT_GAIN_WITH_ERROR = false;

   private final YoVariableRegistry registry;

   private final EnumYoVariable<LegConfigurationType> requestedState;
   private final GenericStateMachine<LegConfigurationType, AbstractLegConfigurationState> stateMachine;

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

   private final double kneeRangeOfMotion;
   private final double kneeMidRangeOfMotion;

   public LegConfigurationControlModule(RobotSide robotSide, HighLevelHumanoidControllerToolbox controllerToolbox, StraightLegWalkingParameters straightLegWalkingParameters,
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

      OneDoFJoint kneePitchJoint = controllerToolbox.getFullRobotModel().getLegJoint(robotSide, LegJointName.KNEE_PITCH);
      double kneeLimitUpper = kneePitchJoint.getJointLimitUpper();
      if (Double.isNaN(kneeLimitUpper) || Double.isInfinite(kneeLimitUpper))
         kneeLimitUpper = Math.PI;
      double kneeLimitLower = kneePitchJoint.getJointLimitLower();
      if (Double.isNaN(kneeLimitLower) || Double.isInfinite(kneeLimitLower))
         kneeLimitLower = -Math.PI;
      kneeRangeOfMotion = MathTools.square(kneeLimitUpper - kneeLimitLower);
      kneeMidRangeOfMotion = 0.5 * (kneeLimitUpper + kneeLimitLower);

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
      stateMachine = new GenericStateMachine<>(namePrefix + "State", namePrefix + "SwitchTime", LegConfigurationType.class, time, registry);
      requestedState = EnumYoVariable.create(namePrefix + "RequestedState", "", LegConfigurationType.class, registry, true);
      requestedState.set(null);

      setupStateMachine(controllerToolbox, robotSide);

      if (straightLegWalkingParameters.attemptToStraightenLegs())
         stateMachine.setCurrentState(LegConfigurationType.STRAIGHT);
      else
         stateMachine.setCurrentState(LegConfigurationType.BENT);

      parentRegistry.addChild(registry);
   }

   private void setupStateMachine(HighLevelHumanoidControllerToolbox controllerToolbox, RobotSide robotSide)
   {
      OneDoFJoint hipPitchJoint = controllerToolbox.getFullRobotModel().getLegJoint(robotSide, LegJointName.HIP_PITCH);
      OneDoFJoint kneePitchJoint = controllerToolbox.getFullRobotModel().getLegJoint(robotSide, LegJointName.KNEE_PITCH);
      OneDoFJoint anklePitchJoint = controllerToolbox.getFullRobotModel().getLegJoint(robotSide, LegJointName.ANKLE_PITCH);

      List<AbstractLegConfigurationState> states = new ArrayList<>();

      AbstractLegConfigurationState straighteningToStraightState = new StraightenToStraightControlState(hipPitchJoint, kneePitchJoint, anklePitchJoint, straighteningSpeed);
      AbstractLegConfigurationState straightState = new StraightKneeControlState(hipPitchJoint, kneePitchJoint, anklePitchJoint);
      AbstractLegConfigurationState bentState = new BentKneeControlState(hipPitchJoint, kneePitchJoint, anklePitchJoint);
      AbstractLegConfigurationState collapseState = new CollapseKneeControlState(hipPitchJoint, kneePitchJoint, anklePitchJoint);
      AbstractLegConfigurationState straighteningToControlState = new StraightenToControllableControlState(hipPitchJoint, kneePitchJoint, anklePitchJoint, straighteningSpeed);
      AbstractLegConfigurationState controlledState = new ControllableKneeControlState(hipPitchJoint, kneePitchJoint, anklePitchJoint);
      states.add(straighteningToStraightState);
      states.add(straightState);
      states.add(bentState);
      states.add(collapseState);
      states.add(straighteningToControlState);
      states.add(controlledState);

      straighteningToStraightState.setDefaultNextState(LegConfigurationType.STRAIGHT);
      straighteningToControlState.setDefaultNextState(LegConfigurationType.CONTROLLABLE);
      collapseState.setDefaultNextState(LegConfigurationType.BENT);

      for (AbstractLegConfigurationState fromState : states)
      {
         for (AbstractLegConfigurationState toState : states)
         {
            StateMachineTools.addRequestedStateTransition(requestedState, false, fromState, toState);
         }
      }

      for (AbstractLegConfigurationState state : states)
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

   public void setKneeAngleState(LegConfigurationType controlType)
   {
      requestedState.set(controlType);
   }

   public LegConfigurationType getCurrentKneeControlState()
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

   private abstract class AbstractLegConfigurationState extends FinishableState<LegConfigurationType>
   {
      protected final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
      protected final PrivilegedAccelerationCommand privilegedAccelerationCommand = new PrivilegedAccelerationCommand();

      public AbstractLegConfigurationState(LegConfigurationType stateEnum)
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

      public PrivilegedAccelerationCommand getPrivilegedAccelerationCommand()
      {
         return privilegedAccelerationCommand;
      }

      protected double computeKneeAcceleration(double kp, double kd, double desiredPosition, double currentPosition, double currentVelocity, double maxAcceleration)
      {
         double qdd = 2.0 * kp * (desiredPosition - currentPosition) / kneeRangeOfMotion;
         qdd -= kd * currentVelocity;
         return MathTools.clamp(qdd, maxAcceleration);
      }

   }

   private class StraightenToStraightControlState extends StraighteningKneeControlState
   {
      public StraightenToStraightControlState(OneDoFJoint hipPitchJoint, OneDoFJoint kneePitchJoint, OneDoFJoint anklePitchJoint,
            DoubleYoVariable straighteningSpeed)
      {
         super(LegConfigurationType.STRAIGHTEN_TO_STRAIGHT, hipPitchJoint, kneePitchJoint, anklePitchJoint, straighteningSpeed);
      }
   }

   private class StraightenToControllableControlState extends StraighteningKneeControlState
   {
      public StraightenToControllableControlState(OneDoFJoint hipPitchJoint, OneDoFJoint kneePitchJoint, OneDoFJoint anklePitchJoint,
            DoubleYoVariable straighteningSpeed)
      {
         super(LegConfigurationType.STRAIGHTEN_TO_CONTROLLABLE, hipPitchJoint, kneePitchJoint, anklePitchJoint, straighteningSpeed);
      }
   }

   private class StraighteningKneeControlState extends AbstractLegConfigurationState
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

      public StraighteningKneeControlState(LegConfigurationType stateEnum, OneDoFJoint hipPitchJoint, OneDoFJoint kneePitchJoint, OneDoFJoint anklePitchJoint,
            DoubleYoVariable straighteningSpeed)
      {
         super(stateEnum);

         this.kneePitchJoint = kneePitchJoint;
         this.yoStraighteningSpeed = straighteningSpeed;

         privilegedConfigurationCommand.addJoint(hipPitchJoint, PrivilegedConfigurationOption.AT_ZERO);
         privilegedConfigurationCommand.addJoint(kneePitchJoint, Double.NaN);
         privilegedConfigurationCommand.addJoint(anklePitchJoint, PrivilegedConfigurationOption.AT_ZERO);

         privilegedAccelerationCommand.addJoint(hipPitchJoint, Double.NaN);
         privilegedAccelerationCommand.addJoint(kneePitchJoint, Double.NaN);
         privilegedAccelerationCommand.addJoint(anklePitchJoint, Double.NaN);

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
         double currentVelocity = kneePitchJoint.getQd();

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
            double absoluteError = Math.abs(currentPosition - desiredAngleWhenStraight.getDoubleValue()) / (2.0 * Math.PI);
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

         double privilegedKneeAcceleration = computeKneeAcceleration(kneeStraightPrivilegedPositionGain.getDoubleValue(), kneeStraightPrivilegedVelocityGain.getDoubleValue(),
                                                                 desiredPrivilegedPosition, currentPosition, currentVelocity, privilegedMaxAcceleration.getDoubleValue());
         double privilegedHipPitchAcceleration = -0.5 * privilegedKneeAcceleration;
         double privilegedAnklePitchAcceleration = -0.5 * privilegedKneeAcceleration;

         privilegedAccelerationCommand.setOneDoFJoint(hipPitchJointIndex, privilegedHipPitchAcceleration);
         privilegedAccelerationCommand.setOneDoFJoint(kneePitchJointIndex, privilegedKneeAcceleration);
         privilegedAccelerationCommand.setOneDoFJoint(anklePitchJointIndex, privilegedAnklePitchAcceleration);

         privilegedAccelerationCommand.setWeight(hipPitchJointIndex, legPitchPrivilegedWeight.getDoubleValue());
         privilegedAccelerationCommand.setWeight(kneePitchJointIndex, kneeStraightPrivilegedWeight.getDoubleValue());
         privilegedAccelerationCommand.setWeight(anklePitchJointIndex, legPitchPrivilegedWeight.getDoubleValue());

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

   private class StraightKneeControlState extends AbstractLegConfigurationState
   {
      private final JointspaceFeedbackControlCommand jointspaceFeedbackControlCommand = new JointspaceFeedbackControlCommand();

      private static final int hipPitchJointIndex = 0;
      private static final int kneePitchJointIndex = 1;
      private static final int anklePitchJointIndex = 2;

      private final OneDoFJoint kneePitchJoint;

      public StraightKneeControlState(OneDoFJoint hipPitchJoint, OneDoFJoint kneePitchJoint, OneDoFJoint anklePitchJoint)
      {
         super(LegConfigurationType.STRAIGHT);

         this.kneePitchJoint = kneePitchJoint;

         jointspaceFeedbackControlCommand.addJoint(kneePitchJoint, Double.NaN, Double.NaN, Double.NaN);

         privilegedConfigurationCommand.addJoint(hipPitchJoint, PrivilegedConfigurationOption.AT_ZERO);
         privilegedConfigurationCommand.addJoint(kneePitchJoint, Double.NaN);
         privilegedConfigurationCommand.addJoint(anklePitchJoint, PrivilegedConfigurationOption.AT_ZERO);

         privilegedAccelerationCommand.addJoint(hipPitchJoint, Double.NaN);
         privilegedAccelerationCommand.addJoint(kneePitchJoint, Double.NaN);
         privilegedAccelerationCommand.addJoint(anklePitchJoint, Double.NaN);
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

         // directly set the privileged accelerations
         double privilegedKneeAcceleration = computeKneeAcceleration(kneeStraightPrivilegedPositionGain.getDoubleValue(), kneeStraightPrivilegedVelocityGain.getDoubleValue(),
                                                                     desiredAngleWhenStraight.getDoubleValue(), kneePitchJoint.getQ(), kneePitchJoint.getQd(),
                                                                     privilegedMaxAcceleration.getDoubleValue());
         double privilegedHipPitchAcceleration = -0.5 * privilegedKneeAcceleration;
         double privilegedAnklePitchAcceleration = -0.5 * privilegedKneeAcceleration;

         privilegedAccelerationCommand.setOneDoFJoint(hipPitchJointIndex, privilegedHipPitchAcceleration);
         privilegedAccelerationCommand.setOneDoFJoint(kneePitchJointIndex, privilegedKneeAcceleration);
         privilegedAccelerationCommand.setOneDoFJoint(anklePitchJointIndex, privilegedAnklePitchAcceleration);

         privilegedAccelerationCommand.setWeight(hipPitchJointIndex, legPitchPrivilegedWeight.getDoubleValue());
         privilegedAccelerationCommand.setWeight(kneePitchJointIndex, kneeStraightPrivilegedWeight.getDoubleValue());
         privilegedAccelerationCommand.setWeight(anklePitchJointIndex, legPitchPrivilegedWeight.getDoubleValue());
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

   private class BentKneeControlState extends AbstractLegConfigurationState
   {
      private static final int hipPitchJointIndex = 0;
      private static final int kneePitchJointIndex = 1;
      private static final int anklePitchJointIndex = 2;

      private final OneDoFJoint kneePitchJoint;

      public BentKneeControlState(OneDoFJoint hipPitchJoint, OneDoFJoint kneePitchJoint, OneDoFJoint anklePitchJoint)
      {
         super(LegConfigurationType.BENT);

         this.kneePitchJoint = kneePitchJoint;

         privilegedConfigurationCommand.addJoint(hipPitchJoint, PrivilegedConfigurationOption.AT_ZERO);
         privilegedConfigurationCommand.addJoint(kneePitchJoint, PrivilegedConfigurationOption.AT_MID_RANGE);
         privilegedConfigurationCommand.addJoint(anklePitchJoint, PrivilegedConfigurationOption.AT_ZERO);

         privilegedAccelerationCommand.addJoint(hipPitchJoint, Double.NaN);
         privilegedAccelerationCommand.addJoint(kneePitchJoint, Double.NaN);
         privilegedAccelerationCommand.addJoint(anklePitchJoint, Double.NaN);
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

         // directly set the privileged accelerations
         double privilegedKneeAcceleration = computeKneeAcceleration(kneeBentPrivilegedPositionGain.getDoubleValue(), kneeBentPrivilegedVelocityGain.getDoubleValue(),
                                                                     kneeMidRangeOfMotion, kneePitchJoint.getQ(), kneePitchJoint.getQd(),
                                                                     privilegedMaxAcceleration.getDoubleValue());
         double privilegedHipPitchAcceleration = -0.5 * privilegedKneeAcceleration;
         double privilegedAnklePitchAcceleration = -0.5 * privilegedKneeAcceleration;

         privilegedAccelerationCommand.setOneDoFJoint(hipPitchJointIndex, privilegedHipPitchAcceleration);
         privilegedAccelerationCommand.setOneDoFJoint(kneePitchJointIndex, privilegedKneeAcceleration);
         privilegedAccelerationCommand.setOneDoFJoint(anklePitchJointIndex, privilegedAnklePitchAcceleration);

         privilegedAccelerationCommand.setWeight(hipPitchJointIndex, legPitchPrivilegedWeight.getDoubleValue());
         privilegedAccelerationCommand.setWeight(kneePitchJointIndex, kneeBentPrivilegedWeight.getDoubleValue());
         privilegedAccelerationCommand.setWeight(anklePitchJointIndex, legPitchPrivilegedWeight.getDoubleValue());
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

   private class CollapseKneeControlState extends AbstractLegConfigurationState
   {
      private static final int hipPitchJointIndex = 0;
      private static final int kneePitchJointIndex = 1;
      private static final int anklePitchJointIndex = 2;

      private final OneDoFJoint kneePitchJoint;

      public CollapseKneeControlState(OneDoFJoint hipPitchJoint, OneDoFJoint kneePitchJoint, OneDoFJoint anklePitchJoint)
      {
         super(LegConfigurationType.COLLAPSE);

         this.kneePitchJoint = kneePitchJoint;

         privilegedConfigurationCommand.addJoint(hipPitchJoint, PrivilegedConfigurationOption.AT_ZERO);
         privilegedConfigurationCommand.addJoint(kneePitchJoint, Double.NaN);
         privilegedConfigurationCommand.addJoint(anklePitchJoint, PrivilegedConfigurationOption.AT_ZERO);

         privilegedAccelerationCommand.addJoint(hipPitchJoint, Double.NaN);
         privilegedAccelerationCommand.addJoint(kneePitchJoint, Double.NaN);
         privilegedAccelerationCommand.addJoint(anklePitchJoint, Double.NaN);
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

         double desiredKneePosition = InterpolationTools.linearInterpolate(desiredAngleWhenStraight.getDoubleValue(), kneeMidRangeOfMotion,
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

         // directly set the privileged accelerations
         double privilegedKneeAcceleration = computeKneeAcceleration(kneeBentPrivilegedPositionGain.getDoubleValue(), kneeBentPrivilegedVelocityGain.getDoubleValue(),
                                                                     desiredKneePosition, kneePitchJoint.getQ(), kneePitchJoint.getQd(),
                                                                     privilegedMaxAcceleration.getDoubleValue());
         double privilegedHipPitchAcceleration = -0.5 * privilegedKneeAcceleration;
         double privilegedAnklePitchAcceleration = -0.5 * privilegedKneeAcceleration;

         privilegedAccelerationCommand.setOneDoFJoint(hipPitchJointIndex, privilegedHipPitchAcceleration);
         privilegedAccelerationCommand.setOneDoFJoint(kneePitchJointIndex, privilegedKneeAcceleration);
         privilegedAccelerationCommand.setOneDoFJoint(anklePitchJointIndex, privilegedAnklePitchAcceleration);

         privilegedAccelerationCommand.setWeight(hipPitchJointIndex, legPitchPrivilegedWeight.getDoubleValue());
         privilegedAccelerationCommand.setWeight(kneePitchJointIndex, kneeBentPrivilegedWeight.getDoubleValue());
         privilegedAccelerationCommand.setWeight(anklePitchJointIndex, legPitchPrivilegedWeight.getDoubleValue());

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

   private class ControllableKneeControlState extends AbstractLegConfigurationState
   {
      private static final int hipPitchJointIndex = 0;
      private static final int kneePitchJointIndex = 1;
      private static final int anklePitchJointIndex = 2;

      private final OneDoFJoint kneePitchJoint;

      public ControllableKneeControlState(OneDoFJoint hipPitchJoint, OneDoFJoint kneePitchJoint, OneDoFJoint anklePitchJoint)
      {
         super(LegConfigurationType.CONTROLLABLE);

         this.kneePitchJoint = kneePitchJoint;

         privilegedConfigurationCommand.addJoint(hipPitchJoint, PrivilegedConfigurationOption.AT_ZERO);
         privilegedConfigurationCommand.addJoint(kneePitchJoint, Double.NaN);
         privilegedConfigurationCommand.addJoint(anklePitchJoint, PrivilegedConfigurationOption.AT_ZERO);

         privilegedAccelerationCommand.addJoint(hipPitchJoint, Double.NaN);
         privilegedAccelerationCommand.addJoint(kneePitchJoint, Double.NaN);
         privilegedAccelerationCommand.addJoint(anklePitchJoint, Double.NaN);
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

         // directly set the privileged accelerations
         double privilegedKneeAcceleration = computeKneeAcceleration(kneeBentPrivilegedPositionGain.getDoubleValue(), kneeBentPrivilegedVelocityGain.getDoubleValue(),
                                                                     desiredAngle.getDoubleValue(), kneePitchJoint.getQ(), kneePitchJoint.getQd(),
                                                                     privilegedMaxAcceleration.getDoubleValue());
         double privilegedHipPitchAcceleration = -0.5 * privilegedKneeAcceleration;
         double privilegedAnklePitchAcceleration = -0.5 * privilegedKneeAcceleration;

         privilegedAccelerationCommand.setOneDoFJoint(hipPitchJointIndex, privilegedHipPitchAcceleration);
         privilegedAccelerationCommand.setOneDoFJoint(kneePitchJointIndex, privilegedKneeAcceleration);
         privilegedAccelerationCommand.setOneDoFJoint(anklePitchJointIndex, privilegedAnklePitchAcceleration);

         privilegedAccelerationCommand.setWeight(hipPitchJointIndex, legPitchPrivilegedWeight.getDoubleValue());
         privilegedAccelerationCommand.setWeight(kneePitchJointIndex, kneeBentPrivilegedWeight.getDoubleValue());
         privilegedAccelerationCommand.setWeight(anklePitchJointIndex, legPitchPrivilegedWeight.getDoubleValue());
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
