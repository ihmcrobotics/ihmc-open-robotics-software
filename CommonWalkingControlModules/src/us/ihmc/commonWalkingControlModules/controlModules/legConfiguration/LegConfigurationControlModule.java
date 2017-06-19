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
      STRAIGHTEN_TO_STRAIGHT, STRAIGHT, STRAIGHTEN_TO_CONTROLLABLE, COLLAPSE, BENT
   }

   private static final boolean ONLY_MOVE_PRIV_POS_IF_NOT_BENDING = true;
   private static final boolean SCALE_STRAIGHT_GAIN_WITH_ERROR = false;

   private final YoVariableRegistry registry;

   private final EnumYoVariable<LegConfigurationType> requestedState;
   private final GenericStateMachine<LegConfigurationType, AbstractLegConfigurationState> stateMachine;

   private final DoubleYoVariable legPitchPrivilegedWeight;

   private final DoubleYoVariable kneeStraightPrivilegedWeight;
   private final DoubleYoVariable kneeStraightPrivilegedPositionGain;
   private final DoubleYoVariable kneeStraightPrivilegedVelocityGain;

   private final DoubleYoVariable kneeBentPrivilegedWeight;
   private final DoubleYoVariable kneeBentPrivilegedPositionGain;
   private final DoubleYoVariable kneeBentPrivilegedVelocityGain;

   private final DoubleYoVariable kneePitchPrivilegedConfiguration;
   private final DoubleYoVariable privilegedMaxAcceleration;

   private final DoubleYoVariable desiredAngleWhenStraight;

   private final DoubleYoVariable straighteningSpeed;
   private final DoubleYoVariable collapsingDuration;

   private final OneDoFJoint kneePitchJoint;

   private static final int hipPitchJointIndex = 0;
   private static final int kneePitchJointIndex = 1;
   private static final int anklePitchJointIndex = 2;
   private final PrivilegedAccelerationCommand privilegedAccelerationCommand = new PrivilegedAccelerationCommand();

   private final double kneeRangeOfMotion;
   private final double kneeMidRangeOfMotion;

   private double kneePitchPrivilegedConfigurationGain;
   private double kneePitchPrivilegedVelocityGain;
   private double kneePitchPrivilegedConfigurationWeight;

   public LegConfigurationControlModule(RobotSide robotSide, HighLevelHumanoidControllerToolbox controllerToolbox, StraightLegWalkingParameters straightLegWalkingParameters,
                                        YoVariableRegistry parentRegistry)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String namePrefix = sidePrefix + "Knee";
      registry = new YoVariableRegistry(sidePrefix + getClass().getSimpleName());

      kneePitchJoint = controllerToolbox.getFullRobotModel().getLegJoint(robotSide, LegJointName.KNEE_PITCH);
      double kneeLimitUpper = kneePitchJoint.getJointLimitUpper();
      if (Double.isNaN(kneeLimitUpper) || Double.isInfinite(kneeLimitUpper))
         kneeLimitUpper = Math.PI;
      double kneeLimitLower = kneePitchJoint.getJointLimitLower();
      if (Double.isNaN(kneeLimitLower) || Double.isInfinite(kneeLimitLower))
         kneeLimitLower = -Math.PI;
      kneeRangeOfMotion = MathTools.square(kneeLimitUpper - kneeLimitLower);
      kneeMidRangeOfMotion = 0.5 * (kneeLimitUpper + kneeLimitLower);

      OneDoFJoint hipPitchJoint = controllerToolbox.getFullRobotModel().getLegJoint(robotSide, LegJointName.HIP_PITCH);
      OneDoFJoint anklePitchJoint = controllerToolbox.getFullRobotModel().getLegJoint(robotSide, LegJointName.ANKLE_PITCH);
      privilegedAccelerationCommand.addJoint(hipPitchJoint, Double.NaN);
      privilegedAccelerationCommand.addJoint(kneePitchJoint, Double.NaN);
      privilegedAccelerationCommand.addJoint(anklePitchJoint, Double.NaN);

      legPitchPrivilegedWeight = new DoubleYoVariable(sidePrefix + "LegPitchPrivilegedWeight", registry);

      kneeStraightPrivilegedWeight = new DoubleYoVariable(sidePrefix + "KneeStraightPrivilegedWeight", registry);
      kneeStraightPrivilegedPositionGain = new DoubleYoVariable(sidePrefix + "KneeStraightPrivilegedKp", registry);
      kneeStraightPrivilegedVelocityGain = new DoubleYoVariable(sidePrefix + "KneeStraightPrivilegedKv", registry);

      kneeBentPrivilegedWeight = new DoubleYoVariable(sidePrefix + "KneeBentPrivilegedWeight", registry);
      kneeBentPrivilegedPositionGain = new DoubleYoVariable(sidePrefix + "KneeBentPrivilegedKp", registry);
      kneeBentPrivilegedVelocityGain = new DoubleYoVariable(sidePrefix + "KneeBentPrivilegedKv", registry);

      kneePitchPrivilegedConfiguration = new DoubleYoVariable(sidePrefix + "KneePitchPrivilegedConfiguration", registry);
      privilegedMaxAcceleration = new DoubleYoVariable(namePrefix + "PrivilegedMaxAcceleration", registry);

      legPitchPrivilegedWeight.set(straightLegWalkingParameters.getLegPitchPrivilegedWeight());

      kneeStraightPrivilegedWeight.set(straightLegWalkingParameters.getKneeStraightLegPrivilegedWeight());
      kneeStraightPrivilegedPositionGain.set(straightLegWalkingParameters.getKneeStraightLegPrivilegedConfigurationGain());
      kneeStraightPrivilegedVelocityGain.set(straightLegWalkingParameters.getKneeStraightLegPrivilegedVelocityGain());

      kneeBentPrivilegedWeight.set(straightLegWalkingParameters.getKneeBentLegPrivilegedWeight());
      kneeBentPrivilegedPositionGain.set(straightLegWalkingParameters.getKneeBentLegPrivilegedConfigurationGain());
      kneeBentPrivilegedVelocityGain.set(straightLegWalkingParameters.getKneeBentLegPrivilegedVelocityGain());

      privilegedMaxAcceleration.set(straightLegWalkingParameters.getPrivilegedMaxAcceleration());

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

      setupStateMachine();

      if (straightLegWalkingParameters.attemptToStraightenLegs())
         stateMachine.setCurrentState(LegConfigurationType.STRAIGHT);
      else
         stateMachine.setCurrentState(LegConfigurationType.BENT);

      parentRegistry.addChild(registry);
   }

   private void setupStateMachine()
   {
      List<AbstractLegConfigurationState> states = new ArrayList<>();

      AbstractLegConfigurationState straighteningToStraightState = new StraightenToStraightControlState(straighteningSpeed);
      AbstractLegConfigurationState straightState = new StraightKneeControlState();
      AbstractLegConfigurationState bentState = new BentKneeControlState();
      AbstractLegConfigurationState collapseState = new CollapseKneeControlState();
      AbstractLegConfigurationState straighteningToControlState = new StraightenToControllableControlState(straighteningSpeed);
      states.add(straighteningToStraightState);
      states.add(straightState);
      states.add(bentState);
      states.add(collapseState);
      states.add(straighteningToControlState);

      straighteningToStraightState.setDefaultNextState(LegConfigurationType.STRAIGHT);
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

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return privilegedAccelerationCommand;
   }

   private abstract class AbstractLegConfigurationState extends FinishableState<LegConfigurationType>
   {
      public AbstractLegConfigurationState(LegConfigurationType stateEnum)
      {
         super(stateEnum);
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
      public StraightenToStraightControlState(DoubleYoVariable straighteningSpeed)
      {
         super(LegConfigurationType.STRAIGHTEN_TO_STRAIGHT, straighteningSpeed);
      }
   }

   private class StraightenToControllableControlState extends StraighteningKneeControlState
   {
      public StraightenToControllableControlState(DoubleYoVariable straighteningSpeed)
      {
         super(LegConfigurationType.STRAIGHTEN_TO_CONTROLLABLE, straighteningSpeed);
      }
   }

   private class StraighteningKneeControlState extends AbstractLegConfigurationState
   {
      private final DoubleYoVariable yoStraighteningSpeed;

      private double startingPosition;

      private double timeUntilStraight;
      private double straighteningSpeed;

      private double dwellTime;
      private double desiredPrivilegedPosition;

      private double previousTime;

      public StraighteningKneeControlState(LegConfigurationType stateEnum, DoubleYoVariable straighteningSpeed)
      {
         super(stateEnum);

         this.yoStraighteningSpeed = straighteningSpeed;
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

         double privilegedKneeAcceleration = computeKneeAcceleration(gainModifier * kneeStraightPrivilegedPositionGain.getDoubleValue(), kneeStraightPrivilegedVelocityGain.getDoubleValue(),
                                                                     desiredPrivilegedPosition, currentPosition, currentVelocity, privilegedMaxAcceleration.getDoubleValue());
         double privilegedHipPitchAcceleration = -0.5 * privilegedKneeAcceleration;
         double privilegedAnklePitchAcceleration = -0.5 * privilegedKneeAcceleration;

         privilegedAccelerationCommand.setOneDoFJoint(hipPitchJointIndex, privilegedHipPitchAcceleration);
         privilegedAccelerationCommand.setOneDoFJoint(kneePitchJointIndex, privilegedKneeAcceleration);
         privilegedAccelerationCommand.setOneDoFJoint(anklePitchJointIndex, privilegedAnklePitchAcceleration);

         privilegedAccelerationCommand.setWeight(hipPitchJointIndex, legPitchPrivilegedWeight.getDoubleValue());
         privilegedAccelerationCommand.setWeight(kneePitchJointIndex, kneeStraightPrivilegedWeight.getDoubleValue());
         privilegedAccelerationCommand.setWeight(anklePitchJointIndex, legPitchPrivilegedWeight.getDoubleValue());

         kneePitchPrivilegedConfiguration.set(desiredPrivilegedPosition);

         kneePitchPrivilegedConfigurationGain = gainModifier * kneeStraightPrivilegedPositionGain.getDoubleValue();
         kneePitchPrivilegedVelocityGain = kneeStraightPrivilegedVelocityGain.getDoubleValue();
         kneePitchPrivilegedConfigurationWeight = kneeStraightPrivilegedWeight.getDoubleValue();

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
      public StraightKneeControlState()
      {
         super(LegConfigurationType.STRAIGHT);
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

         double privilegedKneeAcceleration = computeKneeAcceleration(gainModifier * kneeStraightPrivilegedPositionGain.getDoubleValue(), kneeStraightPrivilegedVelocityGain.getDoubleValue(),
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

         kneePitchPrivilegedConfiguration.set(desiredAngleWhenStraight.getDoubleValue());

         kneePitchPrivilegedConfigurationGain = gainModifier * kneeStraightPrivilegedPositionGain.getDoubleValue();
         kneePitchPrivilegedVelocityGain = kneeStraightPrivilegedVelocityGain.getDoubleValue();
         kneePitchPrivilegedConfigurationWeight = kneeStraightPrivilegedWeight.getDoubleValue();
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

   private class BentKneeControlState extends AbstractLegConfigurationState
   {
      public BentKneeControlState()
      {
         super(LegConfigurationType.BENT);
      }

      @Override
      public boolean isDone()
      {
         return false;
      }

      @Override
      public void doAction()
      {
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

         kneePitchPrivilegedConfiguration.set(kneeMidRangeOfMotion);

         kneePitchPrivilegedConfigurationGain = kneeBentPrivilegedPositionGain.getDoubleValue();
         kneePitchPrivilegedVelocityGain = kneeBentPrivilegedVelocityGain.getDoubleValue();
         kneePitchPrivilegedConfigurationWeight = kneeBentPrivilegedWeight.getDoubleValue();
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
      public CollapseKneeControlState()
      {
         super(LegConfigurationType.COLLAPSE);
      }

      @Override
      public boolean isDone()
      {
         return getTimeInCurrentState() > collapsingDuration.getDoubleValue();
      }

      @Override
      public void doAction()
      {
         double desiredKneePosition = InterpolationTools.linearInterpolate(desiredAngleWhenStraight.getDoubleValue(), kneeMidRangeOfMotion,
               getTimeInCurrentState() / collapsingDuration.getDoubleValue());

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

         kneePitchPrivilegedConfiguration.set(desiredKneePosition);

         kneePitchPrivilegedConfigurationGain = kneeBentPrivilegedPositionGain.getDoubleValue();
         kneePitchPrivilegedVelocityGain = kneeBentPrivilegedVelocityGain.getDoubleValue();
         kneePitchPrivilegedConfigurationWeight = kneeBentPrivilegedWeight.getDoubleValue();

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
}
