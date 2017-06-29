package us.ihmc.commonWalkingControlModules.controlModules.legConfiguration;

import us.ihmc.commonWalkingControlModules.configurations.StraightLegWalkingParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.InterpolationTools;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.controllers.YoPDGains;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
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

   private static final boolean ONLY_MOVE_PRIV_POS_IF_NOT_BENDING = false;

   private final YoVariableRegistry registry;

   private final PrivilegedAccelerationCommand privilegedAccelerationCommand = new PrivilegedAccelerationCommand();

   private final StraightLegWalkingParameters straightLegWalkingParameters;

   private final YoEnum<LegConfigurationType> requestedState;
   private final GenericStateMachine<LegConfigurationType, FinishableState<LegConfigurationType>> stateMachine;

   private final YoDouble legPitchPrivilegedWeight;

   private final YoDouble kneeStraightPrivilegedWeight;
   private final YoDouble straightJointSpacePositionGain;
   private final YoDouble straightJointSpaceVelocityGain;
   private final YoDouble straightActuatorSpacePositionGain;
   private final YoDouble straightActuatorSpaceVelocityGain;

   private final YoDouble kneeBentPrivilegedWeight;
   private final YoDouble bentJointSpacePositionGain;
   private final YoDouble bentJointSpaceVelocityGain;
   private final YoDouble bentActuatorSpaceVelocityGain;
   private final YoDouble bentActuatorSpacePositionGain;

   private final YoDouble kneePitchPrivilegedConfiguration;
   private final YoDouble kneePitchPrivilegedError;

   private final YoDouble kneePrivilegedPAction;
   private final YoDouble kneePrivilegedDAction;
   private final YoDouble privilegedMaxAcceleration;

   private final YoDouble desiredAngleWhenStraight;

   private final YoDouble straighteningSpeed;
   private final YoDouble collapsingDuration;

   private final YoDouble desiredVirtualActuatorLength;
   private final YoDouble currentVirtualActuatorLength;
   private final YoDouble currentVirtualActuatorVelocity;

   private final OneDoFJoint kneePitchJoint;

   private static final int hipPitchJointIndex = 0;
   private static final int kneePitchJointIndex = 1;
   private static final int anklePitchJointIndex = 2;

   private final double kneeRangeOfMotion;
   private final double kneeMidRangeOfMotion;

   private final double thighLength;
   private final double shinLength;

   private double jointSpaceConfigurationGain;
   private double jointSpaceVelocityGain;
   private double actuatorSpaceConfigurationGain;
   private double actuatorSpaceVelocityGain;

   private double kneePitchPrivilegedConfigurationWeight;

   public LegConfigurationControlModule(RobotSide robotSide, HighLevelHumanoidControllerToolbox controllerToolbox, StraightLegWalkingParameters straightLegWalkingParameters,
                                        YoVariableRegistry parentRegistry)
   {
      this.straightLegWalkingParameters = straightLegWalkingParameters;

      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String namePrefix = sidePrefix + "Leg";
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

      legPitchPrivilegedWeight = new YoDouble(sidePrefix + "LegPitchPrivilegedWeight", registry);

      kneeStraightPrivilegedWeight = new YoDouble(sidePrefix + "KneeStraightPrivilegedWeight", registry);
      straightJointSpacePositionGain = new YoDouble(sidePrefix + "StraightLegJointSpaceKp", registry);
      straightJointSpaceVelocityGain = new YoDouble(sidePrefix + "StraightLegJointSpaceKv", registry);
      straightActuatorSpacePositionGain = new YoDouble(sidePrefix + "StraightLegActuatorSpaceKp", registry);
      straightActuatorSpaceVelocityGain = new YoDouble(sidePrefix + "StraightLegActuatorSpaceKv", registry);

      kneeBentPrivilegedWeight = new YoDouble(sidePrefix + "KneeBentPrivilegedWeight", registry);
      bentJointSpacePositionGain = new YoDouble(sidePrefix + "BentLegJointSpaceKp", registry);
      bentJointSpaceVelocityGain = new YoDouble(sidePrefix + "BentLegJointSpaceKv", registry);
      bentActuatorSpacePositionGain = new YoDouble(sidePrefix + "BentLegActuatorSpaceKp", registry);
      bentActuatorSpaceVelocityGain = new YoDouble(sidePrefix + "BentLegActuatorSpaceKv", registry);

      kneePitchPrivilegedConfiguration = new YoDouble(sidePrefix + "KneePitchPrivilegedConfiguration", registry);
      privilegedMaxAcceleration = new YoDouble(sidePrefix + "LegPrivilegedMaxAcceleration", registry);

      kneePitchPrivilegedError = new YoDouble(sidePrefix + "KneePitchPrivilegedError", registry);
      kneePrivilegedPAction = new YoDouble(sidePrefix + "KneePrivilegedPAction", registry);
      kneePrivilegedDAction = new YoDouble(sidePrefix + "KneePrivilegedDAction", registry);

      legPitchPrivilegedWeight.set(straightLegWalkingParameters.getLegPitchPrivilegedWeight());

      kneeStraightPrivilegedWeight.set(straightLegWalkingParameters.getKneeStraightLegPrivilegedWeight());
      straightJointSpacePositionGain.set(straightLegWalkingParameters.getStraightLegJointSpacePrivilegedConfigurationGain());
      straightJointSpaceVelocityGain.set(straightLegWalkingParameters.getStraightLegJointSpacePrivilegedVelocityGain());
      straightActuatorSpacePositionGain.set(straightLegWalkingParameters.getStraightLegActuatorSpacePrivilegedConfigurationGain());
      straightActuatorSpaceVelocityGain.set(straightLegWalkingParameters.getStraightLegActuatorSpacePrivilegedVelocityGain());

      kneeBentPrivilegedWeight.set(straightLegWalkingParameters.getKneeBentLegPrivilegedWeight());
      bentJointSpacePositionGain.set(straightLegWalkingParameters.getBentLegJointSpacePrivilegedConfigurationGain());
      bentJointSpaceVelocityGain.set(straightLegWalkingParameters.getBentLegJointSpacePrivilegedVelocityGain());
      bentActuatorSpacePositionGain.set(straightLegWalkingParameters.getBentLegActuatorSpacePrivilegedConfigurationGain());
      bentActuatorSpaceVelocityGain.set(straightLegWalkingParameters.getBentLegActuatorSpacePrivilegedVelocityGain());

      privilegedMaxAcceleration.set(straightLegWalkingParameters.getPrivilegedMaxAcceleration());

      desiredAngleWhenStraight = new YoDouble(namePrefix + "DesiredAngleWhenStraight", registry);
      desiredAngleWhenStraight.set(straightLegWalkingParameters.getStraightKneeAngle());

      straighteningSpeed = new YoDouble(namePrefix + "SupportKneeStraighteningSpeed", registry);
      straighteningSpeed.set(straightLegWalkingParameters.getSpeedForSupportKneeStraightening());

      collapsingDuration = new YoDouble(namePrefix + "SupportKneeCollapsingDuration", registry);
      collapsingDuration.set(straightLegWalkingParameters.getSupportKneeCollapsingDuration());

      desiredVirtualActuatorLength = new YoDouble(namePrefix + "DesiredVirtualActuatorLength", registry);
      currentVirtualActuatorLength = new YoDouble(namePrefix + "CurrentVirtualActuatorLength", registry);
      currentVirtualActuatorVelocity = new YoDouble(namePrefix + "CurrentVirtualActuatorVelocity", registry);

      // set up states and state machine
      YoDouble time = controllerToolbox.getYoTime();
      stateMachine = new GenericStateMachine<>(namePrefix + "State", namePrefix + "SwitchTime", LegConfigurationType.class, time, registry);
      requestedState = YoEnum.create(namePrefix + "RequestedState", "", LegConfigurationType.class, registry, true);
      requestedState.set(null);

      // compute leg segment lengths
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      ReferenceFrame hipPitchFrame = fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.HIP_PITCH).getFrameAfterJoint();
      FramePoint hipPoint = new FramePoint(hipPitchFrame);
      FramePoint kneePoint = new FramePoint(fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH).getFrameBeforeJoint());
      kneePoint.changeFrame(hipPitchFrame);

      thighLength = hipPoint.distance(kneePoint);

      ReferenceFrame kneePitchFrame = fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH).getFrameAfterJoint();
      kneePoint.setToZero(kneePitchFrame);
      FramePoint anklePoint = new FramePoint(fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.ANKLE_PITCH).getFrameBeforeJoint());
      anklePoint.changeFrame(kneePitchFrame);

      shinLength = kneePoint.distance(anklePoint);

      setupStateMachine();

      if (straightLegWalkingParameters.attemptToStraightenLegs())
         stateMachine.setCurrentState(LegConfigurationType.STRAIGHT);
      else
         stateMachine.setCurrentState(LegConfigurationType.BENT);

      parentRegistry.addChild(registry);
   }

   private void setupStateMachine()
   {
      List<FinishableState<LegConfigurationType>> states = new ArrayList<>();

      FinishableState<LegConfigurationType> straighteningToStraightState = new StraightenToStraightControlState(straighteningSpeed);
      FinishableState<LegConfigurationType> straightState = new StraightKneeControlState();
      FinishableState<LegConfigurationType> bentState = new BentKneeControlState();
      FinishableState<LegConfigurationType> collapseState = new CollapseKneeControlState();
      FinishableState<LegConfigurationType> straighteningToControlState = new StraightenToControllableControlState(straighteningSpeed);
      states.add(straighteningToStraightState);
      states.add(straightState);
      states.add(bentState);
      states.add(collapseState);
      states.add(straighteningToControlState);

      straighteningToStraightState.setDefaultNextState(LegConfigurationType.STRAIGHT);
      collapseState.setDefaultNextState(LegConfigurationType.BENT);

      for (FinishableState<LegConfigurationType> fromState : states)
      {
         for (FinishableState<LegConfigurationType> toState : states)
         {
            StateMachineTools.addRequestedStateTransition(requestedState, false, fromState, toState);
         }
      }

      for (FinishableState<LegConfigurationType> state : states)
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

      double privilegedKneeAcceleration = computeKneeAcceleration();
      double privilegedHipPitchAcceleration = -0.5 * privilegedKneeAcceleration;
      double privilegedAnklePitchAcceleration = -0.5 * privilegedKneeAcceleration;

      privilegedAccelerationCommand.setOneDoFJoint(hipPitchJointIndex, privilegedHipPitchAcceleration);
      privilegedAccelerationCommand.setOneDoFJoint(kneePitchJointIndex, privilegedKneeAcceleration);
      privilegedAccelerationCommand.setOneDoFJoint(anklePitchJointIndex, privilegedAnklePitchAcceleration);

      privilegedAccelerationCommand.setWeight(hipPitchJointIndex, legPitchPrivilegedWeight.getDoubleValue());
      privilegedAccelerationCommand.setWeight(kneePitchJointIndex, kneePitchPrivilegedConfigurationWeight);
      privilegedAccelerationCommand.setWeight(anklePitchJointIndex, legPitchPrivilegedWeight.getDoubleValue());
   }

   private double computeKneeAcceleration()
   {
      double currentPosition = kneePitchJoint.getQ();

      double desiredVirtualLength = computeVirtualActuatorLength(kneePitchPrivilegedConfiguration.getDoubleValue());
      double currentVirtualLength = computeVirtualActuatorLength(currentPosition);

      desiredVirtualActuatorLength.set(desiredVirtualLength);
      currentVirtualActuatorLength.set(currentVirtualLength);

      double error = kneePitchPrivilegedConfiguration.getDoubleValue() - currentPosition;
      double virtualError = desiredVirtualLength - currentVirtualLength;
      kneePitchPrivilegedError.set(error);

      double currentVirtualVelocity = computeVirtualActuatorVelocity(currentPosition, kneePitchJoint.getQd());
      currentVirtualActuatorVelocity.set(currentVirtualVelocity);

      double percentDistanceToMidRange = Math.abs(currentPosition - kneeMidRangeOfMotion) / (2.0 * kneeRangeOfMotion);

      double jointSpaceKp = 2.0 * jointSpaceConfigurationGain / kneeRangeOfMotion;

      double jointSpacePAction = jointSpaceKp * error;
      double actuatorSpacePAction = -actuatorSpaceConfigurationGain * virtualError;

      double jointSpaceDAction = jointSpaceVelocityGain * -kneePitchJoint.getQd();
      double actuatorSpaceDAction = actuatorSpaceVelocityGain * currentVirtualVelocity;

      double pAction, dAction;

      if (straightLegWalkingParameters.blendPrivilegedConfigurationPositionError())
         pAction = InterpolationTools.linearInterpolate(jointSpacePAction, actuatorSpacePAction, percentDistanceToMidRange);
      else
         pAction = jointSpacePAction;

      if (straightLegWalkingParameters.blendPrivilegedConfigurationVelocityError())
         dAction = InterpolationTools.linearInterpolate(jointSpaceDAction, actuatorSpaceDAction, percentDistanceToMidRange);
      else
         dAction = jointSpaceDAction;


      kneePrivilegedPAction.set(pAction);
      kneePrivilegedDAction.set(dAction);

      return MathTools.clamp(kneePrivilegedPAction.getDoubleValue() + kneePrivilegedDAction.getDoubleValue(), privilegedMaxAcceleration.getDoubleValue());
   }

   private double computeVirtualActuatorLength(double kneePitchAngle)
   {
      double length = Math.pow(thighLength, 2.0) + Math.pow(shinLength, 2.0) + 2.0 * thighLength * shinLength * Math.cos(kneePitchAngle);
      return Math.sqrt(length);
   }

   private double computeVirtualActuatorVelocity(double kneePitchAngle, double kneePitchVelocity)
   {
      double virtualLength = computeVirtualActuatorLength(kneePitchAngle);
      double velocity = -thighLength * shinLength / virtualLength * kneePitchVelocity * Math.sin(kneePitchAngle);
      return velocity;
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

   private class StraightenToStraightControlState extends StraighteningKneeControlState
   {
      public StraightenToStraightControlState(YoDouble straighteningSpeed)
      {
         super(LegConfigurationType.STRAIGHTEN_TO_STRAIGHT, straighteningSpeed);
      }
   }

   private class StraightenToControllableControlState extends StraighteningKneeControlState
   {
      public StraightenToControllableControlState(YoDouble straighteningSpeed)
      {
         super(LegConfigurationType.STRAIGHTEN_TO_CONTROLLABLE, straighteningSpeed);
      }
   }

   private class StraighteningKneeControlState extends FinishableState<LegConfigurationType>
   {
      private final YoDouble yoStraighteningSpeed;

      private double startingPosition;
      private double previousKneePitchAngle;

      private double timeUntilStraight;
      private double straighteningSpeed;

      private double dwellTime;
      private double desiredPrivilegedPosition;

      private double previousTime;

      public StraighteningKneeControlState(LegConfigurationType stateEnum, YoDouble straighteningSpeed)
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

         if (ONLY_MOVE_PRIV_POS_IF_NOT_BENDING)
         {
            if (currentPosition > previousKneePitchAngle && currentPosition > startingPosition) // the knee is bending
               dwellTime += estimatedDT;
            else
               desiredPrivilegedPosition -= estimatedDT * straighteningSpeed;
         }
         else
         {
            desiredPrivilegedPosition -= estimatedDT * straighteningSpeed;
         }

         kneePitchPrivilegedConfiguration.set(desiredPrivilegedPosition);

         jointSpaceConfigurationGain = straightJointSpacePositionGain.getDoubleValue();
         jointSpaceVelocityGain = straightJointSpaceVelocityGain.getDoubleValue();
         actuatorSpaceConfigurationGain = straightActuatorSpacePositionGain.getDoubleValue();
         actuatorSpaceVelocityGain = straightActuatorSpaceVelocityGain.getDoubleValue();

         kneePitchPrivilegedConfigurationWeight = kneeStraightPrivilegedWeight.getDoubleValue();

         previousKneePitchAngle = currentPosition;

         if (isDone())
            transitionToDefaultNextState();
      }



      @Override
      public void doTransitionIntoAction()
      {
         startingPosition = kneePitchJoint.getQ();
         previousKneePitchAngle = kneePitchJoint.getQ();

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

   private class StraightKneeControlState extends FinishableState<LegConfigurationType>
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
         kneePitchPrivilegedConfiguration.set(desiredAngleWhenStraight.getDoubleValue());

         jointSpaceConfigurationGain = straightJointSpacePositionGain.getDoubleValue();
         jointSpaceVelocityGain = straightJointSpaceVelocityGain.getDoubleValue();
         actuatorSpaceConfigurationGain = straightActuatorSpacePositionGain.getDoubleValue();
         actuatorSpaceVelocityGain = straightActuatorSpaceVelocityGain.getDoubleValue();

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

   private class BentKneeControlState extends FinishableState<LegConfigurationType>
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
         kneePitchPrivilegedConfiguration.set(kneeMidRangeOfMotion);

         jointSpaceConfigurationGain = bentJointSpacePositionGain.getDoubleValue();
         jointSpaceVelocityGain = bentJointSpaceVelocityGain.getDoubleValue();
         actuatorSpaceConfigurationGain = bentActuatorSpacePositionGain.getDoubleValue();
         actuatorSpaceVelocityGain = bentActuatorSpaceVelocityGain.getDoubleValue();

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

   private class CollapseKneeControlState extends FinishableState<LegConfigurationType>
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

         kneePitchPrivilegedConfiguration.set(desiredKneePosition);

         jointSpaceConfigurationGain = bentJointSpacePositionGain.getDoubleValue();
         jointSpaceVelocityGain = bentJointSpaceVelocityGain.getDoubleValue();
         actuatorSpaceConfigurationGain = bentActuatorSpacePositionGain.getDoubleValue();
         actuatorSpaceVelocityGain = bentActuatorSpaceVelocityGain.getDoubleValue();

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
