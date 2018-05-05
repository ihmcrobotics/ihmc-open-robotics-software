package us.ihmc.commonWalkingControlModules.controlModules.legConfiguration;

import us.ihmc.commonWalkingControlModules.configurations.LegConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedJointSpaceCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.InterpolationTools;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class LegConfigurationControlModule
{
   public enum LegConfigurationType
   {
      STRAIGHTEN, STRAIGHT, COLLAPSE, BENT
   }

   public enum LegControlWeight
   {
      HIGH, MEDIUM, LOW
   }

   private static final double minimumDampingScale = 0.2;
   private static final boolean scaleDamping = false;

   private static final boolean ONLY_MOVE_PRIV_POS_IF_NOT_BENDING = true;

   private final YoVariableRegistry registry;

   private final PrivilegedJointSpaceCommand privilegedAccelerationCommand = new PrivilegedJointSpaceCommand();

   private final YoEnum<LegConfigurationType> requestedState;
   private final YoEnum<LegControlWeight> legControlWeight;

   private final StateMachine<LegConfigurationType, State> stateMachine;

   private final YoDouble highPrivilegedWeight;
   private final YoDouble mediumPrivilegedWeight;
   private final YoDouble lowPrivilegedWeight;

   private final YoDouble straightJointSpacePositionGain;
   private final YoDouble straightJointSpaceVelocityGain;
   private final YoDouble straightActuatorSpacePositionGain;
   private final YoDouble straightActuatorSpaceVelocityGain;

   private final YoDouble bentJointSpacePositionGain;
   private final YoDouble bentJointSpaceVelocityGain;
   private final YoDouble bentActuatorSpaceVelocityGain;
   private final YoDouble bentActuatorSpacePositionGain;

   private final YoDouble kneePitchPrivilegedConfiguration;
   private final YoDouble kneePitchPrivilegedError;

   private final YoDouble jointSpacePAction;
   private final YoDouble jointSpaceDAction;
   private final YoDouble jointSpaceAction;

   private final YoDouble actuatorSpacePAction;
   private final YoDouble actuatorSpaceDAction;
   private final YoDouble actuatorSpaceAction;

   private final YoDouble privilegedMaxAcceleration;

   private final YoBoolean useFullyExtendedLeg;
   private final YoBoolean useBracingAngle;
   private final YoDouble desiredAngle;
   private final YoDouble desiredAngleWhenStraight;
   private final YoDouble desiredAngleWhenExtended;
   private final YoDouble desiredAngleWhenBracing;

   private final YoDouble desiredFractionOfMidRangeForCollapsed;

   private final YoDouble straighteningAcceleration;
   private final YoDouble collapsingDuration;
   private final YoDouble collapsingDurationFractionOfStep;

   private final YoDouble desiredVirtualActuatorLength;
   private final YoDouble currentVirtualActuatorLength;
   private final YoDouble currentVirtualActuatorVelocity;

   private final OneDoFJoint kneePitchJoint;

   private final YoDouble dampingActionScaleFactor;

   private static final int hipPitchJointIndex = 0;
   private static final int kneePitchJointIndex = 1;
   private static final int anklePitchJointIndex = 2;

   private double jointSpaceConfigurationGain;
   private double jointSpaceVelocityGain;
   private double actuatorSpaceConfigurationGain;
   private double actuatorSpaceVelocityGain;

   private final double kneeRangeOfMotion;
   private final double kneeSquareRangeOfMotion;
   private final double kneeMidRangeOfMotion;

   private final double thighLength;
   private final double shinLength;

   public LegConfigurationControlModule(RobotSide robotSide, HighLevelHumanoidControllerToolbox controllerToolbox,
                                        LegConfigurationParameters legConfigurationParameters, YoVariableRegistry parentRegistry)
   {
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
      kneeSquareRangeOfMotion = MathTools.square(kneeLimitUpper - kneeLimitLower);
      kneeRangeOfMotion = kneeLimitUpper - kneeLimitLower;
      kneeMidRangeOfMotion = 0.5 * (kneeLimitUpper + kneeLimitLower);

      OneDoFJoint hipPitchJoint = controllerToolbox.getFullRobotModel().getLegJoint(robotSide, LegJointName.HIP_PITCH);
      OneDoFJoint anklePitchJoint = controllerToolbox.getFullRobotModel().getLegJoint(robotSide, LegJointName.ANKLE_PITCH);
      privilegedAccelerationCommand.addJoint(hipPitchJoint, Double.NaN);
      privilegedAccelerationCommand.addJoint(kneePitchJoint, Double.NaN);
      privilegedAccelerationCommand.addJoint(anklePitchJoint, Double.NaN);

      highPrivilegedWeight = new YoDouble(sidePrefix + "HighPrivilegedWeight", registry);
      mediumPrivilegedWeight = new YoDouble(sidePrefix + "MediumPrivilegedWeight", registry);
      lowPrivilegedWeight = new YoDouble(sidePrefix + "LowPrivilegedWeight", registry);

      straightJointSpacePositionGain = new YoDouble(sidePrefix + "StraightLegJointSpaceKp", registry);
      straightJointSpaceVelocityGain = new YoDouble(sidePrefix + "StraightLegJointSpaceKv", registry);
      straightActuatorSpacePositionGain = new YoDouble(sidePrefix + "StraightLegActuatorSpaceKp", registry);
      straightActuatorSpaceVelocityGain = new YoDouble(sidePrefix + "StraightLegActuatorSpaceKv", registry);

      bentJointSpacePositionGain = new YoDouble(sidePrefix + "BentLegJointSpaceKp", registry);
      bentJointSpaceVelocityGain = new YoDouble(sidePrefix + "BentLegJointSpaceKv", registry);
      bentActuatorSpacePositionGain = new YoDouble(sidePrefix + "BentLegActuatorSpaceKp", registry);
      bentActuatorSpaceVelocityGain = new YoDouble(sidePrefix + "BentLegActuatorSpaceKv", registry);

      kneePitchPrivilegedConfiguration = new YoDouble(sidePrefix + "KneePitchPrivilegedConfiguration", registry);
      privilegedMaxAcceleration = new YoDouble(sidePrefix + "LegPrivilegedMaxAcceleration", registry);

      kneePitchPrivilegedError = new YoDouble(sidePrefix + "KneePitchPrivilegedError", registry);
      jointSpacePAction = new YoDouble(sidePrefix + "KneePrivilegedJointSpacePAction", registry);
      jointSpaceDAction = new YoDouble(sidePrefix + "KneePrivilegedJointSpaceDAction", registry);
      jointSpaceAction = new YoDouble(sidePrefix + "KneePrivilegedJointSpaceAction", registry);
      actuatorSpacePAction = new YoDouble(sidePrefix + "KneePrivilegedActuatorSpacePAction", registry);
      actuatorSpaceDAction = new YoDouble(sidePrefix + "KneePrivilegedActuatorSpaceDAction", registry);
      actuatorSpaceAction = new YoDouble(sidePrefix + "KneePrivilegedActuatorSpaceAction", registry);

      highPrivilegedWeight.set(legConfigurationParameters.getLegPrivilegedHighWeight());
      mediumPrivilegedWeight.set(legConfigurationParameters.getLegPrivilegedMediumWeight());
      lowPrivilegedWeight.set(legConfigurationParameters.getLegPrivilegedLowWeight());

      LegConfigurationGains straightLegGains = legConfigurationParameters.getStraightLegGains();
      LegConfigurationGains bentLegGains = legConfigurationParameters.getBentLegGains();

      straightJointSpacePositionGain.set(straightLegGains.getJointSpaceKp());
      straightJointSpaceVelocityGain.set(straightLegGains.getJointSpaceKd());
      straightActuatorSpacePositionGain.set(straightLegGains.getActuatorSpaceKp());
      straightActuatorSpaceVelocityGain.set(straightLegGains.getActuatorSpaceKd());

      bentJointSpacePositionGain.set(bentLegGains.getJointSpaceKp());
      bentJointSpaceVelocityGain.set(bentLegGains.getJointSpaceKd());
      bentActuatorSpacePositionGain.set(bentLegGains.getActuatorSpaceKp());
      bentActuatorSpaceVelocityGain.set(bentLegGains.getActuatorSpaceKd());

      privilegedMaxAcceleration.set(legConfigurationParameters.getPrivilegedMaxAcceleration());

      dampingActionScaleFactor = new YoDouble(namePrefix + "DampingActionScaleFactor", registry);

      useFullyExtendedLeg = new YoBoolean(namePrefix + "UseFullyExtendedLeg", registry);
      useBracingAngle = new YoBoolean(namePrefix + "UseBracingLeg", registry);

      desiredAngle = new YoDouble(namePrefix + "DesiredAngle", registry);

      desiredAngleWhenStraight = new YoDouble(namePrefix + "DesiredAngleWhenStraight", registry);
      desiredAngleWhenExtended = new YoDouble(namePrefix + "DesiredAngleWhenExtended", registry);
      desiredAngleWhenBracing = new YoDouble(namePrefix + "DesiredAngleWhenBracing", registry);
      desiredAngleWhenStraight.set(legConfigurationParameters.getKneeAngleWhenStraight());
      desiredAngleWhenExtended.set(legConfigurationParameters.getKneeAngleWhenExtended());
      desiredAngleWhenBracing.set(legConfigurationParameters.getKneeAngleWhenBracing());

      desiredFractionOfMidRangeForCollapsed = new YoDouble(namePrefix + "DesiredFractionOfMidRangeForCollapsed", registry);
      desiredFractionOfMidRangeForCollapsed.set(legConfigurationParameters.getDesiredFractionOfMidrangeForCollapsedAngle());

      straighteningAcceleration = new YoDouble(namePrefix + "SupportKneeStraighteningAcceleration", registry);
      straighteningAcceleration.set(legConfigurationParameters.getAccelerationForSupportKneeStraightening());

      collapsingDuration = new YoDouble(namePrefix + "SupportKneeCollapsingDuration", registry);
      collapsingDurationFractionOfStep = new YoDouble(namePrefix + "SupportKneeCollapsingDurationFractionOfStep", registry);
      collapsingDurationFractionOfStep.set(legConfigurationParameters.getSupportKneeCollapsingDurationFractionOfStep());

      desiredVirtualActuatorLength = new YoDouble(namePrefix + "DesiredVirtualActuatorLength", registry);
      currentVirtualActuatorLength = new YoDouble(namePrefix + "CurrentVirtualActuatorLength", registry);
      currentVirtualActuatorVelocity = new YoDouble(namePrefix + "CurrentVirtualActuatorVelocity", registry);

      // set up states and state machine
      YoDouble time = controllerToolbox.getYoTime();
      requestedState = YoEnum.create(namePrefix + "RequestedState", "", LegConfigurationType.class, registry, true);
      requestedState.set(null);
      legControlWeight = YoEnum.create(namePrefix + "LegControlWeight", "", LegControlWeight.class, registry, false);

      // compute leg segment lengths
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      ReferenceFrame hipPitchFrame = fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.HIP_PITCH).getFrameAfterJoint();
      FramePoint3D hipPoint = new FramePoint3D(hipPitchFrame);
      FramePoint3D kneePoint = new FramePoint3D(fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH).getFrameBeforeJoint());
      kneePoint.changeFrame(hipPitchFrame);

      thighLength = hipPoint.distance(kneePoint);

      ReferenceFrame kneePitchFrame = fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH).getFrameAfterJoint();
      kneePoint.setToZero(kneePitchFrame);
      FramePoint3D anklePoint = new FramePoint3D(fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.ANKLE_PITCH).getFrameBeforeJoint());
      anklePoint.changeFrame(kneePitchFrame);

      shinLength = kneePoint.distance(anklePoint);

      stateMachine = setupStateMachine(namePrefix, legConfigurationParameters.attemptToStraightenLegs(), time);

      parentRegistry.addChild(registry);
   }

   private StateMachine<LegConfigurationType, State> setupStateMachine(String namePrefix, boolean attemptToStraightenLegs, DoubleProvider timeProvider)
   {
      StateMachineFactory<LegConfigurationType, State> factory = new StateMachineFactory<>(LegConfigurationType.class);
      factory.setNamePrefix(namePrefix).setRegistry(registry).buildYoClock(timeProvider);

      factory.addStateAndDoneTransition(LegConfigurationType.STRAIGHTEN, new StraighteningKneeControlState(straighteningAcceleration),
                                        LegConfigurationType.STRAIGHT);
      factory.addState(LegConfigurationType.STRAIGHT, new StraightKneeControlState());
      factory.addState(LegConfigurationType.BENT, new BentKneeControlState());
      factory.addState(LegConfigurationType.COLLAPSE, new CollapseKneeControlState());

      for (LegConfigurationType from : LegConfigurationType.values())
      {
         factory.addRequestedTransition(from, requestedState);
         factory.addRequestedTransition(from, from, requestedState);
      }

      return factory.build(attemptToStraightenLegs ? LegConfigurationType.STRAIGHT : LegConfigurationType.BENT);
   }

   public void initialize()
   {
   }

   public void doControl()
   {
      if (useBracingAngle.getBooleanValue())
         desiredAngle.set(desiredAngleWhenBracing.getDoubleValue());
      else if (useFullyExtendedLeg.getBooleanValue())
         desiredAngle.set(desiredAngleWhenExtended.getDoubleValue());
      else
         desiredAngle.set(desiredAngleWhenStraight.getDoubleValue());

      stateMachine.doActionAndTransition();

      double kneePitchPrivilegedConfigurationWeight;
      if (legControlWeight.getEnumValue() == LegControlWeight.LOW)
         kneePitchPrivilegedConfigurationWeight = lowPrivilegedWeight.getDoubleValue();
      else if (legControlWeight.getEnumValue() == LegControlWeight.MEDIUM)
         kneePitchPrivilegedConfigurationWeight = mediumPrivilegedWeight.getDoubleValue();
      else
         kneePitchPrivilegedConfigurationWeight = highPrivilegedWeight.getDoubleValue();

      double privilegedKneeAcceleration = computeKneeAcceleration();
      double privilegedHipPitchAcceleration = -0.5 * privilegedKneeAcceleration;
      double privilegedAnklePitchAcceleration = -0.5 * privilegedKneeAcceleration;

      privilegedAccelerationCommand.setOneDoFJoint(hipPitchJointIndex, privilegedHipPitchAcceleration);
      privilegedAccelerationCommand.setOneDoFJoint(kneePitchJointIndex, privilegedKneeAcceleration);
      privilegedAccelerationCommand.setOneDoFJoint(anklePitchJointIndex, privilegedAnklePitchAcceleration);

      privilegedAccelerationCommand.setWeight(hipPitchJointIndex, kneePitchPrivilegedConfigurationWeight);
      privilegedAccelerationCommand.setWeight(kneePitchJointIndex, kneePitchPrivilegedConfigurationWeight);
      privilegedAccelerationCommand.setWeight(anklePitchJointIndex, kneePitchPrivilegedConfigurationWeight);
   }

   public void setStepDuration(double stepDuration)
   {
      collapsingDuration.set(collapsingDurationFractionOfStep.getDoubleValue() * stepDuration);
   }

   public void setFullyExtendLeg(boolean fullyExtendLeg)
   {
      useFullyExtendedLeg.set(fullyExtendLeg);
   }

   public void prepareForLegBracing()
   {
      useBracingAngle.set(true);
   }

   public void doNotBrace()
   {
      useBracingAngle.set(false);
   }

   public void setLegControlWeight(LegControlWeight legControlWeight)
   {
      this.legControlWeight.set(legControlWeight);
   }

   private double computeKneeAcceleration()
   {
      double currentPosition = kneePitchJoint.getQ();

      double jointError = kneePitchPrivilegedConfiguration.getDoubleValue() - currentPosition;
      kneePitchPrivilegedError.set(jointError);

      // modify gains based on error. If there's a big error, don't damp velocities
      double percentError = Math.abs(jointError) / (0.5 * kneeRangeOfMotion);
      double dampingActionScaleFactor;
      if (scaleDamping)
         dampingActionScaleFactor = MathTools.clamp(1.0 - (1.0 - minimumDampingScale) * percentError, 0.0, 1.0);
      else
         dampingActionScaleFactor = 1.0;
      this.dampingActionScaleFactor.set(dampingActionScaleFactor);

      double jointSpaceAction = computeJointSpaceAction(dampingActionScaleFactor);
      double actuatorSpaceAction = computeActuatorSpaceAction(dampingActionScaleFactor);

      double desiredAcceleration = jointSpaceAction + actuatorSpaceAction;

      return MathTools.clamp(desiredAcceleration, privilegedMaxAcceleration.getDoubleValue());
   }

   private double computeJointSpaceAction(double dampingActionScaleFactor)
   {
      double jointError = kneePitchPrivilegedConfiguration.getDoubleValue() - kneePitchJoint.getQ();
      double jointSpaceKp = 2.0 * jointSpaceConfigurationGain / kneeSquareRangeOfMotion;

      double jointSpacePAction = Double.isNaN(jointSpaceKp) ? 0.0 : jointSpaceKp * jointError;
      double jointSpaceDAction = Double.isNaN(jointSpaceVelocityGain) ? 0.0 : dampingActionScaleFactor * jointSpaceVelocityGain * -kneePitchJoint.getQd();

      this.jointSpacePAction.set(jointSpacePAction);
      this.jointSpaceDAction.set(jointSpaceDAction);

      jointSpaceAction.set(jointSpacePAction + jointSpaceDAction);

      return jointSpacePAction + jointSpaceDAction;
   }

   private double computeActuatorSpaceAction(double dampingActionScaleFactor)
   {
      double currentPosition = kneePitchJoint.getQ();

      double desiredVirtualLength = computeVirtualActuatorLength(kneePitchPrivilegedConfiguration.getDoubleValue());
      double currentVirtualLength = computeVirtualActuatorLength(currentPosition);

      desiredVirtualActuatorLength.set(desiredVirtualLength);
      currentVirtualActuatorLength.set(currentVirtualLength);

      double currentVirtualVelocity = computeVirtualActuatorVelocity(currentPosition, kneePitchJoint.getQd());
      currentVirtualActuatorVelocity.set(currentVirtualVelocity);

      double virtualError = desiredVirtualLength - currentVirtualLength;

      double actuatorSpacePAction = Double.isNaN(actuatorSpaceConfigurationGain) ? 0.0 : actuatorSpaceConfigurationGain * virtualError;
      double actuatorSpaceDAction = Double.isNaN(actuatorSpaceVelocityGain) ? 0.0 : dampingActionScaleFactor * actuatorSpaceVelocityGain * currentVirtualVelocity;

      this.actuatorSpacePAction.set(actuatorSpacePAction);
      this.actuatorSpaceDAction.set(actuatorSpaceDAction);

      double actuatorSpaceAcceleration = actuatorSpacePAction + actuatorSpaceDAction;

      double acceleration = computeJointAccelerationFromActuatorAcceleration(currentPosition, kneePitchJoint.getQd(), actuatorSpaceAcceleration);

      this.actuatorSpaceAction.set(acceleration);

      return acceleration;
   }

   private double computeVirtualActuatorLength(double kneePitchAngle)
   {
      double length = Math.pow(thighLength, 2.0) + Math.pow(shinLength, 2.0) + 2.0 * thighLength * shinLength * Math.cos(kneePitchAngle);
      return Math.sqrt(length);
   }

   private double computeVirtualActuatorVelocity(double kneePitchAngle, double kneePitchVelocity)
   {
      double virtualLength = computeVirtualActuatorLength(kneePitchAngle);
      return -thighLength * shinLength / virtualLength * kneePitchVelocity * Math.sin(kneePitchAngle);
   }

   private double computeJointAccelerationFromActuatorAcceleration(double kneePitchAngle, double kneePitchVelocity, double actuatorAcceleration)
   {
      double actuatorLength = computeVirtualActuatorLength(kneePitchAngle);
      double actuatorVelocity = computeVirtualActuatorVelocity(kneePitchAngle, kneePitchVelocity);

      double coriolisAcceleration = thighLength * shinLength / actuatorLength * Math.pow(kneePitchVelocity, 2.0) * Math.cos(kneePitchAngle);
      double centripetalAcceleration = -Math.pow(actuatorVelocity, 2.0) / actuatorLength;
      double regularAcceleration = -thighLength * shinLength / actuatorLength * actuatorAcceleration * Math.sin(kneePitchAngle);

      return regularAcceleration + coriolisAcceleration + centripetalAcceleration;
   }

   public void setKneeAngleState(LegConfigurationType controlType)
   {
      requestedState.set(controlType);
   }

   public LegConfigurationType getCurrentKneeControlState()
   {
      return stateMachine.getCurrentStateKey();
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return privilegedAccelerationCommand;
   }

   private class StraighteningKneeControlState implements State
   {
      private final YoDouble yoStraighteningAcceleration;

      private double startingPosition;
      private double previousKneePitchAngle;

      private double timeUntilStraight;
      private double straighteningVelocity;
      private double straighteningAcceleration;

      private double dwellTime;
      private double desiredPrivilegedPosition;

      private double previousTime;

      public StraighteningKneeControlState(YoDouble straighteningAcceleration)
      {
         this.yoStraighteningAcceleration = straighteningAcceleration;
      }

      @Override
      public boolean isDone(double timeInState)
      {
         return timeInState > (timeUntilStraight + dwellTime);
      }

      @Override
      public void doAction(double timeInState)
      {
         double estimatedDT = estimateDT(timeInState);
         double currentPosition = kneePitchJoint.getQ();

         if (ONLY_MOVE_PRIV_POS_IF_NOT_BENDING)
         {
            if (currentPosition > previousKneePitchAngle && currentPosition > startingPosition) // the knee is bending
            {
               dwellTime += estimatedDT;
            }
            else
            {
               straighteningVelocity += estimatedDT * straighteningAcceleration;
               desiredPrivilegedPosition -= estimatedDT * straighteningVelocity;
            }
         }
         else
         {
            straighteningVelocity += estimatedDT * straighteningAcceleration;
            desiredPrivilegedPosition -= estimatedDT * straighteningVelocity;
         }

         desiredPrivilegedPosition = Math.max(desiredAngle.getDoubleValue(), desiredPrivilegedPosition);
         kneePitchPrivilegedConfiguration.set(desiredPrivilegedPosition);

         jointSpaceConfigurationGain = straightJointSpacePositionGain.getDoubleValue();
         jointSpaceVelocityGain = straightJointSpaceVelocityGain.getDoubleValue();
         actuatorSpaceConfigurationGain = straightActuatorSpacePositionGain.getDoubleValue();
         actuatorSpaceVelocityGain = straightActuatorSpaceVelocityGain.getDoubleValue();

         previousKneePitchAngle = currentPosition;
      }

      @Override
      public void onEntry()
      {
         startingPosition = kneePitchJoint.getQ();
         previousKneePitchAngle = kneePitchJoint.getQ();

         straighteningVelocity = 0.0;
         straighteningAcceleration = yoStraighteningAcceleration.getDoubleValue();

         timeUntilStraight = Math.sqrt(2.0 * (startingPosition - desiredAngle.getDoubleValue()) / straighteningAcceleration);
         timeUntilStraight = Math.max(timeUntilStraight, 0.0);

         desiredPrivilegedPosition = startingPosition;

         previousTime = 0.0;
         dwellTime = 0.0;

         if (useBracingAngle.getBooleanValue())
            legControlWeight.set(LegControlWeight.MEDIUM);
      }

      @Override
      public void onExit()
      {
      }

      private double estimateDT(double timeInState)
      {
         double estimatedDT = timeInState - previousTime;
         previousTime = timeInState;

         return estimatedDT;
      }
   }

   private class StraightKneeControlState implements State
   {
      @Override
      public boolean isDone(double timeInState)
      {
         return false;
      }

      @Override
      public void doAction(double timeInState)
      {
         kneePitchPrivilegedConfiguration.set(desiredAngle.getDoubleValue());

         jointSpaceConfigurationGain = straightJointSpacePositionGain.getDoubleValue();
         jointSpaceVelocityGain = straightJointSpaceVelocityGain.getDoubleValue();
         actuatorSpaceConfigurationGain = straightActuatorSpacePositionGain.getDoubleValue();
         actuatorSpaceVelocityGain = straightActuatorSpaceVelocityGain.getDoubleValue();
      }

      @Override
      public void onEntry()
      {
      }

      @Override
      public void onExit()
      {
      }
   }

   private class BentKneeControlState implements State
   {
      @Override
      public boolean isDone(double timeInState)
      {
         return false;
      }

      @Override
      public void doAction(double timeInState)
      {
         kneePitchPrivilegedConfiguration.set(kneeMidRangeOfMotion);

         jointSpaceConfigurationGain = bentJointSpacePositionGain.getDoubleValue();
         jointSpaceVelocityGain = bentJointSpaceVelocityGain.getDoubleValue();
         actuatorSpaceConfigurationGain = bentActuatorSpacePositionGain.getDoubleValue();
         actuatorSpaceVelocityGain = bentActuatorSpaceVelocityGain.getDoubleValue();
      }

      @Override
      public void onEntry()
      {
         legControlWeight.set(LegControlWeight.LOW);
      }

      @Override
      public void onExit()
      {
      }
   }

   private class CollapseKneeControlState implements State
   {
      @Override
      public boolean isDone(double timeInState)
      {
         return false;
      }

      @Override
      public void doAction(double timeInState)
      {
         double collapsedAngle = desiredFractionOfMidRangeForCollapsed.getDoubleValue() * kneeMidRangeOfMotion;
         //         double alpha = MathTools.clamp(timeInState / collapsingDuration.getDoubleValue(), 0.0, 1.0);
         double alpha = computeQuadraticCollapseFactor(timeInState, collapsingDuration.getDoubleValue());
         double desiredKneePosition = InterpolationTools.linearInterpolate(desiredAngle.getDoubleValue(), collapsedAngle, alpha);
         desiredKneePosition = Math.max(desiredKneePosition, desiredAngleWhenStraight.getDoubleValue());

         kneePitchPrivilegedConfiguration.set(desiredKneePosition);

         jointSpaceConfigurationGain = bentJointSpacePositionGain.getDoubleValue();
         jointSpaceVelocityGain = bentJointSpaceVelocityGain.getDoubleValue();
         actuatorSpaceConfigurationGain = bentActuatorSpacePositionGain.getDoubleValue();
         actuatorSpaceVelocityGain = bentActuatorSpaceVelocityGain.getDoubleValue();
      }

      private double computeConstantCollapseFactor(double timeInState, double duration)
      {
         return Math.max(timeInState / duration, 0.0);
      }

      private double computeQuadraticCollapseFactor(double timeInState, double duration)
      {
         return Math.max(Math.pow(timeInState / duration, 2.0), 0.0);
      }

      @Override
      public void onEntry()
      {
         legControlWeight.set(LegControlWeight.LOW);
      }

      @Override
      public void onExit()
      {
      }
   }
}
