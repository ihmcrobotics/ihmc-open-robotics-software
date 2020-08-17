package us.ihmc.robotiq.simulatedHand;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.List;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.partNames.FingerName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.robotiq.RobotiqGraspMode;
import us.ihmc.robotiq.model.RobotiqHandModel.RobotiqHandJointNameMinimal;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

public class IndividualRobotiqHandController implements RobotController
{
   enum GraspState
   {
      BASIC_OPEN, BASIC_ONLY_THUMB_OPEN, BASIC_CLOSED, BASIC_ONLY_THUMB_CLOSED,
      PINCH_OPEN, PINCH_ONLY_THUMB_OPEN, PINCH_CLOSED, PINCH_ONLY_THUMB_CLOSED,
      WIDE_OPEN, WIDE_ONLY_THUMB_OPEN, WIDE_CLOSED, WIDE_ONLY_THUMB_CLOSED,
      HOOK
   }
   
   private final boolean DEBUG = false;
   
   private final String name = getClass().getSimpleName();
   private final YoRegistry registry;

   private final RobotSide robotSide;

   private final List<RobotiqHandJointNameMinimal> indexJointEnumValues = new ArrayList<RobotiqHandJointNameMinimal>();
   private final List<RobotiqHandJointNameMinimal> middleJointEnumValues = new ArrayList<RobotiqHandJointNameMinimal>();
   private final List<RobotiqHandJointNameMinimal> thumbJointEnumValues = new ArrayList<RobotiqHandJointNameMinimal>();

   private final EnumMap<RobotiqHandJointNameMinimal, OneDegreeOfFreedomJoint> indexJoints = new EnumMap<>(RobotiqHandJointNameMinimal.class);
   private final EnumMap<RobotiqHandJointNameMinimal, OneDegreeOfFreedomJoint> middleJoints = new EnumMap<>(RobotiqHandJointNameMinimal.class);
   private final EnumMap<RobotiqHandJointNameMinimal, OneDegreeOfFreedomJoint> thumbJoints = new EnumMap<>(RobotiqHandJointNameMinimal.class);

   private final List<OneDegreeOfFreedomJoint> allFingerJoints = new ArrayList<>();

   private final YoPolynomial yoPolynomial;
   private final YoDouble yoTime;
   private final YoDouble startTrajectoryTime, currentTrajectoryTime, endTrajectoryTime, trajectoryTime;
   private final YoBoolean hasTrajectoryTimeChanged, isStopped;
   private final LinkedHashMap<OneDegreeOfFreedomJoint, YoDouble> initialDesiredAngles = new LinkedHashMap<>();
   private final LinkedHashMap<OneDegreeOfFreedomJoint, YoDouble> finalDesiredAngles = new LinkedHashMap<>();
   private final LinkedHashMap<OneDegreeOfFreedomJoint, YoDouble> desiredAngles = new LinkedHashMap<>();
   
   private final YoEnum<RobotiqGraspMode> graspMode;
   private final YoEnum<RobotiqGraspMode> desiredGraspMode;
   private final YoEnum<HandConfiguration> handConfiguration;
   private final YoEnum<HandConfiguration> handDesiredConfiguration;
   
   private StateMachine<GraspState, State> stateMachine;

   public IndividualRobotiqHandController(RobotSide robotSide, YoDouble yoTime, YoDouble trajectoryTime, FloatingRootJointRobot simulatedRobot,
         YoRegistry parentRegistry)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      registry = new YoRegistry(sidePrefix + name);
      parentRegistry.addChild(registry);
      this.robotSide = robotSide;
      this.yoTime = yoTime;

      for (RobotiqHandJointNameMinimal jointEnum : RobotiqHandJointNameMinimal.values)
      {
         String jointName = jointEnum.getJointName(robotSide);
         OneDegreeOfFreedomJoint fingerJoint = simulatedRobot.getOneDegreeOfFreedomJoint(jointName);

         YoDouble initialDesiredAngle = new YoDouble("q_d_initial_" + jointName, registry);
         initialDesiredAngles.put(fingerJoint, initialDesiredAngle);

         YoDouble finalDesiredAngle = new YoDouble("q_d_final_" + jointName, registry);
         finalDesiredAngles.put(fingerJoint, finalDesiredAngle);

         YoDouble desiredAngle = new YoDouble("q_d_" + jointName, registry);
         desiredAngles.put(fingerJoint, desiredAngle);

         allFingerJoints.add(fingerJoint);

         switch (jointEnum.getFinger(robotSide))
         {
         case INDEX:
            indexJoints.put(jointEnum, fingerJoint);
            indexJointEnumValues.add(jointEnum);
            break;

         case MIDDLE:
            middleJoints.put(jointEnum, fingerJoint);
            middleJointEnumValues.add(jointEnum);
            break;

         case THUMB:
            thumbJoints.put(jointEnum, fingerJoint);
            thumbJointEnumValues.add(jointEnum);
            break;

         default:
            break;
         }
      }

      startTrajectoryTime = new YoDouble(sidePrefix + "StartTrajectoryTime", registry);
      currentTrajectoryTime = new YoDouble(sidePrefix + "CurrentTrajectoryTime", registry);
      endTrajectoryTime = new YoDouble(sidePrefix + "EndTrajectoryTime", registry);
      this.trajectoryTime = trajectoryTime;
      hasTrajectoryTimeChanged = new YoBoolean(sidePrefix + "HasTrajectoryTimeChanged", registry);
      isStopped = new YoBoolean(sidePrefix + "IsStopped", registry);
      isStopped.set(false);
      trajectoryTime.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            hasTrajectoryTimeChanged.set(true);
         }
      });
      yoPolynomial = new YoPolynomial(sidePrefix + name, 4, registry);
      yoPolynomial.setCubic(0.0, trajectoryTime.getDoubleValue(), 0.0, 0.0, 1.0, 0.0);
      
      graspMode = new YoEnum<>(sidePrefix + "RobotiqGraspMode", registry, RobotiqGraspMode.class);
      graspMode.set(RobotiqGraspMode.BASIC_MODE);
      desiredGraspMode = new YoEnum<>(sidePrefix + "RobotiqDesiredGraspMode", registry, RobotiqGraspMode.class);
      desiredGraspMode.set(RobotiqGraspMode.BASIC_MODE);
      handConfiguration = new YoEnum<>(sidePrefix + "RobotiqHandConfiguration", registry, HandConfiguration.class);
      handConfiguration.set(HandConfiguration.OPEN);
      handDesiredConfiguration = new YoEnum<>(sidePrefix + "RobotiqHandDesiredConfiguration", registry, HandConfiguration.class);
      handDesiredConfiguration.set(HandConfiguration.OPEN);
      
      stateMachine = setupStateMachine(sidePrefix);
   }
   
   private StateMachine<GraspState, State> setupStateMachine(String sidePrefix)
   {
      StateMachineFactory<GraspState, State> factory = new StateMachineFactory<>(GraspState.class);
      factory.setNamePrefix(sidePrefix + "RobotiqGraspStateMachine").setRegistry(registry).buildYoClock(yoTime);
      State stateOpenBasicGrip = new OpenBasicGrip();
      State stateOpenThumbBasicGrip = new OpenThumbBasicGrip();
      State stateClosedBasicGrip = new ClosedBasicGrip();
      State stateClosedThumbBasicGrip = new ClosedThumbBasicGrip();
      State stateOpenPinchGrip = new OpenPinchGrip();
      State stateOpenThumbPinchGrip = new OpenThumbPinchGrip();
      State stateClosedPinchGrip = new ClosedPinchGrip();
      State stateClosedThumbPinchGrip = new ClosedThumbPinchGrip();
      State stateOpenWideGrip = new OpenWideGrip();
      State stateOpenThumbWideGrip = new OpenThumbWideGrip();
      State stateClosedWideGrip = new ClosedWideGrip();
      State stateClosedThumbWideGrip = new ClosedThumbWideGrip();
      State stateHookGrip = new HookGrip();
      
      factory.addState(GraspState.BASIC_OPEN, stateOpenBasicGrip);
      factory.addState(GraspState.BASIC_ONLY_THUMB_OPEN, stateOpenThumbBasicGrip);
      factory.addState(GraspState.BASIC_CLOSED, stateClosedBasicGrip);
      factory.addState(GraspState.BASIC_ONLY_THUMB_CLOSED, stateClosedThumbBasicGrip);
      factory.addState(GraspState.PINCH_OPEN, stateOpenPinchGrip);
      factory.addState(GraspState.PINCH_ONLY_THUMB_OPEN, stateOpenThumbPinchGrip);
      factory.addState(GraspState.PINCH_CLOSED, stateClosedPinchGrip);
      factory.addState(GraspState.PINCH_ONLY_THUMB_CLOSED, stateClosedThumbPinchGrip);
      factory.addState(GraspState.WIDE_OPEN, stateOpenWideGrip);
      factory.addState(GraspState.WIDE_ONLY_THUMB_OPEN, stateOpenThumbWideGrip);
      factory.addState(GraspState.WIDE_CLOSED, stateClosedWideGrip);
      factory.addState(GraspState.WIDE_ONLY_THUMB_CLOSED, stateClosedThumbWideGrip);
      factory.addState(GraspState.HOOK, stateHookGrip);
      
      StateTransitionCondition openBasicGripCondition = time -> desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.BASIC_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.OPEN);
      
      StateTransitionCondition openThumbBasicGripCondition = time -> desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.BASIC_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.OPEN_THUMB);
      
      StateTransitionCondition openFingersBasicGripCondition = time -> desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.BASIC_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.OPEN_FINGERS);
      
      StateTransitionCondition closedBasicGripCondition = time -> desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.BASIC_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.CLOSE);
      
      StateTransitionCondition closedThumbBasicGripCondition = time -> desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.BASIC_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.CLOSE_THUMB);
      
      StateTransitionCondition closedFingersBasicGripCondition = time -> desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.BASIC_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.CLOSE_FINGERS);
      
      StateTransitionCondition openPinchGripCondition = time -> desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.PINCH_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.OPEN);
      
      StateTransitionCondition openThumbPinchGripCondition = time -> desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.PINCH_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.OPEN_THUMB);
      
      StateTransitionCondition openFingersPinchGripCondition = time -> desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.PINCH_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.OPEN_FINGERS);
      
      StateTransitionCondition closedPinchGripCondition = time -> desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.PINCH_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.CLOSE);
      
      StateTransitionCondition closedThumbPinchGripCondition = time -> desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.PINCH_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.CLOSE_THUMB);
      
      StateTransitionCondition closedFingersPinchGripCondition = time -> desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.PINCH_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.CLOSE_FINGERS);
      
      StateTransitionCondition openWideGripCondition = time -> desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.WIDE_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.OPEN);
      
      StateTransitionCondition openThumbWideGripCondition = time -> desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.WIDE_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.OPEN_THUMB);
      
      StateTransitionCondition openFingersWideGripCondition = time -> desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.WIDE_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.OPEN_FINGERS);
      
      StateTransitionCondition closedWideGripCondition = time -> desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.WIDE_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.CLOSE);
      
      StateTransitionCondition closedThumbWideGripCondition = time -> desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.WIDE_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.CLOSE_THUMB);
      
      StateTransitionCondition closedFingersWideGripCondition = time -> desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.WIDE_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.CLOSE_FINGERS);
      
      StateTransitionCondition hookGripCondition = time -> handDesiredConfiguration.getEnumValue().equals(HandConfiguration.HOOK);
      
      //BASIC_OPEN
      factory.addTransition(GraspState.BASIC_OPEN, GraspState.BASIC_CLOSED, closedBasicGripCondition);
      factory.addTransition(GraspState.BASIC_OPEN, GraspState.BASIC_ONLY_THUMB_CLOSED, closedThumbBasicGripCondition);
      factory.addTransition(GraspState.BASIC_OPEN, GraspState.BASIC_ONLY_THUMB_OPEN, closedFingersBasicGripCondition);
      factory.addTransition(GraspState.BASIC_OPEN, GraspState.PINCH_OPEN, openPinchGripCondition);
      factory.addTransition(GraspState.BASIC_OPEN, GraspState.WIDE_OPEN, openWideGripCondition);
      factory.addTransition(GraspState.BASIC_OPEN, GraspState.HOOK, hookGripCondition);
      
      //BASIC_ONLY_THUMB_OPEN
      factory.addTransition(GraspState.BASIC_ONLY_THUMB_OPEN, GraspState.BASIC_OPEN, openBasicGripCondition);
      factory.addTransition(GraspState.BASIC_ONLY_THUMB_OPEN, GraspState.BASIC_OPEN, openFingersBasicGripCondition);
      factory.addTransition(GraspState.BASIC_ONLY_THUMB_OPEN, GraspState.BASIC_CLOSED, closedBasicGripCondition);
      factory.addTransition(GraspState.BASIC_ONLY_THUMB_OPEN, GraspState.BASIC_CLOSED, closedThumbBasicGripCondition);
      factory.addTransition(GraspState.BASIC_ONLY_THUMB_OPEN, GraspState.PINCH_OPEN, openPinchGripCondition);
      factory.addTransition(GraspState.BASIC_ONLY_THUMB_OPEN, GraspState.WIDE_OPEN, openWideGripCondition);
      factory.addTransition(GraspState.BASIC_ONLY_THUMB_OPEN, GraspState.HOOK, hookGripCondition);
      
      //BASIC_CLOSED
      factory.addTransition(GraspState.BASIC_CLOSED, GraspState.BASIC_OPEN, openBasicGripCondition);
      factory.addTransition(GraspState.BASIC_CLOSED, GraspState.BASIC_ONLY_THUMB_OPEN, openThumbBasicGripCondition);
      factory.addTransition(GraspState.BASIC_CLOSED, GraspState.BASIC_ONLY_THUMB_CLOSED, openFingersBasicGripCondition);
      factory.addTransition(GraspState.BASIC_CLOSED, GraspState.PINCH_OPEN, openPinchGripCondition);
      factory.addTransition(GraspState.BASIC_CLOSED, GraspState.WIDE_OPEN, openWideGripCondition);
      factory.addTransition(GraspState.BASIC_CLOSED, GraspState.HOOK, hookGripCondition);
      
      //BASIC_ONLY_THUMB_CLOSED
      factory.addTransition(GraspState.BASIC_ONLY_THUMB_CLOSED, GraspState.BASIC_OPEN, openBasicGripCondition);
      factory.addTransition(GraspState.BASIC_ONLY_THUMB_CLOSED, GraspState.BASIC_OPEN, openThumbBasicGripCondition);
      factory.addTransition(GraspState.BASIC_ONLY_THUMB_CLOSED, GraspState.BASIC_CLOSED, closedBasicGripCondition);
      factory.addTransition(GraspState.BASIC_ONLY_THUMB_CLOSED, GraspState.BASIC_CLOSED, closedFingersBasicGripCondition);
      factory.addTransition(GraspState.BASIC_ONLY_THUMB_CLOSED, GraspState.PINCH_OPEN, openPinchGripCondition);
      factory.addTransition(GraspState.BASIC_ONLY_THUMB_CLOSED, GraspState.WIDE_OPEN, openWideGripCondition);
      factory.addTransition(GraspState.BASIC_ONLY_THUMB_CLOSED, GraspState.HOOK, hookGripCondition);
      
      //PINCH_OPEN
      factory.addTransition(GraspState.PINCH_OPEN, GraspState.PINCH_CLOSED, closedPinchGripCondition);
      factory.addTransition(GraspState.PINCH_OPEN, GraspState.PINCH_ONLY_THUMB_CLOSED, closedThumbPinchGripCondition);
      factory.addTransition(GraspState.PINCH_OPEN, GraspState.PINCH_ONLY_THUMB_OPEN, closedFingersPinchGripCondition);
      factory.addTransition(GraspState.PINCH_OPEN, GraspState.BASIC_OPEN, openBasicGripCondition);
      factory.addTransition(GraspState.PINCH_OPEN, GraspState.WIDE_OPEN, openWideGripCondition);
      factory.addTransition(GraspState.PINCH_OPEN, GraspState.HOOK, hookGripCondition);
      
      //PINCH_ONLY_THUMB_OPEN
      factory.addTransition(GraspState.PINCH_ONLY_THUMB_OPEN, GraspState.PINCH_OPEN, openPinchGripCondition);
      factory.addTransition(GraspState.PINCH_ONLY_THUMB_OPEN, GraspState.PINCH_OPEN, openFingersPinchGripCondition);
      factory.addTransition(GraspState.PINCH_ONLY_THUMB_OPEN, GraspState.PINCH_CLOSED, closedPinchGripCondition);
      factory.addTransition(GraspState.PINCH_ONLY_THUMB_OPEN, GraspState.PINCH_CLOSED, closedThumbPinchGripCondition);
      factory.addTransition(GraspState.PINCH_ONLY_THUMB_OPEN, GraspState.BASIC_OPEN, openBasicGripCondition);
      factory.addTransition(GraspState.PINCH_ONLY_THUMB_OPEN, GraspState.WIDE_OPEN, openWideGripCondition);
      
      //PINCH_CLOSED
      factory.addTransition(GraspState.PINCH_CLOSED, GraspState.PINCH_OPEN, openPinchGripCondition);
      factory.addTransition(GraspState.PINCH_CLOSED, GraspState.PINCH_ONLY_THUMB_OPEN, openThumbPinchGripCondition);
      factory.addTransition(GraspState.PINCH_CLOSED, GraspState.PINCH_ONLY_THUMB_CLOSED, openFingersPinchGripCondition);
      factory.addTransition(GraspState.PINCH_CLOSED, GraspState.BASIC_OPEN, openBasicGripCondition);
      factory.addTransition(GraspState.PINCH_CLOSED, GraspState.WIDE_OPEN, openWideGripCondition);
      
      //PINCH_ONLY_THUMB_CLOSED
      factory.addTransition(GraspState.PINCH_ONLY_THUMB_CLOSED, GraspState.PINCH_OPEN, openPinchGripCondition);
      factory.addTransition(GraspState.PINCH_ONLY_THUMB_CLOSED, GraspState.PINCH_OPEN, openThumbPinchGripCondition);
      factory.addTransition(GraspState.PINCH_ONLY_THUMB_CLOSED, GraspState.PINCH_CLOSED, closedPinchGripCondition);
      factory.addTransition(GraspState.PINCH_ONLY_THUMB_CLOSED, GraspState.PINCH_CLOSED, closedFingersPinchGripCondition);
      factory.addTransition(GraspState.PINCH_ONLY_THUMB_CLOSED, GraspState.BASIC_OPEN, openBasicGripCondition);
      factory.addTransition(GraspState.PINCH_ONLY_THUMB_CLOSED, GraspState.WIDE_OPEN, openWideGripCondition);
      
      //WIDE_OPEN
      factory.addTransition(GraspState.WIDE_OPEN, GraspState.WIDE_CLOSED, closedWideGripCondition);
      factory.addTransition(GraspState.WIDE_OPEN, GraspState.WIDE_ONLY_THUMB_CLOSED, closedThumbWideGripCondition);
      factory.addTransition(GraspState.WIDE_OPEN, GraspState.WIDE_ONLY_THUMB_OPEN, closedFingersWideGripCondition);
      factory.addTransition(GraspState.WIDE_OPEN, GraspState.BASIC_OPEN, openBasicGripCondition);
      factory.addTransition(GraspState.WIDE_OPEN, GraspState.PINCH_OPEN, openPinchGripCondition);
      factory.addTransition(GraspState.WIDE_OPEN, GraspState.HOOK, hookGripCondition);
      
      //WIDE_ONLY_THUMB_OPEN
      factory.addTransition(GraspState.WIDE_ONLY_THUMB_OPEN, GraspState.WIDE_OPEN, openWideGripCondition);
      factory.addTransition(GraspState.WIDE_ONLY_THUMB_OPEN, GraspState.WIDE_OPEN, openFingersWideGripCondition);
      factory.addTransition(GraspState.WIDE_ONLY_THUMB_OPEN, GraspState.WIDE_CLOSED, closedWideGripCondition);
      factory.addTransition(GraspState.WIDE_ONLY_THUMB_OPEN, GraspState.WIDE_CLOSED, closedThumbWideGripCondition);
      factory.addTransition(GraspState.WIDE_ONLY_THUMB_OPEN, GraspState.BASIC_OPEN, openBasicGripCondition);
      factory.addTransition(GraspState.WIDE_ONLY_THUMB_OPEN, GraspState.PINCH_OPEN, openPinchGripCondition);
      
      //WIDE_CLOSED
      factory.addTransition(GraspState.WIDE_CLOSED, GraspState.WIDE_OPEN, openWideGripCondition);
      factory.addTransition(GraspState.WIDE_CLOSED, GraspState.WIDE_ONLY_THUMB_OPEN, openThumbWideGripCondition);
      factory.addTransition(GraspState.WIDE_CLOSED, GraspState.WIDE_ONLY_THUMB_CLOSED, openFingersWideGripCondition);
      factory.addTransition(GraspState.WIDE_CLOSED, GraspState.BASIC_OPEN, openBasicGripCondition);
      factory.addTransition(GraspState.WIDE_CLOSED, GraspState.PINCH_OPEN, openPinchGripCondition);
      
      //WIDE_ONLY_THUMB_CLOSED
      factory.addTransition(GraspState.WIDE_ONLY_THUMB_CLOSED, GraspState.WIDE_OPEN, openWideGripCondition);
      factory.addTransition(GraspState.WIDE_ONLY_THUMB_CLOSED, GraspState.WIDE_OPEN, openThumbWideGripCondition);
      factory.addTransition(GraspState.WIDE_ONLY_THUMB_CLOSED, GraspState.WIDE_CLOSED, closedWideGripCondition);
      factory.addTransition(GraspState.WIDE_ONLY_THUMB_CLOSED, GraspState.WIDE_CLOSED, closedFingersWideGripCondition);
      factory.addTransition(GraspState.WIDE_ONLY_THUMB_CLOSED, GraspState.BASIC_OPEN, openBasicGripCondition);
      factory.addTransition(GraspState.WIDE_ONLY_THUMB_CLOSED, GraspState.PINCH_OPEN, openPinchGripCondition);
      
      //HOOK
      factory.addTransition(GraspState.HOOK, GraspState.BASIC_OPEN, openBasicGripCondition);
      factory.addTransition(GraspState.HOOK, GraspState.PINCH_OPEN, openPinchGripCondition);
      factory.addTransition(GraspState.HOOK, GraspState.WIDE_OPEN, openWideGripCondition);

      return factory.build(GraspState.BASIC_OPEN);
   }
   
   private class OpenBasicGrip implements State
   {
      @Override
      public void doAction(double timeInState)
      {
      }

      @Override
      public void onEntry()
      {
         isStopped.set(false);
         graspMode.set(RobotiqGraspMode.BASIC_MODE);
         handConfiguration.set(HandConfiguration.OPEN);
         computeAllFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getOpenBasicGripDesiredConfiguration(robotSide));
      }

      @Override
      public void onExit()
      {
      }
   }
   
   private class OpenThumbBasicGrip implements State
   {
      @Override
      public void doAction(double timeInState)
      {
      }

      @Override
      public void onEntry()
      {
         isStopped.set(false);
         graspMode.set(RobotiqGraspMode.BASIC_MODE);
         if(handDesiredConfiguration.getEnumValue().equals(HandConfiguration.OPEN_THUMB))
         {
            handConfiguration.set(HandConfiguration.OPEN_THUMB);
            computeThumbFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getOpenBasicGripDesiredConfiguration(robotSide));
         }
         else if(handDesiredConfiguration.getEnumValue().equals(HandConfiguration.CLOSE_FINGERS))
         {
            handConfiguration.set(HandConfiguration.CLOSE_FINGERS);
            computeIndexFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getClosedBasicGripDesiredConfiguration(robotSide));
            computeMiddleFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getClosedBasicGripDesiredConfiguration(robotSide));
         }
      }

      @Override
      public void onExit()
      {
      }
   }
   
   private class ClosedBasicGrip implements State
   {
      @Override
      public void doAction(double timeInState)
      {
      }

      @Override
      public void onEntry()
      {
         isStopped.set(false);
         graspMode.set(RobotiqGraspMode.BASIC_MODE);
         handConfiguration.set(HandConfiguration.CLOSE);
         computeAllFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getClosedBasicGripDesiredConfiguration(robotSide));
      }

      @Override
      public void onExit()
      {
      }
   }
   
   private class ClosedThumbBasicGrip implements State
   {
      @Override
      public void doAction(double timeInState)
      {
      }

      @Override
      public void onEntry()
      {
         isStopped.set(false);
         graspMode.set(RobotiqGraspMode.BASIC_MODE);
         if(handDesiredConfiguration.getEnumValue().equals(HandConfiguration.CLOSE_THUMB))
         {
            handConfiguration.set(HandConfiguration.CLOSE_THUMB);
            computeThumbFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getClosedBasicGripDesiredConfiguration(robotSide));
         }
         else if(handDesiredConfiguration.getEnumValue().equals(HandConfiguration.OPEN_FINGERS))
         {
            handConfiguration.set(HandConfiguration.OPEN_FINGERS);
            computeIndexFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getOpenBasicGripDesiredConfiguration(robotSide));
            computeMiddleFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getOpenBasicGripDesiredConfiguration(robotSide));
         }
      }

      @Override
      public void onExit()
      {
      }
   }
   
   private class OpenPinchGrip implements State
   {
      @Override
      public void doAction(double timeInState)
      {
      }

      @Override
      public void onEntry()
      {
         isStopped.set(false);
         graspMode.set(RobotiqGraspMode.PINCH_MODE);
         handConfiguration.set(HandConfiguration.OPEN);
         computeAllFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getOpenPinchGripDesiredConfiguration(robotSide));
      }

      @Override
      public void onExit()
      {
      }
   }
   
   private class OpenThumbPinchGrip implements State
   {
      @Override
      public void doAction(double timeInState)
      {
      }

      @Override
      public void onEntry()
      {
         isStopped.set(false);
         graspMode.set(RobotiqGraspMode.PINCH_MODE);
         if(handDesiredConfiguration.getEnumValue().equals(HandConfiguration.OPEN_THUMB))
         {
            handConfiguration.set(HandConfiguration.OPEN_THUMB);
            computeThumbFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getOpenPinchGripDesiredConfiguration(robotSide));
         }
         else if(handDesiredConfiguration.getEnumValue().equals(HandConfiguration.CLOSE_FINGERS))
         {
            handConfiguration.set(HandConfiguration.CLOSE_FINGERS);
            computeIndexFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getClosedPinchGripDesiredConfiguration(robotSide));
            computeMiddleFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getClosedPinchGripDesiredConfiguration(robotSide));
         }
      }

      @Override
      public void onExit()
      {
      }
   }
   
   private class ClosedPinchGrip implements State
   {
      @Override
      public void doAction(double timeInState)
      {
      }

      @Override
      public void onEntry()
      {
         isStopped.set(false);
         graspMode.set(RobotiqGraspMode.PINCH_MODE);
         handConfiguration.set(HandConfiguration.CLOSE);
         computeAllFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getClosedPinchGripDesiredConfiguration(robotSide));
      }

      @Override
      public void onExit()
      {
      }
   }
   
   private class ClosedThumbPinchGrip implements State
   {
      @Override
      public void doAction(double timeInState)
      {
      }

      @Override
      public void onEntry()
      {
         isStopped.set(false);
         graspMode.set(RobotiqGraspMode.PINCH_MODE);
         if(handDesiredConfiguration.getEnumValue().equals(HandConfiguration.CLOSE_THUMB))
         {
            handConfiguration.set(HandConfiguration.CLOSE_THUMB);
            computeThumbFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getClosedPinchGripDesiredConfiguration(robotSide));
         }
         else if(handDesiredConfiguration.getEnumValue().equals(HandConfiguration.OPEN_FINGERS))
         {
            handConfiguration.set(HandConfiguration.OPEN_FINGERS);
            computeIndexFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getOpenPinchGripDesiredConfiguration(robotSide));
            computeMiddleFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getOpenPinchGripDesiredConfiguration(robotSide));
         }
      }

      @Override
      public void onExit()
      {
      }
   }
   
   private class OpenWideGrip implements State
   {
      @Override
      public void doAction(double timeInState)
      {
      }

      @Override
      public void onEntry()
      {
         isStopped.set(false);
         graspMode.set(RobotiqGraspMode.WIDE_MODE);
         handConfiguration.set(HandConfiguration.OPEN);
         computeAllFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getOpenWideGripDesiredConfiguration(robotSide));
      }

      @Override
      public void onExit()
      {
      }
   }
   
   private class OpenThumbWideGrip implements State
   {
      @Override
      public void doAction(double timeInState)
      {
      }

      @Override
      public void onEntry()
      {
         isStopped.set(false);
         graspMode.set(RobotiqGraspMode.WIDE_MODE);
         if(handDesiredConfiguration.getEnumValue().equals(HandConfiguration.OPEN_THUMB))
         {
            handConfiguration.set(HandConfiguration.OPEN_THUMB);
            computeThumbFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getOpenWideGripDesiredConfiguration(robotSide));
         }
         else if(handDesiredConfiguration.getEnumValue().equals(HandConfiguration.CLOSE_FINGERS))
         {
            handConfiguration.set(HandConfiguration.CLOSE_FINGERS);
            computeIndexFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getClosedWideGripDesiredConfiguration(robotSide));
            computeMiddleFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getClosedWideGripDesiredConfiguration(robotSide));
         }
      }

      @Override
      public void onExit()
      {
      }
   }
   
   private class ClosedWideGrip implements State
   {
      @Override
      public void doAction(double timeInState)
      {
      }

      @Override
      public void onEntry()
      {
         isStopped.set(false);
         graspMode.set(RobotiqGraspMode.WIDE_MODE);
         handConfiguration.set(HandConfiguration.CLOSE);
         computeAllFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getClosedWideGripDesiredConfiguration(robotSide));
      }

      @Override
      public void onExit()
      {
      }
   }
   
   private class ClosedThumbWideGrip implements State
   {
      @Override
      public void doAction(double timeInState)
      {
      }

      @Override
      public void onEntry()
      {
         isStopped.set(false);
         graspMode.set(RobotiqGraspMode.WIDE_MODE);
         if(handDesiredConfiguration.getEnumValue().equals(HandConfiguration.CLOSE_THUMB))
         {
            handConfiguration.set(HandConfiguration.CLOSE_THUMB);
            computeThumbFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getClosedWideGripDesiredConfiguration(robotSide));
         }
         else if(handDesiredConfiguration.getEnumValue().equals(HandConfiguration.OPEN_FINGERS))
         {
            handConfiguration.set(HandConfiguration.OPEN_FINGERS);
            computeIndexFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getOpenWideGripDesiredConfiguration(robotSide));
            computeMiddleFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getOpenWideGripDesiredConfiguration(robotSide));
         }
      }

      @Override
      public void onExit()
      {
      }
   }
   
   private class HookGrip implements State
   {
      @Override
      public void doAction(double timeInState)
      {
         
      }

      @Override
      public void onEntry()
      {
         isStopped.set(false);
         graspMode.set(RobotiqGraspMode.BASIC_MODE);
         handConfiguration.set(HandConfiguration.HOOK);
         computeIndexFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getOpenBasicGripDesiredConfiguration(robotSide));
         computeMiddleFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getClosedBasicGripDesiredConfiguration(robotSide));
         computeThumbFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getClosedBasicGripDesiredConfiguration(robotSide));
      }

      @Override
      public void onExit()
      {
      }
   }
   
   @Override
   public void doControl()
   {
      stateMachine.doActionAndTransition();
      computeDesiredJointAngles();
   }
   
   public void open()
   {
      handDesiredConfiguration.set(HandConfiguration.OPEN);
   }
   
   public void openThumb()
   {
      handDesiredConfiguration.set(HandConfiguration.OPEN_THUMB);
   }
   
   public void openFingers()
   {
      handDesiredConfiguration.set(HandConfiguration.OPEN_FINGERS);
   }
   
   public void close()
   {
      handDesiredConfiguration.set(HandConfiguration.CLOSE);
   }
   
   public void closeThumb()
   {
      handDesiredConfiguration.set(HandConfiguration.CLOSE_THUMB);
   }
   
   public void closeFingers()
   {
      handDesiredConfiguration.set(HandConfiguration.CLOSE_FINGERS);
   }
   
   public void hook()
   {
      handDesiredConfiguration.set(HandConfiguration.HOOK);
   }

   public void crush()
   {
      close();
   }
   
   public void crushThumb()
   {
      closeThumb();
   }

   public void basicGrip()
   {
      desiredGraspMode.set(RobotiqGraspMode.BASIC_MODE);
   }
   
   public void pinchGrip()
   {
      desiredGraspMode.set(RobotiqGraspMode.PINCH_MODE);
   }
   
   public void wideGrip()
   {
      desiredGraspMode.set(RobotiqGraspMode.WIDE_MODE);
   }

   public void stop()
   {
      if (!isStopped.getBooleanValue())
      {
         isStopped.set(true);
      }
   }

   public void reset()
   {
      isStopped.set(false);
      for (int i = 0; i < allFingerJoints.size(); i++)
      {
         OneDegreeOfFreedomJoint fingerJoint = allFingerJoints.get(i);
         finalDesiredAngles.get(fingerJoint).set(0.0);
      }
   }

   private void computeAllFinalDesiredAngles(double percent, EnumMap<RobotiqHandJointNameMinimal, Double> handDesiredConfiguration)
   {
      computeIndexFinalDesiredAngles(percent, handDesiredConfiguration);

      computeMiddleFinalDesiredAngles(percent, handDesiredConfiguration);

      computeThumbFinalDesiredAngles(percent, handDesiredConfiguration);
   }

   private void computeOneFingerDesiredAngles(double percent, EnumMap<RobotiqHandJointNameMinimal, Double> fingerDesiredConfiguration, FingerName fingerName)
   {
      switch (fingerName)
      {
      case INDEX:
         computeIndexFinalDesiredAngles(percent, fingerDesiredConfiguration);
         break;

      case MIDDLE:
         computeMiddleFinalDesiredAngles(percent, fingerDesiredConfiguration);
         break;

      case THUMB:
         computeThumbFinalDesiredAngles(percent, fingerDesiredConfiguration);
      default:
         break;
      }
   }

   private void computeThumbFinalDesiredAngles(double percent, EnumMap<RobotiqHandJointNameMinimal, Double> fingerDesiredConfiguration)
   {
      for (int i = 0; i < thumbJointEnumValues.size(); i++)
      {
         RobotiqHandJointNameMinimal fingerJointEnum = thumbJointEnumValues.get(i);
         OneDegreeOfFreedomJoint fingerJoint = thumbJoints.get(fingerJointEnum);
         double qDesired = percent * fingerDesiredConfiguration.get(fingerJointEnum);
         finalDesiredAngles.get(fingerJoint).set(qDesired);
      }
      initializeTrajectory();
   }

   private void computeMiddleFinalDesiredAngles(double percent, EnumMap<RobotiqHandJointNameMinimal, Double> fingerDesiredConfiguration)
   {
      for (int i = 0; i < middleJointEnumValues.size(); i++)
      {
         RobotiqHandJointNameMinimal fingerJointEnum = middleJointEnumValues.get(i);
         OneDegreeOfFreedomJoint fingerJoint = middleJoints.get(fingerJointEnum);
         double qDesired = percent * fingerDesiredConfiguration.get(fingerJointEnum);
         finalDesiredAngles.get(fingerJoint).set(qDesired);
      }
      initializeTrajectory();
   }

   private void computeIndexFinalDesiredAngles(double percent, EnumMap<RobotiqHandJointNameMinimal, Double> fingerDesiredConfiguration)
   {
      for (int i = 0; i < indexJointEnumValues.size(); i++)
      {
         RobotiqHandJointNameMinimal fingerJointEnum = indexJointEnumValues.get(i);
         OneDegreeOfFreedomJoint fingerJoint = indexJoints.get(fingerJointEnum);
         double qDesired = percent * fingerDesiredConfiguration.get(fingerJointEnum);
         finalDesiredAngles.get(fingerJoint).set(qDesired);
      }
      initializeTrajectory();
   }

   /**
    * Only place where the SCS robot should be modified
    */
   public void writeDesiredJointAngles()
   {
      for (int i = 0; i < allFingerJoints.size(); i++)
      {
         OneDegreeOfFreedomJoint fingerJoint = allFingerJoints.get(i);
         fingerJoint.setqDesired(desiredAngles.get(fingerJoint).getDoubleValue());
      }
   }

   private void initializeTrajectory()
   {
      for (int i = 0; i < allFingerJoints.size(); i++)
      {
         OneDegreeOfFreedomJoint fingerJoint = allFingerJoints.get(i);
         initialDesiredAngles.get(fingerJoint).set(desiredAngles.get(fingerJoint).getDoubleValue());
      }

      startTrajectoryTime.set(yoTime.getDoubleValue());
      endTrajectoryTime.set(startTrajectoryTime.getDoubleValue() + trajectoryTime.getDoubleValue());

      if (hasTrajectoryTimeChanged.getBooleanValue())
      {
         yoPolynomial.setCubic(0.0, trajectoryTime.getDoubleValue(), 0.0, 0.0, 1.0, 0.0);
         hasTrajectoryTimeChanged.set(false);
      }
   }

   private void computeDesiredJointAngles()
   {
      if (!isStopped.getBooleanValue())
      {
         currentTrajectoryTime.set(yoTime.getDoubleValue() - startTrajectoryTime.getDoubleValue());
         currentTrajectoryTime.set(MathTools.clamp(currentTrajectoryTime.getDoubleValue(), 0.0, trajectoryTime.getDoubleValue()));
      }
      yoPolynomial.compute(currentTrajectoryTime.getDoubleValue());
      double alpha = MathTools.clamp(yoPolynomial.getPosition(), 0.0, 1.0);

      for (int i = 0; i < allFingerJoints.size(); i++)
      {
         OneDegreeOfFreedomJoint fingerJoint = allFingerJoints.get(i);

         double q_d_initial = initialDesiredAngles.get(fingerJoint).getDoubleValue();
         double q_d_final = finalDesiredAngles.get(fingerJoint).getDoubleValue();
         double q_d = (1.0 - alpha) * q_d_initial + alpha * q_d_final;
         desiredAngles.get(fingerJoint).set(q_d);
         
         if (DEBUG && alpha > 0.0 && alpha < 1.0)
            PrintTools.debug(this, fingerJoint.getName() + "Desired q : " + q_d);
      }
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public String getName()
   {
      return robotSide.getCamelCaseNameForStartOfExpression() + getClass().getSimpleName();
   }

   @Override
   public String getDescription()
   {
      return "Simulated controller for the " + robotSide.getLowerCaseName() + " Robotiq hands.";
   }
}
