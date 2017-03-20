package us.ihmc.robotiq.simulatedHand;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.List;

import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.robotics.partNames.FingerName;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.commons.PrintTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachine;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransition;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.robotiq.RobotiqGraspMode;
import us.ihmc.robotiq.model.RobotiqHandModel.RobotiqHandJointNameMinimal;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

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
   private final YoVariableRegistry registry;

   private final RobotSide robotSide;

   private final List<RobotiqHandJointNameMinimal> indexJointEnumValues = new ArrayList<RobotiqHandJointNameMinimal>();
   private final List<RobotiqHandJointNameMinimal> middleJointEnumValues = new ArrayList<RobotiqHandJointNameMinimal>();
   private final List<RobotiqHandJointNameMinimal> thumbJointEnumValues = new ArrayList<RobotiqHandJointNameMinimal>();

   private final EnumMap<RobotiqHandJointNameMinimal, OneDegreeOfFreedomJoint> indexJoints = new EnumMap<>(RobotiqHandJointNameMinimal.class);
   private final EnumMap<RobotiqHandJointNameMinimal, OneDegreeOfFreedomJoint> middleJoints = new EnumMap<>(RobotiqHandJointNameMinimal.class);
   private final EnumMap<RobotiqHandJointNameMinimal, OneDegreeOfFreedomJoint> thumbJoints = new EnumMap<>(RobotiqHandJointNameMinimal.class);

   private final List<OneDegreeOfFreedomJoint> allFingerJoints = new ArrayList<>();

   private final YoPolynomial yoPolynomial;
   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable startTrajectoryTime, currentTrajectoryTime, endTrajectoryTime, trajectoryTime;
   private final BooleanYoVariable hasTrajectoryTimeChanged, isStopped;
   private final LinkedHashMap<OneDegreeOfFreedomJoint, DoubleYoVariable> initialDesiredAngles = new LinkedHashMap<>();
   private final LinkedHashMap<OneDegreeOfFreedomJoint, DoubleYoVariable> finalDesiredAngles = new LinkedHashMap<>();
   private final LinkedHashMap<OneDegreeOfFreedomJoint, DoubleYoVariable> desiredAngles = new LinkedHashMap<>();
   
   private final EnumYoVariable<RobotiqGraspMode> graspMode;
   private final EnumYoVariable<RobotiqGraspMode> desiredGraspMode;
   private final EnumYoVariable<HandConfiguration> handConfiguration;
   private final EnumYoVariable<HandConfiguration> handDesiredConfiguration;
   
   private StateMachine<GraspState> stateMachine;

   public IndividualRobotiqHandController(RobotSide robotSide, DoubleYoVariable yoTime, DoubleYoVariable trajectoryTime, FloatingRootJointRobot simulatedRobot,
         YoVariableRegistry parentRegistry)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      registry = new YoVariableRegistry(sidePrefix + name);
      parentRegistry.addChild(registry);
      this.robotSide = robotSide;
      this.yoTime = yoTime;

      for (RobotiqHandJointNameMinimal jointEnum : RobotiqHandJointNameMinimal.values)
      {
         String jointName = jointEnum.getJointName(robotSide);
         OneDegreeOfFreedomJoint fingerJoint = simulatedRobot.getOneDegreeOfFreedomJoint(jointName);

         DoubleYoVariable initialDesiredAngle = new DoubleYoVariable("q_d_initial_" + jointName, registry);
         initialDesiredAngles.put(fingerJoint, initialDesiredAngle);

         DoubleYoVariable finalDesiredAngle = new DoubleYoVariable("q_d_final_" + jointName, registry);
         finalDesiredAngles.put(fingerJoint, finalDesiredAngle);

         DoubleYoVariable desiredAngle = new DoubleYoVariable("q_d_" + jointName, registry);
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

      startTrajectoryTime = new DoubleYoVariable(sidePrefix + "StartTrajectoryTime", registry);
      currentTrajectoryTime = new DoubleYoVariable(sidePrefix + "CurrentTrajectoryTime", registry);
      endTrajectoryTime = new DoubleYoVariable(sidePrefix + "EndTrajectoryTime", registry);
      this.trajectoryTime = trajectoryTime;
      hasTrajectoryTimeChanged = new BooleanYoVariable(sidePrefix + "HasTrajectoryTimeChanged", registry);
      isStopped = new BooleanYoVariable(sidePrefix + "IsStopped", registry);
      isStopped.set(false);
      trajectoryTime.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            hasTrajectoryTimeChanged.set(true);
         }
      });
      yoPolynomial = new YoPolynomial(sidePrefix + name, 4, registry);
      yoPolynomial.setCubic(0.0, trajectoryTime.getDoubleValue(), 0.0, 0.0, 1.0, 0.0);
      
      graspMode = new EnumYoVariable<>(sidePrefix + "RobotiqGraspMode", registry, RobotiqGraspMode.class);
      graspMode.set(RobotiqGraspMode.BASIC_MODE);
      desiredGraspMode = new EnumYoVariable<>(sidePrefix + "RobotiqDesiredGraspMode", registry, RobotiqGraspMode.class);
      desiredGraspMode.set(RobotiqGraspMode.BASIC_MODE);
      handConfiguration = new EnumYoVariable<>(sidePrefix + "RobotiqHandConfiguration", registry, HandConfiguration.class);
      handConfiguration.set(HandConfiguration.OPEN);
      handDesiredConfiguration = new EnumYoVariable<>(sidePrefix + "RobotiqHandDesiredConfiguration", registry, HandConfiguration.class);
      handDesiredConfiguration.set(HandConfiguration.OPEN);
      
      stateMachine = new StateMachine<>(sidePrefix + "RobotiqGraspStateMachine", "FingerTrajectoryTime", GraspState.class, yoTime, registry);
      setupStateMachine();
   }
   
   private void setupStateMachine()
   {
      State<GraspState> stateOpenBasicGrip = new OpenBasicGrip();
      State<GraspState> stateOpenThumbBasicGrip = new OpenThumbBasicGrip();
      State<GraspState> stateClosedBasicGrip = new ClosedBasicGrip();
      State<GraspState> stateClosedThumbBasicGrip = new ClosedThumbBasicGrip();
      State<GraspState> stateOpenPinchGrip = new OpenPinchGrip();
      State<GraspState> stateOpenThumbPinchGrip = new OpenThumbPinchGrip();
      State<GraspState> stateClosedPinchGrip = new ClosedPinchGrip();
      State<GraspState> stateClosedThumbPinchGrip = new ClosedThumbPinchGrip();
      State<GraspState> stateOpenWideGrip = new OpenWideGrip();
      State<GraspState> stateOpenThumbWideGrip = new OpenThumbWideGrip();
      State<GraspState> stateClosedWideGrip = new ClosedWideGrip();
      State<GraspState> stateClosedThumbWideGrip = new ClosedThumbWideGrip();
      State<GraspState> stateHookGrip = new HookGrip();
      
      stateMachine.addState(stateOpenBasicGrip);
      stateMachine.addState(stateOpenThumbBasicGrip);
      stateMachine.addState(stateClosedBasicGrip);
      stateMachine.addState(stateClosedThumbBasicGrip);
      stateMachine.addState(stateOpenPinchGrip);
      stateMachine.addState(stateOpenThumbPinchGrip);
      stateMachine.addState(stateClosedPinchGrip);
      stateMachine.addState(stateClosedThumbPinchGrip);
      stateMachine.addState(stateOpenWideGrip);
      stateMachine.addState(stateOpenThumbWideGrip);
      stateMachine.addState(stateClosedWideGrip);
      stateMachine.addState(stateClosedThumbWideGrip);
      stateMachine.addState(stateHookGrip);
      stateMachine.setCurrentState(GraspState.BASIC_OPEN);
      
      StateTransitionCondition openBasicGripCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.BASIC_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.OPEN);
         }
      };
      
      StateTransitionCondition openThumbBasicGripCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.BASIC_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.OPEN_THUMB);
         }
      };
      
      StateTransitionCondition openFingersBasicGripCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.BASIC_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.OPEN_FINGERS);
         }
      };
      
      StateTransitionCondition closedBasicGripCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.BASIC_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.CLOSE);
         }
      };
      
      StateTransitionCondition closedThumbBasicGripCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.BASIC_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.CLOSE_THUMB);
         }
      };
      
      StateTransitionCondition closedFingersBasicGripCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.BASIC_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.CLOSE_FINGERS);
         }
      };
      
      StateTransitionCondition openPinchGripCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.PINCH_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.OPEN);
         }
      };
      
      StateTransitionCondition openThumbPinchGripCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.PINCH_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.OPEN_THUMB);
         }
      };
      
      StateTransitionCondition openFingersPinchGripCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.PINCH_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.OPEN_FINGERS);
         }
      };
      
      StateTransitionCondition closedPinchGripCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.PINCH_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.CLOSE);
         }
      };
      
      StateTransitionCondition closedThumbPinchGripCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.PINCH_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.CLOSE_THUMB);
         }
      };
      
      StateTransitionCondition closedFingersPinchGripCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.PINCH_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.CLOSE_FINGERS);
         }
      };
      
      StateTransitionCondition openWideGripCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.WIDE_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.OPEN);
         }
      };
      
      StateTransitionCondition openThumbWideGripCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.WIDE_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.OPEN_THUMB);
         }
      };
      
      StateTransitionCondition openFingersWideGripCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.WIDE_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.OPEN_FINGERS);
         }
      };
      
      StateTransitionCondition closedWideGripCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.WIDE_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.CLOSE);
         }
      };
      
      StateTransitionCondition closedThumbWideGripCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.WIDE_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.CLOSE_THUMB);
         }
      };
      
      StateTransitionCondition closedFingersWideGripCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return desiredGraspMode.getEnumValue().equals(RobotiqGraspMode.WIDE_MODE) && handDesiredConfiguration.getEnumValue().equals(HandConfiguration.CLOSE_FINGERS);
         }
      };
      
      StateTransitionCondition hookGripCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return handDesiredConfiguration.getEnumValue().equals(HandConfiguration.HOOK);
         }
      };
      
      //BASIC_OPEN
      stateOpenBasicGrip.addStateTransition(new StateTransition<GraspState>(GraspState.BASIC_CLOSED, closedBasicGripCondition));
      stateOpenBasicGrip.addStateTransition(new StateTransition<GraspState>(GraspState.BASIC_ONLY_THUMB_CLOSED, closedThumbBasicGripCondition));
      stateOpenBasicGrip.addStateTransition(new StateTransition<GraspState>(GraspState.BASIC_ONLY_THUMB_OPEN, closedFingersBasicGripCondition));
      stateOpenBasicGrip.addStateTransition(new StateTransition<GraspState>(GraspState.PINCH_OPEN, openPinchGripCondition));
      stateOpenBasicGrip.addStateTransition(new StateTransition<GraspState>(GraspState.WIDE_OPEN, openWideGripCondition));
      stateOpenBasicGrip.addStateTransition(new StateTransition<GraspState>(GraspState.HOOK, hookGripCondition));
      
      //BASIC_ONLY_THUMB_OPEN
      stateOpenThumbBasicGrip.addStateTransition(new StateTransition<GraspState>(GraspState.BASIC_OPEN, openBasicGripCondition));
      stateOpenThumbBasicGrip.addStateTransition(new StateTransition<GraspState>(GraspState.BASIC_OPEN, openFingersBasicGripCondition));
      stateOpenThumbBasicGrip.addStateTransition(new StateTransition<GraspState>(GraspState.BASIC_CLOSED, closedBasicGripCondition));
      stateOpenThumbBasicGrip.addStateTransition(new StateTransition<GraspState>(GraspState.BASIC_CLOSED, closedThumbBasicGripCondition));
      stateOpenThumbBasicGrip.addStateTransition(new StateTransition<GraspState>(GraspState.PINCH_OPEN, openPinchGripCondition));
      stateOpenThumbBasicGrip.addStateTransition(new StateTransition<GraspState>(GraspState.WIDE_OPEN, openWideGripCondition));
      stateOpenThumbBasicGrip.addStateTransition(new StateTransition<GraspState>(GraspState.HOOK, hookGripCondition));
      
      //BASIC_CLOSED
      stateClosedBasicGrip.addStateTransition(new StateTransition<GraspState>(GraspState.BASIC_OPEN, openBasicGripCondition));
      stateClosedBasicGrip.addStateTransition(new StateTransition<GraspState>(GraspState.BASIC_ONLY_THUMB_OPEN, openThumbBasicGripCondition));
      stateClosedBasicGrip.addStateTransition(new StateTransition<GraspState>(GraspState.BASIC_ONLY_THUMB_CLOSED, openFingersBasicGripCondition));
      stateClosedBasicGrip.addStateTransition(new StateTransition<GraspState>(GraspState.PINCH_OPEN, openPinchGripCondition));
      stateClosedBasicGrip.addStateTransition(new StateTransition<GraspState>(GraspState.WIDE_OPEN, openWideGripCondition));
      stateClosedBasicGrip.addStateTransition(new StateTransition<GraspState>(GraspState.HOOK, hookGripCondition));
      
      //BASIC_ONLY_THUMB_CLOSED
      stateClosedThumbBasicGrip.addStateTransition(new StateTransition<GraspState>(GraspState.BASIC_OPEN, openBasicGripCondition));
      stateClosedThumbBasicGrip.addStateTransition(new StateTransition<GraspState>(GraspState.BASIC_OPEN, openThumbBasicGripCondition));
      stateClosedThumbBasicGrip.addStateTransition(new StateTransition<GraspState>(GraspState.BASIC_CLOSED, closedBasicGripCondition));
      stateClosedThumbBasicGrip.addStateTransition(new StateTransition<GraspState>(GraspState.BASIC_CLOSED, closedFingersBasicGripCondition));
      stateClosedThumbBasicGrip.addStateTransition(new StateTransition<GraspState>(GraspState.PINCH_OPEN, openPinchGripCondition));
      stateClosedThumbBasicGrip.addStateTransition(new StateTransition<GraspState>(GraspState.WIDE_OPEN, openWideGripCondition));
      stateClosedThumbBasicGrip.addStateTransition(new StateTransition<GraspState>(GraspState.HOOK, hookGripCondition));
      
      //PINCH_OPEN
      stateOpenPinchGrip.addStateTransition(new StateTransition<GraspState>(GraspState.PINCH_CLOSED, closedPinchGripCondition));
      stateOpenPinchGrip.addStateTransition(new StateTransition<GraspState>(GraspState.PINCH_ONLY_THUMB_CLOSED, closedThumbPinchGripCondition));
      stateOpenPinchGrip.addStateTransition(new StateTransition<GraspState>(GraspState.PINCH_ONLY_THUMB_OPEN, closedFingersPinchGripCondition));
      stateOpenPinchGrip.addStateTransition(new StateTransition<GraspState>(GraspState.BASIC_OPEN, openBasicGripCondition));
      stateOpenPinchGrip.addStateTransition(new StateTransition<GraspState>(GraspState.WIDE_OPEN, openWideGripCondition));
      stateOpenPinchGrip.addStateTransition(new StateTransition<GraspState>(GraspState.HOOK, hookGripCondition));
      
      //PINCH_ONLY_THUMB_OPEN
      stateOpenThumbPinchGrip.addStateTransition(new StateTransition<GraspState>(GraspState.PINCH_OPEN, openPinchGripCondition));
      stateOpenThumbPinchGrip.addStateTransition(new StateTransition<GraspState>(GraspState.PINCH_OPEN, openFingersPinchGripCondition));
      stateOpenThumbPinchGrip.addStateTransition(new StateTransition<GraspState>(GraspState.PINCH_CLOSED, closedPinchGripCondition));
      stateOpenThumbPinchGrip.addStateTransition(new StateTransition<GraspState>(GraspState.PINCH_CLOSED, closedThumbPinchGripCondition));
      stateOpenThumbPinchGrip.addStateTransition(new StateTransition<GraspState>(GraspState.BASIC_OPEN, openBasicGripCondition));
      stateOpenThumbPinchGrip.addStateTransition(new StateTransition<GraspState>(GraspState.WIDE_OPEN, openWideGripCondition));
      
      //PINCH_CLOSED
      stateClosedPinchGrip.addStateTransition(new StateTransition<GraspState>(GraspState.PINCH_OPEN, openPinchGripCondition));
      stateClosedPinchGrip.addStateTransition(new StateTransition<GraspState>(GraspState.PINCH_ONLY_THUMB_OPEN, openThumbPinchGripCondition));
      stateClosedPinchGrip.addStateTransition(new StateTransition<GraspState>(GraspState.PINCH_ONLY_THUMB_CLOSED, openFingersPinchGripCondition));
      stateClosedPinchGrip.addStateTransition(new StateTransition<GraspState>(GraspState.BASIC_OPEN, openBasicGripCondition));
      stateClosedPinchGrip.addStateTransition(new StateTransition<GraspState>(GraspState.WIDE_OPEN, openWideGripCondition));
      
      //PINCH_ONLY_THUMB_CLOSED
      stateClosedThumbPinchGrip.addStateTransition(new StateTransition<GraspState>(GraspState.PINCH_OPEN, openPinchGripCondition));
      stateClosedThumbPinchGrip.addStateTransition(new StateTransition<GraspState>(GraspState.PINCH_OPEN, openThumbPinchGripCondition));
      stateClosedThumbPinchGrip.addStateTransition(new StateTransition<GraspState>(GraspState.PINCH_CLOSED, closedPinchGripCondition));
      stateClosedThumbPinchGrip.addStateTransition(new StateTransition<GraspState>(GraspState.PINCH_CLOSED, closedFingersPinchGripCondition));
      stateClosedThumbPinchGrip.addStateTransition(new StateTransition<GraspState>(GraspState.BASIC_OPEN, openBasicGripCondition));
      stateClosedThumbPinchGrip.addStateTransition(new StateTransition<GraspState>(GraspState.WIDE_OPEN, openWideGripCondition));
      
      //WIDE_OPEN
      stateOpenWideGrip.addStateTransition(new StateTransition<GraspState>(GraspState.WIDE_CLOSED, closedWideGripCondition));
      stateOpenWideGrip.addStateTransition(new StateTransition<GraspState>(GraspState.WIDE_ONLY_THUMB_CLOSED, closedThumbWideGripCondition));
      stateOpenWideGrip.addStateTransition(new StateTransition<GraspState>(GraspState.WIDE_ONLY_THUMB_OPEN, closedFingersWideGripCondition));
      stateOpenWideGrip.addStateTransition(new StateTransition<GraspState>(GraspState.BASIC_OPEN, openBasicGripCondition));
      stateOpenWideGrip.addStateTransition(new StateTransition<GraspState>(GraspState.PINCH_OPEN, openPinchGripCondition));
      stateOpenWideGrip.addStateTransition(new StateTransition<GraspState>(GraspState.HOOK, hookGripCondition));
      
      //WIDE_ONLY_THUMB_OPEN
      stateOpenThumbWideGrip.addStateTransition(new StateTransition<GraspState>(GraspState.WIDE_OPEN, openWideGripCondition));
      stateOpenThumbWideGrip.addStateTransition(new StateTransition<GraspState>(GraspState.WIDE_OPEN, openFingersWideGripCondition));
      stateOpenThumbWideGrip.addStateTransition(new StateTransition<GraspState>(GraspState.WIDE_CLOSED, closedWideGripCondition));
      stateOpenThumbWideGrip.addStateTransition(new StateTransition<GraspState>(GraspState.WIDE_CLOSED, closedThumbWideGripCondition));
      stateOpenThumbWideGrip.addStateTransition(new StateTransition<GraspState>(GraspState.BASIC_OPEN, openBasicGripCondition));
      stateOpenThumbWideGrip.addStateTransition(new StateTransition<GraspState>(GraspState.PINCH_OPEN, openPinchGripCondition));
      
      //WIDE_CLOSED
      stateClosedWideGrip.addStateTransition(new StateTransition<GraspState>(GraspState.WIDE_OPEN, openWideGripCondition));
      stateClosedWideGrip.addStateTransition(new StateTransition<GraspState>(GraspState.WIDE_ONLY_THUMB_OPEN, openThumbWideGripCondition));
      stateClosedWideGrip.addStateTransition(new StateTransition<GraspState>(GraspState.WIDE_ONLY_THUMB_CLOSED, openFingersWideGripCondition));
      stateClosedWideGrip.addStateTransition(new StateTransition<GraspState>(GraspState.BASIC_OPEN, openBasicGripCondition));
      stateClosedWideGrip.addStateTransition(new StateTransition<GraspState>(GraspState.PINCH_OPEN, openPinchGripCondition));
      
      //WIDE_ONLY_THUMB_CLOSED
      stateClosedThumbWideGrip.addStateTransition(new StateTransition<GraspState>(GraspState.WIDE_OPEN, openWideGripCondition));
      stateClosedThumbWideGrip.addStateTransition(new StateTransition<GraspState>(GraspState.WIDE_OPEN, openThumbWideGripCondition));
      stateClosedThumbWideGrip.addStateTransition(new StateTransition<GraspState>(GraspState.WIDE_CLOSED, closedWideGripCondition));
      stateClosedThumbWideGrip.addStateTransition(new StateTransition<GraspState>(GraspState.WIDE_CLOSED, closedFingersWideGripCondition));
      stateClosedThumbWideGrip.addStateTransition(new StateTransition<GraspState>(GraspState.BASIC_OPEN, openBasicGripCondition));
      stateClosedThumbWideGrip.addStateTransition(new StateTransition<GraspState>(GraspState.PINCH_OPEN, openPinchGripCondition));
      
      //HOOK
      stateHookGrip.addStateTransition(new StateTransition<GraspState>(GraspState.BASIC_OPEN, openBasicGripCondition));
      stateHookGrip.addStateTransition(new StateTransition<GraspState>(GraspState.PINCH_OPEN, openPinchGripCondition));
      stateHookGrip.addStateTransition(new StateTransition<GraspState>(GraspState.WIDE_OPEN, openWideGripCondition));
   }
   
   private class OpenBasicGrip extends State<GraspState>
   {
      public OpenBasicGrip()
      {
         super(GraspState.BASIC_OPEN);
      }

      @Override
      public void doAction()
      {
      }

      @Override
      public void doTransitionIntoAction()
      {
         isStopped.set(false);
         graspMode.set(RobotiqGraspMode.BASIC_MODE);
         handConfiguration.set(HandConfiguration.OPEN);
         computeAllFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getOpenBasicGripDesiredConfiguration(robotSide));
      }

      @Override
      public void doTransitionOutOfAction()
      {
      }
   }
   
   private class OpenThumbBasicGrip extends State<GraspState>
   {
      public OpenThumbBasicGrip()
      {
         super(GraspState.BASIC_ONLY_THUMB_OPEN);
      }

      @Override
      public void doAction()
      {
      }

      @Override
      public void doTransitionIntoAction()
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
      public void doTransitionOutOfAction()
      {
      }
   }
   
   private class ClosedBasicGrip extends State<GraspState>
   {
      public ClosedBasicGrip()
      {
         super(GraspState.BASIC_CLOSED);
      }

      @Override
      public void doAction()
      {
      }

      @Override
      public void doTransitionIntoAction()
      {
         isStopped.set(false);
         graspMode.set(RobotiqGraspMode.BASIC_MODE);
         handConfiguration.set(HandConfiguration.CLOSE);
         computeAllFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getClosedBasicGripDesiredConfiguration(robotSide));
      }

      @Override
      public void doTransitionOutOfAction()
      {
      }
   }
   
   private class ClosedThumbBasicGrip extends State<GraspState>
   {
      public ClosedThumbBasicGrip()
      {
         super(GraspState.BASIC_ONLY_THUMB_CLOSED);
      }

      @Override
      public void doAction()
      {
      }

      @Override
      public void doTransitionIntoAction()
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
      public void doTransitionOutOfAction()
      {
      }
   }
   
   private class OpenPinchGrip extends State<GraspState>
   {
      public OpenPinchGrip()
      {
         super(GraspState.PINCH_OPEN);
      }

      @Override
      public void doAction()
      {
      }

      @Override
      public void doTransitionIntoAction()
      {
         isStopped.set(false);
         graspMode.set(RobotiqGraspMode.PINCH_MODE);
         handConfiguration.set(HandConfiguration.OPEN);
         computeAllFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getOpenPinchGripDesiredConfiguration(robotSide));
      }

      @Override
      public void doTransitionOutOfAction()
      {
      }
   }
   
   private class OpenThumbPinchGrip extends State<GraspState>
   {
      public OpenThumbPinchGrip()
      {
         super(GraspState.PINCH_ONLY_THUMB_OPEN);
      }

      @Override
      public void doAction()
      {
      }

      @Override
      public void doTransitionIntoAction()
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
      public void doTransitionOutOfAction()
      {
      }
   }
   
   private class ClosedPinchGrip extends State<GraspState>
   {
      public ClosedPinchGrip()
      {
         super(GraspState.PINCH_CLOSED);
      }

      @Override
      public void doAction()
      {
      }

      @Override
      public void doTransitionIntoAction()
      {
         isStopped.set(false);
         graspMode.set(RobotiqGraspMode.PINCH_MODE);
         handConfiguration.set(HandConfiguration.CLOSE);
         computeAllFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getClosedPinchGripDesiredConfiguration(robotSide));
      }

      @Override
      public void doTransitionOutOfAction()
      {
      }
   }
   
   private class ClosedThumbPinchGrip extends State<GraspState>
   {
      public ClosedThumbPinchGrip()
      {
         super(GraspState.PINCH_ONLY_THUMB_CLOSED);
      }

      @Override
      public void doAction()
      {
      }

      @Override
      public void doTransitionIntoAction()
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
      public void doTransitionOutOfAction()
      {
      }
   }
   
   private class OpenWideGrip extends State<GraspState>
   {
      public OpenWideGrip()
      {
         super(GraspState.WIDE_OPEN);
      }

      @Override
      public void doAction()
      {
      }

      @Override
      public void doTransitionIntoAction()
      {
         isStopped.set(false);
         graspMode.set(RobotiqGraspMode.WIDE_MODE);
         handConfiguration.set(HandConfiguration.OPEN);
         computeAllFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getOpenWideGripDesiredConfiguration(robotSide));
      }

      @Override
      public void doTransitionOutOfAction()
      {
      }
   }
   
   private class OpenThumbWideGrip extends State<GraspState>
   {
      public OpenThumbWideGrip()
      {
         super(GraspState.WIDE_ONLY_THUMB_OPEN);
      }

      @Override
      public void doAction()
      {
      }

      @Override
      public void doTransitionIntoAction()
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
      public void doTransitionOutOfAction()
      {
      }
   }
   
   private class ClosedWideGrip extends State<GraspState>
   {
      public ClosedWideGrip()
      {
         super(GraspState.WIDE_CLOSED);
      }

      @Override
      public void doAction()
      {
      }

      @Override
      public void doTransitionIntoAction()
      {
         isStopped.set(false);
         graspMode.set(RobotiqGraspMode.WIDE_MODE);
         handConfiguration.set(HandConfiguration.CLOSE);
         computeAllFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getClosedWideGripDesiredConfiguration(robotSide));
      }

      @Override
      public void doTransitionOutOfAction()
      {
      }
   }
   
   private class ClosedThumbWideGrip extends State<GraspState>
   {
      public ClosedThumbWideGrip()
      {
         super(GraspState.WIDE_ONLY_THUMB_CLOSED);
      }

      @Override
      public void doAction()
      {
      }

      @Override
      public void doTransitionIntoAction()
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
      public void doTransitionOutOfAction()
      {
      }
   }
   
   private class HookGrip extends State<GraspState>
   {
      public HookGrip()
      {
         super(GraspState.HOOK);
      }

      @Override
      public void doAction()
      {
         
      }

      @Override
      public void doTransitionIntoAction()
      {
         isStopped.set(false);
         graspMode.set(RobotiqGraspMode.BASIC_MODE);
         handConfiguration.set(HandConfiguration.HOOK);
         computeIndexFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getOpenBasicGripDesiredConfiguration(robotSide));
         computeMiddleFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getClosedBasicGripDesiredConfiguration(robotSide));
         computeThumbFinalDesiredAngles(1.0, RobotiqHandsDesiredConfigurations.getClosedBasicGripDesiredConfiguration(robotSide));
      }

      @Override
      public void doTransitionOutOfAction()
      {
      }
   }
   
   @Override
   public void doControl()
   {
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
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
   public YoVariableRegistry getYoVariableRegistry()
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
