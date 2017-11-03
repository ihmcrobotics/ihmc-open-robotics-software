package us.ihmc.valkyrie.fingers;

import java.util.EnumMap;
import java.util.LinkedHashMap;

import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachine;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransition;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.LowLevelOneDoFJointDesiredDataHolderList;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

public class ValkyrieFingerSetController implements RobotController
{
   enum GraspState
   {
      OPEN, CLOSED
   }

   public static final boolean DEBUG = false;

   private final boolean runningOnRealRobot;

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry controllerRegistry;
   private final YoVariableRegistry registry;

   private final RobotSide robotSide;

   private final YoPolynomial yoPolynomial;
   private final YoDouble yoTime;
   private final YoDouble startTrajectoryTime, currentTrajectoryTime, endTrajectoryTime, trajectoryTime;
   private final YoBoolean hasTrajectoryTimeChanged, isStopped;

   private final LinkedHashMap<ValkyrieHandJointName, YoDouble> initialDesiredAngles = new LinkedHashMap<>();
   private final LinkedHashMap<ValkyrieHandJointName, YoDouble> finalDesiredAngles = new LinkedHashMap<>();
   private final LinkedHashMap<ValkyrieHandJointName, YoDouble> desiredAngles = new LinkedHashMap<>();

   private final EnumMap<ValkyrieHandJointName, YoDouble> realRobotControlVariables = new EnumMap<>(ValkyrieHandJointName.class);
   private final EnumMap<ValkyrieHandJointName, RevoluteJoint> revoluteJointMap = new EnumMap<>(ValkyrieHandJointName.class);
   private final EnumMap<ValkyrieHandJointName, JointDesiredOutput> desiredOutputMap = new EnumMap<>(ValkyrieHandJointName.class);

   private final YoEnum<HandConfiguration> handConfiguration;
   private final YoEnum<HandConfiguration> handDesiredConfiguration;
   private StateMachine<GraspState> stateMachine;

   public ValkyrieFingerSetController(RobotSide robotSide, YoDouble yoTime, YoDouble trajectoryTime, FullRobotModel fullRobotModel, LowLevelOneDoFJointDesiredDataHolderList lowLevelDesiredJointData, boolean runningOnRealRobot, YoVariableRegistry parentRegistry,  YoVariableRegistry controllerRegistry)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      this.controllerRegistry = controllerRegistry;
      registry = new YoVariableRegistry(sidePrefix + name);
      parentRegistry.addChild(registry);
      this.robotSide = robotSide;
      this.yoTime = yoTime;
      this.runningOnRealRobot = runningOnRealRobot;

      mapJointsAndVariables(fullRobotModel, lowLevelDesiredJointData);

      startTrajectoryTime = new YoDouble(sidePrefix + "StartTrajectoryTime", registry);
      currentTrajectoryTime = new YoDouble(sidePrefix + "CurrentTrajectoryTime", registry);
      endTrajectoryTime = new YoDouble(sidePrefix + "EndTrajectoryTime", registry);
      this.trajectoryTime = trajectoryTime;
      hasTrajectoryTimeChanged = new YoBoolean(sidePrefix + "HasTrajectoryTimeChanged", registry);
      isStopped = new YoBoolean(sidePrefix + "IsStopped", registry);
      isStopped.set(false);
      trajectoryTime.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            hasTrajectoryTimeChanged.set(true);
         }
      });
      yoPolynomial = new YoPolynomial(sidePrefix + name, 4, registry);
      yoPolynomial.setCubic(0.0, trajectoryTime.getDoubleValue(), 0.0, 0.0, 1.0, 0.0);

      handConfiguration = new YoEnum<>(sidePrefix + "ValkyrieHandConfiguration", registry, HandConfiguration.class);
      handConfiguration.set(HandConfiguration.OPEN);
      handDesiredConfiguration = new YoEnum<>(sidePrefix + "ValkyrieHandDesiredConfiguration", registry, HandConfiguration.class);
      handDesiredConfiguration.set(HandConfiguration.OPEN);

      stateMachine = new StateMachine<>(sidePrefix + "ValkyrieGraspStateMachine", "FingerTrajectoryTime", GraspState.class, yoTime, registry);
      setupStateMachine();
   }

   private void mapJointsAndVariables(FullRobotModel fullRobotModel, LowLevelOneDoFJointDesiredDataHolderList lowLevelDesiredJointData)
   {
      for (ValkyrieHandJointName jointEnum : ValkyrieHandJointName.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         YoDouble initialDesiredAngle = new YoDouble("q_d_initial_" + sidePrefix + jointEnum, registry);
         initialDesiredAngles.put(jointEnum, initialDesiredAngle);

         YoDouble finalDesiredAngle = new YoDouble("q_d_final_" + sidePrefix + jointEnum, registry);
         finalDesiredAngles.put(jointEnum, finalDesiredAngle);

         YoDouble desiredAngle = new YoDouble("q_d_" + sidePrefix + jointEnum, registry);
         desiredAngles.put(jointEnum, desiredAngle);
      }

      for (ValkyrieHandJointName simulatedFingerJoint : ValkyrieHandJointName.values)
      {
         revoluteJointMap.put(simulatedFingerJoint, simulatedFingerJoint.getRelatedRevoluteJoint(robotSide, fullRobotModel));
         desiredOutputMap.put(simulatedFingerJoint, lowLevelDesiredJointData.getJointDesiredOutput(revoluteJointMap.get(simulatedFingerJoint)));
         
      }

      if (runningOnRealRobot)
      {
         for (ValkyrieHandJointName realRobotFingerJointEnum : ValkyrieHandJointName.values)
         {
//            realRobotControlVariables.put(realRobotFingerJointEnum, realRobotFingerJointEnum.getRelatedControlVariable(robotSide, controllerRegistry));
         }
      }
   }

   private void setupStateMachine()
   {
      State<GraspState> stateOpenGrip = new OpenGrip();
      State<GraspState> stateClosedGrip = new ClosedGrip();

      stateMachine.addState(stateOpenGrip);
      stateMachine.addState(stateClosedGrip);
      stateMachine.setCurrentState(GraspState.OPEN);

      StateTransitionCondition openGripCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return handDesiredConfiguration.getEnumValue().equals(HandConfiguration.OPEN);
         }
      };

      StateTransitionCondition closedGripCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return handDesiredConfiguration.getEnumValue().equals(HandConfiguration.CLOSE);
         }
      };

      // OPEN
      stateOpenGrip.addStateTransition(new StateTransition<GraspState>(GraspState.CLOSED, closedGripCondition));

      // CLOSED
      stateClosedGrip.addStateTransition(new StateTransition<GraspState>(GraspState.OPEN, openGripCondition));
   }

   private class OpenGrip extends State<GraspState>
   {
      public OpenGrip()
      {
         super(GraspState.OPEN);
      }

      @Override
      public void doAction()
      {
      }

      @Override
      public void doTransitionIntoAction()
      {
         isStopped.set(false);
         handConfiguration.set(HandConfiguration.OPEN);
         computeAllFinalDesiredAngles(1.0, ValkyrieFingerPoseDefinitions.getOpenDesiredDefinition(robotSide));
      }

      @Override
      public void doTransitionOutOfAction()
      {
      }
   }

   private class ClosedGrip extends State<GraspState>
   {
      public ClosedGrip()
      {
         super(GraspState.CLOSED);
      }

      @Override
      public void doAction()
      {
      }

      @Override
      public void doTransitionIntoAction()
      {
         isStopped.set(false);
         handConfiguration.set(HandConfiguration.CLOSE);
         computeAllFinalDesiredAngles(1.0, ValkyrieFingerPoseDefinitions.getClosedDesiredDefinition(robotSide));
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

   public void close()
   {
      handDesiredConfiguration.set(HandConfiguration.CLOSE);
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

      for (ValkyrieHandJointName controllableJoint : ValkyrieHandJointName.values)
      {
         finalDesiredAngles.get(controllableJoint).set(0.0);
      }
   }

   private void computeAllFinalDesiredAngles(double percent, EnumMap<ValkyrieHandJointName, Double> desiredDefinition)
   {
      for (ValkyrieHandJointName controllableJoint : ValkyrieHandJointName.values)
      {
         double qDesired = percent * desiredDefinition.get(controllableJoint);
         finalDesiredAngles.get(controllableJoint).set(qDesired);
      }

      initializeTrajectory();
   }

   /**
    * Only place where the actual values should be modified.
    */
   public void writeDesiredJointAngles()
   {
      if (runningOnRealRobot)
      {
         for (ValkyrieHandJointName controllableJoint : ValkyrieHandJointName.values)
         {
            realRobotControlVariables.get(controllableJoint).set(desiredAngles.get(controllableJoint).getDoubleValue());
         }
      }

      for (ValkyrieHandJointName controllableJoint : ValkyrieHandJointName.values)
      {
         double desiredValue = desiredAngles.get(controllableJoint).getDoubleValue();

//         double alpha = MathTools.clipToMinMax(yoPolynomial.getPosition(), 0.0, 1.0);
//         if (alpha > 0.0 && alpha < 1.0)
//            PrintTools.debug(DEBUG, this, controllableJoint.name() + "Desired q : " + desiredValue);

         switch (controllableJoint)
         {
         case ThumbRoll:
            desiredOutputMap.get(ValkyrieHandJointName.ThumbRoll).setDesiredPosition(desiredValue);
            break;
         case ThumbPitch1:
            desiredOutputMap.get(ValkyrieHandJointName.ThumbPitch1).setDesiredPosition(desiredValue / 3.0);
            desiredOutputMap.get(ValkyrieHandJointName.ThumbPitch2).setDesiredPosition(desiredValue / 3.0);
            desiredOutputMap.get(ValkyrieHandJointName.ThumbPitch3).setDesiredPosition(desiredValue / 3.0);
            break;
         case IndexFingerPitch1:
            desiredOutputMap.get(ValkyrieHandJointName.IndexFingerPitch1).setDesiredPosition(desiredValue / 3.0);
            desiredOutputMap.get(ValkyrieHandJointName.IndexFingerPitch2).setDesiredPosition(desiredValue / 3.0);
            desiredOutputMap.get(ValkyrieHandJointName.IndexFingerPitch3).setDesiredPosition(desiredValue / 3.0);
            break;
         case MiddleFingerPitch1:
            desiredOutputMap.get(ValkyrieHandJointName.MiddleFingerPitch1).setDesiredPosition(desiredValue / 3.0);
            desiredOutputMap.get(ValkyrieHandJointName.MiddleFingerPitch2).setDesiredPosition(desiredValue / 3.0);
            desiredOutputMap.get(ValkyrieHandJointName.MiddleFingerPitch3).setDesiredPosition(desiredValue / 3.0);
            break;
         case PinkyPitch1:
            desiredOutputMap.get(ValkyrieHandJointName.PinkyPitch1).setDesiredPosition(desiredValue / 3.0);
            desiredOutputMap.get(ValkyrieHandJointName.PinkyPitch2).setDesiredPosition(desiredValue / 3.0);
            desiredOutputMap.get(ValkyrieHandJointName.PinkyPitch3).setDesiredPosition(desiredValue / 3.0);
            break;
         }
      }
   }

   private void initializeTrajectory()
   {
      for (ValkyrieHandJointName controllableJoint : ValkyrieHandJointName.values)
      {
         initialDesiredAngles.get(controllableJoint).set(desiredAngles.get(controllableJoint).getDoubleValue());
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

      for (ValkyrieHandJointName controllableJoint : ValkyrieHandJointName.values)
      {
         double q_d_initial = initialDesiredAngles.get(controllableJoint).getDoubleValue();
         double q_d_final = finalDesiredAngles.get(controllableJoint).getDoubleValue();
         double q_d = (1.0 - alpha) * q_d_initial + alpha * q_d_final;
         desiredAngles.get(controllableJoint).set(q_d);
      }
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return robotSide.getCamelCaseNameForStartOfExpression() + getClass().getSimpleName();
   }

   @Override
   public String getDescription()
   {
      return "Controller for " + robotSide.getLowerCaseName() + " Valkyrie fingers in both simulation and real robot environments.";
   }

   @Override
   public void initialize()
   {
      // Do nothing
   }
}
