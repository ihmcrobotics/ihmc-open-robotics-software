package us.ihmc.valkyrie.fingers;

import java.util.EnumMap;
import java.util.LinkedHashMap;

import us.ihmc.SdfLoader.SDFBaseFullRobotModel;
import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.math.trajectories.YoPolynomial;
import us.ihmc.yoUtilities.stateMachines.State;
import us.ihmc.yoUtilities.stateMachines.StateMachine;
import us.ihmc.yoUtilities.stateMachines.StateTransition;
import us.ihmc.yoUtilities.stateMachines.StateTransitionCondition;

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
   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable startTrajectoryTime, currentTrajectoryTime, endTrajectoryTime, trajectoryTime;
   private final BooleanYoVariable hasTrajectoryTimeChanged, isStopped;
   
   private final LinkedHashMap<ValkyrieRealRobotFingerJoint, DoubleYoVariable> initialDesiredAngles = new LinkedHashMap<>();
   private final LinkedHashMap<ValkyrieRealRobotFingerJoint, DoubleYoVariable> finalDesiredAngles = new LinkedHashMap<>();
   private final LinkedHashMap<ValkyrieRealRobotFingerJoint, DoubleYoVariable> desiredAngles = new LinkedHashMap<>();

   private final EnumMap<ValkyrieRealRobotFingerJoint, DoubleYoVariable> realRobotControlVariables = new EnumMap<>(ValkyrieRealRobotFingerJoint.class);
   private final EnumMap<ValkyrieSimulatedFingerJoint, RevoluteJoint> revoluteJointMap = new EnumMap<>(ValkyrieSimulatedFingerJoint.class);
   
   private final EnumYoVariable<FingerState> fingerState;
   private final EnumYoVariable<FingerState> desiredFingerState;
   private StateMachine<GraspState> stateMachine;
         
   public ValkyrieFingerSetController(RobotSide robotSide, DoubleYoVariable yoTime, DoubleYoVariable trajectoryTime, SDFBaseFullRobotModel fullRobotModel, boolean runningOnRealRobot, YoVariableRegistry parentRegistry,  YoVariableRegistry controllerRegistry)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      this.controllerRegistry = controllerRegistry;
      registry = new YoVariableRegistry(sidePrefix + name);
      parentRegistry.addChild(registry);
      this.robotSide = robotSide;
      this.yoTime = yoTime;
      this.runningOnRealRobot = runningOnRealRobot;
      
      mapJointsAndVariables(fullRobotModel);
      
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

      fingerState = new EnumYoVariable<>(sidePrefix + "ValkyrieFingerState", registry, FingerState.class);
      fingerState.set(FingerState.OPEN);
      desiredFingerState = new EnumYoVariable<>(sidePrefix + "ValkyrieDesiredFingerState", registry, FingerState.class);
      desiredFingerState.set(FingerState.OPEN);
      
      stateMachine = new StateMachine<>(sidePrefix + "ValkyrieGraspStateMachine", "FingerTrajectoryTime", GraspState.class, yoTime, registry);
      setupStateMachine();
   }

   private void mapJointsAndVariables(SDFBaseFullRobotModel fullRobotModel)
   {
      for (ValkyrieRealRobotFingerJoint jointEnum : ValkyrieRealRobotFingerJoint.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         DoubleYoVariable initialDesiredAngle = new DoubleYoVariable("q_d_initial_" + sidePrefix + jointEnum, registry);
         initialDesiredAngles.put(jointEnum, initialDesiredAngle);
         
         DoubleYoVariable finalDesiredAngle = new DoubleYoVariable("q_d_final_" + sidePrefix + jointEnum, registry);
         finalDesiredAngles.put(jointEnum, finalDesiredAngle);

         DoubleYoVariable desiredAngle = new DoubleYoVariable("q_d_" + sidePrefix + jointEnum, registry);
         desiredAngles.put(jointEnum, desiredAngle);
      }
      
      for (ValkyrieSimulatedFingerJoint simulatedFingerJoint : ValkyrieSimulatedFingerJoint.values)
      {
         revoluteJointMap.put(simulatedFingerJoint, simulatedFingerJoint.getRelatedRevoluteJoint(robotSide, fullRobotModel));
      }
      
      if (runningOnRealRobot)
      {
         for (ValkyrieRealRobotFingerJoint realRobotFingerJointEnum : ValkyrieRealRobotFingerJoint.values)
         {
            realRobotControlVariables.put(realRobotFingerJointEnum, realRobotFingerJointEnum.getRelatedControlVariable(robotSide, controllerRegistry));
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
            return desiredFingerState.getEnumValue().equals(FingerState.OPEN);
         }
      };
      
      StateTransitionCondition closedGripCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return desiredFingerState.getEnumValue().equals(FingerState.CLOSE);
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
         fingerState.set(FingerState.OPEN);
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
         fingerState.set(FingerState.CLOSE);
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
      desiredFingerState.set(FingerState.OPEN);
   }
   
   public void close()
   {
      desiredFingerState.set(FingerState.CLOSE);
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
      
      for (ValkyrieRealRobotFingerJoint controllableJoint : ValkyrieRealRobotFingerJoint.values)
      {
         finalDesiredAngles.get(controllableJoint).set(0.0);
      }
   }
   
   private void computeAllFinalDesiredAngles(double percent, EnumMap<ValkyrieRealRobotFingerJoint, Double> desiredDefinition)
   {
      for (ValkyrieRealRobotFingerJoint controllableJoint : ValkyrieRealRobotFingerJoint.values)
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
         for (ValkyrieRealRobotFingerJoint controllableJoint : ValkyrieRealRobotFingerJoint.values)
         {
            realRobotControlVariables.get(controllableJoint).set(desiredAngles.get(controllableJoint).getDoubleValue());
         }
      }
      
      for (ValkyrieRealRobotFingerJoint controllableJoint : ValkyrieRealRobotFingerJoint.values)
      {
         double desiredValue = desiredAngles.get(controllableJoint).getDoubleValue();

//         double alpha = MathTools.clipToMinMax(yoPolynomial.getPosition(), 0.0, 1.0);
//         if (alpha > 0.0 && alpha < 1.0)
//            PrintTools.debug(DEBUG, this, controllableJoint.name() + "Desired q : " + desiredValue);
         
         switch (controllableJoint)
         {
         case ThumbRoll:
            revoluteJointMap.get(ValkyrieSimulatedFingerJoint.ThumbRoll).setqDesired(desiredValue);
            break;
         case Thumb:
            revoluteJointMap.get(ValkyrieSimulatedFingerJoint.ThumbPitch1).setqDesired(desiredValue / 3.0);
            revoluteJointMap.get(ValkyrieSimulatedFingerJoint.ThumbPitch2).setqDesired(desiredValue / 3.0);
            revoluteJointMap.get(ValkyrieSimulatedFingerJoint.ThumbPitch3).setqDesired(desiredValue / 3.0);
            break;
         case Index:
            revoluteJointMap.get(ValkyrieSimulatedFingerJoint.IndexFingerPitch1).setqDesired(desiredValue / 3.0);
            revoluteJointMap.get(ValkyrieSimulatedFingerJoint.IndexFingerPitch2).setqDesired(desiredValue / 3.0);
            revoluteJointMap.get(ValkyrieSimulatedFingerJoint.IndexFingerPitch3).setqDesired(desiredValue / 3.0);
            break;
         case Middle:
            revoluteJointMap.get(ValkyrieSimulatedFingerJoint.MiddleFingerPitch1).setqDesired(desiredValue / 3.0);
            revoluteJointMap.get(ValkyrieSimulatedFingerJoint.MiddleFingerPitch2).setqDesired(desiredValue / 3.0);
            revoluteJointMap.get(ValkyrieSimulatedFingerJoint.MiddleFingerPitch3).setqDesired(desiredValue / 3.0);
            break;
         case Pinky:
            revoluteJointMap.get(ValkyrieSimulatedFingerJoint.PinkyPitch1).setqDesired(desiredValue / 3.0);
            revoluteJointMap.get(ValkyrieSimulatedFingerJoint.PinkyPitch2).setqDesired(desiredValue / 3.0);
            revoluteJointMap.get(ValkyrieSimulatedFingerJoint.PinkyPitch3).setqDesired(desiredValue / 3.0);
            break;
         }
      }
   }

   private void initializeTrajectory()
   {
      for (ValkyrieRealRobotFingerJoint controllableJoint : ValkyrieRealRobotFingerJoint.values)
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
         currentTrajectoryTime.set(MathTools.clipToMinMax(currentTrajectoryTime.getDoubleValue(), 0.0, trajectoryTime.getDoubleValue()));
      }
      
      yoPolynomial.compute(currentTrajectoryTime.getDoubleValue());
      double alpha = MathTools.clipToMinMax(yoPolynomial.getPosition(), 0.0, 1.0);

      for (ValkyrieRealRobotFingerJoint controllableJoint : ValkyrieRealRobotFingerJoint.values)
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
