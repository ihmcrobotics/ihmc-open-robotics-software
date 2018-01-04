package us.ihmc.valkyrie.fingers;

import java.util.EnumMap;
import java.util.Map;

import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPIDGains;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.partNames.FingerName;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachine;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachineTools;
import us.ihmc.valkyrie.parameters.ValkyrieStateEstimatorParameters;
import us.ihmc.valkyrieRosControl.dataHolders.YoEffortJointHandleHolder;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class ValkyrieFingerSetController implements RobotController
{
   private static final double MIN_ACTUATOR_POSITION = 0.0;
   private static final double MAX_ACTUATOR_POSITION = 3.6;

   private final ValkyrieHandJointName[] controlledJoints = ValkyrieHandJointName.controllableJoints;

   enum GraspState
   {
      OPEN, CLOSE
   }

   public static final boolean DEBUG = false;

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry;

   private final RobotSide robotSide;

   private final YoPolynomial fingerPolynomial, thumbPolynomial;
   private final YoDouble trajectoryTime;
   private final YoDouble thumbCloseDelay;
   private final YoDouble fingerOpenDelay;

   private final Map<ValkyrieHandJointName, YoDouble> initialDesiredAngles = new EnumMap<>(ValkyrieHandJointName.class);
   private final Map<ValkyrieHandJointName, YoDouble> finalDesiredAngles = new EnumMap<>(ValkyrieHandJointName.class);
   private final Map<ValkyrieHandJointName, YoDouble> closedAngles = new EnumMap<>(ValkyrieHandJointName.class);
   private final Map<ValkyrieHandJointName, YoDouble> openedAngles = new EnumMap<>(ValkyrieHandJointName.class);
   private final Map<ValkyrieHandJointName, YoDouble> desiredAngles = new EnumMap<>(ValkyrieHandJointName.class);
   private final Map<ValkyrieHandJointName, YoDouble> desiredVelocities = new EnumMap<>(ValkyrieHandJointName.class);
   private final Map<ValkyrieHandJointName, PIDController> pidControllers = new EnumMap<>(ValkyrieHandJointName.class);

   private final EnumMap<ValkyrieHandJointName, YoEffortJointHandleHolder> jointHandles;

   private final StateMachine<GraspState> stateMachine;
   private final YoEnum<GraspState> requestedState;

   private final double controlDT;

   /**
    * FIXME: This is a hack. As of the 11/11/2017, Valkyrie only provides actuator space measurement and commands.
    * This us unnecessary for the right hand, but for the left hand, the actuator and joint motions are opposite.
    * Ideally, we should be using the transmission ratio used in {@link ValkyrieStateEstimatorParameters#configureSensorProcessing(us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing)},
    * but hopefully the API with the robot will change, making this unnecessary.
    */
   private final boolean flipErrorSign;

   public ValkyrieFingerSetController(RobotSide robotSide, YoDouble yoTime, double controlDT, YoPIDGains gains, YoDouble trajectoryTime,
                                      YoDouble thumbCloseDelay, YoDouble fingerOpenDelay,
                                      EnumMap<ValkyrieHandJointName, YoEffortJointHandleHolder> jointHandles, YoVariableRegistry parentRegistry)
   {
      this.robotSide = robotSide;
      this.controlDT = controlDT;
      this.trajectoryTime = trajectoryTime;
      this.thumbCloseDelay = thumbCloseDelay;
      this.fingerOpenDelay = fingerOpenDelay;
      this.jointHandles = jointHandles;

      flipErrorSign = robotSide == RobotSide.LEFT;

      String sidePrefix = robotSide.getCamelCaseName();
      registry = new YoVariableRegistry(sidePrefix + name);

      mapJointsAndVariables(gains);

      fingerPolynomial = new YoPolynomial(sidePrefix + "FingersPolynomial", 4, registry);
      thumbPolynomial = new YoPolynomial(sidePrefix + "ThumbPolynomial", 4, registry);

      stateMachine = new StateMachine<>(sidePrefix + "GripCurrent", "SwitchTime", GraspState.class, yoTime, registry);
      requestedState = new YoEnum<>(sidePrefix + "RequestedGrip", registry, GraspState.class, true);
      requestedState.set(null);
      setupStateMachine();

      parentRegistry.addChild(registry);
   }

   private void mapJointsAndVariables(YoPIDGains gains)
   {
      for (ValkyrieHandJointName jointEnum : controlledJoints)
      {
         String jointName = jointEnum.getJointName(robotSide);
         YoDouble initialDesiredAngle = new YoDouble(jointName + "InitialAngle", registry);
         initialDesiredAngles.put(jointEnum, initialDesiredAngle);

         YoDouble finalDesiredAngle = new YoDouble(jointName + "FinalAngle", registry);
         finalDesiredAngles.put(jointEnum, finalDesiredAngle);

         YoDouble desiredAngle = new YoDouble("q_d_" + jointName, registry);
         desiredAngle.setToNaN();
         desiredAngles.put(jointEnum, desiredAngle);

         YoDouble desiredVelocity = new YoDouble("qd_d_" + jointName, registry);
         desiredVelocities.put(jointEnum, desiredVelocity);

         PIDController pidController = new PIDController(gains, jointEnum.getPascalCaseJointName(robotSide), registry);
         pidControllers.put(jointEnum, pidController);

         YoDouble closedAngle = new YoDouble(jointName + "ClosedAngle", registry);
         closedAngle.set(ValkyrieFingerControlParameters.getClosedDesiredDefinition(robotSide).get(jointEnum));
         closedAngles.put(jointEnum, closedAngle);

         YoDouble openedAngle = new YoDouble(jointName + "OpenedAngle", registry);
         openedAngle.set(ValkyrieFingerControlParameters.getOpenDesiredDefinition(robotSide).get(jointEnum));
         openedAngles.put(jointEnum, openedAngle);
      }
   }

   private void setupStateMachine()
   {
      GripState stateOpenGrip = new OpenGrip();
      GripState stateClosedGrip = new ClosedGrip();

      stateMachine.addState(stateOpenGrip);
      stateMachine.addState(stateClosedGrip);
      stateMachine.setCurrentState(GraspState.OPEN);

      StateMachineTools.addRequestedStateTransition(requestedState, true, stateOpenGrip, stateClosedGrip);
      StateMachineTools.addRequestedStateTransition(requestedState, true, stateClosedGrip, stateOpenGrip);
   }

   public void requestState(GraspState requestedState)
   {
      this.requestedState.set(requestedState);
   }

   private class OpenGrip extends GripState
   {
      public OpenGrip()
      {
         super(GraspState.OPEN);
      }

      @Override
      public void doTransitionIntoAction()
      {
         fingerPolynomial.setCubic(0.0, trajectoryTime.getDoubleValue(), 0.0, 1.0);
         thumbPolynomial.setCubic(0.0, trajectoryTime.getDoubleValue(), 0.0, 1.0);

         for (ValkyrieHandJointName jointEnum : controlledJoints)
         {
            double qInitial;
            YoDouble desiredAngle = desiredAngles.get(jointEnum);
            if (desiredAngle.isNaN())
               qInitial = jointHandles.get(jointEnum).getOneDoFJoint().getQ();
            else
               qInitial = desiredAngle.getDoubleValue();
            initialDesiredAngles.get(jointEnum).set(qInitial);
            double qFinal = openedAngles.get(jointEnum).getDoubleValue();
            finalDesiredAngles.get(jointEnum).set(qFinal);
         }
      }

      @Override
      protected double getFingerDelay()
      {
         return fingerOpenDelay.getDoubleValue();
      }

      @Override
      protected double getThumbDelay()
      {
         return 0.0;
      }
   }

   private class ClosedGrip extends GripState
   {
      public ClosedGrip()
      {
         super(GraspState.CLOSE);
      }

      @Override
      public void doTransitionIntoAction()
      {
         fingerPolynomial.setCubic(0.0, trajectoryTime.getDoubleValue(), 0.0, 1.0);
         thumbPolynomial.setCubic(0.0, trajectoryTime.getDoubleValue(), 0.0, 1.0);

         for (ValkyrieHandJointName jointEnum : controlledJoints)
         {
            double qInitial;
            YoDouble desiredAngle = desiredAngles.get(jointEnum);
            if (desiredAngle.isNaN())
               qInitial = jointHandles.get(jointEnum).getOneDoFJoint().getQ();
            else
               qInitial = desiredAngle.getDoubleValue();
            initialDesiredAngles.get(jointEnum).set(qInitial);
            double qFinal = closedAngles.get(jointEnum).getDoubleValue();
            finalDesiredAngles.get(jointEnum).set(qFinal);
         }
      }

      @Override
      protected double getFingerDelay()
      {
         return 0.0;
      }

      @Override
      protected double getThumbDelay()
      {
         return thumbCloseDelay.getDoubleValue();
      }
   }

   public abstract class GripState extends FinishableState<GraspState>
   {

      public GripState(GraspState stateEnum)
      {
         super(stateEnum);
      }

      @Override
      public void doAction()
      {
         double fingerTime = getTimeInCurrentState() - getFingerDelay();
         double fingerAlpha, fingerAlphaDot;

         if (fingerTime <= 0.0)
         {
            fingerAlpha = 0.0;
            fingerAlphaDot = 0.0;
         }
         else if (fingerTime >= trajectoryTime.getDoubleValue())
         {
            fingerAlpha = 1.0;
            fingerAlphaDot = 0.0;
         }
         else
         {
            fingerPolynomial.compute(fingerTime);
            fingerAlpha = fingerPolynomial.getPosition();
            fingerAlphaDot = fingerPolynomial.getVelocity();
         }

         double thumbTime = getTimeInCurrentState() - getThumbDelay();
         double thumbAlpha, thumbAlphaDot;
         
         if (thumbTime <= 0.0)
         {
            thumbAlpha = 0.0;
            thumbAlphaDot = 0.0;
         }
         else if (thumbTime >= trajectoryTime.getDoubleValue())
         {
            thumbAlpha = 1.0;
            thumbAlphaDot = 0.0;
         }
         else
         {
            thumbPolynomial.compute(thumbTime);
            thumbAlpha = thumbPolynomial.getPosition();
            thumbAlphaDot = thumbPolynomial.getVelocity();
         }

         for (ValkyrieHandJointName jointEnum : controlledJoints)
         {
            if (jointEnum.getFingerName() == FingerName.THUMB)
            {
               double q0 = initialDesiredAngles.get(jointEnum).getDoubleValue();
               double qf = finalDesiredAngles.get(jointEnum).getDoubleValue();
               desiredAngles.get(jointEnum).set(TupleTools.interpolate(q0, qf, thumbAlpha));
               desiredVelocities.get(jointEnum).set(thumbAlphaDot * (qf - q0));
            }
            else
            {
               double q0 = initialDesiredAngles.get(jointEnum).getDoubleValue();
               double qf = finalDesiredAngles.get(jointEnum).getDoubleValue();
               desiredAngles.get(jointEnum).set(TupleTools.interpolate(q0, qf, fingerAlpha));
               desiredVelocities.get(jointEnum).set(fingerAlphaDot * (qf - q0));
            }
         }
      }

      protected abstract double getFingerDelay();

      protected abstract double getThumbDelay();

      @Override
      public boolean isDone()
      {
         return getTimeInCurrentState() >= trajectoryTime.getDoubleValue();
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

      for (ValkyrieHandJointName jointEnum : controlledJoints)
      {
         PIDController pidController = pidControllers.get(jointEnum);
         YoEffortJointHandleHolder handle = jointHandles.get(jointEnum);
         OneDoFJoint joint = handle.getOneDoFJoint();

         double q = joint.getQ();
         double q_d = desiredAngles.get(jointEnum).getDoubleValue();
         double qd = joint.getQd();
         double qd_d = desiredVelocities.get(jointEnum).getDoubleValue();

         if (flipErrorSign)
         {
            double q_err = -(q_d - q);
            q_d = q + q_err;
            qd_d = -qd_d;
         }

         double tau = pidController.compute(q, q_d, qd, qd_d, controlDT);

         if (handle.getQ() <= MIN_ACTUATOR_POSITION && tau < 0.0)
         {
            tau = 0.0;
            pidController.resetIntegrator();
         }
         else if (handle.getQ() >= MAX_ACTUATOR_POSITION && tau > 0.0)
         {
            tau = 0.0;
            pidController.resetIntegrator();
         }
         handle.setDesiredEffort(tau);
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
