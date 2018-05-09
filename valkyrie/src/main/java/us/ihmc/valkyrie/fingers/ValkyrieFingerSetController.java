package us.ihmc.valkyrie.fingers;

import java.util.EnumMap;
import java.util.Map;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPIDGains;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.partNames.FingerName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlFingerStateEstimator;
import us.ihmc.valkyrieRosControl.dataHolders.YoEffortJointHandleHolder;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class ValkyrieFingerSetController implements RobotController
{
   private static final double MIN_ACTUATOR_POSITION = 0.0;
   private static final double MAX_ACTUATOR_POSITION = 3.6;

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

   private final Map<ValkyrieFingerMotorName, YoDouble> initialDesiredAngles = new EnumMap<>(ValkyrieFingerMotorName.class);
   private final Map<ValkyrieFingerMotorName, YoDouble> finalDesiredAngles = new EnumMap<>(ValkyrieFingerMotorName.class);
   private final Map<ValkyrieFingerMotorName, YoDouble> closedAngles = new EnumMap<>(ValkyrieFingerMotorName.class);
   private final Map<ValkyrieFingerMotorName, YoDouble> openedAngles = new EnumMap<>(ValkyrieFingerMotorName.class);
   private final Map<ValkyrieFingerMotorName, YoDouble> desiredAngles = new EnumMap<>(ValkyrieFingerMotorName.class);
   private final Map<ValkyrieFingerMotorName, YoDouble> desiredVelocities = new EnumMap<>(ValkyrieFingerMotorName.class);
   private final Map<ValkyrieFingerMotorName, PIDController> pidControllers = new EnumMap<>(ValkyrieFingerMotorName.class);

   private final EnumMap<ValkyrieFingerMotorName, YoEffortJointHandleHolder> jointHandles;

   private final StateMachine<GraspState, State> stateMachine;
   private final YoEnum<GraspState> requestedState;

   private final double controlDT;

   private final ValkyrieRosControlFingerStateEstimator fingerStateEstimator;
   private final YoPIDGains gains;

   public ValkyrieFingerSetController(RobotSide robotSide, YoDouble yoTime, double controlDT, ValkyrieRosControlFingerStateEstimator fingerStateEstimator,
                                      YoPIDGains gains, YoDouble trajectoryTime, YoDouble thumbCloseDelay, YoDouble fingerOpenDelay,
                                      EnumMap<ValkyrieFingerMotorName, YoEffortJointHandleHolder> jointHandles, YoVariableRegistry parentRegistry)
   {
      this.robotSide = robotSide;
      this.controlDT = controlDT;
      this.fingerStateEstimator = fingerStateEstimator;
      this.gains = gains;
      this.trajectoryTime = trajectoryTime;
      this.thumbCloseDelay = thumbCloseDelay;
      this.fingerOpenDelay = fingerOpenDelay;
      this.jointHandles = jointHandles;

      String sidePrefix = robotSide.getCamelCaseName();
      registry = new YoVariableRegistry(sidePrefix + name);

      mapJointsAndVariables(gains);

      fingerPolynomial = new YoPolynomial(sidePrefix + "FingersPolynomial", 4, registry);
      thumbPolynomial = new YoPolynomial(sidePrefix + "ThumbPolynomial", 4, registry);

      requestedState = new YoEnum<>(sidePrefix + "RequestedGrip", registry, GraspState.class, true);
      requestedState.set(null);
      stateMachine = setupStateMachine(sidePrefix, yoTime);

      parentRegistry.addChild(registry);
   }

   private void mapJointsAndVariables(YoPIDGains gains)
   {
      for (ValkyrieFingerMotorName jointEnum : ValkyrieFingerMotorName.values)
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

         PIDController pidController = new PIDController(jointEnum.getPascalCaseJointName(robotSide), registry);
         pidControllers.put(jointEnum, pidController);

         YoDouble closedAngle = new YoDouble(jointName + "ClosedAngle", registry);
         closedAngle.set(ValkyrieFingerControlParameters.getClosedDesiredDefinition(robotSide).get(jointEnum));
         closedAngles.put(jointEnum, closedAngle);

         YoDouble openedAngle = new YoDouble(jointName + "OpenedAngle", registry);
         openedAngle.set(ValkyrieFingerControlParameters.getOpenDesiredDefinition(robotSide).get(jointEnum));
         openedAngles.put(jointEnum, openedAngle);
      }
   }

   private StateMachine<GraspState, State> setupStateMachine(String sidePrefix, DoubleProvider time)
   {
      StateMachineFactory<GraspState, State> factory = new StateMachineFactory<>(GraspState.class);
      factory.setNamePrefix(sidePrefix + "GripCurrent").setRegistry(registry).buildYoClock(time);

      GripState stateOpenGrip = new OpenGrip();
      GripState stateClosedGrip = new ClosedGrip();

      factory.addState(GraspState.OPEN, stateOpenGrip);
      factory.addState(GraspState.CLOSE, stateClosedGrip);
      factory.addRequestedTransition(GraspState.OPEN, GraspState.CLOSE, requestedState, true);
      factory.addRequestedTransition(GraspState.CLOSE, GraspState.OPEN, requestedState, true);

      return factory.build(GraspState.OPEN);
   }

   public void requestState(GraspState requestedState)
   {
      this.requestedState.set(requestedState);
   }

   private class OpenGrip extends GripState
   {
      @Override
      public void onEntry()
      {
         fingerPolynomial.setCubic(0.0, trajectoryTime.getDoubleValue(), 0.0, 1.0);
         thumbPolynomial.setCubic(0.0, trajectoryTime.getDoubleValue(), 0.0, 1.0);

         for (ValkyrieFingerMotorName jointEnum : ValkyrieFingerMotorName.values)
         {
            double qInitial;
            YoDouble desiredAngle = desiredAngles.get(jointEnum);
            if (desiredAngle.isNaN())
               qInitial = fingerStateEstimator.getMotorBasedFingerJointPosition(robotSide, jointEnum.getCorrespondingJointName(1));
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
      @Override
      public void onEntry()
      {
         fingerPolynomial.setCubic(0.0, trajectoryTime.getDoubleValue(), 0.0, 1.0);
         thumbPolynomial.setCubic(0.0, trajectoryTime.getDoubleValue(), 0.0, 1.0);

         for (ValkyrieFingerMotorName jointEnum : ValkyrieFingerMotorName.values)
         {
            double qInitial;
            YoDouble desiredAngle = desiredAngles.get(jointEnum);
            if (desiredAngle.isNaN())
               qInitial = fingerStateEstimator.getMotorBasedFingerJointPosition(robotSide, jointEnum.getCorrespondingJointName(1));
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

   public abstract class GripState implements State
   {
      @Override
      public void doAction(double timeInState)
      {
         double fingerTime = timeInState - getFingerDelay();
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

         double thumbTime = timeInState - getThumbDelay();
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

         for (ValkyrieFingerMotorName jointEnum : ValkyrieFingerMotorName.values)
         {
            if (jointEnum.getFingerName() == FingerName.THUMB)
            {
               double q0 = initialDesiredAngles.get(jointEnum).getDoubleValue();
               double qf = finalDesiredAngles.get(jointEnum).getDoubleValue();
               desiredAngles.get(jointEnum).set(EuclidCoreTools.interpolate(q0, qf, thumbAlpha));
               desiredVelocities.get(jointEnum).set(thumbAlphaDot * (qf - q0));
            }
            else
            {
               double q0 = initialDesiredAngles.get(jointEnum).getDoubleValue();
               double qf = finalDesiredAngles.get(jointEnum).getDoubleValue();
               desiredAngles.get(jointEnum).set(EuclidCoreTools.interpolate(q0, qf, fingerAlpha));
               desiredVelocities.get(jointEnum).set(fingerAlphaDot * (qf - q0));
            }
         }
      }

      protected abstract double getFingerDelay();

      protected abstract double getThumbDelay();

      @Override
      public boolean isDone(double timeInState)
      {
         return timeInState >= trajectoryTime.getDoubleValue();
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

      for (ValkyrieFingerMotorName jointEnum : ValkyrieFingerMotorName.values)
      {
         PIDController pidController = pidControllers.get(jointEnum);
         pidController.setGains(gains);
         double fingerJointTransmissionScale = fingerStateEstimator.getFingerJointTransmissionScale(robotSide, jointEnum.getCorrespondingJointName(1));
         pidController.setProportionalGain(gains.getKp() / fingerJointTransmissionScale);
         pidController.setIntegralGain(gains.getKi() / fingerJointTransmissionScale);
         YoEffortJointHandleHolder handle = jointHandles.get(jointEnum);

         double q = fingerStateEstimator.getMotorBasedFingerJointPosition(robotSide, jointEnum.getCorrespondingJointName(1));
         double q_d = desiredAngles.get(jointEnum).getDoubleValue();
         double qd = handle.getQd();
         double qd_d = desiredVelocities.get(jointEnum).getDoubleValue();

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
         handle.getDesiredJointData().setDesiredTorque(tau);
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
