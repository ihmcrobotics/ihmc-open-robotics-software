package us.ihmc.valkyrie.fingers;

import java.util.EnumMap;
import java.util.Map;

import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import controller_msgs.msg.dds.TrajectoryPoint1DMessage;
import controller_msgs.msg.dds.ValkyrieHandFingerTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.partNames.FingerName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlFingerStateEstimator;
import us.ihmc.valkyrieRosControl.dataHolders.YoEffortJointHandleHolder;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ValkyrieFingerSetController implements RobotController
{
   private final double trajectoryTime = ValkyrieHandFingerTrajectoryMessageConversion.trajectoryTime;
   private final double extendedTrajectoryTime = trajectoryTime * ValkyrieHandFingerTrajectoryMessageConversion.extendedTimeRatioForThumb;

   private static final double MIN_ACTUATOR_POSITION = 0.0;
   private static final double MAX_ACTUATOR_POSITION = 3.6;

   public static final boolean DEBUG = false;

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry;

   private final RobotSide robotSide;

   private final ValkyrieFingerSetTrajectoryGenerator<ValkyrieFingerMotorName> fingerSetTrajectoryGenerator;
   private final EnumMap<ValkyrieFingerMotorName, YoDouble> desiredAngles = new EnumMap<>(ValkyrieFingerMotorName.class);
   private final EnumMap<ValkyrieFingerMotorName, YoDouble> desiredVelocities = new EnumMap<>(ValkyrieFingerMotorName.class);
   private final Map<ValkyrieFingerMotorName, PIDController> pidControllers = new EnumMap<>(ValkyrieFingerMotorName.class);

   private final EnumMap<ValkyrieFingerMotorName, YoEffortJointHandleHolder> jointHandles;

   private final double controlDT;

   private final ValkyrieRosControlFingerStateEstimator fingerStateEstimator;
   private final EnumMap<ValkyrieFingerMotorName, PIDGainsReadOnly> gains;

   public ValkyrieFingerSetController(RobotSide robotSide, YoDouble yoTime, double controlDT, ValkyrieRosControlFingerStateEstimator fingerStateEstimator,
                                      EnumMap<ValkyrieFingerMotorName, PIDGainsReadOnly> gains,
                                      EnumMap<ValkyrieFingerMotorName, YoEffortJointHandleHolder> jointHandles, YoVariableRegistry parentRegistry)
   {
      this.robotSide = robotSide;
      this.controlDT = controlDT;
      this.fingerStateEstimator = fingerStateEstimator;
      this.gains = gains;
      this.jointHandles = jointHandles;

      String sidePrefix = robotSide.getCamelCaseName();
      registry = new YoVariableRegistry(sidePrefix + name);

      mapJointsAndVariables();
      fingerSetTrajectoryGenerator = new ValkyrieFingerSetTrajectoryGenerator<>(ValkyrieFingerMotorName.class, robotSide, yoTime, desiredAngles, registry);

      parentRegistry.addChild(registry);
   }

   private void mapJointsAndVariables()
   {
      for (ValkyrieFingerMotorName fingerMotorNameEnum : ValkyrieFingerMotorName.values)
      {
         String jointName = fingerMotorNameEnum.getJointName(robotSide);

         YoDouble desiredAngle = new YoDouble("q_d_" + jointName, registry);
         desiredAngle.set(fingerStateEstimator.getFingerMotorPosition(robotSide, fingerMotorNameEnum));
         desiredAngles.put(fingerMotorNameEnum, desiredAngle);

         YoDouble desiredVelocity = new YoDouble("qd_d_" + jointName, registry);
         desiredVelocities.put(fingerMotorNameEnum, desiredVelocity);

         PIDController pidController = new PIDController(fingerMotorNameEnum.getPascalCaseJointName(robotSide), registry);
         pidControllers.put(fingerMotorNameEnum, pidController);
      }
   }

   @Override
   public void doControl()
   {
      // update trajectory generators.
      fingerSetTrajectoryGenerator.doControl();

      // set desired values.
      for (ValkyrieFingerMotorName fingerMotorNameEnum : ValkyrieFingerMotorName.values)
      {
         desiredAngles.get(fingerMotorNameEnum).set(fingerSetTrajectoryGenerator.getDesired(fingerMotorNameEnum));
         desiredVelocities.get(fingerMotorNameEnum).set(fingerSetTrajectoryGenerator.getDesiredVelocity(fingerMotorNameEnum));
      }

      // PID control.
      for (ValkyrieFingerMotorName fingerMotorNameEnum : ValkyrieFingerMotorName.values)
      {
         PIDGainsReadOnly gainsToUse = gains.get(fingerMotorNameEnum);
         PIDController pidController = pidControllers.get(fingerMotorNameEnum);
         pidController.setGains(gainsToUse);
         pidController.setProportionalGain(gainsToUse.getKp());
         pidController.setIntegralGain(gainsToUse.getKi());
         YoEffortJointHandleHolder handle = jointHandles.get(fingerMotorNameEnum);

         double q = handle.getQ();
         double q_d = desiredAngles.get(fingerMotorNameEnum).getDoubleValue();
         double qd = handle.getQd();
         double qd_d = desiredVelocities.get(fingerMotorNameEnum).getDoubleValue();

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

   public void getDesiredHandConfiguration(HandConfiguration handConfiguration)
   {
      fingerSetTrajectoryGenerator.clearTrajectories();
      switch (handConfiguration)
      {
         case CLOSE:
            for (ValkyrieFingerMotorName fingerMotorName : ValkyrieFingerMotorName.values)
            {
               double desiredFingerMotor = ValkyrieFingerControlParameters.getDesiredFingerMotorPosition(robotSide, fingerMotorName, 1.0);
               double time = fingerMotorName.getFingerName() == FingerName.THUMB ? extendedTrajectoryTime : trajectoryTime;
               fingerSetTrajectoryGenerator.appendWayPoint(fingerMotorName, time, desiredFingerMotor);
            }
            break;

         case OPEN:
            for (ValkyrieFingerMotorName fingerMotorName : ValkyrieFingerMotorName.values)
            {
               double desiredFingerMotor = ValkyrieFingerControlParameters.getDesiredFingerMotorPosition(robotSide, fingerMotorName, 0.0);
               double time = fingerMotorName.getFingerName() == FingerName.THUMB ? extendedTrajectoryTime : trajectoryTime;
               fingerSetTrajectoryGenerator.appendWayPoint(fingerMotorName, time, desiredFingerMotor);
            }
            break;

         case STOP:
            for (ValkyrieFingerMotorName fingerMotorName : ValkyrieFingerMotorName.values)
            {
               double currentEstimatedPosition = fingerStateEstimator.getFingerMotorPosition(robotSide, fingerMotorName);
               fingerSetTrajectoryGenerator.appendStopPoint(fingerMotorName, currentEstimatedPosition);
            }
            break;

         default:

            break;
      }
      fingerSetTrajectoryGenerator.executeTrajectories();
   }

   public void getHandFingerTrajectoryMessage(ValkyrieHandFingerTrajectoryMessage handFingerTrajectoryMessage)
   {
      fingerSetTrajectoryGenerator.clearTrajectories();

      for (ValkyrieFingerMotorName fingerMotorName : ValkyrieFingerMotorName.values)
      {
         int indexOfTrajectory = hasTrajectory(handFingerTrajectoryMessage.getValkyrieFingerMotorNames(), fingerMotorName);

         if (indexOfTrajectory == -1)
         {
            fingerSetTrajectoryGenerator.appendStopPoint(fingerMotorName, fingerStateEstimator.getFingerMotorPosition(robotSide, fingerMotorName));
         }
         else
         {
            Object<OneDoFJointTrajectoryMessage> jointTrajectoryMessages = handFingerTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages();
            Object<TrajectoryPoint1DMessage> trajectoryPoints = jointTrajectoryMessages.get(indexOfTrajectory).getTrajectoryPoints();

            for (int i = 0; i < trajectoryPoints.size(); i++)
            {
               TrajectoryPoint1DMessage trajectoryPoint1DMessage = trajectoryPoints.get(i);
               double wayPointTime = trajectoryPoint1DMessage.getTime();
               double wayPointPosition = ValkyrieFingerControlParameters.getDesiredFingerMotorPosition(robotSide,
                                                                                                       fingerMotorName,
                                                                                                       trajectoryPoint1DMessage.getPosition());
               fingerSetTrajectoryGenerator.appendWayPoint(fingerMotorName, wayPointTime, wayPointPosition);
            }
         }
      }
      fingerSetTrajectoryGenerator.executeTrajectories();
   }

   public void initializeDesiredTrajectoryGenerator()
   {
      for (ValkyrieFingerMotorName fingerMotorNameEnum : ValkyrieFingerMotorName.values)
      {
         desiredAngles.get(fingerMotorNameEnum).set(fingerStateEstimator.getFingerMotorPosition(robotSide, fingerMotorNameEnum));
      }
   }

   private int hasTrajectory(us.ihmc.idl.IDLSequence.Byte namesInMessage, ValkyrieFingerMotorName fingerMotorName)
   {
      for (int i = 0; i < namesInMessage.size(); i++)
         if (fingerMotorName == ValkyrieFingerMotorName.fromByte(namesInMessage.get(i)))
            return i;

      return -1;
   }
}
