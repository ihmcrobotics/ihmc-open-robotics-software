package us.ihmc.valkyrie.hands.athena;

import java.util.EnumMap;
import java.util.Map;

import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.partNames.FingerName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.valkyrie.hands.athena.AthenaHandModel.AthenaFingerMotorName;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlAthenaStateEstimator;
import us.ihmc.valkyrieRosControl.dataHolders.YoEffortJointHandleHolder;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import valkyrie_msgs.msg.dds.AthenaTrajectoryMessage;

public class AthenaController implements RobotController
{
   private final double trajectoryTime = AthenaTrajectoryMessageConversion.trajectoryTime;
   private final double extendedTrajectoryTime = trajectoryTime * AthenaTrajectoryMessageConversion.extendedTimeRatioForThumb;

   private static final double MIN_ACTUATOR_POSITION = 0.0;
   private static final double MAX_ACTUATOR_POSITION = 3.6;

   public static final boolean DEBUG = false;

   private final String name = getClass().getSimpleName();
   private final YoRegistry registry;

   private final RobotSide robotSide;

   private final AthenaTrajectoryGenerator<AthenaFingerMotorName> fingerSetTrajectoryGenerator;
   private final EnumMap<AthenaFingerMotorName, YoDouble> desiredAngles = new EnumMap<>(AthenaFingerMotorName.class);
   private final EnumMap<AthenaFingerMotorName, YoDouble> desiredVelocities = new EnumMap<>(AthenaFingerMotorName.class);
   private final Map<AthenaFingerMotorName, PIDController> pidControllers = new EnumMap<>(AthenaFingerMotorName.class);

   private final EnumMap<AthenaFingerMotorName, YoEffortJointHandleHolder> jointHandles;

   private final double controlDT;

   private final ValkyrieRosControlAthenaStateEstimator fingerStateEstimator;
   private final EnumMap<AthenaFingerMotorName, PIDGainsReadOnly> gains;

   public AthenaController(RobotSide robotSide,
                           YoDouble yoTime,
                           double controlDT,
                           ValkyrieRosControlAthenaStateEstimator fingerStateEstimator,
                           EnumMap<AthenaFingerMotorName, PIDGainsReadOnly> gains,
                           EnumMap<AthenaFingerMotorName, YoEffortJointHandleHolder> jointHandles,
                           YoRegistry parentRegistry)
   {
      this.robotSide = robotSide;
      this.controlDT = controlDT;
      this.fingerStateEstimator = fingerStateEstimator;
      this.gains = gains;
      this.jointHandles = jointHandles;

      String sidePrefix = robotSide.getCamelCaseName();
      registry = new YoRegistry(sidePrefix + name);

      mapJointsAndVariables();
      fingerSetTrajectoryGenerator = new AthenaTrajectoryGenerator<>(AthenaFingerMotorName.class, robotSide, yoTime, desiredAngles, registry);

      parentRegistry.addChild(registry);
   }

   private void mapJointsAndVariables()
   {
      for (AthenaFingerMotorName fingerMotorNameEnum : AthenaFingerMotorName.values)
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
      for (AthenaFingerMotorName fingerMotorNameEnum : AthenaFingerMotorName.values)
      {
         desiredAngles.get(fingerMotorNameEnum).set(fingerSetTrajectoryGenerator.getDesired(fingerMotorNameEnum));
         desiredVelocities.get(fingerMotorNameEnum).set(fingerSetTrajectoryGenerator.getDesiredVelocity(fingerMotorNameEnum));
      }

      // PID control.
      for (AthenaFingerMotorName fingerMotorNameEnum : AthenaFingerMotorName.values)
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
   public YoRegistry getYoRegistry()
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
            for (AthenaFingerMotorName fingerMotorName : AthenaFingerMotorName.values)
            {
               double desiredFingerMotor = AthenaFingerControlParameters.getDesiredFingerMotorPosition(robotSide, fingerMotorName, 1.0);
               double time = fingerMotorName.getFingerName() == FingerName.THUMB ? extendedTrajectoryTime : trajectoryTime;
               fingerSetTrajectoryGenerator.appendWayPoint(fingerMotorName, time, desiredFingerMotor);
            }
            break;

         case OPEN:
            for (AthenaFingerMotorName fingerMotorName : AthenaFingerMotorName.values)
            {
               double desiredFingerMotor = AthenaFingerControlParameters.getDesiredFingerMotorPosition(robotSide, fingerMotorName, 0.0);
               double time = fingerMotorName.getFingerName() == FingerName.THUMB ? extendedTrajectoryTime : trajectoryTime;
               fingerSetTrajectoryGenerator.appendWayPoint(fingerMotorName, time, desiredFingerMotor);
            }
            break;

         case STOP:
            for (AthenaFingerMotorName fingerMotorName : AthenaFingerMotorName.values)
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

   public void getAthenaTrajectoryMessage(AthenaTrajectoryMessage athenaTrajectoryMessage)
   {
      fingerSetTrajectoryGenerator.clearTrajectories();

      for (AthenaFingerMotorName fingerMotorName : AthenaFingerMotorName.values)
      {
         int indexOfTrajectory = hasTrajectory(athenaTrajectoryMessage.getFingerMotorNames(), fingerMotorName);

         if (indexOfTrajectory == -1)
         {
            fingerSetTrajectoryGenerator.appendStopPoint(fingerMotorName, fingerStateEstimator.getFingerMotorPosition(robotSide, fingerMotorName));
         }
         else
         {
            Object<OneDoFJointTrajectoryMessage> jointTrajectoryMessages = athenaTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages();
            Object<TrajectoryPoint1DMessage> trajectoryPoints = jointTrajectoryMessages.get(indexOfTrajectory).getTrajectoryPoints();

            for (int i = 0; i < trajectoryPoints.size(); i++)
            {
               TrajectoryPoint1DMessage trajectoryPoint1DMessage = trajectoryPoints.get(i);
               double wayPointTime = trajectoryPoint1DMessage.getTime();
               double wayPointPosition = AthenaFingerControlParameters.getDesiredFingerMotorPosition(robotSide,
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
      for (AthenaFingerMotorName fingerMotorNameEnum : AthenaFingerMotorName.values)
      {
         desiredAngles.get(fingerMotorNameEnum).set(fingerStateEstimator.getFingerMotorPosition(robotSide, fingerMotorNameEnum));
      }
   }

   private int hasTrajectory(us.ihmc.idl.IDLSequence.Byte namesInMessage, AthenaFingerMotorName fingerMotorName)
   {
      for (int i = 0; i < namesInMessage.size(); i++)
         if (fingerMotorName == AthenaFingerMotorName.fromByte(namesInMessage.get(i)))
            return i;

      return -1;
   }
}
