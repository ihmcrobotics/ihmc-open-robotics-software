package us.ihmc.valkyrie.fingers;

import java.util.EnumMap;
import java.util.Map;

import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.valkyrie.fingers.trajectories.GoalPositionTrajectory;
import us.ihmc.valkyrie.fingers.trajectories.LinearTrajectory;
import us.ihmc.valkyrie.fingers.trajectories.SinusoidalTrajectory;
import us.ihmc.valkyrie.fingers.trajectories.TrajectoryGeneratorStateMachine;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This FingerSetController is for simulating finger trajectory generation given by the FingerTrajectoryMessages.
 * Once OneDegreeOfFreedomJoint of finger joint is initiated in scs, Valkyrie can move finger by this FingerSetController.
 */
public class IndividualValkyrieFingerSetController implements RobotController
{
   enum TrajectoryType
   {
      Linear, Sinusoidal
   }

   private static final double MIN_ACTUATOR_POSITION = 0.0;
   private static final double MAX_ACTUATOR_POSITION = 3.6;

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry;

   private final RobotSide robotSide;
   private final YoDouble yoTime;

   private final TrajectoryType trajectoryType = TrajectoryType.Sinusoidal;
   private final Map<ValkyrieFingerMotorName, GoalPositionTrajectory> trajectoryInterfaces = new EnumMap<>(ValkyrieFingerMotorName.class);
   private final Map<ValkyrieFingerMotorName, TrajectoryGeneratorStateMachine> trajectoryGenerators = new EnumMap<>(ValkyrieFingerMotorName.class);

   private final Map<ValkyrieFingerMotorName, YoDouble> desiredAngles = new EnumMap<>(ValkyrieFingerMotorName.class);
   private final Map<ValkyrieFingerMotorName, YoDouble> desiredVelocities = new EnumMap<>(ValkyrieFingerMotorName.class);
   private final Map<ValkyrieFingerMotorName, PIDController> pidControllers = new EnumMap<>(ValkyrieFingerMotorName.class);

   public IndividualValkyrieFingerSetController(RobotSide robotSide, YoDouble yoTime, FloatingRootJointRobot simulatedRobot, YoVariableRegistry parentRegistry)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      registry = new YoVariableRegistry(sidePrefix + name);
      parentRegistry.addChild(registry);
      this.robotSide = robotSide;
      this.yoTime = yoTime;

      for (ValkyrieFingerMotorName jointEnum : ValkyrieFingerMotorName.values)
      {
         String jointName = jointEnum.getJointName(robotSide);
         OneDegreeOfFreedomJoint fingerJoint = simulatedRobot.getOneDegreeOfFreedomJoint(jointName);

         GoalPositionTrajectory trajectoryInterface;
         switch (trajectoryType)
         {
         case Linear:
            trajectoryInterface = new LinearTrajectory(0.0);
            break;
         case Sinusoidal:
            trajectoryInterface = new SinusoidalTrajectory(0.0);
            break;
         default:
            trajectoryInterface = new LinearTrajectory(0.0);
            break;
         }
         trajectoryInterface.setUpperLimit(MAX_ACTUATOR_POSITION);
         trajectoryInterface.setLowerLimit(MIN_ACTUATOR_POSITION);

         TrajectoryGeneratorStateMachine trajectoryGenerator = new TrajectoryGeneratorStateMachine(jointName + "TrajectoryGenerator", yoTime, registry,
                                                                                                   trajectoryInterface);

         trajectoryInterfaces.put(jointEnum, trajectoryInterface);
         trajectoryGenerators.put(jointEnum, trajectoryGenerator);

         YoDouble desiredAngle = new YoDouble("q_desired_" + jointName, registry);
         desiredAngle.setToNaN();
         desiredAngles.put(jointEnum, desiredAngle);

         YoDouble desiredVelocity = new YoDouble("qd_desired_" + jointName, registry);
         desiredVelocities.put(jointEnum, desiredVelocity);

         PIDController pidController = new PIDController(jointEnum.getPascalCaseJointName(robotSide), registry);
         pidControllers.put(jointEnum, pidController);
      }
   }

   @Override
   public void initialize()
   {
      // Do nothing
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
   public void doControl()
   {
      for (ValkyrieFingerMotorName jointEnum : ValkyrieFingerMotorName.values)
      {
         trajectoryGenerators.get(jointEnum).doControl();
         desiredAngles.get(jointEnum).set(trajectoryGenerators.get(jointEnum).getValue());
         desiredVelocities.get(jointEnum).set(trajectoryGenerators.get(jointEnum).getVelocity());
      }
   }

   public void executeTrajectory(ValkyrieFingerMotorName jointEnum, double trajectoryTime, double delayTime, double desiredPosition)
   {
      trajectoryGenerators.get(jointEnum).executeTrajectory(trajectoryTime, delayTime, desiredPosition);
   }

   public void writeDesiredJointAngles()
   {
      for (ValkyrieFingerMotorName jointEnum : ValkyrieFingerMotorName.values)
      {
         trajectoryGenerators.get(jointEnum).doControl();
         desiredAngles.get(jointEnum).set(trajectoryGenerators.get(jointEnum).getValue());
         desiredVelocities.get(jointEnum).set(trajectoryGenerators.get(jointEnum).getVelocity());
      }
   }
}