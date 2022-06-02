package us.ihmc.quadrupedRobotics.controller;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotDataLogger.RobotVisualizer;
import us.ihmc.robotModels.OutputWriter;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimFloatingJointBasics;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataPublisher;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.stateEstimation.humanoid.StateEstimatorController;
import us.ihmc.yoVariables.registry.YoRegistry;

public class QuadrupedSimulationController implements Controller
{
   private static final boolean PIN_ROBOT_IN_AIR = false;
   private static final Vector3D pinPosition = new Vector3D(0.0, 0.0, 1.0);
   private static final Vector3D zeroAngularVelocity = new Vector3D();
   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);
   private final Robot sdfRobot;
   private final SensorReader sensorReader;
   private final OutputWriter outputWriter;
   private final RobotController gaitControlManager;
   private StateEstimatorController stateEstimator; //not implemented yet
   private final RobotConfigurationDataPublisher robotConfigurationDataPublisher;
   private boolean firstTick = true;

   private final RobotVisualizer robotVisualizer;

   private final SensorDataContext sensorDataContext = new SensorDataContext();

   public QuadrupedSimulationController(Robot simulationRobot, SensorReader sensorReader, OutputWriter outputWriter, RobotController gaitControlManager, StateEstimatorController stateEstimator,
                                        RobotConfigurationDataPublisher robotConfigurationDataPublisher)
   {
      this(simulationRobot, sensorReader, outputWriter, gaitControlManager, stateEstimator, robotConfigurationDataPublisher, null);
   }

   public QuadrupedSimulationController(Robot simulationRobot, SensorReader sensorReader, OutputWriter outputWriter, RobotController gaitControlManager, StateEstimatorController stateEstimator,
                                        RobotConfigurationDataPublisher robotConfigurationDataPublisher, RobotVisualizer robotVisualizer)
   {
      this.sdfRobot = simulationRobot;
      this.robotConfigurationDataPublisher = robotConfigurationDataPublisher;
      this.sensorReader = sensorReader;
      this.outputWriter = outputWriter;
      this.gaitControlManager = gaitControlManager;
      this.stateEstimator = stateEstimator;
      this.robotVisualizer = robotVisualizer;
      registry.addChild(gaitControlManager.getYoRegistry());

      if (robotVisualizer != null)
      {
         robotVisualizer.setMainRegistry(getYoRegistry(), null);
      }
   }

   @Override
   public void initialize()
   {

   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public void doControl()
   {
      long timestamp = sensorReader.read(sensorDataContext);
      sensorReader.compute(timestamp, sensorDataContext);
      if(stateEstimator != null)
      {
         if(firstTick)
         {
            stateEstimator.initialize();
            firstTick = false;
         }
         stateEstimator.doControl();
      }
      gaitControlManager.doControl();
      if(robotConfigurationDataPublisher != null)
      {
         robotConfigurationDataPublisher.write();
      }

      outputWriter.write();

      if (robotVisualizer != null)
      {
         robotVisualizer.update(timestamp);
      }

      if(PIN_ROBOT_IN_AIR)
      {
         SimFloatingJointBasics rootJoint = sdfRobot.getFloatingRootJoint();
         rootJoint.getJointPose().getPosition().set(pinPosition);
         rootJoint.getJointPose().getOrientation().setToZero();
         rootJoint.getJointTwist().setToZero();
         rootJoint.getJointAcceleration().setToZero();
         rootJoint.setPinned(true);
      }
   }
}
