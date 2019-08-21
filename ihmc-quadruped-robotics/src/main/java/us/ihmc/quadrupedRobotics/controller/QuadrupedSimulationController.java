package us.ihmc.quadrupedRobotics.controller;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotDataLogger.RobotVisualizer;
import us.ihmc.robotModels.OutputWriter;
import us.ihmc.sensorProcessing.communication.producers.DRCPoseCommunicator;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.stateEstimation.humanoid.StateEstimatorController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedSimulationController implements RobotController
{
   private static final boolean PIN_ROBOT_IN_AIR = false;
   private static final Vector3D pinPosition = new Vector3D(0.0, 0.0, 1.0);
   private static final Vector3D zeroAngularVelocity = new Vector3D();
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final FloatingRootJointRobot sdfRobot;
   private final SensorReader sensorReader;
   private final OutputWriter outputWriter;
   private final RobotController gaitControlManager;
   private StateEstimatorController stateEstimator; //not implemented yet
   private final DRCPoseCommunicator poseCommunicator;
   private boolean firstTick = true;

   private final RobotVisualizer robotVisualizer;

   private final SensorDataContext sensorDataContext = new SensorDataContext();

   public QuadrupedSimulationController(FloatingRootJointRobot simulationRobot, SensorReader sensorReader, OutputWriter outputWriter, RobotController gaitControlManager, StateEstimatorController stateEstimator,
                                        DRCPoseCommunicator poseCommunicator)
   {
      this(simulationRobot, sensorReader, outputWriter, gaitControlManager, stateEstimator, poseCommunicator, null);
   }

   public QuadrupedSimulationController(FloatingRootJointRobot simulationRobot, SensorReader sensorReader, OutputWriter outputWriter, RobotController gaitControlManager, StateEstimatorController stateEstimator,
                                        DRCPoseCommunicator poseCommunicator, RobotVisualizer robotVisualizer)
   {
      this.sdfRobot = simulationRobot;
      this.poseCommunicator = poseCommunicator;
      this.sensorReader = sensorReader;
      this.outputWriter = outputWriter;
      this.gaitControlManager = gaitControlManager;
      this.stateEstimator = stateEstimator;
      this.robotVisualizer = robotVisualizer;
      registry.addChild(gaitControlManager.getYoVariableRegistry());

      if (robotVisualizer != null)
      {
         robotVisualizer.setMainRegistry(getYoVariableRegistry(), null, null);
      }
   }

   @Override
   public void initialize()
   {

   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public String getDescription()
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
      if(poseCommunicator != null)
      {
         poseCommunicator.write();
      }

      outputWriter.write();

      if (robotVisualizer != null)
      {
         robotVisualizer.update(timestamp);
      }

      if(PIN_ROBOT_IN_AIR)
      {
         sdfRobot.setPositionInWorld(pinPosition);
         sdfRobot.setOrientation(0.0, 0.0,0.0);
         sdfRobot.setLinearVelocity(zeroAngularVelocity);
         sdfRobot.setAngularVelocity(zeroAngularVelocity);
         sdfRobot.getRootJoint().setAcceleration(zeroAngularVelocity);
         sdfRobot.getRootJoint().setAngularAccelerationInBody(zeroAngularVelocity);
      }
   }
}
