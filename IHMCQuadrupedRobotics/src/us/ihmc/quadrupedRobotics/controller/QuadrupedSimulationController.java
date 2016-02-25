package us.ihmc.quadrupedRobotics.controller;

import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.OutputWriter;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.quadrupedRobotics.stateEstimator.QuadrupedStateEstimator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.sensorProcessing.communication.producers.DRCPoseCommunicator;
import us.ihmc.simulationconstructionset.robotController.RawSensorReader;
import us.ihmc.simulationconstructionset.robotController.RobotController;

public class QuadrupedSimulationController implements RobotController
{
   private static final boolean PIN_ROBOT_IN_AIR = false;
   private static final Vector3d pinPosition = new Vector3d(0.0, 0.0, 1.0);
   private static final Vector3d zeroAngularVelocity = new Vector3d();
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final SDFRobot sdfRobot;
   private final RawSensorReader sensorReader;
   private final OutputWriter outputWriter;
   private final RobotController gaitControlManager;
   private RobotController headController; //not implemented yet
   private QuadrupedStateEstimator stateEstimator; //not implemented yet
   private final DRCPoseCommunicator poseCommunicator;
   
   public QuadrupedSimulationController(SDFRobot simulationRobot, RawSensorReader sensorReader, OutputWriter outputWriter, QuadrupedControllerManager gaitControlManager, QuadrupedStateEstimator stateEstimator,
         DRCPoseCommunicator poseCommunicator, RobotController headController)
   {
      this.sdfRobot = simulationRobot;
      this.poseCommunicator = poseCommunicator;
      this.sensorReader = sensorReader;
      this.outputWriter = outputWriter;
      this.gaitControlManager = gaitControlManager;
      this.stateEstimator = stateEstimator;
      this.headController = headController;
      registry.addChild(gaitControlManager.getYoVariableRegistry());
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
      sensorReader.read();
      stateEstimator.doControl();
      gaitControlManager.doControl();
      if(poseCommunicator != null)
      {
         poseCommunicator.write();
      }

      if(headController != null)
         headController.doControl();

      outputWriter.write();
      
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
