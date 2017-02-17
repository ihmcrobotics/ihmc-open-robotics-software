package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.euclid.tuple3D.Vector3D;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;

public class JointStateUpdaterTest
{
   private static final Vector3D X = new Vector3D(1.0, 0.0, 0.0);
   private static final Vector3D Y = new Vector3D(0.0, 1.0, 0.0);
   private static final Vector3D Z = new Vector3D(0.0, 0.0, 1.0);

   private static final Random random = new Random(1776L);
   private static final double EPS = 1e-10;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testConstructorNormalCase()
   {
      YoVariableRegistry registry = new YoVariableRegistry("Blop");
      
      Vector3D[] jointAxes = {X, Y, Z, Z, X, Z, Z, X, Y, Y};
      ScrewTestTools.RandomFloatingChain randomFloatingChain = new ScrewTestTools.RandomFloatingChain(random, jointAxes);
      ArrayList<RevoluteJoint> joints = new ArrayList<RevoluteJoint>(randomFloatingChain.getRevoluteJoints());
      
      FullInverseDynamicsStructure inverseDynamicsStructure = createFullInverseDynamicsStructure(randomFloatingChain, joints);

      ArrayList<RevoluteJoint> jointsWithPositionSensor = new ArrayList<RevoluteJoint>(joints);
      ArrayList<RevoluteJoint> jointsWithVelocitySensor = new ArrayList<RevoluteJoint>(joints);
      SensorProcessing sensorMap = createJointSensorDataSource(registry, jointsWithPositionSensor, jointsWithVelocitySensor);
      
      try
      {
         new JointStateUpdater(inverseDynamicsStructure, sensorMap, null, registry);
      }
      catch (Exception e)
      {
         fail("Could not create JointStateUpdater. StackTrace:");
         e.printStackTrace();
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout=300000)
   public void testInitializingAndReading()
   {
      YoVariableRegistry registry = new YoVariableRegistry("Blop");
      
      Vector3D[] jointAxes = {X, Y, Z, Z, X, Z, Z, X, Y, Y};
      ScrewTestTools.RandomFloatingChain randomFloatingChain = new ScrewTestTools.RandomFloatingChain(random, jointAxes);
      ArrayList<RevoluteJoint> joints = new ArrayList<RevoluteJoint>(randomFloatingChain.getRevoluteJoints());
      
      FullInverseDynamicsStructure inverseDynamicsStructure = createFullInverseDynamicsStructure(randomFloatingChain, joints);

      // Test constructor is working for normal case
      ArrayList<RevoluteJoint> jointsWithPositionSensor = new ArrayList<RevoluteJoint>(joints);
      ArrayList<RevoluteJoint> jointsWithVelocitySensor = new ArrayList<RevoluteJoint>(joints);
      SensorProcessing sensorMap = createJointSensorDataSource(registry, jointsWithPositionSensor, jointsWithVelocitySensor);
      
      JointStateUpdater jointStateUpdater = new JointStateUpdater(inverseDynamicsStructure, sensorMap, null, registry);
      
      fillSensorsWithRandomPositionsAndVelocities(jointsWithPositionSensor, jointsWithVelocitySensor, sensorMap);
      
      jointStateUpdater.initialize();
      
      readAndCheckJointPositions(jointsWithPositionSensor, sensorMap);
      readAndCheckJointVelocities(jointsWithVelocitySensor, sensorMap);
      
      for (int i = 0; i < 1000; i++)
      {
         fillSensorsWithRandomPositionsAndVelocities(jointsWithPositionSensor, jointsWithVelocitySensor, sensorMap);
         
         jointStateUpdater.updateJointState();
         
         readAndCheckJointPositions(jointsWithPositionSensor, sensorMap);
         readAndCheckJointVelocities(jointsWithVelocitySensor, sensorMap);
      }
   }

   private static void readAndCheckJointVelocities(ArrayList<RevoluteJoint> jointsWithVelocitySensor, SensorOutputMapReadOnly sensorMap)
   {
      for (OneDoFJoint joint : jointsWithVelocitySensor)
      {
         double sensorValue = sensorMap.getJointVelocityProcessedOutput(joint);
         double robotJointValue = joint.getQd();
         
         assertEquals(sensorValue, robotJointValue, EPS);
      }
   }

   private static void readAndCheckJointPositions(ArrayList<RevoluteJoint> jointsWithPositionSensor, SensorOutputMapReadOnly sensorMap)
   {
      for (OneDoFJoint joint : jointsWithPositionSensor)
      {
         double sensorValue = sensorMap.getJointPositionProcessedOutput(joint);
         double robotJointValue = joint.getQ();
         
         assertEquals(sensorValue, robotJointValue, EPS);
      }
   }

   private static void fillSensorsWithRandomPositionsAndVelocities(ArrayList<RevoluteJoint> jointsWithPositionSensor,
         ArrayList<RevoluteJoint> jointsWithVelocitySensor, SensorProcessing sensorMap)
   {
      for (OneDoFJoint joint : jointsWithPositionSensor)
      {
         double randPosition = RandomTools.generateRandomDouble(random, -5000.0, 5000.0);
         sensorMap.setJointPositionSensorValue(joint, randPosition);
      }
      
      for (OneDoFJoint joint : jointsWithVelocitySensor)
      {
         double randVelocity = RandomTools.generateRandomDouble(random, -5000.0, 5000.0);
         sensorMap.setJointVelocitySensorValue(joint, randVelocity);
      }
   }

   private static SensorProcessing createJointSensorDataSource(YoVariableRegistry registry, ArrayList<RevoluteJoint> jointsWithPositionSensor,
         ArrayList<RevoluteJoint> jointsWithVelocitySensor)
   {
      StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions = createSensorDefinitions(jointsWithPositionSensor, jointsWithVelocitySensor);

      SensorProcessingConfiguration sensorProcessingConfiguration = new SensorProcessingConfiguration()
      {
         @Override
         public SensorNoiseParameters getSensorNoiseParameters()
         {
            return null;
         }
         
         @Override
         public double getEstimatorDT()
         {
            return 1e-3;
         }
         
         @Override
         public void configureSensorProcessing(SensorProcessing sensorProcessing)
         {
         }
      };
      SensorProcessing sensorDataSource = new SensorProcessing(stateEstimatorSensorDefinitions, sensorProcessingConfiguration, registry);
    
      return sensorDataSource;
   }

   private static StateEstimatorSensorDefinitions createSensorDefinitions(ArrayList<RevoluteJoint> jointsWithPositionSensor,
         ArrayList<RevoluteJoint> jointsWithVelocitySensor)
   {
      StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions = new StateEstimatorSensorDefinitions();
      for (OneDoFJoint joint : jointsWithPositionSensor)
         stateEstimatorSensorDefinitions.addJointSensorDefinition(joint);
      return stateEstimatorSensorDefinitions;
   }

   private static FullInverseDynamicsStructure createFullInverseDynamicsStructure(ScrewTestTools.RandomFloatingChain randomFloatingChain,
         ArrayList<RevoluteJoint> joints)
   {
      int indexOfEstimationParentJoint = RandomTools.generateRandomInt(random, 0, joints.size() - 1);
      RigidBody estimationLink = joints.get(indexOfEstimationParentJoint).getSuccessor();
      SixDoFJoint rootInverseDynamicsJoint = randomFloatingChain.getRootJoint();
      RigidBody elevator = randomFloatingChain.getElevator();
      FullInverseDynamicsStructure inverseDynamicsStructure = new FullInverseDynamicsStructure(elevator, estimationLink, rootInverseDynamicsJoint);
      return inverseDynamicsStructure;
   }
}
