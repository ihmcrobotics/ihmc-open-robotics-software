package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.fail;

import java.util.ArrayList;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools.RandomFloatingRevoluteJointChain;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;

public class JointStateUpdaterTest
{
   private static final Vector3D X = new Vector3D(1.0, 0.0, 0.0);
   private static final Vector3D Y = new Vector3D(0.0, 1.0, 0.0);
   private static final Vector3D Z = new Vector3D(0.0, 0.0, 1.0);

   private static final Random random = new Random(1776L);
   private static final double EPS = 1e-10;

	@Test
   public void testConstructorNormalCase()
   {
      YoRegistry registry = new YoRegistry("Blop");
      
      Vector3D[] jointAxes = {X, Y, Z, Z, X, Z, Z, X, Y, Y};
      RandomFloatingRevoluteJointChain randomFloatingChain = new RandomFloatingRevoluteJointChain(random, jointAxes);
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

	@Test
   public void testInitializingAndReading()
   {
      YoRegistry registry = new YoRegistry("Blop");
      
      Vector3D[] jointAxes = {X, Y, Z, Z, X, Z, Z, X, Y, Y};
      RandomFloatingRevoluteJointChain randomFloatingChain = new RandomFloatingRevoluteJointChain(random, jointAxes);
      ArrayList<RevoluteJoint> joints = new ArrayList<RevoluteJoint>(randomFloatingChain.getRevoluteJoints());
      
      FullInverseDynamicsStructure inverseDynamicsStructure = createFullInverseDynamicsStructure(randomFloatingChain, joints);

      // Test constructor is working for normal case
      ArrayList<RevoluteJoint> jointsWithPositionSensor = new ArrayList<RevoluteJoint>(joints);
      ArrayList<RevoluteJoint> jointsWithVelocitySensor = new ArrayList<RevoluteJoint>(joints);
      SensorProcessing sensorMap = createJointSensorDataSource(registry, jointsWithPositionSensor, jointsWithVelocitySensor);
      
      JointStateUpdater jointStateUpdater = new JointStateUpdater(inverseDynamicsStructure, sensorMap, null, registry);
      
      fillSensorsWithRandomPositionsAndVelocities(jointsWithPositionSensor, jointsWithVelocitySensor, sensorMap);

      new DefaultParameterReader().readParametersInRegistry(registry);
      
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
      for (OneDoFJointBasics joint : jointsWithVelocitySensor)
      {
         double sensorValue = sensorMap.getOneDoFJointOutput(joint).getVelocity();
         double robotJointValue = joint.getQd();
         
         assertEquals(sensorValue, robotJointValue, EPS);
      }
   }

   private static void readAndCheckJointPositions(ArrayList<RevoluteJoint> jointsWithPositionSensor, SensorOutputMapReadOnly sensorMap)
   {
      for (OneDoFJointBasics joint : jointsWithPositionSensor)
      {
         double sensorValue = sensorMap.getOneDoFJointOutput(joint).getPosition();
         double robotJointValue = joint.getQ();
         
         assertEquals(sensorValue, robotJointValue, EPS);
      }
   }

   private static void fillSensorsWithRandomPositionsAndVelocities(ArrayList<RevoluteJoint> jointsWithPositionSensor,
         ArrayList<RevoluteJoint> jointsWithVelocitySensor, SensorProcessing sensorMap)
   {
      for (OneDoFJointBasics joint : jointsWithPositionSensor)
      {
         double randPosition = RandomNumbers.nextDouble(random, -5000.0, 5000.0);
         sensorMap.setJointPositionSensorValue(joint, randPosition);
      }
      
      for (OneDoFJointBasics joint : jointsWithVelocitySensor)
      {
         double randVelocity = RandomNumbers.nextDouble(random, -5000.0, 5000.0);
         sensorMap.setJointVelocitySensorValue(joint, randVelocity);
      }
   }

   private static SensorProcessing createJointSensorDataSource(YoRegistry registry, ArrayList<RevoluteJoint> jointsWithPositionSensor,
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
      for (OneDoFJointBasics joint : jointsWithPositionSensor)
         stateEstimatorSensorDefinitions.addJointSensorDefinition(joint);
      return stateEstimatorSensorDefinitions;
   }

   private static FullInverseDynamicsStructure createFullInverseDynamicsStructure(RandomFloatingRevoluteJointChain randomFloatingChain,
         ArrayList<RevoluteJoint> joints)
   {
      int indexOfEstimationParentJoint = RandomNumbers.nextInt(random, 0, joints.size() - 1);
      RigidBodyBasics estimationLink = joints.get(indexOfEstimationParentJoint).getSuccessor();
      SixDoFJoint rootInverseDynamicsJoint = randomFloatingChain.getRootJoint();
      RigidBodyBasics elevator = randomFloatingChain.getElevator();
      FullInverseDynamicsStructure inverseDynamicsStructure = new FullInverseDynamicsStructure(elevator, estimationLink, rootInverseDynamicsJoint);
      return inverseDynamicsStructure;
   }
}
