package us.ihmc.darpaRoboticsChallenge.stateEstimation.kinematicsBasedStateEstimator;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.darpaRoboticsChallenge.stateEstimation.DRCSimulatedSensorNoiseParameters;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.SensorFilterParameters;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTestTools;
import us.ihmc.utilities.screwTheory.SixDoFJoint;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class JointStateUpdaterTest
{
   private static final Vector3d X = new Vector3d(1.0, 0.0, 0.0);
   private static final Vector3d Y = new Vector3d(0.0, 1.0, 0.0);
   private static final Vector3d Z = new Vector3d(0.0, 0.0, 1.0);

   private static final Random random = new Random(1776L);
   private static final double EPS = 1e-10;
   
   @Test
   public void testConstructorNormalCase()
   {
      YoVariableRegistry registry = new YoVariableRegistry("Blop");
      
      Vector3d[] jointAxes = {X, Y, Z, Z, X, Z, Z, X, Y, Y};
      ScrewTestTools.RandomFloatingChain randomFloatingChain = new ScrewTestTools.RandomFloatingChain(random, jointAxes);
      ArrayList<RevoluteJoint> joints = new ArrayList<RevoluteJoint>(randomFloatingChain.getRevoluteJoints());
      
      FullInverseDynamicsStructure inverseDynamicsStructure = createFullInverseDynamicsStructure(randomFloatingChain, joints);

      ArrayList<RevoluteJoint> jointsWithPositionSensor = new ArrayList<RevoluteJoint>(joints);
      ArrayList<RevoluteJoint> jointsWithVelocitySensor = new ArrayList<RevoluteJoint>(joints);
      SensorProcessing sensorMap = createJointSensorDataSource(registry, jointsWithPositionSensor, jointsWithVelocitySensor);
      
      try
      {
         new JointStateUpdater(inverseDynamicsStructure, sensorMap, registry);
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
      YoVariableRegistry registry = new YoVariableRegistry("Blop");
      
      Vector3d[] jointAxes = {X, Y, Z, Z, X, Z, Z, X, Y, Y};
      ScrewTestTools.RandomFloatingChain randomFloatingChain = new ScrewTestTools.RandomFloatingChain(random, jointAxes);
      ArrayList<RevoluteJoint> joints = new ArrayList<RevoluteJoint>(randomFloatingChain.getRevoluteJoints());
      
      FullInverseDynamicsStructure inverseDynamicsStructure = createFullInverseDynamicsStructure(randomFloatingChain, joints);

      // Test constructor is working for normal case
      ArrayList<RevoluteJoint> jointsWithPositionSensor = new ArrayList<RevoluteJoint>(joints);
      ArrayList<RevoluteJoint> jointsWithVelocitySensor = new ArrayList<RevoluteJoint>(joints);
      SensorProcessing sensorMap = createJointSensorDataSource(registry, jointsWithPositionSensor, jointsWithVelocitySensor);
      
      JointStateUpdater jointStateUpdater = new JointStateUpdater(inverseDynamicsStructure, sensorMap, registry);
      
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
      
      SensorFilterParameters sensorFilterParameters = createParametersForNoFiltering();
      
      SensorNoiseParameters sensorNoiseParameters = DRCSimulatedSensorNoiseParameters.createNoiseParametersForEstimatorJerryTuningSeptember2013();
      
      SensorProcessing sensorProcessing = new SensorProcessing(stateEstimatorSensorDefinitions, sensorFilterParameters, sensorNoiseParameters, registry);
      return sensorProcessing;
   }

   private static SensorFilterParameters createParametersForNoFiltering()
   {
      double updateDT = 1e-3;
      SensorFilterParameters sensorFilterParameters = new SensorFilterParameters(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, 0.0, updateDT, false, false, false, Double.POSITIVE_INFINITY, null);
      return sensorFilterParameters;
   }

   private static StateEstimatorSensorDefinitions createSensorDefinitions(ArrayList<RevoluteJoint> jointsWithPositionSensor,
         ArrayList<RevoluteJoint> jointsWithVelocitySensor)
   {
      StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions = new StateEstimatorSensorDefinitions();
      for (OneDoFJoint joint : jointsWithPositionSensor)
         stateEstimatorSensorDefinitions.addJointPositionSensorDefinition(joint);
      for (OneDoFJoint joint : jointsWithVelocitySensor)
         stateEstimatorSensorDefinitions.addJointVelocitySensorDefinition(joint);
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
