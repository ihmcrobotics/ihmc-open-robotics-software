package us.ihmc.darpaRoboticsChallenge.stateEstimation.kinematicsBasedStateEstimator;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.sensorProcessing.simulatedSensors.JointAndIMUSensorMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorFilterParameters;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.JointAndIMUSensorDataSource;
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
      JointAndIMUSensorDataSource jointAndIMUSensorDataSource = createJointSensorDataSource(registry, jointsWithPositionSensor, jointsWithVelocitySensor);
      
      try
      {
         JointAndIMUSensorMap sensorMap = jointAndIMUSensorDataSource.getSensorMap();
         new JointStateUpdater(inverseDynamicsStructure, sensorMap, registry);
      }
      catch (Exception e)
      {
         fail("Could not create JointStateUpdater. StackTrace:");
         e.printStackTrace();
      }
   }

   @Test
   public void testConstructorNotEnoughPositionSensors()
   {
      YoVariableRegistry registry = new YoVariableRegistry("Blop");
      
      Vector3d[] jointAxes = {X, Y, Z, Z, X, Z, Z, X, Y, Y};
      ScrewTestTools.RandomFloatingChain randomFloatingChain = new ScrewTestTools.RandomFloatingChain(random, jointAxes);
      ArrayList<RevoluteJoint> joints = new ArrayList<RevoluteJoint>(randomFloatingChain.getRevoluteJoints());
      
      FullInverseDynamicsStructure inverseDynamicsStructure = createFullInverseDynamicsStructure(randomFloatingChain, joints);

      ArrayList<RevoluteJoint> jointsWithPositionSensor = new ArrayList<RevoluteJoint>(joints.subList(0, joints.size() - 1));
      ArrayList<RevoluteJoint> jointsWithVelocitySensor = new ArrayList<RevoluteJoint>(joints);
      JointAndIMUSensorDataSource jointAndIMUSensorDataSource = createJointSensorDataSource(registry, jointsWithPositionSensor, jointsWithVelocitySensor);
      
      JointStateUpdater jointStateUpdater;
      try
      {
         JointAndIMUSensorMap sensorMap = jointAndIMUSensorDataSource.getSensorMap();
         jointStateUpdater = new JointStateUpdater(inverseDynamicsStructure, sensorMap, registry);
      }
      catch (Exception e)
      {
         jointStateUpdater = null;
      }
      
      if (jointStateUpdater != null)
         fail("RuntimeException expected, not enough joint position sensors to create the JointStateUpdater.");
   }

   @Test
   public void testConstructorNotEnoughVelocitySensors()
   {
      YoVariableRegistry registry = new YoVariableRegistry("Blop");
      
      Vector3d[] jointAxes = {X, Y, Z, Z, X, Z, Z, X, Y, Y};
      ScrewTestTools.RandomFloatingChain randomFloatingChain = new ScrewTestTools.RandomFloatingChain(random, jointAxes);
      ArrayList<RevoluteJoint> joints = new ArrayList<RevoluteJoint>(randomFloatingChain.getRevoluteJoints());
      
      FullInverseDynamicsStructure inverseDynamicsStructure = createFullInverseDynamicsStructure(randomFloatingChain, joints);

      ArrayList<RevoluteJoint> jointsWithPositionSensor = new ArrayList<RevoluteJoint>(joints);
      ArrayList<RevoluteJoint> jointsWithVelocitySensor = new ArrayList<RevoluteJoint>(joints.subList(0, joints.size() - 1));
      JointAndIMUSensorDataSource jointAndIMUSensorDataSource = createJointSensorDataSource(registry, jointsWithPositionSensor, jointsWithVelocitySensor);
      
      JointStateUpdater jointStateUpdater;
      try
      {
         JointAndIMUSensorMap sensorMap = jointAndIMUSensorDataSource.getSensorMap();
         jointStateUpdater = new JointStateUpdater(inverseDynamicsStructure, sensorMap, registry);
      }
      catch (Exception e)
      {
         jointStateUpdater = null;
      }
      
      if (jointStateUpdater != null)
         fail("RuntimeException expected, not enough joint velocity sensors to create the JointStateUpdater.");
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
      JointAndIMUSensorDataSource jointAndIMUSensorDataSource = createJointSensorDataSource(registry, jointsWithPositionSensor, jointsWithVelocitySensor);
      
      JointAndIMUSensorMap sensorMap = jointAndIMUSensorDataSource.getSensorMap();
      JointStateUpdater jointStateUpdater = new JointStateUpdater(inverseDynamicsStructure, sensorMap, registry);
      
      fillSensorsWithRandomPositionsAndVelocities(jointsWithPositionSensor, jointsWithVelocitySensor, jointAndIMUSensorDataSource);
      
      jointStateUpdater.initialize();
      
      readAndCheckJointPositions(jointsWithPositionSensor, jointAndIMUSensorDataSource);
      readAndCheckJointVelocities(jointsWithVelocitySensor, jointAndIMUSensorDataSource);
      
      for (int i = 0; i < 1000; i++)
      {
         fillSensorsWithRandomPositionsAndVelocities(jointsWithPositionSensor, jointsWithVelocitySensor, jointAndIMUSensorDataSource);
         
         jointStateUpdater.updateJointState();
         
         readAndCheckJointPositions(jointsWithPositionSensor, jointAndIMUSensorDataSource);
         readAndCheckJointVelocities(jointsWithVelocitySensor, jointAndIMUSensorDataSource);
      }
   }

   private static void readAndCheckJointVelocities(ArrayList<RevoluteJoint> jointsWithVelocitySensor, JointAndIMUSensorDataSource jointAndIMUSensorDataSource)
   {
      for (OneDoFJoint joint : jointsWithVelocitySensor)
      {
         double sensorValue = jointAndIMUSensorDataSource.getSensorMap().getJointVelocitySensorPort(joint).getData()[0];
         double robotJointValue = joint.getQd();
         
         assertEquals(sensorValue, robotJointValue, EPS);
      }
   }

   private static void readAndCheckJointPositions(ArrayList<RevoluteJoint> jointsWithPositionSensor, JointAndIMUSensorDataSource jointAndIMUSensorDataSource)
   {
      for (OneDoFJoint joint : jointsWithPositionSensor)
      {
         double sensorValue = jointAndIMUSensorDataSource.getSensorMap().getJointPositionSensorPort(joint).getData()[0];
         double robotJointValue = joint.getQ();
         
         assertEquals(sensorValue, robotJointValue, EPS);
      }
   }

   private static void fillSensorsWithRandomPositionsAndVelocities(ArrayList<RevoluteJoint> jointsWithPositionSensor,
         ArrayList<RevoluteJoint> jointsWithVelocitySensor, JointAndIMUSensorDataSource jointAndIMUSensorDataSource)
   {
      for (OneDoFJoint joint : jointsWithPositionSensor)
      {
         double randPosition = RandomTools.generateRandomDouble(random, -5000.0, 5000.0);
         jointAndIMUSensorDataSource.setJointPositionSensorValue(joint, randPosition);
      }
      
      for (OneDoFJoint joint : jointsWithVelocitySensor)
      {
         double randVelocity = RandomTools.generateRandomDouble(random, -5000.0, 5000.0);
         jointAndIMUSensorDataSource.setJointVelocitySensorValue(joint, randVelocity);
      }
   }

   private static JointAndIMUSensorDataSource createJointSensorDataSource(YoVariableRegistry registry, ArrayList<RevoluteJoint> jointsWithPositionSensor,
         ArrayList<RevoluteJoint> jointsWithVelocitySensor)
   {
      StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions = createSensorDefinitions(jointsWithPositionSensor, jointsWithVelocitySensor);
      
      SensorFilterParameters sensorFilterParameters = createParametersForNoFiltering();
      
      JointAndIMUSensorDataSource jointAndIMUSensorDataSource = new JointAndIMUSensorDataSource(stateEstimatorSensorDefinitions, sensorFilterParameters, registry);
      return jointAndIMUSensorDataSource;
   }

   private static SensorFilterParameters createParametersForNoFiltering()
   {
      double updateDT = 1e-3;
      SensorFilterParameters sensorFilterParameters = new SensorFilterParameters(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, 0.0, updateDT, false, false);
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
