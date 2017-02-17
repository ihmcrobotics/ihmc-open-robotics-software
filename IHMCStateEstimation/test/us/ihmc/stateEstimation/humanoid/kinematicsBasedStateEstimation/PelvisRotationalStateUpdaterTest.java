package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import static org.junit.Assert.fail;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;

public class PelvisRotationalStateUpdaterTest
{
   private static final Vector3D X = new Vector3D(1.0, 0.0, 0.0);
   private static final Vector3D Y = new Vector3D(0.0, 1.0, 0.0);
   private static final Vector3D Z = new Vector3D(0.0, 0.0, 1.0);

   private static final Random random = new Random(2843L);

   private static final double EPS = 1e-10;

   private final List<IMUSensorReadOnly> imuSensors = new ArrayList<>();

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testConstructorWithOneIMU()
   {
      YoVariableRegistry registry = new YoVariableRegistry("Blop");

      Vector3D[] jointAxes = {X, Y, Z, Z, X, Z, Z, X, Y, Y};
      ScrewTestTools.RandomFloatingChain randomFloatingChain = new ScrewTestTools.RandomFloatingChain(random, jointAxes);
      ArrayList<RevoluteJoint> joints = new ArrayList<RevoluteJoint>(randomFloatingChain.getRevoluteJoints());
      
      FullInverseDynamicsStructure inverseDynamicsStructure = createFullInverseDynamicsStructure(randomFloatingChain, joints);

      StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions = new StateEstimatorSensorDefinitions();
      createAndAddIMUDefinitions(joints, stateEstimatorSensorDefinitions, 1);
      
      buildSensorConfigurations(stateEstimatorSensorDefinitions, registry);
      
      try
      {
         new IMUBasedPelvisRotationalStateUpdater(inverseDynamicsStructure, imuSensors, 1.0e-3, registry);
      }
      catch (Exception e)
      {
         fail("Could not create PelvisRotationalStateUpdater. Stack Trace:");
         e.printStackTrace();
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testConstructorWithZeroIMUSensor()
   {
      YoVariableRegistry registry = new YoVariableRegistry("Blop");

      Vector3D[] jointAxes = {X, Y, Z, Z, X, Z, Z, X, Y, Y};
      ScrewTestTools.RandomFloatingChain randomFloatingChain = new ScrewTestTools.RandomFloatingChain(random, jointAxes);
      ArrayList<RevoluteJoint> joints = new ArrayList<RevoluteJoint>(randomFloatingChain.getRevoluteJoints());
      
      FullInverseDynamicsStructure inverseDynamicsStructure = createFullInverseDynamicsStructure(randomFloatingChain, joints);

      StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions = new StateEstimatorSensorDefinitions();
      buildSensorConfigurations(stateEstimatorSensorDefinitions, registry);
      
      PelvisRotationalStateUpdaterInterface pelvisRotationalStateUpdater;
      try
      {
         pelvisRotationalStateUpdater = new IMUBasedPelvisRotationalStateUpdater(inverseDynamicsStructure, imuSensors, 1.0e-3, registry);
      }
      catch (Exception e)
      {
         pelvisRotationalStateUpdater = null;
      }

      if (pelvisRotationalStateUpdater != null)
         fail("RuntimeException expected, no orientation sensor attached to the sensor map.");
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout=300000)
   public void testInitializeAndReadWithOneIMU()
   {
      YoVariableRegistry registry = new YoVariableRegistry("Blop");

      Vector3D[] jointAxes = {X, Y, Z, Z, X, Z, Z, X, Y, Y};
      ScrewTestTools.RandomFloatingChain randomFloatingChain = new ScrewTestTools.RandomFloatingChain(random, jointAxes);
      ArrayList<RevoluteJoint> joints = new ArrayList<RevoluteJoint>(randomFloatingChain.getRevoluteJoints());
      
      FullInverseDynamicsStructure inverseDynamicsStructure = createFullInverseDynamicsStructure(randomFloatingChain, joints);

      StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions = new StateEstimatorSensorDefinitions();
      createAndAddIMUDefinitions(joints, stateEstimatorSensorDefinitions, 1);
      
      SensorProcessing jointAndIMUSensorDataSource = buildSensorConfigurations(stateEstimatorSensorDefinitions, registry);

      PelvisRotationalStateUpdaterInterface pelvisRotationalStateUpdater = new IMUBasedPelvisRotationalStateUpdater(inverseDynamicsStructure, imuSensors, 1.0e-3, registry);

      
      Quaternion rotationExpected = new Quaternion();
      Twist twistExpected = new Twist();
      Quaternion rotationEstimated = new Quaternion();
      Twist twistEstimated = new Twist();
      FloatingInverseDynamicsJoint rootJoint = inverseDynamicsStructure.getRootJoint();

      setRandomRobotConfigurationAndUpdateSensors(joints, inverseDynamicsStructure, stateEstimatorSensorDefinitions, jointAndIMUSensorDataSource);
      
      rootJoint.getRotation(rotationExpected);
      rootJoint.getJointTwist(twistExpected);
      
      // Reset the root joint state configuration so the test fails if the PelvisRotationalStateUpdater actually does not do anything.
      rootJoint.setPositionAndRotation(new RigidBodyTransform());
      rootJoint.setVelocity(new DenseMatrix64F(6, 1), 0);
      
      // Need to initialize the sensor data source manually
      jointAndIMUSensorDataSource.initialize();
      pelvisRotationalStateUpdater.initialize();

      rootJoint.getRotation(rotationEstimated);
      rootJoint.getJointTwist(twistEstimated);
      
      EuclidCoreTestTools.assertQuaternionEqualsUsingDifference(rotationExpected, rotationEstimated, EPS);
      EuclidCoreTestTools.assertTuple3DEquals(twistExpected.getAngularPartCopy(), twistEstimated.getAngularPartCopy(), EPS);
      
      for (int i = 0; i < 1000; i++)
      {
         setRandomRobotConfigurationAndUpdateSensors(joints, inverseDynamicsStructure, stateEstimatorSensorDefinitions, jointAndIMUSensorDataSource);
         
         rootJoint.getRotation(rotationExpected);
         rootJoint.getJointTwist(twistExpected);
         
         // Reset the root joint state configuration so the test fails if the PelvisRotationalStateUpdater actually does not do anything.
         rootJoint.setPositionAndRotation(new RigidBodyTransform());
         rootJoint.setVelocity(new DenseMatrix64F(6, 1), 0);
         
         // Need to run the sensor data source manually
         jointAndIMUSensorDataSource.startComputation(0, 0, -1);
         pelvisRotationalStateUpdater.updateRootJointOrientationAndAngularVelocity();

         rootJoint.getRotation(rotationEstimated);
         rootJoint.getJointTwist(twistEstimated);
         
         EuclidCoreTestTools.assertQuaternionEqualsUsingDifference(rotationExpected, rotationEstimated, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(twistExpected.getAngularPartCopy(), twistEstimated.getAngularPartCopy(), EPS);
      }
   }

   private void setRandomRobotConfigurationAndUpdateSensors(ArrayList<RevoluteJoint> joints, FullInverseDynamicsStructure inverseDynamicsStructure,
         StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions, SensorProcessing jointAndIMUSensorDataSource)
   {
      ScrewTestTools.setRandomPositionAndOrientation(inverseDynamicsStructure.getRootJoint(), random);
      ScrewTestTools.setRandomVelocity(inverseDynamicsStructure.getRootJoint(), random);
      ScrewTestTools.setRandomPositions(joints, random);
      ScrewTestTools.setRandomVelocities(joints, random);
      inverseDynamicsStructure.getElevator().updateFramesRecursively();
      inverseDynamicsStructure.updateInternalState();
      
      for (int i = 0; i < stateEstimatorSensorDefinitions.getIMUSensorDefinitions().size(); i++)
      {
         IMUDefinition imuDefinition = stateEstimatorSensorDefinitions.getIMUSensorDefinitions().get(i);
         IMUSensorReadOnly imuSensor = imuSensors.get(i);
         
         RigidBodyTransform transformFromMeasFrameToWorld = imuSensor.getMeasurementFrame().getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
         RotationMatrix rotationFromMeasFrameToWorld = new RotationMatrix();
         transformFromMeasFrameToWorld.getRotation(rotationFromMeasFrameToWorld);
         jointAndIMUSensorDataSource.setOrientationSensorValue(imuDefinition, rotationFromMeasFrameToWorld);
      }

      for (int i = 0; i < stateEstimatorSensorDefinitions.getIMUSensorDefinitions().size(); i++)
      {
         IMUDefinition imuDefinition = stateEstimatorSensorDefinitions.getIMUSensorDefinitions().get(i);
         RigidBody measurementLink = imuDefinition.getRigidBody();
         Twist twistIMU = new Twist();
         inverseDynamicsStructure.getTwistCalculator().getTwistOfBody(twistIMU, measurementLink);
         twistIMU.changeFrame(imuSensors.get(i).getMeasurementFrame());
         twistIMU.changeBodyFrameNoRelativeTwist(imuSensors.get(i).getMeasurementFrame());
         
         Vector3D sensorValue = twistIMU.getAngularPartCopy();
         jointAndIMUSensorDataSource.setAngularVelocitySensorValue(imuDefinition, sensorValue);
      }
   }
   
   private SensorProcessing buildSensorConfigurations(StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions, YoVariableRegistry registry)
   {
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
      
      imuSensors.clear();
      imuSensors.addAll(sensorDataSource.getIMUProcessedOutputs());
      
      return sensorDataSource;
   }

   private void createAndAddIMUDefinitions(ArrayList<RevoluteJoint> joints, StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions, int numberOfIMUs)
   {
      for (int i = 0; i < numberOfIMUs; i++)
      {
         IMUDefinition imuDefinition = createRandomIMUDefinition(String.valueOf(i), joints);
         stateEstimatorSensorDefinitions.addIMUSensorDefinition(imuDefinition);
      }
   }

   private IMUDefinition createRandomIMUDefinition(String suffix, ArrayList<RevoluteJoint> joints)
   {
      int indexOfIMUParentJoint = RandomTools.generateRandomInt(random, 0, joints.size() - 1);
      RigidBody rigidBody = joints.get(indexOfIMUParentJoint).getSuccessor();
      RigidBodyTransform transformFromIMUToJoint = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
      IMUDefinition imuDefinition = new IMUDefinition("IMU" + suffix, rigidBody, transformFromIMUToJoint);
      return imuDefinition;
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
