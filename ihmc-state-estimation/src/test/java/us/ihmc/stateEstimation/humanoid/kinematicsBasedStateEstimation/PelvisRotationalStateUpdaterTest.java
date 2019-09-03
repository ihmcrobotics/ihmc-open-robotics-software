package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import static us.ihmc.robotics.Assert.fail;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools.RandomFloatingRevoluteJointChain;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class PelvisRotationalStateUpdaterTest
{
   private static final Vector3D X = new Vector3D(1.0, 0.0, 0.0);
   private static final Vector3D Y = new Vector3D(0.0, 1.0, 0.0);
   private static final Vector3D Z = new Vector3D(0.0, 0.0, 1.0);

   private static final Random random = new Random(2843L);

   private static final double EPS = 1e-10;

   private final List<IMUSensorReadOnly> imuSensors = new ArrayList<>();

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

	@Test
   public void testConstructorWithOneIMU()
   {
      YoVariableRegistry registry = new YoVariableRegistry("Blop");

      Vector3D[] jointAxes = {X, Y, Z, Z, X, Z, Z, X, Y, Y};
      RandomFloatingRevoluteJointChain randomFloatingChain = new RandomFloatingRevoluteJointChain(random, jointAxes);
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

	@Test
   public void testConstructorWithZeroIMUSensor()
   {
      YoVariableRegistry registry = new YoVariableRegistry("Blop");

      Vector3D[] jointAxes = {X, Y, Z, Z, X, Z, Z, X, Y, Y};
      RandomFloatingRevoluteJointChain randomFloatingChain = new RandomFloatingRevoluteJointChain(random, jointAxes);
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

	@Test
   public void testInitializeAndReadWithOneIMU()
   {
      YoVariableRegistry registry = new YoVariableRegistry("Blop");

      Vector3D[] jointAxes = {X, Y, Z, Z, X, Z, Z, X, Y, Y};
      RandomFloatingRevoluteJointChain randomFloatingChain = new RandomFloatingRevoluteJointChain(random, jointAxes);
      ArrayList<RevoluteJoint> joints = new ArrayList<RevoluteJoint>(randomFloatingChain.getRevoluteJoints());
      
      FullInverseDynamicsStructure inverseDynamicsStructure = createFullInverseDynamicsStructure(randomFloatingChain, joints);

      StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions = new StateEstimatorSensorDefinitions();
      createAndAddIMUDefinitions(joints, stateEstimatorSensorDefinitions, 1);
      
      SensorProcessing jointAndIMUSensorDataSource = buildSensorConfigurations(stateEstimatorSensorDefinitions, registry);

      PelvisRotationalStateUpdaterInterface pelvisRotationalStateUpdater = new IMUBasedPelvisRotationalStateUpdater(inverseDynamicsStructure, imuSensors, 1.0e-3, registry);
      new DefaultParameterReader().readParametersInRegistry(registry);

      
      Quaternion rotationExpected = new Quaternion();
      Twist twistExpected = new Twist();
      Quaternion rotationEstimated = new Quaternion();
      Twist twistEstimated = new Twist();
      FloatingJointBasics rootJoint = inverseDynamicsStructure.getRootJoint();

      setRandomRobotConfigurationAndUpdateSensors(joints, inverseDynamicsStructure, stateEstimatorSensorDefinitions, jointAndIMUSensorDataSource);
      
      rotationExpected.set(rootJoint.getJointPose().getOrientation());
      twistExpected.setIncludingFrame(rootJoint.getJointTwist());
      
      // Reset the root joint state configuration so the test fails if the PelvisRotationalStateUpdater actually does not do anything.
      rootJoint.setJointConfiguration(new RigidBodyTransform());
      rootJoint.setJointVelocity(0, new DenseMatrix64F(6, 1));
      
      // Need to initialize the sensor data source manually
      jointAndIMUSensorDataSource.initialize();
      pelvisRotationalStateUpdater.initialize();

      rotationEstimated.set(rootJoint.getJointPose().getOrientation());
      twistEstimated.setIncludingFrame(rootJoint.getJointTwist());
      
      EuclidCoreTestTools.assertQuaternionGeometricallyEquals(rotationExpected, rotationEstimated, EPS);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(twistExpected.getAngularPart()), new Vector3D(twistEstimated.getAngularPart()), EPS);
      
      for (int i = 0; i < 1000; i++)
      {
         setRandomRobotConfigurationAndUpdateSensors(joints, inverseDynamicsStructure, stateEstimatorSensorDefinitions, jointAndIMUSensorDataSource);
         
         rotationExpected.set(rootJoint.getJointPose().getOrientation());
         twistExpected.setIncludingFrame(rootJoint.getJointTwist());
         
         // Reset the root joint state configuration so the test fails if the PelvisRotationalStateUpdater actually does not do anything.
         rootJoint.setJointConfiguration(new RigidBodyTransform());
         rootJoint.setJointVelocity(0, new DenseMatrix64F(6, 1));
         
         // Need to run the sensor data source manually
         jointAndIMUSensorDataSource.startComputation(0, 0, -1);
         pelvisRotationalStateUpdater.updateRootJointOrientationAndAngularVelocity();

         rotationEstimated.set(rootJoint.getJointPose().getOrientation());
         twistEstimated.setIncludingFrame(rootJoint.getJointTwist());
         
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(rotationExpected, rotationEstimated, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(twistExpected.getAngularPart()), new Vector3D(twistEstimated.getAngularPart()), EPS);
      }
   }

   private void setRandomRobotConfigurationAndUpdateSensors(ArrayList<RevoluteJoint> joints, FullInverseDynamicsStructure inverseDynamicsStructure,
         StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions, SensorProcessing jointAndIMUSensorDataSource)
   {
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, inverseDynamicsStructure.getRootJoint());
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, inverseDynamicsStructure.getRootJoint());
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -Math.PI / 2.0, Math.PI / 2.0, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
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
         RigidBodyBasics measurementLink = imuDefinition.getRigidBody();
         Twist twistIMU = new Twist();
         measurementLink.getBodyFixedFrame().getTwistOfFrame(twistIMU);
         twistIMU.changeFrame(imuSensors.get(i).getMeasurementFrame());
         twistIMU.setBodyFrame(imuSensors.get(i).getMeasurementFrame());
         
         Vector3D sensorValue = new Vector3D(twistIMU.getAngularPart());
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
      int indexOfIMUParentJoint = RandomNumbers.nextInt(random, 0, joints.size() - 1);
      RigidBodyBasics rigidBody = joints.get(indexOfIMUParentJoint).getSuccessor();
      RigidBodyTransform transformFromIMUToJoint = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      IMUDefinition imuDefinition = new IMUDefinition("IMU" + suffix, rigidBody, transformFromIMUToJoint);
      return imuDefinition;
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
