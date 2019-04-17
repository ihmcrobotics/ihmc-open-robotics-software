package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.ArrayList;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.IMUMount;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SimulatedSensorHolderAndReaderFromRobotFactory implements SensorReaderFactory
{
   private final YoVariableRegistry registry = new YoVariableRegistry("SensorReaderFactory");
   private final Robot robot;

   private final ArrayList<IMUMount> imuMounts = new ArrayList<IMUMount>();
   private final ArrayList<WrenchCalculatorInterface> groundContactPointBasedWrenchCalculators = new ArrayList<WrenchCalculatorInterface>();

   private SimulatedSensorHolderAndReader simulatedSensorHolderAndReader;
   private StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions;
   private final SensorProcessingConfiguration sensorProcessingConfiguration;

   public SimulatedSensorHolderAndReaderFromRobotFactory(Robot robot, SensorProcessingConfiguration sensorProcessingConfiguration)
   {
      this.robot = robot;
      this.sensorProcessingConfiguration = sensorProcessingConfiguration;

      robot.getIMUMounts(imuMounts);
      robot.getForceSensors(groundContactPointBasedWrenchCalculators);
   }

   @Override
   public void build(FloatingJointBasics rootJoint, IMUDefinition[] imuDefinition, ForceSensorDefinition[] forceSensorDefinitions,
                     JointDesiredOutputList estimatorDesiredJointDataHolder, YoVariableRegistry parentRegistry)
   {
      ArrayList<Joint> rootJoints = robot.getRootJoints();
      if (rootJoints.size() > 1)
      {
         throw new RuntimeException("Robot has more than 1 rootJoint");
      }

      final Joint scsRootJoint = rootJoints.get(0);
      if (!(scsRootJoint instanceof FloatingJoint))
         throw new RuntimeException("Not FloatingJoint rootjoint found");

      SCSToInverseDynamicsJointMap scsToInverseDynamicsJointMap = SCSToInverseDynamicsJointMap.createByName((FloatingJoint) scsRootJoint, rootJoint);
      StateEstimatorSensorDefinitionsFromRobotFactory stateEstimatorSensorDefinitionsFromRobotFactory = new StateEstimatorSensorDefinitionsFromRobotFactory(scsToInverseDynamicsJointMap,
                                                                                                                                                            imuMounts,
                                                                                                                                                            groundContactPointBasedWrenchCalculators);

      this.stateEstimatorSensorDefinitions = stateEstimatorSensorDefinitionsFromRobotFactory.getStateEstimatorSensorDefinitions();
      Map<IMUMount, IMUDefinition> imuDefinitions = stateEstimatorSensorDefinitionsFromRobotFactory.getIMUDefinitions();
      Map<WrenchCalculatorInterface, ForceSensorDefinition> forceSensors = stateEstimatorSensorDefinitionsFromRobotFactory.getForceSensorDefinitions();
      this.simulatedSensorHolderAndReader = new SimulatedSensorHolderAndReader(stateEstimatorSensorDefinitions, sensorProcessingConfiguration,
                                                                               robot.getYoTime(), registry);

      createAndAddOrientationSensors(imuDefinitions, registry);
      createAndAddAngularVelocitySensors(imuDefinitions, registry);
      createAndAddForceSensors(forceSensors, registry);
      createAndAddLinearAccelerationSensors(imuDefinitions, registry);
      createAndAddOneDoFPositionAndVelocitySensors(scsToInverseDynamicsJointMap);

      parentRegistry.addChild(registry);
   }

   private void createAndAddForceSensors(Map<WrenchCalculatorInterface, ForceSensorDefinition> forceSensorDefinitions2, YoVariableRegistry registry)
   {
      for (Entry<WrenchCalculatorInterface, ForceSensorDefinition> forceSensorDefinitionEntry : forceSensorDefinitions2.entrySet())
      {
         WrenchCalculatorInterface groundContactPointBasedWrenchCalculator = forceSensorDefinitionEntry.getKey();
         simulatedSensorHolderAndReader.addForceTorqueSensorPort(forceSensorDefinitionEntry.getValue(), groundContactPointBasedWrenchCalculator);
      }
   }

   @Override
   public SimulatedSensorHolderAndReader getSensorReader()
   {
      return simulatedSensorHolderAndReader;
   }

   private void createAndAddOneDoFPositionAndVelocitySensors(SCSToInverseDynamicsJointMap scsToInverseDynamicsJointMap)
   {
      ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints = new ArrayList<OneDegreeOfFreedomJoint>(scsToInverseDynamicsJointMap.getSCSOneDegreeOfFreedomJoints());

      for (OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint : oneDegreeOfFreedomJoints)
      {
         OneDoFJointBasics oneDoFJoint = scsToInverseDynamicsJointMap.getInverseDynamicsOneDoFJoint(oneDegreeOfFreedomJoint);

         simulatedSensorHolderAndReader.addJointPositionSensorPort(oneDoFJoint, oneDegreeOfFreedomJoint.getQYoVariable());
         simulatedSensorHolderAndReader.addJointVelocitySensorPort(oneDoFJoint, oneDegreeOfFreedomJoint.getQDYoVariable());
         simulatedSensorHolderAndReader.addJointTorqueSensorPort(oneDoFJoint, oneDegreeOfFreedomJoint.getTauYoVariable());
      }
   }

   private void createAndAddOrientationSensors(Map<IMUMount, IMUDefinition> imuDefinitions, YoVariableRegistry registry)
   {
      Set<IMUMount> imuMounts = imuDefinitions.keySet();

      for (IMUMount imuMount : imuMounts)
      {
         IMUDefinition imuDefinition = imuDefinitions.get(imuMount);

         simulatedSensorHolderAndReader.addOrientationSensorPort(imuDefinition, new QuaternionProvider()
         {
            private final Quaternion orientation = new Quaternion();

            @Override
            public QuaternionReadOnly getValue()
            {
               imuMount.getOrientation(orientation);
               return orientation;
            }
         });
      }
   }

   private void createAndAddAngularVelocitySensors(Map<IMUMount, IMUDefinition> imuDefinitions, YoVariableRegistry registry)
   {
      Set<IMUMount> imuMounts = imuDefinitions.keySet();

      for (IMUMount imuMount : imuMounts)
      {
         IMUDefinition imuDefinition = imuDefinitions.get(imuMount);

         simulatedSensorHolderAndReader.addAngularVelocitySensorPort(imuDefinition, new Vector3DProvider()
         {
            private final Vector3D angularVelocity = new Vector3D();

            @Override
            public Vector3DReadOnly getValue()
            {
               imuMount.getAngularVelocityInBody(angularVelocity);
               return angularVelocity;
            }
         });
      }
   }

   private void createAndAddLinearAccelerationSensors(Map<IMUMount, IMUDefinition> imuDefinitions, YoVariableRegistry registry)
   {
      Set<IMUMount> imuMounts = imuDefinitions.keySet();

      for (IMUMount imuMount : imuMounts)
      {
         IMUDefinition imuDefinition = imuDefinitions.get(imuMount);

         simulatedSensorHolderAndReader.addLinearAccelerationSensorPort(imuDefinition, new Vector3DProvider()
         {
            private final Vector3D linearAcceleration = new Vector3D();

            @Override
            public Vector3DReadOnly getValue()
            {
               imuMount.getLinearAccelerationInBody(linearAcceleration);
               return linearAcceleration;
            }
         });
      }
   }

   @Override
   public StateEstimatorSensorDefinitions getStateEstimatorSensorDefinitions()
   {
      return stateEstimatorSensorDefinitions;
   }

   @Override
   public boolean useStateEstimator()
   {
      return true;
   }
}
