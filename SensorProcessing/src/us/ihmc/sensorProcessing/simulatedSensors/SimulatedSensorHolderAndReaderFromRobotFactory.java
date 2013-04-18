package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Set;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.sensorProcessing.signalCorruption.GaussianOrientationCorruptor;
import us.ihmc.sensorProcessing.signalCorruption.GaussianVectorCorruptor;
import us.ihmc.sensorProcessing.signalCorruption.RandomWalkBiasVectorCorruptor;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;

import com.yobotics.simulationconstructionset.IMUMount;
import com.yobotics.simulationconstructionset.KinematicPoint;
import com.yobotics.simulationconstructionset.OneDegreeOfFreedomJoint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class SimulatedSensorHolderAndReaderFromRobotFactory
{
   private final SCSToInverseDynamicsJointMap scsToInverseDynamicsJointMap;
   private final Robot robot;
   private final double controlDT;
   private final SensorNoiseParameters sensorNoiseParameters;

   private final LinkedHashMap<IMUMount, IMUDefinition> imuDefinitions;
   private final LinkedHashMap<KinematicPoint, PointVelocitySensorDefinition> pointVelocitySensorDefinitions;
   private final SimulatedSensorHolderAndReader simulatedSensorHolderAndReader;

   public SimulatedSensorHolderAndReaderFromRobotFactory(SCSToInverseDynamicsJointMap scsToInverseDynamicsJointMap, Robot robot,
           SensorNoiseParameters sensorNoiseParameters, double controlDT, ArrayList<IMUMount> imuMounts, ArrayList<KinematicPoint> velocityPoints,
           YoVariableRegistry registry)
   {
      this.scsToInverseDynamicsJointMap = scsToInverseDynamicsJointMap;
      this.robot = robot;
      this.sensorNoiseParameters = sensorNoiseParameters;

      this.controlDT = controlDT;

      this.imuDefinitions = generateIMUDefinitions(imuMounts);
      this.pointVelocitySensorDefinitions = generatePointVelocitySensorDefinitions(velocityPoints);

      simulatedSensorHolderAndReader = new SimulatedSensorHolderAndReader();

      createAndAddOneDoFPositionAndVelocitySensors();
      createAndAddOrientationSensors(imuDefinitions, registry);
      createAndAddAngularVelocitySensors(imuDefinitions, registry);
      createAndAddLinearAccelerationSensors(imuDefinitions, registry);
      createAndAddPointVelocitySensors(velocityPoints, registry);
   }

   public SimulatedSensorHolderAndReader getSimulatedSensorHolderAndReader()
   {
      return simulatedSensorHolderAndReader;
   }

   private LinkedHashMap<IMUMount, IMUDefinition> generateIMUDefinitions(ArrayList<IMUMount> imuMounts)
   {
      LinkedHashMap<IMUMount, IMUDefinition> imuDefinitions = new LinkedHashMap<IMUMount, IMUDefinition>();

      for (IMUMount imuMount : imuMounts)
      {
         RigidBody rigidBody = scsToInverseDynamicsJointMap.getRigidBody(imuMount.getParentJoint());
         Transform3D transformFromMountToJoint = new Transform3D();
         imuMount.getTransformFromMountToJoint(transformFromMountToJoint);
         IMUDefinition imuDefinition = new IMUDefinition(imuMount.getName(), rigidBody, transformFromMountToJoint);
         imuDefinitions.put(imuMount, imuDefinition);
      }

      return imuDefinitions;
   }


   private LinkedHashMap<KinematicPoint, PointVelocitySensorDefinition> generatePointVelocitySensorDefinitions(ArrayList<KinematicPoint> velocityPoints)
   {
      LinkedHashMap<KinematicPoint, PointVelocitySensorDefinition> pointVelocitySensorDefinitions = new LinkedHashMap<KinematicPoint,
                                                                                                       PointVelocitySensorDefinition>();

      for (KinematicPoint kinematicPoint : velocityPoints)
      {
         RigidBody rigidBody = scsToInverseDynamicsJointMap.getRigidBody(kinematicPoint.getParentJoint());
         Vector3d offset = new Vector3d();
         kinematicPoint.getOffset(offset);

         PointVelocitySensorDefinition pointVelocitySensorDefinition = new PointVelocitySensorDefinition(kinematicPoint.getName(), rigidBody, offset);
         pointVelocitySensorDefinitions.put(kinematicPoint, pointVelocitySensorDefinition);
      }

      return pointVelocitySensorDefinitions;
   }


   public void createAndAddOneDoFPositionAndVelocitySensors()
   {
      ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      robot.getAllOneDegreeOfFreedomJoints(oneDegreeOfFreedomJoints);

      for (OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint : oneDegreeOfFreedomJoints)
      {
         OneDoFJoint oneDoFJoint = scsToInverseDynamicsJointMap.getInverseDynamicsOneDoFJoint(oneDegreeOfFreedomJoint);

         SimulatedOneDoFJointPositionSensor positionSensor = new SimulatedOneDoFJointPositionSensor(oneDegreeOfFreedomJoint.getName(), oneDegreeOfFreedomJoint);
         simulatedSensorHolderAndReader.addJointPositionSensorPort(oneDoFJoint, positionSensor);

         SimulatedOneDoFJointVelocitySensor velocitySensor = new SimulatedOneDoFJointVelocitySensor(oneDegreeOfFreedomJoint.getName(), oneDegreeOfFreedomJoint);
         simulatedSensorHolderAndReader.addJointVelocitySensorPort(oneDoFJoint, velocitySensor);
      }
   }


   public void createAndAddOrientationSensors(LinkedHashMap<IMUMount, IMUDefinition> imuDefinitions, YoVariableRegistry registry)
   {
      Set<IMUMount> imuMounts = imuDefinitions.keySet();

      for (IMUMount imuMount : imuMounts)
      {
         String sensorName = imuMount.getName() + "Orientation";
         IMUDefinition imuDefinition = imuDefinitions.get(imuMount);

         SimulatedOrientationSensorFromRobot orientationSensor = new SimulatedOrientationSensorFromRobot(sensorName, imuMount, registry);

         if (sensorNoiseParameters != null)
         {
            double orientationMeasurementStandardDeviation = sensorNoiseParameters.getOrientationMeasurementStandardDeviation();

            GaussianOrientationCorruptor orientationCorruptor = new GaussianOrientationCorruptor(sensorName, 12334255L, registry);
            orientationCorruptor.setStandardDeviation(orientationMeasurementStandardDeviation);
            orientationSensor.addSignalCorruptor(orientationCorruptor);
         }

         simulatedSensorHolderAndReader.addOrientationSensorPort(imuDefinition, orientationSensor);
      }
   }


   public void createAndAddAngularVelocitySensors(LinkedHashMap<IMUMount, IMUDefinition> imuDefinitions, YoVariableRegistry registry)
   {
      Set<IMUMount> imuMounts = imuDefinitions.keySet();

      for (IMUMount imuMount : imuMounts)
      {
         String sensorName = imuMount.getName() + "AngularVelocity";
         IMUDefinition imuDefinition = imuDefinitions.get(imuMount);

         SimulatedAngularVelocitySensorFromRobot angularVelocitySensor = new SimulatedAngularVelocitySensorFromRobot(sensorName, imuMount, registry);

         if (sensorNoiseParameters != null)
         {
            GaussianVectorCorruptor angularVelocityCorruptor = new GaussianVectorCorruptor(1235L, sensorName, registry);
            double angularVelocityMeasurementStandardDeviation = sensorNoiseParameters.getAngularVelocityMeasurementStandardDeviation();

            angularVelocityCorruptor.setStandardDeviation(angularVelocityMeasurementStandardDeviation);
            angularVelocitySensor.addSignalCorruptor(angularVelocityCorruptor);

            RandomWalkBiasVectorCorruptor biasVectorCorruptor = new RandomWalkBiasVectorCorruptor(1236L, sensorName, controlDT, registry);

            Vector3d initialAngularVelocityBias = new Vector3d();
            sensorNoiseParameters.getInitialAngularVelocityBias(initialAngularVelocityBias);
            biasVectorCorruptor.setBias(initialAngularVelocityBias);

            double angularVelocityBiasProcessNoiseStandardDeviation = sensorNoiseParameters.getAngularVelocityBiasProcessNoiseStandardDeviation();
            biasVectorCorruptor.setStandardDeviation(angularVelocityBiasProcessNoiseStandardDeviation);

            angularVelocitySensor.addSignalCorruptor(biasVectorCorruptor);
         }

         simulatedSensorHolderAndReader.addAngularVelocitySensorPort(imuDefinition, angularVelocitySensor);
      }
   }


   public void createAndAddLinearAccelerationSensors(LinkedHashMap<IMUMount, IMUDefinition> imuDefinitions, YoVariableRegistry registry)
   {
      Set<IMUMount> imuMounts = imuDefinitions.keySet();

      for (IMUMount imuMount : imuMounts)
      {
         String sensorName = imuMount.getName() + "LinearAcceleration";
         IMUDefinition imuDefinition = imuDefinitions.get(imuMount);

         SimulatedLinearAccelerationSensorFromRobot linearAccelerationSensor = new SimulatedLinearAccelerationSensorFromRobot(sensorName, imuMount, registry);

         if (sensorNoiseParameters != null)
         {
            GaussianVectorCorruptor linearAccelerationCorruptor = new GaussianVectorCorruptor(1237L, sensorName, registry);
            double linearAccelerationMeasurementStandardDeviation = sensorNoiseParameters.getLinearAccelerationMeasurementStandardDeviation();
            linearAccelerationCorruptor.setStandardDeviation(linearAccelerationMeasurementStandardDeviation);
            linearAccelerationSensor.addSignalCorruptor(linearAccelerationCorruptor);

            RandomWalkBiasVectorCorruptor biasVectorCorruptor = new RandomWalkBiasVectorCorruptor(1286L, sensorName, controlDT, registry);

            Vector3d initialLinearAccelerationBias = new Vector3d();
            sensorNoiseParameters.getInitialLinearVelocityBias(initialLinearAccelerationBias);
            biasVectorCorruptor.setBias(initialLinearAccelerationBias);

            double linearAccelerationBiasProcessNoiseStandardDeviation = sensorNoiseParameters.getLinearAccelerationBiasProcessNoiseStandardDeviation();
            biasVectorCorruptor.setStandardDeviation(linearAccelerationBiasProcessNoiseStandardDeviation);
            linearAccelerationSensor.addSignalCorruptor(biasVectorCorruptor);
         }

         simulatedSensorHolderAndReader.addLinearAccelerationSensorPort(imuDefinition, linearAccelerationSensor);
      }
   }

   public void createAndAddPointVelocitySensors(ArrayList<KinematicPoint> velocityPoints, YoVariableRegistry registry)
   {
      for (KinematicPoint kinematicPoint : velocityPoints)
      {
         String sensorName = kinematicPoint.getName() + "PointVelocity";

         SimulatedPointVelocitySensorFromRobot pointVelocitySensor = new SimulatedPointVelocitySensorFromRobot(sensorName, kinematicPoint, registry);


         if (sensorNoiseParameters != null)
         {
            GaussianVectorCorruptor pointVelocityCorruptor = new GaussianVectorCorruptor(1257L, sensorName, registry);

            double pointVelocityMeasurementStandardDeviation = sensorNoiseParameters.getPointVelocityMeasurementStandardDeviation();
            pointVelocityCorruptor.setStandardDeviation(pointVelocityMeasurementStandardDeviation);
            pointVelocitySensor.addSignalCorruptor(pointVelocityCorruptor);
         }

         PointVelocitySensorDefinition pointVelocitySensorDefinition = pointVelocitySensorDefinitions.get(kinematicPoint);

         simulatedSensorHolderAndReader.addPointVelocitySensorPort(pointVelocitySensorDefinition, pointVelocitySensor);
      }
   }
}
