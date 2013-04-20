package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.ArrayList;
import java.util.Map;
import java.util.Set;

import javax.vecmath.Vector3d;

import us.ihmc.sensorProcessing.signalCorruption.GaussianOrientationCorruptor;
import us.ihmc.sensorProcessing.signalCorruption.GaussianVectorCorruptor;
import us.ihmc.sensorProcessing.signalCorruption.RandomWalkBiasVectorCorruptor;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

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

   private final Map<IMUMount, IMUDefinition> imuDefinitions;
   private final Map<KinematicPoint, PointPositionSensorDefinition> pointPositionSensorDefinitions;
   private final Map<KinematicPoint, PointVelocitySensorDefinition> pointVelocitySensorDefinitions;
   private final SimulatedSensorHolderAndReader simulatedSensorHolderAndReader;

   public SimulatedSensorHolderAndReaderFromRobotFactory(StateEstimatorSensorDefinitionsFromRobotFactory stateEstimatorSensorDefinitionsFromRobotFactory,
         SCSToInverseDynamicsJointMap scsToInverseDynamicsJointMap, Robot robot,
           SensorNoiseParameters sensorNoiseParameters, double controlDT, ArrayList<IMUMount> imuMounts, 
           ArrayList<KinematicPoint> positionPoints, ArrayList<KinematicPoint> velocityPoints,
           YoVariableRegistry registry)
   {
      this.scsToInverseDynamicsJointMap = scsToInverseDynamicsJointMap;
      this.robot = robot;
      this.sensorNoiseParameters = sensorNoiseParameters;

      this.controlDT = controlDT;

      this.imuDefinitions = stateEstimatorSensorDefinitionsFromRobotFactory.getIMUDefinitions();
      this.pointPositionSensorDefinitions = stateEstimatorSensorDefinitionsFromRobotFactory.getPointPositionSensorDefinitions();
      this.pointVelocitySensorDefinitions = stateEstimatorSensorDefinitionsFromRobotFactory.getPointVelocitySensorDefinitions();

      simulatedSensorHolderAndReader = new SimulatedSensorHolderAndReader();

      createAndAddOneDoFPositionAndVelocitySensors();
      createAndAddOrientationSensors(imuDefinitions, registry);
      createAndAddAngularVelocitySensors(imuDefinitions, registry);
      createAndAddLinearAccelerationSensors(imuDefinitions, registry);
      
      createAndAddPointPositionSensors(positionPoints, registry);
      createAndAddPointVelocitySensors(velocityPoints, registry);
   }

   public SimulatedSensorHolderAndReader getSimulatedSensorHolderAndReader()
   {
      return simulatedSensorHolderAndReader;
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


   public void createAndAddOrientationSensors(Map<IMUMount, IMUDefinition> imuDefinitions, YoVariableRegistry registry)
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


   public void createAndAddAngularVelocitySensors(Map<IMUMount, IMUDefinition> imuDefinitions, YoVariableRegistry registry)
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


   public void createAndAddLinearAccelerationSensors(Map<IMUMount, IMUDefinition> imuDefinitions, YoVariableRegistry registry)
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

   public void createAndAddPointPositionSensors(ArrayList<KinematicPoint> positionPoints, YoVariableRegistry registry)
   {
      for (KinematicPoint kinematicPoint : positionPoints)
      {
         String sensorName = kinematicPoint.getName() + "PointPosition";

         SimulatedPointPositionSensorFromRobot pointPositionSensor = new SimulatedPointPositionSensorFromRobot(sensorName, kinematicPoint, registry);


         if (sensorNoiseParameters != null)
         {
            GaussianVectorCorruptor pointPositionCorruptor = new GaussianVectorCorruptor(1257L, sensorName, registry);

            double pointPositionMeasurementStandardDeviation = sensorNoiseParameters.getPointPositionMeasurementStandardDeviation();
            pointPositionCorruptor.setStandardDeviation(pointPositionMeasurementStandardDeviation);
            pointPositionSensor.addSignalCorruptor(pointPositionCorruptor);
         }

         PointPositionSensorDefinition pointPositionSensorDefinition = pointPositionSensorDefinitions.get(kinematicPoint);

         simulatedSensorHolderAndReader.addPointPositionSensorPort(pointPositionSensorDefinition, pointPositionSensor);
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
