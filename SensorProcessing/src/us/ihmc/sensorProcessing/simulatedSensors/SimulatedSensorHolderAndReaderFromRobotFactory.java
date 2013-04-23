package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.ArrayList;
import java.util.Map;
import java.util.Set;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.sensorProcessing.signalCorruption.GaussianOrientationCorruptor;
import us.ihmc.sensorProcessing.signalCorruption.GaussianVectorCorruptor;
import us.ihmc.sensorProcessing.signalCorruption.RandomWalkBiasVectorCorruptor;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.SixDoFJoint;

import com.yobotics.simulationconstructionset.FloatingJoint;
import com.yobotics.simulationconstructionset.IMUMount;
import com.yobotics.simulationconstructionset.Joint;
import com.yobotics.simulationconstructionset.KinematicPoint;
import com.yobotics.simulationconstructionset.OneDegreeOfFreedomJoint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.UnreasonableAccelerationException;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class SimulatedSensorHolderAndReaderFromRobotFactory
{
   private final YoVariableRegistry registry;
   private final Robot robot;
   private final double controlDT;
   private final SensorNoiseParameters sensorNoiseParameters;
   
   private final ArrayList<IMUMount> imuMounts;
   private final ArrayList<KinematicPoint> positionPoints;
   private final ArrayList<KinematicPoint> velocityPoints;
   

   private Map<IMUMount, IMUDefinition> imuDefinitions;
   private Map<KinematicPoint, PointPositionSensorDefinition> pointPositionSensorDefinitions;
   private Map<KinematicPoint, PointVelocitySensorDefinition> pointVelocitySensorDefinitions;
   private SimulatedSensorHolderAndReader simulatedSensorHolderAndReader;
   private StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions;
   
   
   public SimulatedSensorHolderAndReaderFromRobotFactory(Robot robot, SensorNoiseParameters sensorNoiseParameters, double controlDT,
         ArrayList<IMUMount> imuMounts, ArrayList<KinematicPoint> positionPoints, ArrayList<KinematicPoint> velocityPoints, YoVariableRegistry registry)
   {
      
      this.registry = registry;
      this.robot = robot;
      this.sensorNoiseParameters = sensorNoiseParameters;
      
      this.controlDT = controlDT;
      this.imuMounts = imuMounts;
      this.positionPoints = positionPoints;
      this.velocityPoints = velocityPoints;


   }

   public void build(SixDoFJoint sixDoFJoint)
   {
      
      
      ArrayList<Joint> rootJoints = robot.getRootJoints();
      if (rootJoints.size() > 1)
      {
         throw new RuntimeException("Robot has more than 1 rootJoint");
      }

      final Joint rootJoint = rootJoints.get(0);
      if (rootJoint instanceof FloatingJoint)
      {
         SCSToInverseDynamicsJointMap scsToInverseDynamicsJointMap = SCSToInverseDynamicsJointMap.createByName((FloatingJoint) rootJoint, sixDoFJoint);
         StateEstimatorSensorDefinitionsFromRobotFactory stateEstimatorSensorDefinitionsFromRobotFactory = new StateEstimatorSensorDefinitionsFromRobotFactory(
               scsToInverseDynamicsJointMap, robot, controlDT, imuMounts, positionPoints, velocityPoints);
         
         this.stateEstimatorSensorDefinitions = stateEstimatorSensorDefinitionsFromRobotFactory.getStateEstimatorSensorDefinitions();
         this.imuDefinitions = stateEstimatorSensorDefinitionsFromRobotFactory.getIMUDefinitions();
         this.pointPositionSensorDefinitions = stateEstimatorSensorDefinitionsFromRobotFactory.getPointPositionSensorDefinitions();
         this.pointVelocitySensorDefinitions = stateEstimatorSensorDefinitionsFromRobotFactory.getPointVelocitySensorDefinitions();
         
         this.simulatedSensorHolderAndReader = new SimulatedSensorHolderAndReader();
         
         createAndAddOrientationSensors(imuDefinitions, registry);
         createAndAddAngularVelocitySensors(imuDefinitions, registry);
         createAndAddLinearAccelerationSensors(imuDefinitions, registry);
         createAndAddPointPositionSensors(positionPoints, registry);
         createAndAddPointVelocitySensors(velocityPoints, registry);
         createAndAddOneDoFPositionAndVelocitySensors(scsToInverseDynamicsJointMap);
      }
      else
      {
         throw new RuntimeException("Not FloatingJoint rootjoint found");
      }
   }

   public SimulatedSensorHolderAndReader getSimulatedSensorHolderAndReader()
   {
      return simulatedSensorHolderAndReader;
   }

   public void createAndAddOneDoFPositionAndVelocitySensors(SCSToInverseDynamicsJointMap scsToInverseDynamicsJointMap)
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

   public StateEstimatorSensorDefinitions getStateEstimatorSensorDefinitions()
   {
      return stateEstimatorSensorDefinitions;
   }

   public void getCenterOfMassPostion(FramePoint estimatedCoMPosition)
   {
      estimatedCoMPosition.setX(estimatedCoMPosition.getX() + 0.0855);
      estimatedCoMPosition.setZ(estimatedCoMPosition.getZ() + 0.9904); //1.2); 
      
      Point3d comPoint = new Point3d();
      try
      {
         robot.doDynamicsButDoNotIntegrate();
      }
      catch (UnreasonableAccelerationException e)
      {
         throw new RuntimeException("UnreasonableAccelerationException in getCenterOfMassPostion");
      }
      robot.computeCenterOfMass(comPoint);
      
   }
}
