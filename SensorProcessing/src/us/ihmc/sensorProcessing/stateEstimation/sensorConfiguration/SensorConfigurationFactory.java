package us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Map;
import java.util.Set;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.simulatedSensors.IMUDefinition;
import us.ihmc.sensorProcessing.simulatedSensors.PointVelocitySensorDefinition;
import us.ihmc.sensorProcessing.simulatedSensors.SimulatedSensorsFactory;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;

public class SensorConfigurationFactory
{
   private final double orientationMeasurementStandardDeviation = Math.sqrt(1e-2);
   private final double angularVelocityMeasurementStandardDeviation = 1e-1;    // 1e-3;    // 2e-4;
   private final double linearAccelerationMeasurementStandardDeviation = 1e0;    // 1e-1;    // 1.7e-2;
   private final double pointVelocityMeasurementStandardDeviation = 1e-1;    // 1e0; //1e1; //1e-1; //1e-10; //1e-1;    // 1.7e-2;

   private final double angularVelocityBiasProcessNoiseStandardDeviation = Math.sqrt(1e-5);
   private final double linearAccelerationBiasProcessNoiseStandardDeviation = Math.sqrt(1e-4);

   private final Vector3d gravitationalAcceleration;

   public SensorConfigurationFactory(Vector3d gravitationalAcceleration)
   {
      this.gravitationalAcceleration = new Vector3d();
      this.gravitationalAcceleration.set(gravitationalAcceleration);
   }

   public Collection<OrientationSensorConfiguration> createOrientationSensorConfigurations(Map<IMUDefinition,
                     ControlFlowOutputPort<Matrix3d>> orientationSensors)
   {
      ArrayList<OrientationSensorConfiguration> orientationSensorConfigurations = new ArrayList<OrientationSensorConfiguration>();

      Set<IMUDefinition> imuDefinitions = orientationSensors.keySet();
      for (IMUDefinition estimatedIMUDefinition : imuDefinitions)
      {
         String sensorName = estimatedIMUDefinition.getName() + "Orientation";

         DenseMatrix64F orientationNoiseCovariance = createDiagonalCovarianceMatrix(orientationMeasurementStandardDeviation, 3);

         RigidBody estimatedMeasurementBody = estimatedIMUDefinition.getRigidBody();
         ReferenceFrame estimatedMeasurementFrame = SimulatedSensorsFactory.createMeasurementFrame(sensorName, "EstimatedMeasurementFrame",
                                                       estimatedIMUDefinition, estimatedMeasurementBody);

         ControlFlowOutputPort<Matrix3d> outputPort = orientationSensors.get(estimatedIMUDefinition);

         OrientationSensorConfiguration orientationSensorConfiguration = new OrientationSensorConfiguration(outputPort, sensorName, estimatedMeasurementFrame,
                                                                            orientationNoiseCovariance);
         orientationSensorConfigurations.add(orientationSensorConfiguration);
      }


      return orientationSensorConfigurations;
   }

   public ArrayList<AngularVelocitySensorConfiguration> createAngularVelocitySensorConfigurations(Map<IMUDefinition,
                    ControlFlowOutputPort<Vector3d>> angularVelocitySensors)
   {
      ArrayList<AngularVelocitySensorConfiguration> angularVelocitySensorConfigurations = new ArrayList<AngularVelocitySensorConfiguration>();

      Set<IMUDefinition> imuDefinitions = angularVelocitySensors.keySet();
      for (IMUDefinition estimatedIMUDefinition : imuDefinitions)
      {
         String sensorName = estimatedIMUDefinition.getName() + "AngularVelocity";

         DenseMatrix64F angularVelocityNoiseCovariance = createDiagonalCovarianceMatrix(angularVelocityMeasurementStandardDeviation, 3);
         DenseMatrix64F angularVelocityBiasProcessNoiseCovariance = createDiagonalCovarianceMatrix(angularVelocityBiasProcessNoiseStandardDeviation, 3);

         RigidBody estimatedMeasurementBody = estimatedIMUDefinition.getRigidBody();
         ReferenceFrame estimatedMeasurementFrame = SimulatedSensorsFactory.createMeasurementFrame(sensorName, "EstimatedMeasurementFrame",
                                                       estimatedIMUDefinition, estimatedMeasurementBody);

         ControlFlowOutputPort<Vector3d> outputPort = angularVelocitySensors.get(estimatedIMUDefinition);

         AngularVelocitySensorConfiguration angularVelocitySensorConfiguration = new AngularVelocitySensorConfiguration(outputPort, sensorName,
                                                                                    estimatedMeasurementBody, estimatedMeasurementFrame,
                                                                                    angularVelocityNoiseCovariance, angularVelocityBiasProcessNoiseCovariance);
         angularVelocitySensorConfigurations.add(angularVelocitySensorConfiguration);
      }


      return angularVelocitySensorConfigurations;
   }


   public ArrayList<LinearAccelerationSensorConfiguration> createLinearAccelerationSensorConfigurations(Map<IMUDefinition,
                    ControlFlowOutputPort<Vector3d>> linearAccelerationSensors)
   {
      ArrayList<LinearAccelerationSensorConfiguration> linearAccelerationSensorConfigurations = new ArrayList<LinearAccelerationSensorConfiguration>();

      Set<IMUDefinition> imuDefinitions = linearAccelerationSensors.keySet();
      for (IMUDefinition estimatedIMUDefinition : imuDefinitions)
      {
         String sensorName = estimatedIMUDefinition.getName() + "LinearAcceleration";

         DenseMatrix64F linearAccelerationNoiseCovariance = createDiagonalCovarianceMatrix(linearAccelerationMeasurementStandardDeviation, 3);
         DenseMatrix64F linearAccelerationBiasProcessNoiseCovariance = createDiagonalCovarianceMatrix(linearAccelerationBiasProcessNoiseStandardDeviation, 3);

         RigidBody estimatedMeasurementBody = estimatedIMUDefinition.getRigidBody();
         ReferenceFrame estimatedMeasurementFrame = SimulatedSensorsFactory.createMeasurementFrame(sensorName, "EstimatedMeasurementFrame",
                                                       estimatedIMUDefinition, estimatedMeasurementBody);

         ControlFlowOutputPort<Vector3d> outputPort = linearAccelerationSensors.get(estimatedIMUDefinition);

         LinearAccelerationSensorConfiguration linearAccelerationSensorConfiguration = new LinearAccelerationSensorConfiguration(outputPort, sensorName,
                                                                                          estimatedMeasurementBody, estimatedMeasurementFrame,
                                                                                          gravitationalAcceleration.getZ(), linearAccelerationNoiseCovariance,
                                                                                          linearAccelerationBiasProcessNoiseCovariance);

         linearAccelerationSensorConfigurations.add(linearAccelerationSensorConfiguration);
      }

      return linearAccelerationSensorConfigurations;
   }


   public ArrayList<PointVelocitySensorConfiguration> createPointVelocitySensorConfigurations(Map<PointVelocitySensorDefinition,
                    ControlFlowOutputPort<Vector3d>> pointVelocitySensors)
   {
      ArrayList<PointVelocitySensorConfiguration> pointVelocitySensorConfigurations = new ArrayList<PointVelocitySensorConfiguration>();

      Set<PointVelocitySensorDefinition> pointVelocitySensorDefinitions = pointVelocitySensors.keySet();
      for (PointVelocitySensorDefinition pointVelocitySensorDefinition : pointVelocitySensorDefinitions)
      {
         String sensorName = pointVelocitySensorDefinition.getName() + "PointVelocity";

         DenseMatrix64F pointVelocityNoiseCovariance = createDiagonalCovarianceMatrix(pointVelocityMeasurementStandardDeviation, 3);

         RigidBody estimatedMeasurementBody = pointVelocitySensorDefinition.getRigidBody();
         ReferenceFrame estimatedFrameAfterJoint = estimatedMeasurementBody.getParentJoint().getFrameAfterJoint();

         Vector3d offset = new Vector3d();
         pointVelocitySensorDefinition.getOffset(offset);
         FramePoint estimatedVelocityPoint = new FramePoint(estimatedFrameAfterJoint, offset);

         ControlFlowOutputPort<Vector3d> outputPort = pointVelocitySensors.get(pointVelocitySensorDefinition);

         PointVelocitySensorConfiguration pointVelocitySensorConfiguration = new PointVelocitySensorConfiguration(outputPort, sensorName,
                                                                                estimatedMeasurementBody, estimatedVelocityPoint, pointVelocityNoiseCovariance);

         pointVelocitySensorConfigurations.add(pointVelocitySensorConfiguration);
      }

      return pointVelocitySensorConfigurations;
   }

   private static DenseMatrix64F createDiagonalCovarianceMatrix(double standardDeviation, int size)
   {
      DenseMatrix64F orientationCovarianceMatrix = new DenseMatrix64F(size, size);
      CommonOps.setIdentity(orientationCovarianceMatrix);
      CommonOps.scale(MathTools.square(standardDeviation), orientationCovarianceMatrix);

      return orientationCovarianceMatrix;
   }
}
