package us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration;

import java.util.ArrayList;
import java.util.Map;
import java.util.Set;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;

public class SensorConfigurationFactory
{
   private final double gravitationalAcceleration;
   private final SensorNoiseParameters sensorNoiseParameters;

   public SensorConfigurationFactory(SensorNoiseParameters sensorNoiseParameters, double gravitationalAcceleration)
   {
      this.gravitationalAcceleration = gravitationalAcceleration;
      this.sensorNoiseParameters = sensorNoiseParameters;
   }

   public ArrayList<OrientationSensorConfiguration> createOrientationSensorConfigurations(Map<IMUDefinition,
                     ControlFlowOutputPort<RotationMatrix>> orientationSensors)
   {
      ArrayList<OrientationSensorConfiguration> orientationSensorConfigurations = new ArrayList<OrientationSensorConfiguration>();

      Set<IMUDefinition> imuDefinitions = orientationSensors.keySet();
      for (IMUDefinition estimatedIMUDefinition : imuDefinitions)
      {
         String sensorName = estimatedIMUDefinition.getName() + "Orientation";

         double orientationMeasurementStandardDeviation = sensorNoiseParameters.getOrientationMeasurementStandardDeviation();
         DenseMatrix64F orientationNoiseCovariance = createDiagonalCovarianceMatrix(orientationMeasurementStandardDeviation, 3);

         RigidBody estimatedMeasurementBody = estimatedIMUDefinition.getRigidBody();
         ReferenceFrame estimatedMeasurementFrame = createMeasurementFrame(sensorName, "EstimatedMeasurementFrame", estimatedIMUDefinition,
                                                       estimatedMeasurementBody);

         ControlFlowOutputPort<RotationMatrix> outputPort = orientationSensors.get(estimatedIMUDefinition);

         OrientationSensorConfiguration orientationSensorConfiguration = new OrientationSensorConfiguration(outputPort, sensorName, estimatedMeasurementFrame,
                                                                            orientationNoiseCovariance);
         orientationSensorConfigurations.add(orientationSensorConfiguration);
      }


      return orientationSensorConfigurations;
   }

   public ArrayList<AngularVelocitySensorConfiguration> createAngularVelocitySensorConfigurations(Map<IMUDefinition,
                    ControlFlowOutputPort<Vector3D>> angularVelocitySensors)
   {
      ArrayList<AngularVelocitySensorConfiguration> angularVelocitySensorConfigurations = new ArrayList<AngularVelocitySensorConfiguration>();

      Set<IMUDefinition> imuDefinitions = angularVelocitySensors.keySet();
      for (IMUDefinition estimatedIMUDefinition : imuDefinitions)
      {
         String sensorName = estimatedIMUDefinition.getName() + "AngularVelocity";

         double angularVelocityMeasurementStandardDeviation = sensorNoiseParameters.getAngularVelocityMeasurementStandardDeviation();
         DenseMatrix64F angularVelocityNoiseCovariance = createDiagonalCovarianceMatrix(angularVelocityMeasurementStandardDeviation, 3);
         double angularVelocityBiasProcessNoiseStandardDeviation = sensorNoiseParameters.getAngularVelocityBiasProcessNoiseStandardDeviation();
         DenseMatrix64F angularVelocityBiasProcessNoiseCovariance = createDiagonalCovarianceMatrix(angularVelocityBiasProcessNoiseStandardDeviation, 3);

         RigidBody estimatedMeasurementBody = estimatedIMUDefinition.getRigidBody();
         ReferenceFrame estimatedMeasurementFrame = createMeasurementFrame(sensorName, "EstimatedMeasurementFrame", estimatedIMUDefinition,
                                                       estimatedMeasurementBody);

         ControlFlowOutputPort<Vector3D> outputPort = angularVelocitySensors.get(estimatedIMUDefinition);

         AngularVelocitySensorConfiguration angularVelocitySensorConfiguration = new AngularVelocitySensorConfiguration(outputPort, sensorName,
                                                                                    estimatedMeasurementBody, estimatedMeasurementFrame,
                                                                                    angularVelocityNoiseCovariance, angularVelocityBiasProcessNoiseCovariance);
         angularVelocitySensorConfigurations.add(angularVelocitySensorConfiguration);
      }


      return angularVelocitySensorConfigurations;
   }


   public ArrayList<LinearAccelerationSensorConfiguration> createLinearAccelerationSensorConfigurations(Map<IMUDefinition,
                    ControlFlowOutputPort<Vector3D>> linearAccelerationSensors)
   {
      ArrayList<LinearAccelerationSensorConfiguration> linearAccelerationSensorConfigurations = new ArrayList<LinearAccelerationSensorConfiguration>();

      Set<IMUDefinition> imuDefinitions = linearAccelerationSensors.keySet();
      for (IMUDefinition estimatedIMUDefinition : imuDefinitions)
      {
         String sensorName = estimatedIMUDefinition.getName() + "LinearAcceleration";

         double linearAccelerationMeasurementStandardDeviation = sensorNoiseParameters.getLinearAccelerationMeasurementStandardDeviation();
         DenseMatrix64F linearAccelerationNoiseCovariance = createDiagonalCovarianceMatrix(linearAccelerationMeasurementStandardDeviation, 3);
         double linearAccelerationBiasProcessNoiseStandardDeviation = sensorNoiseParameters.getLinearAccelerationBiasProcessNoiseStandardDeviation();
         DenseMatrix64F linearAccelerationBiasProcessNoiseCovariance = createDiagonalCovarianceMatrix(linearAccelerationBiasProcessNoiseStandardDeviation, 3);

         RigidBody estimatedMeasurementBody = estimatedIMUDefinition.getRigidBody();
         ReferenceFrame estimatedMeasurementFrame = createMeasurementFrame(sensorName, "EstimatedMeasurementFrame", estimatedIMUDefinition,
                                                       estimatedMeasurementBody);

         ControlFlowOutputPort<Vector3D> outputPort = linearAccelerationSensors.get(estimatedIMUDefinition);

         LinearAccelerationSensorConfiguration linearAccelerationSensorConfiguration = new LinearAccelerationSensorConfiguration(outputPort, sensorName,
                                                                                          estimatedMeasurementBody, estimatedMeasurementFrame,
                                                                                          gravitationalAcceleration, linearAccelerationNoiseCovariance,
                                                                                          linearAccelerationBiasProcessNoiseCovariance);

         linearAccelerationSensorConfigurations.add(linearAccelerationSensorConfiguration);
      }

      return linearAccelerationSensorConfigurations;
   }

   private static DenseMatrix64F createDiagonalCovarianceMatrix(double standardDeviation, int size)
   {
      DenseMatrix64F orientationCovarianceMatrix = new DenseMatrix64F(size, size);
      CommonOps.setIdentity(orientationCovarianceMatrix);
      CommonOps.scale(MathTools.square(standardDeviation), orientationCovarianceMatrix);

      return orientationCovarianceMatrix;
   }

   public static ReferenceFrame createMeasurementFrame(String sensorName, String frameName, IMUDefinition imuDefinition, RigidBody measurementBody)
   {
      RigidBodyTransform transformFromIMUToJoint = new RigidBodyTransform();
      imuDefinition.getTransformFromIMUToJoint(transformFromIMUToJoint);

      ReferenceFrame perfectFrameAfterJoint = measurementBody.getParentJoint().getFrameAfterJoint();

      if (transformFromIMUToJoint.epsilonEquals(new RigidBodyTransform(), 1e-10))
      {
         return perfectFrameAfterJoint;
      }

      ReferenceFrame perfectMeasurementFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(sensorName + frameName, perfectFrameAfterJoint,
                                                  transformFromIMUToJoint);

      return perfectMeasurementFrame;
   }
}
