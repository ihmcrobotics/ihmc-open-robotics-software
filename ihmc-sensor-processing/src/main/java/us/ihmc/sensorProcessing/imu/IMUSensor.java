package us.ihmc.sensorProcessing.imu;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;

public class IMUSensor implements IMUSensorReadOnly
{
   private final String sensorName;

   private final ReferenceFrame measurementFrame;
   private final RigidBodyBasics measurementLink;

   private final Quaternion orientationMeasurement = new Quaternion();
   private final RigidBodyTransform transformFromIMUToJoint = new RigidBodyTransform();

   private final Vector3D angularVelocityMeasurement = new Vector3D();
   private final Vector3D linearAccelerationMeasurement = new Vector3D();

   private final DMatrixRMaj orientationNoiseCovariance;
   private final DMatrixRMaj angularVelocityNoiseCovariance;
   private final DMatrixRMaj angularVelocityBiasProcessNoiseCovariance;
   private final DMatrixRMaj linearAccelerationNoiseCovariance;
   private final DMatrixRMaj linearAccelerationBiasProcessNoiseCovariance;

   public IMUSensor(IMUDefinition imuDefinition, SensorNoiseParameters sensorNoiseParameters)
   {
      sensorName = imuDefinition.getName();

      measurementFrame = imuDefinition.getIMUFrame();
      measurementLink = imuDefinition.getRigidBody();
      imuDefinition.getTransformFromIMUToJoint(transformFromIMUToJoint);

      if (sensorNoiseParameters != null)
      {
         orientationNoiseCovariance = createDiagonalCovarianceMatrix(sensorNoiseParameters.getOrientationMeasurementStandardDeviation());

         angularVelocityNoiseCovariance = createDiagonalCovarianceMatrix(sensorNoiseParameters.getAngularVelocityMeasurementStandardDeviation());
         angularVelocityBiasProcessNoiseCovariance = createDiagonalCovarianceMatrix(sensorNoiseParameters.getAngularVelocityBiasProcessNoiseStandardDeviation());

         linearAccelerationNoiseCovariance = createDiagonalCovarianceMatrix(sensorNoiseParameters.getLinearAccelerationMeasurementStandardDeviation());
         linearAccelerationBiasProcessNoiseCovariance = createDiagonalCovarianceMatrix(sensorNoiseParameters.getLinearAccelerationBiasProcessNoiseStandardDeviation());
      }
      else
      {
         orientationNoiseCovariance = createDiagonalCovarianceMatrix(0.0);

         angularVelocityNoiseCovariance = createDiagonalCovarianceMatrix(0.0);
         angularVelocityBiasProcessNoiseCovariance = createDiagonalCovarianceMatrix(0.0);

         linearAccelerationNoiseCovariance = createDiagonalCovarianceMatrix(0.0);
         linearAccelerationBiasProcessNoiseCovariance = createDiagonalCovarianceMatrix(0.0);
      }
   }

   @Override
   public String getSensorName()
   {
      return sensorName;
   }

   @Override
   public ReferenceFrame getMeasurementFrame()
   {
      return measurementFrame;
   }

   @Override
   public RigidBodyBasics getMeasurementLink()
   {
      return measurementLink;
   }

   @Override 
   public RigidBodyTransformReadOnly getTransformFromIMUToJoint()
   {
      return transformFromIMUToJoint;
   }

   public void setOrientationMeasurement(Orientation3DReadOnly newOrientation)
   {
      orientationMeasurement.set(newOrientation);
   }

   public void setAngularVelocityMeasurement(Vector3DReadOnly newAngularOrientation)
   {
      angularVelocityMeasurement.set(newAngularOrientation);
   }

   public void setLinearAccelerationMeasurement(Vector3DReadOnly newLinearAcceleration)
   {
      linearAccelerationMeasurement.set(newLinearAcceleration);
   }

   @Override
   public Vector3DReadOnly getAngularVelocityMeasurement()
   {
      return angularVelocityMeasurement;
   }

   @Override
   public Vector3DReadOnly getLinearAccelerationMeasurement()
   {
      return linearAccelerationMeasurement;
   }

   @Override
   public QuaternionReadOnly getOrientationMeasurement()
   {
      return orientationMeasurement;
   }

   public void getAngularVelocityMeasurement(Vector3D angularVelocityToPack)
   {
      angularVelocityToPack.set(angularVelocityMeasurement);
   }

   public void getLinearAccelerationMeasurement(Vector3D linearAccelerationToPack)
   {
      linearAccelerationToPack.set(linearAccelerationMeasurement);
   }

   @Override
   public void getOrientationNoiseCovariance(DMatrixRMaj noiseCovarianceToPack)
   {
      noiseCovarianceToPack.set(orientationNoiseCovariance);
   }

   @Override
   public void getAngularVelocityNoiseCovariance(DMatrixRMaj noiseCovarianceToPack)
   {
      noiseCovarianceToPack.set(angularVelocityNoiseCovariance);
   }

   @Override
   public void getAngularVelocityBiasProcessNoiseCovariance(DMatrixRMaj biasProcessNoiseCovarianceToPack)
   {
      biasProcessNoiseCovarianceToPack.set(angularVelocityBiasProcessNoiseCovariance);
   }

   @Override
   public void getLinearAccelerationNoiseCovariance(DMatrixRMaj noiseCovarianceToPack)
   {
      noiseCovarianceToPack.set(linearAccelerationNoiseCovariance);
   }

   @Override
   public void getLinearAccelerationBiasProcessNoiseCovariance(DMatrixRMaj biasProcessNoiseCovarianceToPack)
   {
      biasProcessNoiseCovarianceToPack.set(linearAccelerationBiasProcessNoiseCovariance);
   }

   private static DMatrixRMaj createDiagonalCovarianceMatrix(double standardDeviation)
   {
      DMatrixRMaj orientationCovarianceMatrix = new DMatrixRMaj(3, 3);
      CommonOps_DDRM.setIdentity(orientationCovarianceMatrix);
      CommonOps_DDRM.scale(MathTools.square(standardDeviation), orientationCovarianceMatrix);

      return orientationCovarianceMatrix;
   }

   @Override
   public String toString()
   {
      return sensorName + ", body: " + measurementLink;
   }
}
