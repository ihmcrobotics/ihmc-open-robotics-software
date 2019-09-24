package us.ihmc.sensorProcessing.imu;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
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
   private final Vector3D angularVelocityMeasurement = new Vector3D();
   private final Vector3D linearAccelerationMeasurement = new Vector3D();

   private final DenseMatrix64F orientationNoiseCovariance;
   private final DenseMatrix64F angularVelocityNoiseCovariance;
   private final DenseMatrix64F angularVelocityBiasProcessNoiseCovariance;
   private final DenseMatrix64F linearAccelerationNoiseCovariance;
   private final DenseMatrix64F linearAccelerationBiasProcessNoiseCovariance;

   public IMUSensor(IMUDefinition imuDefinition, SensorNoiseParameters sensorNoiseParameters)
   {
      sensorName = imuDefinition.getName();

      measurementFrame = imuDefinition.getIMUFrame();
      measurementLink = imuDefinition.getRigidBody();

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
   public void getOrientationNoiseCovariance(DenseMatrix64F noiseCovarianceToPack)
   {
      noiseCovarianceToPack.set(orientationNoiseCovariance);
   }

   @Override
   public void getAngularVelocityNoiseCovariance(DenseMatrix64F noiseCovarianceToPack)
   {
      noiseCovarianceToPack.set(angularVelocityNoiseCovariance);
   }

   @Override
   public void getAngularVelocityBiasProcessNoiseCovariance(DenseMatrix64F biasProcessNoiseCovarianceToPack)
   {
      biasProcessNoiseCovarianceToPack.set(angularVelocityBiasProcessNoiseCovariance);
   }

   @Override
   public void getLinearAccelerationNoiseCovariance(DenseMatrix64F noiseCovarianceToPack)
   {
      noiseCovarianceToPack.set(linearAccelerationNoiseCovariance);
   }

   @Override
   public void getLinearAccelerationBiasProcessNoiseCovariance(DenseMatrix64F biasProcessNoiseCovarianceToPack)
   {
      biasProcessNoiseCovarianceToPack.set(linearAccelerationBiasProcessNoiseCovariance);
   }

   private static DenseMatrix64F createDiagonalCovarianceMatrix(double standardDeviation)
   {
      DenseMatrix64F orientationCovarianceMatrix = new DenseMatrix64F(3, 3);
      CommonOps.setIdentity(orientationCovarianceMatrix);
      CommonOps.scale(MathTools.square(standardDeviation), orientationCovarianceMatrix);

      return orientationCovarianceMatrix;
   }
}
