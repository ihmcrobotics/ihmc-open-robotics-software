package us.ihmc.sensorProcessing.imu;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;

public class IMUSensor implements IMUSensorReadOnly
{
   private final String sensorName;

   private final ReferenceFrame measurementFrame;
   private final RigidBody measurementLink;

   private final RotationMatrix orientationMeasurement = new RotationMatrix();
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
   public RigidBody getMeasurementLink()
   {
      return measurementLink;
   }

   public void setOrientationMeasurement(RotationMatrixReadOnly newOrientation)
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
   public void getAngularVelocityMeasurement(Vector3DBasics angularVelocityToPack)
   {
      angularVelocityToPack.set(angularVelocityMeasurement);
   }

   @Override
   public void getLinearAccelerationMeasurement(Vector3DBasics linearAccelerationToPack)
   {
      linearAccelerationToPack.set(linearAccelerationMeasurement);
   }

   @Override
   public void getOrientationMeasurement(RotationMatrix orientationToPack)
   {
      orientationToPack.set(orientationMeasurement);
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
