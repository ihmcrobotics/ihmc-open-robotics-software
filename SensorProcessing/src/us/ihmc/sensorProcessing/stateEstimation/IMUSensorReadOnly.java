package us.ihmc.sensorProcessing.stateEstimation;

import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix3f;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public interface IMUSensorReadOnly
{
   public abstract String getSensorName();

   public abstract ReferenceFrame getMeasurementFrame();

   public abstract RigidBody getMeasurementLink();

   public abstract void getOrientationMeasurement(Matrix3d orientationToPack);

   public abstract void getAngularVelocityMeasurement(Vector3d angularVelocityToPack);

   public abstract void getLinearAccelerationMeasurement(Vector3d linearAccelerationToPack);

   public abstract void getOrientationMeasurement(Matrix3f orientationToPack);

   public abstract void getAngularVelocityMeasurement(Vector3f angularVelocityToPack);

   public abstract void getLinearAccelerationMeasurement(Vector3f linearAccelerationToPack);

   public abstract void getOrientationNoiseCovariance(DenseMatrix64F noiseCovarianceToPack);

   public abstract void getAngularVelocityNoiseCovariance(DenseMatrix64F noiseCovarianceToPack);

   public abstract void getAngularVelocityBiasProcessNoiseCovariance(DenseMatrix64F biasProcessNoiseCovarianceToPack);

   public abstract void getLinearAccelerationNoiseCovariance(DenseMatrix64F noiseCovarianceToPack);

   public abstract void getLinearAccelerationBiasProcessNoiseCovariance(DenseMatrix64F biasProcessNoiseCovarianceToPack);
}
