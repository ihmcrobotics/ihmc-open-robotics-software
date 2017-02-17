package us.ihmc.sensorProcessing.stateEstimation;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public interface IMUSensorReadOnly
{
   public abstract String getSensorName();

   public abstract ReferenceFrame getMeasurementFrame();

   public abstract RigidBody getMeasurementLink();

   public abstract void getOrientationMeasurement(RotationMatrix orientationToPack);

   public abstract void getAngularVelocityMeasurement(Vector3DBasics angularVelocityToPack);

   public abstract void getLinearAccelerationMeasurement(Vector3DBasics linearAccelerationToPack);

   public abstract void getOrientationNoiseCovariance(DenseMatrix64F noiseCovarianceToPack);

   public abstract void getAngularVelocityNoiseCovariance(DenseMatrix64F noiseCovarianceToPack);

   public abstract void getAngularVelocityBiasProcessNoiseCovariance(DenseMatrix64F biasProcessNoiseCovarianceToPack);

   public abstract void getLinearAccelerationNoiseCovariance(DenseMatrix64F noiseCovarianceToPack);

   public abstract void getLinearAccelerationBiasProcessNoiseCovariance(DenseMatrix64F biasProcessNoiseCovarianceToPack);
}
