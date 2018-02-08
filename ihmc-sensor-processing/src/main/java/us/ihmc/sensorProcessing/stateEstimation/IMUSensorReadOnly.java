package us.ihmc.sensorProcessing.stateEstimation;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.screwTheory.RigidBody;

public interface IMUSensorReadOnly
{
   public abstract String getSensorName();

   public abstract ReferenceFrame getMeasurementFrame();

   public abstract RigidBody getMeasurementLink();

   public abstract QuaternionReadOnly getOrientationMeasurement();

   /**
    * @deprecated use {@link #getOrientationMeasurement()} instead.
    */
   public default void getOrientationMeasurement(RotationMatrix orientationToPack)
   {
      orientationToPack.set(getOrientationMeasurement());
   }

   public abstract Vector3DReadOnly getAngularVelocityMeasurement();

   /**
    * @deprecated use {@link #getAngularVelocityMeasurement()} instead.
    */
   public default void getAngularVelocityMeasurement(Vector3DBasics angularVelocityToPack)
   {
      angularVelocityToPack.set(getAngularVelocityMeasurement());
   }

   public abstract Vector3DReadOnly getLinearAccelerationMeasurement();

   /**
    * @deprecated use {@link #getLinearAccelerationMeasurement()} instead.
    */
   public default void getLinearAccelerationMeasurement(Vector3DBasics linearAccelerationToPack)
   {
      linearAccelerationToPack.set(getLinearAccelerationMeasurement());
   }

   public abstract void getOrientationNoiseCovariance(DenseMatrix64F noiseCovarianceToPack);

   public abstract void getAngularVelocityNoiseCovariance(DenseMatrix64F noiseCovarianceToPack);

   public abstract void getAngularVelocityBiasProcessNoiseCovariance(DenseMatrix64F biasProcessNoiseCovarianceToPack);

   public abstract void getLinearAccelerationNoiseCovariance(DenseMatrix64F noiseCovarianceToPack);

   public abstract void getLinearAccelerationBiasProcessNoiseCovariance(DenseMatrix64F biasProcessNoiseCovarianceToPack);
}
