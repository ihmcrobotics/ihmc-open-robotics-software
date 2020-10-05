package us.ihmc.sensorProcessing.stateEstimation;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

public interface IMUSensorReadOnly
{
   public abstract String getSensorName();

   public abstract ReferenceFrame getMeasurementFrame();

   public abstract RigidBodyBasics getMeasurementLink();

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

   public abstract void getOrientationNoiseCovariance(DMatrixRMaj noiseCovarianceToPack);

   public abstract void getAngularVelocityNoiseCovariance(DMatrixRMaj noiseCovarianceToPack);

   public abstract void getAngularVelocityBiasProcessNoiseCovariance(DMatrixRMaj biasProcessNoiseCovarianceToPack);

   public abstract void getLinearAccelerationNoiseCovariance(DMatrixRMaj noiseCovarianceToPack);

   public abstract void getLinearAccelerationBiasProcessNoiseCovariance(DMatrixRMaj biasProcessNoiseCovarianceToPack);
}
