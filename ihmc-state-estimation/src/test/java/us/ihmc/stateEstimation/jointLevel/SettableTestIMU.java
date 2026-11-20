package us.ihmc.stateEstimation.jointLevel;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;

/**
 * Public, settable {@link IMUSensorReadOnly} for estimator tests: a body-fixed measurement frame on a chosen
 * link, a settable angular velocity AND specific force (linear acceleration), positive-definite covariances,
 * and the ability to corrupt the bias covariance for the NaN-hardening tests. Unlike the package-private
 * {@code JointLevelKFTestFixture.TestIMU}, the specific force is settable (needed to feed a gravity-consistent
 * reading at a tilt / in free fall) and the class is reusable across test packages.
 */
public class SettableTestIMU implements IMUSensorReadOnly
{
   private final String name;
   private final RigidBodyBasics measurementLink;
   private final ReferenceFrame measurementFrame;
   private final Vector3D angularVelocity = new Vector3D();
   private final Vector3D linearAcceleration = new Vector3D(0.0, 0.0, 9.81); // stationary specific force at identity
   private final Quaternion orientation = new Quaternion();
   private final DMatrixRMaj biasProcessNoiseCovariance = scaledIdentity(1.0e-4);
   private final DMatrixRMaj genericCovariance = scaledIdentity(1.0e-4);

   public SettableTestIMU(String name, RigidBodyBasics measurementLink)
   {
      this.name = name;
      this.measurementLink = measurementLink;
      this.measurementFrame = measurementLink.getBodyFixedFrame();
   }

   public void setAngularVelocity(double x, double y, double z)
   {
      angularVelocity.set(x, y, z);
   }

   public void setLinearAcceleration(double x, double y, double z)
   {
      linearAcceleration.set(x, y, z);
   }

   public void setLinearAcceleration(Vector3DReadOnly acceleration)
   {
      linearAcceleration.set(acceleration);
   }

   /** Corrupts the bias process-noise covariance so it is non-finite (for the NaN-hardening tests). */
   public void setBiasProcessNoiseCovarianceNonFinite()
   {
      biasProcessNoiseCovariance.set(0, 0, Double.NaN);
   }

   public RigidBodyBasics getMeasurementLinkBody()
   {
      return measurementLink;
   }

   @Override
   public String getSensorName()
   {
      return name;
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
   public QuaternionReadOnly getOrientationMeasurement()
   {
      return orientation;
   }

   @Override
   public Vector3DReadOnly getAngularVelocityMeasurement()
   {
      return angularVelocity;
   }

   @Override
   public Vector3DReadOnly getLinearAccelerationMeasurement()
   {
      return linearAcceleration;
   }

   @Override
   public void getOrientationNoiseCovariance(DMatrixRMaj noiseCovarianceToPack)
   {
      noiseCovarianceToPack.set(genericCovariance);
   }

   @Override
   public void getAngularVelocityNoiseCovariance(DMatrixRMaj noiseCovarianceToPack)
   {
      noiseCovarianceToPack.set(genericCovariance);
   }

   @Override
   public void getAngularVelocityBiasProcessNoiseCovariance(DMatrixRMaj biasProcessNoiseCovarianceToPack)
   {
      biasProcessNoiseCovarianceToPack.set(biasProcessNoiseCovariance);
   }

   @Override
   public void getLinearAccelerationNoiseCovariance(DMatrixRMaj noiseCovarianceToPack)
   {
      noiseCovarianceToPack.set(genericCovariance);
   }

   @Override
   public void getLinearAccelerationBiasProcessNoiseCovariance(DMatrixRMaj biasProcessNoiseCovarianceToPack)
   {
      biasProcessNoiseCovarianceToPack.set(genericCovariance);
   }

   private static DMatrixRMaj scaledIdentity(double value)
   {
      DMatrixRMaj matrix = new DMatrixRMaj(3, 3);
      for (int i = 0; i < 3; i++)
         matrix.set(i, i, value);
      return matrix;
   }
}
