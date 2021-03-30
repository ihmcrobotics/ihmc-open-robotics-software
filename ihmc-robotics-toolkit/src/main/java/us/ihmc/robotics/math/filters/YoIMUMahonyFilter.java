package us.ihmc.robotics.math.filters;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This class implements a processor for fusing gyro, accelerometer, and magnetometer (optional)
 * from an IMU and estimating the orientation.
 * <p>
 * It is inspired from the Mahony's filter introduced
 * <a href="http://www.olliw.eu/2013/imu-data-fusing/">here</a>.
 * </p>
 * <p>
 * Although it can be used drastically optimized, this implementation is meant to run at lower rate
 * for which such optimization is not necessary, instead, I preferred improving the clarity of the
 * algorithm.
 * </p>
 */
public class YoIMUMahonyFilter implements ProcessingYoVariable
{
   private static final double MIN_MAGNITUDE = 1.0e-5;

   public static final Vector3DReadOnly ACCELERATION_REFERENCE = Axis3D.Z;
   public static final Vector3DReadOnly NORTH_REFERENCE = Axis3D.X;

   private YoFrameVector3D rawAngularVelocity;
   private YoFrameVector3D rawLinearAcceleration;
   private YoFrameVector3D rawMagneticVector;

   private final YoFrameQuaternion estimatedOrientation;
   private final YoFrameVector3D estimatedAngularVelocity;

   private final YoFrameVector3D yoErrorTerm;
   private final YoFrameVector3D yoIntegralTerm;

   private final YoDouble proportionalGain;
   private final YoDouble integralGain;

   private final YoBoolean enableYawRateBiasCorrection;
   private final YoDouble zeroAngularVelocityThreshold;
   private final YoDouble yawRateBiasGain;
   private final YoDouble yawRateBiasEstimate;
   private final YoFrameVector3D yawRateBiasError;

   private final double updateDT;

   private final YoBoolean hasBeenInitialized;

   private final ReferenceFrame sensorFrame;

   public YoIMUMahonyFilter(String imuName,
                            String nameSuffix,
                            double updateDT,
                            ReferenceFrame sensorFrame,
                            YoFrameQuaternion estimatedOrientation,
                            YoFrameVector3D estimatedAngularVelocity,
                            YoRegistry parentRegistry)
   {
      this(imuName, nameSuffix, updateDT, false, sensorFrame, estimatedOrientation, estimatedAngularVelocity, parentRegistry);
   }

   public YoIMUMahonyFilter(String imuName,
                            String nameSuffix,
                            double updateDT,
                            boolean createYawRateBiasEstimator,
                            ReferenceFrame sensorFrame,
                            YoFrameQuaternion estimatedOrientation,
                            YoFrameVector3D estimatedAngularVelocity,
                            YoRegistry parentRegistry)
   {
      this.updateDT = updateDT;
      this.sensorFrame = sensorFrame;

      YoRegistry registry = new YoRegistry(imuName + "MahonyFilter");
      parentRegistry.addChild(registry);

      estimatedOrientation.checkReferenceFrameMatch(sensorFrame.getRootFrame());
      if (estimatedAngularVelocity != null)
         estimatedAngularVelocity.checkReferenceFrameMatch(sensorFrame);

      this.estimatedOrientation = estimatedOrientation;
      this.estimatedAngularVelocity = estimatedAngularVelocity;

      yoErrorTerm = new YoFrameVector3D("ErrorTerm", nameSuffix, sensorFrame, registry);
      yoIntegralTerm = new YoFrameVector3D("IntegralTerm", nameSuffix, sensorFrame, registry);

      proportionalGain = new YoDouble("ProportionalGain" + nameSuffix, registry);
      integralGain = new YoDouble("IntegralGain" + nameSuffix, registry);

      if (createYawRateBiasEstimator)
      {
         enableYawRateBiasCorrection = new YoBoolean("EnableYawRateBiasCorrection" + nameSuffix, registry);
         zeroAngularVelocityThreshold = new YoDouble("ZeroAngularVelocityThreshold" + nameSuffix, registry);
         yawRateBiasGain = new YoDouble("YawRateBiasFilterGain" + nameSuffix, registry);
         yawRateBiasEstimate = new YoDouble("YawRateBiasEstimate" + nameSuffix, registry);
         yawRateBiasError = new YoFrameVector3D("YawRateBiasError", nameSuffix, sensorFrame, registry);
      }
      else
      {
         enableYawRateBiasCorrection = null;
         zeroAngularVelocityThreshold = null;
         yawRateBiasGain = null;
         yawRateBiasEstimate = null;
         yawRateBiasError = null;
      }

      hasBeenInitialized = new YoBoolean("HasBeenInitialized" + nameSuffix, registry);
   }

   public YoIMUMahonyFilter(String imuName, String namePrefix, String nameSuffix, double updateDT, ReferenceFrame sensorFrame, YoRegistry parentRegistry)
   {
      this(imuName, namePrefix, nameSuffix, updateDT, false, sensorFrame, parentRegistry);
   }

   public YoIMUMahonyFilter(String imuName,
                            String namePrefix,
                            String nameSuffix,
                            double updateDT,
                            boolean createYawRateBiasEstimator,
                            ReferenceFrame sensorFrame,
                            YoRegistry parentRegistry)
   {
      this.updateDT = updateDT;
      this.sensorFrame = sensorFrame;

      YoRegistry registry = new YoRegistry(imuName + "MahonyFilter");
      parentRegistry.addChild(registry);

      estimatedOrientation = new YoFrameQuaternion(namePrefix + "EstimatedOrientation", nameSuffix, sensorFrame.getRootFrame(), registry);
      estimatedAngularVelocity = new YoFrameVector3D(namePrefix + "EstimatedAngularVelocity", nameSuffix, sensorFrame, registry);

      yoErrorTerm = new YoFrameVector3D("ErrorTerm", nameSuffix, sensorFrame, registry);
      yoIntegralTerm = new YoFrameVector3D("IntegralTerm", nameSuffix, sensorFrame, registry);

      proportionalGain = new YoDouble("ProportionalGain" + nameSuffix, registry);
      integralGain = new YoDouble("IntegralGain" + nameSuffix, registry);

      if (createYawRateBiasEstimator)
      {
         enableYawRateBiasCorrection = new YoBoolean("EnableYawRateBiasCorrection" + nameSuffix, registry);
         zeroAngularVelocityThreshold = new YoDouble("ZeroAngularVelocityThreshold" + nameSuffix, registry);
         yawRateBiasGain = new YoDouble("YawRateBiasGain" + nameSuffix, registry);
         yawRateBiasEstimate = new YoDouble("YawRateBiasEstimate" + nameSuffix, registry);
         yawRateBiasError = new YoFrameVector3D("YawRateBiasError", nameSuffix, sensorFrame, registry);
      }
      else
      {
         enableYawRateBiasCorrection = null;
         zeroAngularVelocityThreshold = null;
         yawRateBiasGain = null;
         yawRateBiasEstimate = null;
         yawRateBiasError = null;
      }

      hasBeenInitialized = new YoBoolean("HasBeenInitialized" + nameSuffix, registry);
   }

   /**
    * Sets the input variables to use when updating this filter.
    * 
    * @param inputAngularVelocity    the variable holding the measurements from the gyroscope. Not
    *                                modified.
    * @param inputLinearAcceleration the variable holding the measurements from the accelerometer. Not
    *                                modified.
    */
   public void setInputs(YoFrameVector3D inputAngularVelocity, YoFrameVector3D inputLinearAcceleration)
   {
      setInputs(inputAngularVelocity, inputLinearAcceleration, null);
   }

   /**
    * Sets the input variables to use when updating this filter.
    * 
    * @param inputAngularVelocity    the variable holding the measurements from the gyroscope. Not
    *                                modified.
    * @param inputLinearAcceleration the variable holding the measurements from the accelerometer. Not
    *                                modified.
    * @param inputMagneticVector     the variable holding the measurements from the magnetometer. Not
    *                                modified.
    */
   public void setInputs(YoFrameVector3D inputAngularVelocity, YoFrameVector3D inputLinearAcceleration, YoFrameVector3D inputMagneticVector)
   {
      if (inputAngularVelocity != null)
         inputAngularVelocity.checkReferenceFrameMatch(sensorFrame);
      if (inputLinearAcceleration != null)
         inputLinearAcceleration.checkReferenceFrameMatch(sensorFrame);
      if (inputMagneticVector != null)
         inputMagneticVector.checkReferenceFrameMatch(sensorFrame);

      this.rawAngularVelocity = inputAngularVelocity;
      this.rawLinearAcceleration = inputLinearAcceleration;
      this.rawMagneticVector = inputMagneticVector;
   }

   /**
    * Sets the gains for this filter.
    * 
    * @param proportionalGain gain used to correct the orientation according to the estimated error.
    * @param integralGain     gain used to update the gyroscope bias according to the estimated error.
    */
   public void setGains(double proportionalGain, double integralGain)
   {
      this.proportionalGain.set(proportionalGain);
      this.integralGain.set(integralGain);
   }

   public void setYawDriftParameters(boolean enableCorrection, double zeroAngularVelocityThreshold, double gain)
   {
      enableYawRateBiasCorrection.set(enableCorrection);
      this.zeroAngularVelocityThreshold.set(zeroAngularVelocityThreshold);
      yawRateBiasGain.set(gain);
   }

   /**
    * Mostly useful for test purpose, but can be used to reinitilize this filter by giving
    * {@code false}.
    * 
    * @param value whether this filter has been initialized or not.
    */
   public void setHasBeenInitialized(boolean value)
   {
      hasBeenInitialized.set(value);
   }

   private final Vector3D rotationUpdate = new Vector3D();
   private final Quaternion quaternionUpdate = new Quaternion();
   private final Vector3D angularVelocityUnbiased = new Vector3D();
   private final Vector3D angularVelocityTerm = new Vector3D();

   @Override
   public void update()
   {
      Vector3DReadOnly inputAngularVelocity = rawAngularVelocity;
      Vector3DReadOnly inputLinearAcceleration = rawLinearAcceleration;

      if (rawMagneticVector != null)
         update(inputAngularVelocity, inputLinearAcceleration, (Vector3DReadOnly) rawMagneticVector);
      else
         update(inputAngularVelocity, inputLinearAcceleration);
   }

   public void update(FrameVector3DReadOnly inputAngularVelocity, FrameVector3DReadOnly inputLinearAcceleration)
   {
      update(inputAngularVelocity, inputLinearAcceleration, null);
   }

   public void update(FrameVector3DReadOnly inputAngularVelocity, FrameVector3DReadOnly inputLinearAcceleration, FrameVector3DReadOnly inputMagneticVector)
   {
      inputAngularVelocity.checkReferenceFrameMatch(sensorFrame);
      inputLinearAcceleration.checkReferenceFrameMatch(sensorFrame);
      if (inputMagneticVector != null)
         inputMagneticVector.checkReferenceFrameMatch(sensorFrame);

      update((Vector3DReadOnly) inputAngularVelocity, (Vector3DReadOnly) inputLinearAcceleration, (Vector3DReadOnly) inputMagneticVector);
   }

   public void update(Vector3DReadOnly inputAngularVelocity, Vector3DReadOnly inputLinearAcceleration)
   {
      update(inputAngularVelocity, inputLinearAcceleration, null);
   }

   public void update(Vector3DReadOnly inputAngularVelocity, Vector3DReadOnly inputLinearAcceleration, Vector3DReadOnly inputMagneticVector)
   {
      if (!hasBeenInitialized.getValue())
      {
         initialize(inputLinearAcceleration, inputMagneticVector);
         return;
      }

      boolean success = computeOrientationError((QuaternionReadOnly) estimatedOrientation, inputLinearAcceleration, inputMagneticVector, yoErrorTerm);

      if (success)
      {
         angularVelocityTerm.scaleAdd(proportionalGain.getValue(), yoErrorTerm, inputAngularVelocity);

         if (integralGain.getValue() > 0.0)
         {
            yoIntegralTerm.scaleAdd(integralGain.getValue() * updateDT, yoErrorTerm, yoIntegralTerm);
            angularVelocityTerm.add(yoIntegralTerm);
         }
         else
         {
            yoIntegralTerm.setToZero();
         }

         angularVelocityUnbiased.add(inputAngularVelocity, yoIntegralTerm);
      }
      else
      {
         yoErrorTerm.setToZero();
         angularVelocityTerm.set(inputAngularVelocity);
         angularVelocityUnbiased.set(inputAngularVelocity);
      }

      if (yawRateBiasError != null)
      {
         updateYawRateBiasEstimator(estimatedOrientation, angularVelocityUnbiased, yawRateBiasError);
         if (enableYawRateBiasCorrection.getValue())
         {
            angularVelocityTerm.add(yawRateBiasError);
            angularVelocityUnbiased.add(yawRateBiasError);
         }
      }

      rotationUpdate.setAndScale(updateDT, angularVelocityTerm);
      quaternionUpdate.setRotationVector(rotationUpdate);
      estimatedOrientation.multiply(quaternionUpdate);

      if (estimatedAngularVelocity != null)
         estimatedAngularVelocity.set(angularVelocityUnbiased);
   }

   private void updateYawRateBiasEstimator(QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity, Vector3DBasics yawRateVector)
   {
      if (angularVelocity.lengthSquared() > MathTools.square(zeroAngularVelocityThreshold.getValue()))
         return;

      orientation.transform(angularVelocity, yawRateVector);

      yawRateBiasEstimate.set(EuclidCoreTools.interpolate(yawRateBiasEstimate.getValue(), yawRateVector.getZ(), yawRateBiasGain.getValue()));
      yawRateVector.set(0, 0, -yawRateBiasEstimate.getValue());
      orientation.inverseTransform(yawRateVector);
   }

   @Override
   public void reset()
   {
      hasBeenInitialized.set(false);
   }

   private void initialize(Vector3DReadOnly acceleration, Vector3DReadOnly magneticVector)
   {
      boolean success = computeRotationMatrixFromXZAxes(magneticVector, acceleration, estimatedOrientation);

      if (!success)
         estimatedOrientation.setToZero();
      else
         estimatedOrientation.invert();

      yoIntegralTerm.setToZero();
      yawRateBiasEstimate.set(0.0);
      hasBeenInitialized.set(true);
   }

   // ------------------------------------------------ //
   // Intermediate variables for computing the orientation error
   private final Vector3D m = new Vector3D();
   private final Vector3D mRef = new Vector3D();

   private final Vector3D a = new Vector3D();
   private final Vector3D aRef = new Vector3D();
   // ------------------------------------------------ //

   /**
    * Estimates the error in orientation based on the measurements from the accelerometer and
    * magnetometer.
    * <p>
    * The error in orientation becomes zero when the acceleration is aligned with the z-up vector in
    * world and the magnetic vector lies in the xz-plane in world.
    * </p>
    * <p>
    * If there is no measurement from the magnetometer, i.e. {@code measuredMagneticVector == null},
    * the error is only computed from the accelerometer data.
    * </p>
    * 
    * @param orientation    the current estimate of the sensor orientation with respect to world. Not
    *                       modified.
    * @param acceleration   the measurement from the accelerometer. Not modified.
    * @param magneticVector the measurement from the magnetometer. Can be {@code null}. Not modified.
    * @param errorToPack    the error in the local frame of {@code orientation} to add to the angular
    *                       velocity. Modified.
    * @return whether the method was completed successfully or not.
    */
   private boolean computeOrientationError(QuaternionReadOnly orientation,
                                           Vector3DReadOnly acceleration,
                                           Vector3DReadOnly magneticVector,
                                           Vector3DBasics errorToPack)
   {
      boolean success = false;
      errorToPack.setToZero();

      if (magneticVector != null)
      {
         double norm = magneticVector.length();
         if (norm >= MIN_MAGNITUDE)
         {
            m.setAndScale(1.0 / norm, magneticVector);

            orientation.transform(m, mRef);
            // The magnetometer is only used to correct the heading in world. So we preserve the Z-component and XY magnitude to ensure that no pitch error is generated.
            mRef.setX(EuclidCoreTools.norm(mRef.getX(), mRef.getY()));
            mRef.setY(0.0);
            orientation.inverseTransform(mRef);

            errorToPack.cross(m, mRef);
            success = true;
         }
      }

      if (acceleration != null)
      {
         double norm = acceleration.length();

         if (norm >= MIN_MAGNITUDE)
         {
            a.setAndScale(1.0 / norm, acceleration);

            orientation.inverseTransform(ACCELERATION_REFERENCE, aRef);

            double ex = errorToPack.getX();
            double ey = errorToPack.getY();
            double ez = errorToPack.getZ();
            errorToPack.cross(a, aRef);
            errorToPack.add(ex, ey, ez);
            success = true;
         }
      }

      return success;
   }

   public YoFrameQuaternion getEstimatedOrientation()
   {
      return estimatedOrientation;
   }

   public YoFrameVector3D getEstimatedAngularVelocity()
   {
      return estimatedAngularVelocity;
   }

   public YoFrameVector3D getErrorTerm()
   {
      return yoErrorTerm;
   }

   public YoFrameVector3D getIntegralTerm()
   {
      return yoIntegralTerm;
   }

   /**
    * Computes the rotation matrix describing the orientation of the given x and z axes.
    * 
    * @param xAxis             the x-axis to use in the calculation. Not modified.
    * @param zAxis             the z-axis to use in the calculation. Not modified.
    * @param orientationToPack the rotation matrix in which the result is stored. Modified.
    * @return whether the method succeeded or not.
    */
   private static boolean computeRotationMatrixFromXZAxes(Vector3DReadOnly xAxis, Vector3DReadOnly zAxis, Orientation3DBasics orientationToPack)
   {
      if (xAxis == null)
         xAxis = Axis3D.X;
      if (zAxis == null)
         zAxis = Axis3D.Z;

      double zAxisX = zAxis.getX();
      double zAxisY = zAxis.getY();
      double zAxisZ = zAxis.getZ();
      { // Normalize z-axis
         double length = zAxis.length();

         if (length < MIN_MAGNITUDE)
            return false;

         length = 1.0 / length;
         zAxisX *= length;
         zAxisY *= length;
         zAxisZ *= length;
      }

      // yAxis = zAxis X xAxis
      double yAxisX = zAxisY * xAxis.getZ() - zAxisZ * xAxis.getY();
      double yAxisY = zAxisZ * xAxis.getX() - zAxisX * xAxis.getZ();
      double yAxisZ = zAxisX * xAxis.getY() - zAxisY * xAxis.getX();

      { // Normalize y-axis
         double length = Math.sqrt(EuclidCoreTools.normSquared(yAxisX, yAxisY, yAxisZ));

         if (length < MIN_MAGNITUDE)
            return false;

         length = 1.0 / length;
         yAxisX *= length;
         yAxisY *= length;
         yAxisZ *= length;
      }

      // xAxis = yAxis X zAxis
      double xAxisX = yAxisY * zAxisZ - yAxisZ * zAxisY;
      double xAxisY = yAxisZ * zAxisX - yAxisX * zAxisZ;
      double xAxisZ = yAxisX * zAxisY - yAxisY * zAxisX;

      if (orientationToPack instanceof RotationMatrixBasics)
         ((RotationMatrix) orientationToPack).setUnsafe(xAxisX, yAxisX, zAxisX, xAxisY, yAxisY, zAxisY, xAxisZ, yAxisZ, zAxisZ);
      else
         orientationToPack.setRotationMatrix(xAxisX, yAxisX, zAxisX, xAxisY, yAxisY, zAxisY, xAxisZ, yAxisZ, zAxisZ);

      return true;
   }
}
