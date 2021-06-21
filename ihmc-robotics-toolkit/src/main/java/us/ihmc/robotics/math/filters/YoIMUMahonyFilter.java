package us.ihmc.robotics.math.filters;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TupleTools;
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
   private static final double GRAVITY_DEFAULT_VALUE = 9.81;

   public static final Vector3DReadOnly ACCELERATION_REFERENCE = Axis3D.Z;
   public static final Vector3DReadOnly NORTH_REFERENCE = Axis3D.X;

   private YoFrameVector3D rawAngularVelocity;
   private YoFrameVector3D rawLinearAcceleration;
   private YoFrameVector3D rawMagneticVector;

   private final YoFrameQuaternion estimatedOrientation;
   private final YoFrameVector3D estimatedAngularVelocity;

   private final YoFrameVector3D orientationError;
   private final YoFrameVector3D angularVelocityBias;

   private final YoDouble proportionalGain;
   private final YoDouble integralGain;

   private final YoDouble zeroAngularVelocityThreshold;
   private final YoDouble zeroLinearAccelerationThreshold;
   private final YoDouble yawRateBiasGain;

   private final double updateDT;
   private double gravityMagnitude = GRAVITY_DEFAULT_VALUE;

   private final YoBoolean hasBeenInitialized;

   private final ReferenceFrame sensorFrame;

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
      this(imuName, namePrefix, nameSuffix, updateDT, createYawRateBiasEstimator, sensorFrame, null, null, parentRegistry);
   }

   public YoIMUMahonyFilter(String imuName,
                            String namePrefix,
                            String nameSuffix,
                            double updateDT,
                            ReferenceFrame sensorFrame,
                            YoFrameQuaternion estimatedOrientation,
                            YoFrameVector3D estimatedAngularVelocity,
                            YoRegistry parentRegistry)
   {
      this(imuName, namePrefix, nameSuffix, updateDT, false, sensorFrame, estimatedOrientation, estimatedAngularVelocity, parentRegistry);
   }

   public YoIMUMahonyFilter(String imuName,
                            String namePrefix,
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

      if (estimatedOrientation != null)
         estimatedOrientation.checkReferenceFrameMatch(sensorFrame.getRootFrame());
      else
         estimatedOrientation = new YoFrameQuaternion(namePrefix, nameSuffix, sensorFrame.getRootFrame(), registry);

      if (estimatedAngularVelocity != null)
         estimatedAngularVelocity.checkReferenceFrameMatch(sensorFrame);
      else
         estimatedAngularVelocity = new YoFrameVector3D(namePrefix, nameSuffix, sensorFrame, registry);

      this.estimatedOrientation = estimatedOrientation;
      this.estimatedAngularVelocity = estimatedAngularVelocity;

      orientationError = new YoFrameVector3D(namePrefix + "OrientationError", nameSuffix, sensorFrame, registry);
      angularVelocityBias = new YoFrameVector3D(namePrefix + "AngularVelocityBias", nameSuffix, sensorFrame, registry);

      proportionalGain = new YoDouble(namePrefix + "ProportionalGain" + nameSuffix, registry);
      integralGain = new YoDouble(namePrefix + "IntegralGain" + nameSuffix, registry);

      zeroLinearAccelerationThreshold = new YoDouble(namePrefix + "ZeroLinearAccelerationThreshold" + nameSuffix, registry);

      if (createYawRateBiasEstimator)
      {
         zeroAngularVelocityThreshold = new YoDouble(namePrefix + "ZeroAngularVelocityThreshold" + nameSuffix, registry);
         yawRateBiasGain = new YoDouble(namePrefix + "YawRateBiasGain" + nameSuffix, registry);
      }
      else
      {
         zeroAngularVelocityThreshold = null;
         yawRateBiasGain = null;
      }

      hasBeenInitialized = new YoBoolean(namePrefix + "HasBeenInitialized" + nameSuffix, registry);
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

   /**
    * Sets the gains for this filter.
    * 
    * @param proportionalGain                gain used to correct the orientation according to the
    *                                        estimated error.
    * @param integralGain                    gain used to update the gyroscope bias according to the
    *                                        estimated error.
    * @param zeroLinearAccelerationThreshold when and only when the magnitude of the linear
    *                                        acceleration magnitude is less that this threshold, the
    *                                        integral term is updated. Set to {@code 0.0} to always
    *                                        update the integral term.
    */
   public void setGains(double proportionalGain, double integralGain, double zeroLinearAccelerationThreshold)
   {
      this.proportionalGain.set(proportionalGain);
      this.integralGain.set(integralGain);
      this.zeroLinearAccelerationThreshold.set(zeroLinearAccelerationThreshold);
   }

   public void setGravityMagnitude(double gravityMagnitude)
   {
      this.gravityMagnitude = gravityMagnitude;
   }

   public void setYawDriftParameters(double zeroAngularVelocityThreshold, double gain)
   {
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
      Vector3DReadOnly inputMagneticVector = rawMagneticVector;

      if (inputMagneticVector != null)
         update(inputAngularVelocity, inputLinearAcceleration, inputMagneticVector);
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

      boolean success = computeOrientationError(estimatedOrientation, inputLinearAcceleration, inputMagneticVector, orientationError);

      if (success)
      {
         angularVelocityTerm.scaleAdd(proportionalGain.getValue(), orientationError, inputAngularVelocity);

         boolean hasIntegralTerm = updateIntegralTerm(angularVelocityBias,
                                                      inputMagneticVector != null,
                                                      estimatedOrientation,
                                                      inputAngularVelocity,
                                                      inputLinearAcceleration,
                                                      orientationError);
         if (hasIntegralTerm)
            angularVelocityTerm.add(angularVelocityBias);

         angularVelocityUnbiased.add(inputAngularVelocity, angularVelocityBias);
      }
      else
      {
         orientationError.setToZero();
         angularVelocityTerm.set(inputAngularVelocity);
         angularVelocityUnbiased.set(inputAngularVelocity);
      }

      rotationUpdate.setAndScale(updateDT, angularVelocityTerm);
      quaternionUpdate.setRotationVector(rotationUpdate);
      estimatedOrientation.multiply(quaternionUpdate);

      if (estimatedAngularVelocity != null)
         estimatedAngularVelocity.set(angularVelocityUnbiased);
   }

   @Override
   public void reset()
   {
      hasBeenInitialized.set(false);
   }

   private boolean hasDesiredInitialHeading = false;
   private final Vector3D desiredInitialHeading = new Vector3D();

   public void setDesiredInitialHeading(Vector3DReadOnly desiredInitialHeading)
   {
      this.desiredInitialHeading.set(desiredInitialHeading);
      hasDesiredInitialHeading = true;
   }

   public void initialize(Orientation3DReadOnly initialOrientation)
   {
      estimatedOrientation.set(initialOrientation);
      angularVelocityBias.setToZero();
      hasBeenInitialized.set(true);
   }

   private void initialize(Vector3DReadOnly acceleration, Vector3DReadOnly magneticVector)
   {
      if (magneticVector == null && hasDesiredInitialHeading)
      {
         magneticVector = desiredInitialHeading;
      }

      boolean success = computeRotationMatrixFromXZAxes(magneticVector, acceleration, estimatedOrientation);

      if (!success)
         estimatedOrientation.setToZero();
      else
         estimatedOrientation.invert();

      angularVelocityBias.setToZero();
      hasBeenInitialized.set(true);
   }

   // ------------------------------------------------ //
   // Intermediate variables for computing the orientation error and the integral term.
   private final Vector3D m = new Vector3D();
   private final Vector3D mRef = new Vector3D();

   private final Vector3D a = new Vector3D();
   private final Vector3D aRef = new Vector3D();

   private final Vector3D normalPart = new Vector3D();
   private final Vector3D tangentialPart = new Vector3D();
   // ------------------------------------------------ //

   private boolean updateIntegralTerm(Vector3DBasics integralTerm,
                                      boolean hasMagneticVector,
                                      Orientation3DReadOnly orientation,
                                      Vector3DReadOnly angularVelocity,
                                      Vector3DReadOnly linearAcceleration,
                                      Vector3DReadOnly errorTerm)
   {
      if (!Double.isFinite(integralGain.getValue()) || integralGain.getValue() <= 0.0)
      {
         integralTerm.setToZero();
         return false;
      }

      orientation.inverseTransform(ACCELERATION_REFERENCE, aRef);
      a.setAndScale(gravityMagnitude, aRef); // Acceleration when the IMU is stationary
      a.sub(linearAcceleration, a); // Current acceleration without gravity.

      // Update the integral term when the threshold is not set or the a cc
      if (zeroLinearAccelerationThreshold.getValue() > 0.0 && a.lengthSquared() > MathTools.square(zeroLinearAccelerationThreshold.getValue()))
         return true;

      integralTerm.scaleAdd(integralGain.getValue() * updateDT, errorTerm, integralTerm);

      if (hasMagneticVector)
         return true;

      if (yawRateBiasGain != null)
      {
         if (angularVelocity.lengthSquared() > MathTools.square(zeroAngularVelocityThreshold.getValue()))
            return true;

         double normalPartMagnitude = TupleTools.dot(aRef, integralTerm);

         if (Double.isFinite(normalPartMagnitude) && normalPartMagnitude != 0.0)
         {
            normalPart.setAndScale(normalPartMagnitude, aRef);
            tangentialPart.sub(integralTerm, normalPart);
            double yawRateError = -angularVelocity.dot(aRef);
            double ajustedNormalMagnitude = EuclidCoreTools.interpolate(normalPartMagnitude, yawRateError, yawRateBiasGain.getValue());
            normalPart.scale(ajustedNormalMagnitude / normalPartMagnitude);
            integralTerm.add(normalPart, tangentialPart);
         }
      }
      else
      {
         // If we don't have a magnetic vector, the error around the gravity vector cannot be estimated. So we slowly decay it.
         double normalPartMagnitude = TupleTools.dot(aRef, integralTerm);

         if (Double.isFinite(normalPartMagnitude) && normalPartMagnitude != 0.0)
         {
            normalPart.setAndScale(normalPartMagnitude, aRef);
            tangentialPart.sub(integralTerm, normalPart);
            normalPart.scale(1.0 - integralGain.getValue());
            integralTerm.add(normalPart, tangentialPart);
         }
      }

      return true;
   }

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
         if (Double.isFinite(norm) && norm >= MIN_MAGNITUDE)
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

         if (Double.isFinite(norm) && norm >= MIN_MAGNITUDE)
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
      return orientationError;
   }

   public YoFrameVector3D getIntegralTerm()
   {
      return angularVelocityBias;
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
