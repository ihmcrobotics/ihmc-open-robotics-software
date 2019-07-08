package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

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

   public static final Vector3DReadOnly ACCELERATION_REFERENCE = new Vector3DReadOnly()
   {
      @Override
      public double getX()
      {
         return 0;
      }

      @Override
      public double getY()
      {
         return 0;
      }

      @Override
      public double getZ()
      {
         return 1.0;
      }
   };

   public static final Vector3DReadOnly NORTH_REFERENCE = new Vector3DReadOnly()
   {
      @Override
      public double getX()
      {
         return 1.0;
      }

      @Override
      public double getY()
      {
         return 0;
      }

      @Override
      public double getZ()
      {
         return 0.0;
      }
   };

   private YoFrameVector3D rawAngularVelocity;
   private YoFrameVector3D rawLinearAcceleration;
   private YoFrameVector3D rawMagneticVector;

   private final YoFrameQuaternion estimatedOrientation;
   private final YoFrameVector3D estimatedAngularVelocity;

   private final YoFrameVector3D yoErrorTerm;
   private final YoFrameVector3D yoIntegralTerm;

   private final YoDouble proportionalGain;
   private final YoDouble integralGain;

   private final double updateDT;

   private final YoBoolean hasBeenInitialized;

   private final ReferenceFrame sensorFrame;

   public YoIMUMahonyFilter(String imuName, String nameSuffix, double updateDT, ReferenceFrame sensorFrame, YoFrameQuaternion estimatedOrientation,
                            YoFrameVector3D estimatedAngularVelocity, YoVariableRegistry parentRegistry)
   {
      this.updateDT = updateDT;
      this.sensorFrame = sensorFrame;

      YoVariableRegistry registry = new YoVariableRegistry(imuName + "MahonyFilter");
      parentRegistry.addChild(registry);

      estimatedOrientation.checkReferenceFrameMatch(sensorFrame);
      if (estimatedAngularVelocity != null)
         estimatedAngularVelocity.checkReferenceFrameMatch(sensorFrame);

      this.estimatedOrientation = estimatedOrientation;
      this.estimatedAngularVelocity = estimatedAngularVelocity;

      yoErrorTerm = new YoFrameVector3D("ErrorTerm", nameSuffix, sensorFrame, registry);
      yoIntegralTerm = new YoFrameVector3D("IntegralTerm", nameSuffix, sensorFrame, registry);

      proportionalGain = new YoDouble("ProportionalGain" + nameSuffix, registry);
      integralGain = new YoDouble("IntegralGain" + nameSuffix, registry);

      hasBeenInitialized = new YoBoolean("HasBeenInitialized" + nameSuffix, registry);
   }

   public YoIMUMahonyFilter(String imuName, String namePrefix, String nameSuffix, double updateDT, ReferenceFrame sensorFrame,
                            YoVariableRegistry parentRegistry)
   {
      this.updateDT = updateDT;
      this.sensorFrame = sensorFrame;

      YoVariableRegistry registry = new YoVariableRegistry(imuName + "MahonyFilter");
      parentRegistry.addChild(registry);

      estimatedOrientation = new YoFrameQuaternion(namePrefix, nameSuffix, sensorFrame, registry);
      estimatedAngularVelocity = new YoFrameVector3D(namePrefix, nameSuffix, sensorFrame, registry);

      yoErrorTerm = new YoFrameVector3D("ErrorTerm", nameSuffix, sensorFrame, registry);
      yoIntegralTerm = new YoFrameVector3D("IntegralTerm", nameSuffix, sensorFrame, registry);

      proportionalGain = new YoDouble("ProportionalGain" + nameSuffix, registry);
      integralGain = new YoDouble("IntegralGain" + nameSuffix, registry);

      hasBeenInitialized = new YoBoolean("HasBeenInitialized" + nameSuffix, registry);
   }

   /**
    * Sets the input variables to use when updating this filter.
    * 
    * @param inputAngularVelocity
    *           the variable holding the measurements from the gyroscope. Not modified.
    * @param inputLinearAcceleration
    *           the variable holding the measurements from the accelerometer. Not modified.
    */
   public void setInputs(YoFrameVector3D inputAngularVelocity, YoFrameVector3D inputLinearAcceleration)
   {
      setInputs(inputAngularVelocity, inputLinearAcceleration, null);
   }

   /**
    * Sets the input variables to use when updating this filter.
    * 
    * @param inputAngularVelocity
    *           the variable holding the measurements from the gyroscope. Not modified.
    * @param inputLinearAcceleration
    *           the variable holding the measurements from the accelerometer. Not modified.
    * @param inputMagneticVector
    *           the variable holding the measurements from the magnetometer. Not modified.
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
    * @param proportionalGain
    *           gain used to correct the orientation according to the estimated error.
    * @param integralGain
    *           gain used to update the gyroscope bias according to the estimated error.
    */
   public void setGains(double proportionalGain, double integralGain)
   {
      this.proportionalGain.set(proportionalGain);
      this.integralGain.set(integralGain);
   }

   /**
    * Mostly useful for test purpose, but can be used to reinitilize this filter by giving
    * {@code false}.
    * 
    * @param value
    *           whether this filter has been initialized or not.
    */
   public void setHasBeenInitialized(boolean value)
   {
      hasBeenInitialized.set(value);
   }

   private final Vector3D rotationUpdate = new Vector3D();
   private final Quaternion quaternionUpdate = new Quaternion();

   private final Vector3D totalError = new Vector3D();
   private final Vector3D integralTerm = new Vector3D();
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

      boolean success = computeOrientationError((QuaternionReadOnly) estimatedOrientation, inputLinearAcceleration, inputMagneticVector, quaternionUpdate);
      if (success)
      {
         quaternionUpdate.getRotationVector(totalError);
         yoErrorTerm.set(totalError);

         integralTerm.scaleAdd(integralGain.getValue() * updateDT, yoErrorTerm, yoIntegralTerm);
         yoIntegralTerm.set(integralTerm);

         angularVelocityTerm.scaleAdd(proportionalGain.getValue(), totalError, inputAngularVelocity);
         angularVelocityTerm.add(integralTerm);
      }
      else
      {
         yoErrorTerm.setToZero();
         angularVelocityTerm.set(inputAngularVelocity);
      }

      rotationUpdate.setAndScale(updateDT, angularVelocityTerm);
      quaternionUpdate.setRotationVector(rotationUpdate);
      estimatedOrientation.multiply(quaternionUpdate);

      if (estimatedAngularVelocity != null)
         estimatedAngularVelocity.add(inputAngularVelocity, integralTerm);
   }

   @Override
   public void reset()
   {
      hasBeenInitialized.set(false);
   }

   private void initialize(Vector3DReadOnly acceleration, Vector3DReadOnly magneticVector)
   {
      boolean success = computeOrientationError(estimatedOrientation, acceleration, magneticVector, quaternionUpdate);

      if (!success)
         return;

      estimatedOrientation.multiply(quaternionUpdate);
      yoIntegralTerm.setToZero();

      hasBeenInitialized.set(true);
   }

   private final Vector3D accelerationCurrentDirectionWorld = new Vector3D();
   private final Vector3D accelerationDesiredDirectionWorld = new Vector3D();
   private final Vector3D magneticCurrentDirectionWorld = new Vector3D();
   private final Vector3D magneticDesiredDirectionWorld = new Vector3D();

   private final AxisAngle axisAngleOffset = new AxisAngle();

   private final RotationMatrix rotationMatrixCurrent = new RotationMatrix();
   private final RotationMatrix rotationMatrixDesired = new RotationMatrix();

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
    * @param orientation
    *           the current estimate of the sensor orientation with respect to world. Not modified.
    * @param acceleration
    *           the measurement from the accelerometer. Not modified.
    * @param magneticVector
    *           the measurement from the magnetometer. Can be {@code null}. Not modified.
    * @param errorToPack
    *           the error in orientation expressed in the local frame of {@code orientation}.
    *           Modified.
    * @return whether the method was completed successfully or not.
    */
   private boolean computeOrientationError(QuaternionReadOnly orientation, Vector3DReadOnly acceleration, Vector3DReadOnly magneticVector,
                                           QuaternionBasics errorToPack)
   {
      boolean success;
      double accelerationLength = acceleration.length();

      if (accelerationLength < MIN_MAGNITUDE)
         return false;

      accelerationCurrentDirectionWorld.setAndScale(1.0 / accelerationLength, acceleration);
      orientation.transform(accelerationCurrentDirectionWorld);
      accelerationDesiredDirectionWorld.set(ACCELERATION_REFERENCE);

      boolean useMagneticVector = false;

      if (magneticVector != null)
      {
         double magneticLength = magneticVector.length();

         if (magneticLength > MIN_MAGNITUDE)
         {
            useMagneticVector = true;
            magneticCurrentDirectionWorld.setAndScale(1.0 / magneticLength, magneticVector);
            orientation.transform(magneticCurrentDirectionWorld);
         }
      }

      if (useMagneticVector)
      {
         success = computeRotationMatrixFromXZAxes(magneticCurrentDirectionWorld, accelerationCurrentDirectionWorld, rotationMatrixCurrent);
         if (success)
         {
            // Setting the desired direction to be x+ in the xz-plane
            magneticDesiredDirectionWorld.setX(Math.sqrt(EuclidCoreTools.normSquared(magneticCurrentDirectionWorld.getX(),
                                                                                     magneticCurrentDirectionWorld.getY())));
            magneticDesiredDirectionWorld.setY(0.0);
            magneticDesiredDirectionWorld.setZ(magneticCurrentDirectionWorld.getZ());

            success = computeRotationMatrixFromXZAxes(magneticDesiredDirectionWorld, accelerationDesiredDirectionWorld, rotationMatrixDesired);
            if (success)
            {
               rotationMatrixCurrent.multiplyTransposeThis(rotationMatrixDesired);
               errorToPack.set(rotationMatrixCurrent);
               /*
                * Whether the current/desired directions are expressed in world or local frame, it
                * seems that the rotation axis of the resulting error stored in
                * rotationMatrixCurrent is expressed in world and needs to be transformed as
                * follows. I don't really understand how is that.
                */
               errorToPack.multiply(orientation);
               errorToPack.preMultiplyConjugateOther(orientation);
               // It seems rather important to keep the rotation angle in [-pi,pi] to prevent jumps in the estimated orientation.
               errorToPack.normalizeAndLimitToPi();
               return true;
            }
         }
      }
      EuclidGeometryTools.axisAngleFromFirstToSecondVector3D(accelerationCurrentDirectionWorld, accelerationDesiredDirectionWorld, axisAngleOffset);
      errorToPack.set(axisAngleOffset);

      return true;
   }

   public YoFrameQuaternion getEstimatedOrientation()
   {
      return estimatedOrientation;
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
    * @param xAxis
    *           the x-axis to use in the calculation. Not modified.
    * @param zAxis
    *           the z-axis to use in the calculation. Not modified.
    * @param rotationToPack
    *           the rotation matrix in which the result is stored. Modified.
    * @return whether the method succeeded or not.
    */
   private static boolean computeRotationMatrixFromXZAxes(Vector3DReadOnly xAxis, Vector3DReadOnly zAxis, RotationMatrix rotationToPack)
   {
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

      rotationToPack.setUnsafe(xAxisX, yAxisX, zAxisX, xAxisY, yAxisY, zAxisY, xAxisZ, yAxisZ, zAxisZ);

      return true;
   }
}
