package us.ihmc.robotics.math.filters;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoIMUMahonyFilterTest
{
   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testInitialization() throws Exception
   {
      String imuName = "test";
      String namePrefix = "q_est_";
      String nameSuffix = "";
      double updateDT = 1.0e-3;
      YoRegistry registry = new YoRegistry("test");
      Random random = new Random(3454);
      ReferenceFrame sensorFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
      YoFrameQuaternion actualOrientation = new YoFrameQuaternion("q_act_", sensorFrame, registry);

      YoFrameVector3D inputAngularVelocity = new YoFrameVector3D("angularVelocity", sensorFrame, registry);
      YoFrameVector3D inputLinearAcceleration = new YoFrameVector3D("linearAcceleration", sensorFrame, registry);
      YoFrameVector3D inputMagneticVector = new YoFrameVector3D("magneticDirection", sensorFrame, registry);
      YoIMUMahonyFilter mahonyFilter = new YoIMUMahonyFilter(imuName, namePrefix, nameSuffix, updateDT, sensorFrame, registry);
      mahonyFilter.setInputs(inputAngularVelocity, inputLinearAcceleration, inputMagneticVector);
      mahonyFilter.setGains(0.5, 0.0);
      mahonyFilter.getEstimatedOrientation().set(EuclidCoreRandomTools.nextQuaternion(random));

      actualOrientation.set(EuclidCoreRandomTools.nextQuaternion(random));
      Vector3D zUp = new Vector3D(0.0, 0.0, 1.0);
      actualOrientation.inverseTransform(zUp);
      inputLinearAcceleration.set(zUp);
      inputLinearAcceleration.scale(20.0);

      Vector3D xForward = new Vector3D(1.0, 0.0, 0.0);
      actualOrientation.inverseTransform(xForward);
      inputMagneticVector.set(xForward);
      inputMagneticVector.scale(0.5);

      mahonyFilter.update();
      double currentError = actualOrientation.distance(mahonyFilter.getEstimatedOrientation());

      assertTrue(currentError <= 1.0e-10);
   }

   @Test
   public void testConvergenceToStaticOrientation() throws Exception
   {
      String imuName = "test";
      String namePrefix = "q_est_";
      String nameSuffix = "";
      double updateDT = 1.0e-3;
      YoRegistry registry = new YoRegistry("test");
      Random random = new Random(3454);
      ReferenceFrame sensorFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
      YoFrameQuaternion actualOrientation = new YoFrameQuaternion("q_act_", sensorFrame, registry);

      YoFrameVector3D inputAngularVelocity = new YoFrameVector3D("angularVelocity", sensorFrame, registry);
      YoFrameVector3D inputLinearAcceleration = new YoFrameVector3D("linearAcceleration", sensorFrame, registry);
      YoFrameVector3D inputMagneticVector = new YoFrameVector3D("magneticDirection", sensorFrame, registry);
      YoIMUMahonyFilter mahonyFilter = new YoIMUMahonyFilter(imuName, namePrefix, nameSuffix, updateDT, sensorFrame, registry);
      mahonyFilter.setInputs(inputAngularVelocity, inputLinearAcceleration, inputMagneticVector);
      mahonyFilter.setHasBeenInitialized(true); // So the estimated orientation does not get snapped to the actual

      mahonyFilter.setGains(0.5, 0.0);

      actualOrientation.set(EuclidCoreRandomTools.nextQuaternion(random));
      Vector3D zUp = new Vector3D(0.0, 0.0, 1.0);
      actualOrientation.inverseTransform(zUp);
      inputLinearAcceleration.set(zUp);
      inputLinearAcceleration.scale(20.0);

      Vector3D xForward = new Vector3D(1.0, 0.0, 0.0);
      actualOrientation.inverseTransform(xForward);
      inputMagneticVector.set(xForward);
      inputMagneticVector.scale(0.5);

      double currentError = 0.0;
      double previousError = Double.POSITIVE_INFINITY;

      for (int i = 0; i < 50000; i++)
      {
         mahonyFilter.update();

         currentError = actualOrientation.distance(mahonyFilter.getEstimatedOrientation());
         assertTrue(currentError <= previousError);
         previousError = currentError;
      }

      assertTrue(currentError <= 3.0e-4);
   }

   @Test
   public void testConvergenceToStaticOrientationWithGyroBias() throws Exception
   {
      String imuName = "test";
      String namePrefix = "q_est_";
      String nameSuffix = "";
      double updateDT = 1.0e-3;
      YoRegistry registry = new YoRegistry("test");
      Random random = new Random(3454);
      ReferenceFrame sensorFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
      YoFrameQuaternion actualOrientation = new YoFrameQuaternion("q_act_", sensorFrame, registry);

      YoFrameVector3D inputAngularVelocity = new YoFrameVector3D("angularVelocity", sensorFrame, registry);
      YoFrameVector3D inputLinearAcceleration = new YoFrameVector3D("linearAcceleration", sensorFrame, registry);
      YoFrameVector3D inputMagneticVector = new YoFrameVector3D("magneticDirection", sensorFrame, registry);
      YoIMUMahonyFilter mahonyFilter = new YoIMUMahonyFilter(imuName, namePrefix, nameSuffix, updateDT, sensorFrame, registry);
      mahonyFilter.setInputs(inputAngularVelocity, inputLinearAcceleration, inputMagneticVector);
      mahonyFilter.setHasBeenInitialized(true); // So the estimated orientation does not get snapped to the actual
      mahonyFilter.setGains(0.5, 0.05);

      inputAngularVelocity.set(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 0.1));

      actualOrientation.set(EuclidCoreRandomTools.nextQuaternion(random));
      Vector3D zUp = new Vector3D(0.0, 0.0, 1.0);
      actualOrientation.inverseTransform(zUp);
      inputLinearAcceleration.set(zUp);
      inputLinearAcceleration.scale(20.0);

      Vector3D xForward = new Vector3D(1.0, 0.0, 0.0);
      actualOrientation.inverseTransform(xForward);
      inputMagneticVector.set(xForward);
      inputMagneticVector.scale(0.5);

      double currentError = 0.0;

      for (int i = 0; i < 100000; i++)
      {
         mahonyFilter.update();
         currentError = actualOrientation.distance(mahonyFilter.getEstimatedOrientation());
      }

      System.out.println(currentError);
      assertTrue(currentError <= 3.0e-4);
      Vector3D estimatedBias = new Vector3D();
      estimatedBias.set(mahonyFilter.getIntegralTerm());
      estimatedBias.negate();
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(inputAngularVelocity, estimatedBias, 1.0e-4);
   }

   @Test
   public void testAgainstOriginalAlgorithm() throws Exception
   {
      Random random = new Random(435364);

      double gravity = 9.81;

      String imuName = "test";
      String namePrefix = "q_est_";
      String nameSuffix = "";
      double updateDT = 1.0e-3;
      YoRegistry registry = new YoRegistry("test");
      ReferenceFrame sensorFrame = EuclidFrameRandomTools.nextReferenceFrame(random);

      double kp = 0.5;
      double ki = 0.1;

      YoIMUMahonyFilter mahonyFilter = new YoIMUMahonyFilter(imuName, namePrefix, nameSuffix, updateDT, sensorFrame, registry);
      mahonyFilter.setGains(kp, ki);
      mahonyFilter.setHasBeenInitialized(true); // So the estimated orientation does not get snapped to the actual

      OriginalMahonyFilter originalMahonyFilter = new OriginalMahonyFilter();
      originalMahonyFilter.setSampleFreq(1.0 / updateDT);
      originalMahonyFilter.setGains(kp, ki);

      for (int i = 0; i < 10000; i++)
      {
         Quaternion initialOrientation = EuclidCoreRandomTools.nextQuaternion(random);
         Vector3D angularVelocity = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D linearAcceleration = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, EuclidCoreRandomTools.nextDouble(random, 0.0, 2.0) * gravity);

         mahonyFilter.getEstimatedOrientation().set(initialOrientation);
         mahonyFilter.getIntegralTerm().setToZero();
         mahonyFilter.update(angularVelocity, linearAcceleration);

         originalMahonyFilter.quaternion.set(initialOrientation);
         originalMahonyFilter.reset();
         originalMahonyFilter.updateIMU(angularVelocity, linearAcceleration);

         EuclidCoreTestTools.assertTuple3DEquals(mahonyFilter.getErrorTerm(), originalMahonyFilter.error, 1.0e-12);
         EuclidCoreTestTools.assertTuple3DEquals(mahonyFilter.getIntegralTerm(), originalMahonyFilter.integralFB, 1.0e-12);
         EuclidCoreTestTools.assertTuple4DEquals(mahonyFilter.getEstimatedOrientation(), originalMahonyFilter.quaternion, 1.0e-12);
      }

   }

   private static class OriginalMahonyFilter
   {
      private double sampleFreq;
      private double twoKp; // 2 * proportional gain (Kp)
      private double twoKi; // 2 * integral gain (Ki)
      private final Quaternion quaternion = new Quaternion(); // quaternion of sensor frame relative to auxiliary frame
      private final Vector3D integralFB = new Vector3D(); // integral error terms scaled by Ki
      private final Vector3D error = new Vector3D();

      public OriginalMahonyFilter()
      {
      }

      public void reset()
      {
         integralFB.setToZero();
      }

      public void setSampleFreq(double sampleFreq)
      {
         this.sampleFreq = sampleFreq;
      }

      public void setGains(double kp, double ki)
      {
         twoKp = 2.0 * kp;
         twoKi = 2.0 * ki;
      }

      public void updateAHRS(Vector3DReadOnly gyro, Vector3DReadOnly accelerometer, Vector3DReadOnly magnetometer)
      {
         if (magnetometer != null)
         {
            updateAHRS(gyro.getX(),
                       gyro.getY(),
                       gyro.getZ(),
                       accelerometer.getX(),
                       accelerometer.getY(),
                       accelerometer.getZ(),
                       magnetometer.getX(),
                       magnetometer.getY(),
                       magnetometer.getZ());
         }
         else
         {
            updateIMU(gyro, accelerometer);
         }
      }

      public void updateAHRS(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz)
      {
         double recipNorm;
         double q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
         double hx, hy, bx, bz;
         double halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
         double halfex, halfey, halfez;
         double qa, qb, qc;

         double q0 = quaternion.getS();
         double q1 = quaternion.getX();
         double q2 = quaternion.getY();
         double q3 = quaternion.getZ();

         // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
         if ((mx == 0.0) && (my == 0.0) && (mz == 0.0))
         {
            updateIMU(gx, gy, gz, ax, ay, az);
            return;
         }

         // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
         if (!((ax == 0.0) && (ay == 0.0) && (az == 0.0)))
         {
            // Normalise accelerometer measurement
            recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Normalise magnetometer measurement
            recipNorm = invSqrt(mx * mx + my * my + mz * mz);
            mx *= recipNorm;
            my *= recipNorm;
            mz *= recipNorm;

            // Auxiliary variables to avoid repeated arithmetic
            q0q0 = q0 * q0;
            q0q1 = q0 * q1;
            q0q2 = q0 * q2;
            q0q3 = q0 * q3;
            q1q1 = q1 * q1;
            q1q2 = q1 * q2;
            q1q3 = q1 * q3;
            q2q2 = q2 * q2;
            q2q3 = q2 * q3;
            q3q3 = q3 * q3;

            // Reference direction of Earth's magnetic field
            hx = 2.0 * (mx * (0.5 - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
            hy = 2.0 * (mx * (q1q2 + q0q3) + my * (0.5 - q1q1 - q3q3) + mz * (q2q3 - q0q1));
            bx = Math.sqrt(hx * hx + hy * hy);
            bz = 2.0 * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5 - q1q1 - q2q2));

            // Estimated direction of gravity and magnetic field
            halfvx = q1q3 - q0q2;
            halfvy = q0q1 + q2q3;
            halfvz = q0q0 - 0.5 + q3q3;
            halfwx = bx * (0.5 - q2q2 - q3q3) + bz * (q1q3 - q0q2);
            halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
            halfwz = bx * (q0q2 + q1q3) + bz * (0.5 - q1q1 - q2q2);

            // Error is sum of cross product between estimated direction and measured direction of field vectors
            halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
            halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
            halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);
            error.set(halfex, halfey, halfez);
            error.scale(2.0);

            // Compute and apply integral feedback if enabled
            if (twoKi > 0.0)
            {
               integralFB.addX(twoKi * halfex * (1.0 / sampleFreq)); // integral error scaled by Ki
               integralFB.addY(twoKi * halfey * (1.0 / sampleFreq));
               integralFB.addZ(twoKi * halfez * (1.0 / sampleFreq));
               gx += integralFB.getX(); // apply integral feedback
               gy += integralFB.getY();
               gz += integralFB.getZ();
            }
            else
            {
               integralFB.setToZero(); // prevent integral windup
            }

            // Apply proportional feedback
            gx += twoKp * halfex;
            gy += twoKp * halfey;
            gz += twoKp * halfez;
         }

         // Integrate rate of change of quaternion
         gx *= (0.5 * (1.0 / sampleFreq)); // pre-multiply common factors
         gy *= (0.5 * (1.0 / sampleFreq));
         gz *= (0.5 * (1.0 / sampleFreq));
         qa = q0;
         qb = q1;
         qc = q2;
         q0 += (-qb * gx - qc * gy - q3 * gz);
         q1 += (qa * gx + qc * gz - q3 * gy);
         q2 += (qa * gy - qb * gz + q3 * gx);
         q3 += (qa * gz + qb * gy - qc * gx);

         // Normalise quaternion
         recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
         q0 *= recipNorm;
         q1 *= recipNorm;
         q2 *= recipNorm;
         q3 *= recipNorm;
         quaternion.setUnsafe(q1, q2, q3, q0);
      }

      public void updateIMU(Vector3DReadOnly gyro, Vector3DReadOnly accelerometer)
      {
         updateIMU(gyro.getX(), gyro.getY(), gyro.getZ(), accelerometer.getX(), accelerometer.getY(), accelerometer.getZ());
      }

      public void updateIMU(double gx, double gy, double gz, double ax, double ay, double az)
      {
         double recipNorm;
         double halfvx, halfvy, halfvz;
         double halfex, halfey, halfez;
         double qa, qb, qc;

         double q0 = quaternion.getS();
         double q1 = quaternion.getX();
         double q2 = quaternion.getY();
         double q3 = quaternion.getZ();

         // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
         if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
         {

            // Normalise accelerometer measurement
            recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Estimated direction of gravity and vector perpendicular to magnetic flux
            halfvx = q1 * q3 - q0 * q2;
            halfvy = q0 * q1 + q2 * q3;
            halfvz = q0 * q0 - 0.5f + q3 * q3;

            // Error is sum of cross product between estimated and measured direction of gravity
            halfex = (ay * halfvz - az * halfvy);
            halfey = (az * halfvx - ax * halfvz);
            halfez = (ax * halfvy - ay * halfvx);
            error.set(halfex, halfey, halfez);
            error.scale(2.0);

            // Compute and apply integral feedback if enabled
            if (twoKi > 0.0f)
            {
               integralFB.addX(twoKi * halfex * (1.0f / sampleFreq)); // integral error scaled by Ki
               integralFB.addY(twoKi * halfey * (1.0f / sampleFreq));
               integralFB.addZ(twoKi * halfez * (1.0f / sampleFreq));
               gx += integralFB.getX(); // apply integral feedback
               gy += integralFB.getY();
               gz += integralFB.getZ();
            }
            else
            {
               integralFB.setToZero(); // prevent integral windup
            }

            // Apply proportional feedback
            gx += twoKp * halfex;
            gy += twoKp * halfey;
            gz += twoKp * halfez;
         }

         // Integrate rate of change of quaternion
         gx *= (0.5f * (1.0f / sampleFreq)); // pre-multiply common factors
         gy *= (0.5f * (1.0f / sampleFreq));
         gz *= (0.5f * (1.0f / sampleFreq));
         qa = q0;
         qb = q1;
         qc = q2;
         q0 += (-qb * gx - qc * gy - q3 * gz);
         q1 += (qa * gx + qc * gz - q3 * gy);
         q2 += (qa * gy - qb * gz + q3 * gx);
         q3 += (qa * gz + qb * gy - qc * gx);

         // Normalise quaternion
         recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
         q0 *= recipNorm;
         q1 *= recipNorm;
         q2 *= recipNorm;
         q3 *= recipNorm;
         quaternion.setUnsafe(q1, q2, q3, q0);
      }

      private static double invSqrt(double value)
      {
         return 1.0 / Math.sqrt(value);
      }
   }
}
