package us.ihmc.robotics.math.filters;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.CommonMatrix3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.rotationConversion.RotationMatrixConversion;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.lists.RingBuffer;

/**
 * Online filter for orientation measurements.
 * To summarize, it performs a spline fitting to a moving window of orientation measurements to estimate the orientation, angular velocity, and angular
 * acceleration.
 * <p>
 * Implementation of the Savitzky-Golay filter proposed in <a href="https://hal.science/hal-03603826/document">Geometric Savitzky-Golay Filtering of Noisy
 * Rotations on SO (3) with Simultaneous Angular Velocity and Acceleration Estimation</a>.
 * </p>
 * <p>
 * This implementation is based on the MATLAB code provided from the authors: <a
 * href="https://github.com/MaartenJongeneel/Paper-Savitzky-Golay-filtering-on-SO3">GitHub Repository</a>.
 * </p>
 * <p>
 * The window moving version of the filter is implemented to allow for online filtering.
 * </p>
 *
 * @see SplineBasedOnlinePositionFilter3D
 */
public class SavitzkyGolayOnlineOrientationFilter3D
{
   public static final double EPS = 1.0e-12;

   private final int polynomialOrder;
   private final int windowSize;
   private final RingBuffer<OrientationMesurement> orientationBuffer;

   private final RotationMatrix estimatedOrientation = new RotationMatrix();
   private final Vector3D estimatedAngularVelocity = new Vector3D();
   private final Vector3D estimatedAngularAcceleration = new Vector3D();

   private int backwardIndex = 0;

   public SavitzkyGolayOnlineOrientationFilter3D(int windowSize)
   {
      this(3, windowSize);
   }

   public SavitzkyGolayOnlineOrientationFilter3D(int polynomialOrder, int windowSize)
   {
      this.polynomialOrder = polynomialOrder;
      this.windowSize = windowSize;
      orientationBuffer = new RingBuffer<>(windowSize, OrientationMesurement::new, OrientationMesurement::set);
   }

   public void addOrientation(double time, Orientation3DReadOnly orientation)
   {
      orientationBuffer.add().set(time, orientation);
   }

   public void compute()
   {
      if (!orientationBuffer.isBufferFull())
         return;

      DMatrixRMaj A = new DMatrixRMaj(3 * windowSize, 3 * (polynomialOrder + 1));
      DMatrixRMaj b = new DMatrixRMaj(3 * windowSize, 1);
      Matrix3D Ajk = new Matrix3D();
      RotationMatrix delta = new RotationMatrix();
      Vector3D deltaRotation = new Vector3D();

      OrientationMesurement estimationMeasurement = orientationBuffer.getFromLast(backwardIndex);

      for (int j = 0; j < windowSize; j++)
      {
         OrientationMesurement otherMeasurement = orientationBuffer.getFromLast(j);
         double timeInterval = otherMeasurement.time - estimationMeasurement.time;
         double tPow = 1.0;

         Ajk.setIdentity();
         Ajk.get(3 * j, 0, A);

         for (int k = 1; k <= polynomialOrder; k++)
         {
            tPow *= timeInterval;
            Ajk.setIdentity();
            Ajk.scale(tPow / k);
            Ajk.get(3 * j, 3 * k, A);
         }

         delta.set(otherMeasurement.orientation);
         delta.multiplyTransposeOther(estimationMeasurement.orientation);
         delta.getRotationVector(deltaRotation);
         deltaRotation.get(3 * j, 0, b);
      }

      DMatrixRMaj Apinv = new DMatrixRMaj(3 * (polynomialOrder + 1), 3 * windowSize);
      CommonOps_DDRM.pinv(A, Apinv);
      DMatrixRMaj rho = new DMatrixRMaj(3 * (polynomialOrder + 1), 1);
      CommonOps_DDRM.mult(Apinv, b, rho);

      // Obtain the coefficients of rho
      Vector3D rho0 = new Vector3D(rho.get(0), rho.get(1), rho.get(2));
      Vector3D rho1 = new Vector3D(rho.get(3), rho.get(4), rho.get(5));
      Vector3D rho2 = new Vector3D(rho.get(6), rho.get(7), rho.get(8));

      estimatedOrientation.setRotationVector(rho0);
      estimatedOrientation.append(estimationMeasurement.orientation);
      Matrix3D rho0_dexpSO3 = new Matrix3D();
      dexpSO3(rho0, rho0_dexpSO3);
      Matrix3D rho01_ddexpSO3 = new Matrix3D();
      ddexpSO3(rho0, rho1, rho01_ddexpSO3);
      rho0_dexpSO3.transform(rho1, estimatedAngularVelocity);

      Vector3D temp = new Vector3D();
      rho01_ddexpSO3.transform(rho1, temp);
      rho0_dexpSO3.transform(rho2, estimatedAngularAcceleration);
      estimatedAngularAcceleration.add(temp);
   }

   public void setBackwardIndex(int backwardIndex)
   {
      if (backwardIndex < 0 || backwardIndex >= windowSize)
         throw new IllegalArgumentException("The backward index has to be in [0, " + windowSize + "], but was: " + backwardIndex);
      this.backwardIndex = backwardIndex;
   }

   public int getWindowSize()
   {
      return windowSize;
   }

   public RotationMatrix getEstimatedOrientation()
   {
      return estimatedOrientation;
   }

   public Vector3D getEstimatedAngularVelocity()
   {
      return estimatedAngularVelocity;
   }

   public Vector3D getEstimatedAngularAcceleration()
   {
      return estimatedAngularAcceleration;
   }

   private static class OrientationMesurement
   {
      private double time;
      private final RotationMatrix orientation = new RotationMatrix();

      public void set(OrientationMesurement other)
      {
         set(other.time, other.orientation);
      }

      public void set(double time, Orientation3DReadOnly orientation)
      {
         this.time = time;
         this.orientation.set(orientation);
      }

      @Override
      public String toString()
      {
         return "Time: " + time + ", Orientation: " + orientation.toStringAsYawPitchRoll();
      }
   }

   /**
    * Computes the exponential mapping on SO(3)
    * <p>
    * Exponential on SO(3), Rodrigues formula.
    * </p>
    *
    * @param vector       3 vector, isomorphism to element of so(3)
    * @param matrixToPack element of SO(3)
    */
   public static void expSO3(Vector3DReadOnly vector, CommonMatrix3DBasics matrixToPack)
   {
      RotationMatrixConversion.convertRotationVectorToMatrix(vector, matrixToPack);
   }

   /**
    * Computes the right trivialized tangent d exp on SO(3)
    * <p>
    * Closed form expression of right trivialized tangent d exp on SO(3)
    * </p>
    *
    * @param vector       3 vector, isomorphism to element of so(3)
    * @param matrixToPack diff exponential of a
    */
   public static void dexpSO3(Vector3DReadOnly vector, CommonMatrix3DBasics matrixToPack)
   {
      double phi = vector.norm();

      matrixToPack.setIdentity();

      if (phi != 0)
      {
         double alpha = Math.sin(phi) / phi;
         double beta = MathTools.square(Math.sin(phi / 2.0)) / (MathTools.square(phi / 2.0));

         double vx = vector.getX();
         double vy = vector.getY();
         double vz = vector.getZ();
         double vx2 = vx * vx;
         double vy2 = vy * vy;
         double vz2 = vz * vz;

         // M = I + 0.5 * beta * hat(a) + (1 - alpha) / phi^2 * hat(a) * hat(a)
         // M = I + halfBeta * hat(a) + gamma * hat(a) * hat(a)
         double halfBeta = 0.5 * beta;
         double gamma = (1.0 - alpha) / MathTools.square(phi);
         double m00 = 1.0 - gamma * (vy2 + vz2);
         double m01 = -halfBeta * vz + gamma * vx * vy;
         double m02 = halfBeta * vy + gamma * vx * vz;
         double m10 = halfBeta * vz + gamma * vx * vy;
         double m11 = 1.0 - gamma * (vx2 + vz2);
         double m12 = -halfBeta * vx + gamma * vy * vz;
         double m20 = -halfBeta * vy + gamma * vx * vz;
         double m21 = halfBeta * vx + gamma * vy * vz;
         double m22 = 1.0 - gamma * (vx2 + vy2);
         matrixToPack.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      }
      else // phi == 0
      {
         matrixToPack.setIdentity();
      }
   }

   public static void ddexpSO3(Vector3DReadOnly a, Vector3DReadOnly c, CommonMatrix3DBasics matrixToPack)
   {
      double phi = a.norm();
      double alpha = Math.sin(phi) / phi;
      double beta = MathTools.square(Math.sin(phi / 2.0)) / (MathTools.square(phi / 2.0));
      double halfBeta = 0.5 * beta;
      double aDotc = a.dot(c);
      double oneOverPhi2 = 1.0 / MathTools.square(phi);
      double c0 = 1.0 - alpha;
      double c1 = (alpha - beta) * aDotc;
      double c2 = (beta / 2.0 - 3.0 * oneOverPhi2 * c0) * aDotc;

      double ax = a.getX();
      double ay = a.getY();
      double az = a.getZ();
      double cx = c.getX();
      double cy = c.getY();
      double cz = c.getZ();
      double ax2 = ax * ax;
      double ay2 = ay * ay;
      double az2 = az * az;

      double axy = ax * ay;
      double axz = ax * az;
      double ayz = ay * az;
      double axcx_azcz = ax * cx + az * cz;
      double aycy_azcz = ay * cy + az * cz;
      double aycx_axcy = ay * cx + ax * cy;
      double azcx_axcz = az * cx + ax * cz;
      double azcy_aycz = az * cy + ay * cz;
      // M = 0.5 * beta * hat(c) + 1/phi^2 * (1 - alpha) * (hat(a) * hat(c) + hat(c) * hat(a)) + 1/phi^2 * (alpha - beta) * (a' * c) * hat(a) + 1/phi^2 * (beta/2 - 3/phi^2 * (1 - alpha)) * (a' * c) * hat(a) * hat(a)
      // M = halfBeta * hat(c) + oneOverPhi2 * (c0 * (hat(a) * hat(c) + hat(c) * hat(a)) + c1 * hat(a) + c2 * hat(a) * hat(a))
      double m00 = oneOverPhi2 * (-2.0 * c0 * aycy_azcz - c2 * (ay2 + az2));
      double m01 = -halfBeta * cz + oneOverPhi2 * (c0 * aycx_axcy - c1 * az + c2 * axy);
      double m02 = halfBeta * cy + oneOverPhi2 * (c0 * azcx_axcz + c1 * ay + c2 * axz);
      double m10 = halfBeta * cz + oneOverPhi2 * (c0 * aycx_axcy + c1 * az + c2 * axy);
      double m11 = oneOverPhi2 * (-2.0 * c0 * axcx_azcz - c2 * (ax2 + az2));
      double m12 = -halfBeta * cx + oneOverPhi2 * (c0 * azcy_aycz - c1 * ax + c2 * ayz);
      double m20 = -halfBeta * cy + oneOverPhi2 * (c0 * azcx_axcz - c1 * ay + c2 * axz);
      double m21 = halfBeta * cx + oneOverPhi2 * (c0 * azcy_aycz + c1 * ax + c2 * ayz);
      double m22 = oneOverPhi2 * (-2.0 * c0 * (ax * cx + ay * cy) - c2 * (ax2 + ay2));
      matrixToPack.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }
}
