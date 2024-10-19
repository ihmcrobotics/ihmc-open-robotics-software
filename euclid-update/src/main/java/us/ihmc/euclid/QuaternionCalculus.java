package us.ihmc.euclid;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.rotationConversion.QuaternionConversion;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;

/**
 * This class contains a number of tools related to computing dynamics of quaternions. Each method is guaranteed to be free of allocations, which requires
 * instantiation for some methods.
 *
 * <p>
 * Specifically, it provides tools for exponential
 * ({@link #exp(Vector3DReadOnly, QuaternionBasics)}) and logarithmic maps of quaternions, {@link #log(QuaternionReadOnly, Vector3DBasics)}.
 * </p>
 *
 * <p>
 * Importantly, it also provides the interfaces for the ability to perform spherical linear interpolation, which is smooth interpolation on the
 * SO<sup>3</sup> manifold. This is commonly referred to as the SLERP algorithm, and can be performed calling
 * {@link #interpolate(double, QuaternionReadOnly, QuaternionReadOnly, QuaternionBasics)}.
 * Additional tools are provided to perform SLERP interpolation guaranteeing interpolation along the minimal path, with
 * {@link #interpolate(double, QuaternionReadOnly, QuaternionReadOnly, QuaternionBasics, boolean)}.
 * </p>
 *
 * <p>
 * This class also provides tools for mapping from and to angular velocities and accelerations and quaternion rates and accelerations. These can be done in
 * both
 * the local frame defined by the orientation, or the parent frame. These can be accessed using
 * {@link #computeQDotInParentFrame(QuaternionReadOnly, Vector3DReadOnly, Vector4DBasics)},
 * {@link #computeQDotInRotatedFrame(QuaternionReadOnly, Vector3DReadOnly, Vector4DBasics)},
 * and various other methods.
 * </p>
 * <p>
 * Lastly, this class provides tools for estimating quaternion rates and accelerations via finite differences, provided various quaternion samples. These are
 * useful for testing quaternion changes from generating trajectories, but can be used for a variety of other things. These can be accessed by
 * {@link #computeQDotByFiniteDifferenceCentral(QuaternionReadOnly, QuaternionReadOnly, double, Vector4DBasics)} and
 * {@link #computeQDDotByFiniteDifferenceCentral(QuaternionReadOnly, QuaternionReadOnly, QuaternionReadOnly, double, Vector4DBasics)}.
 * </p>
 */
public class QuaternionCalculus
{
   private final AxisAngle axisAngleForLog = new AxisAngle();
   private final Quaternion tempQ1ForInterpolation = new Quaternion();
   private final AxisAngle axisAngleForPow = new AxisAngle();
   private final Quaternion qConj = new Quaternion();

   private final Vector4D intermediateQDot = new Vector4D();
   private final Vector4D qDotConj = new Vector4D();
   private final Vector3D intermediateAngularAcceleration = new Vector3D();
   private final Vector3D intermediateAngularVelocity = new Vector3D();
   private final Vector4D intermediateQDDot = new Vector4D();
   private final Vector4D pureQuatForMultiply = new Vector4D();

   public QuaternionCalculus()
   {
   }

   /**
    * This computes the logarithm of the quaternion a stores it in {@code resultToPack}, which is a pure quaternion.
    * This is the equation:
    * <p>resultToPack = log(q)</p>
    * where q is a {@link QuaternionReadOnly}.
    *
    * @param q            is a unit quaternion and describes a orientation. Not modified.
    * @param resultToPack is used to store the result and is a pure quaternion (w = 0.0). Modified.
    */
   public void log(QuaternionReadOnly q, Vector4DBasics resultToPack)
   {
      axisAngleForLog.set(q);
      resultToPack.set(axisAngleForLog.getX(), axisAngleForLog.getY(), axisAngleForLog.getZ(), 0.0);
      resultToPack.scale(axisAngleForLog.getAngle());
   }

   /**
    * This computes the logarithm of the quaternion a stores it in {@code resultToPack}, which is a vector.
    * This is the equation:
    * <p>resultToPack = log(q)</p>
    * where q is a {@link QuaternionReadOnly}.
    *
    * @param q            is a unit quaternion and describes a orientation. Not modified.
    * @param resultToPack is used to store the result. Modified.
    */
   public void log(QuaternionReadOnly q, Vector3DBasics resultToPack)
   {
      axisAngleForLog.set(q);
      resultToPack.set(axisAngleForLog.getX(), axisAngleForLog.getY(), axisAngleForLog.getZ());
      resultToPack.scale(axisAngleForLog.getAngle());
   }

   /**
    * Applies the exponential transformation to the rotation vector. This is the same as computing the equivalent quaternion from a rotation vector.
    *
    * @param rotationVector   rotation of which to compute the exponential. Not modified.
    * @param quaternionToPack is used to store the result. Modified.
    */
   public static void exp(Vector3DReadOnly rotationVector, QuaternionBasics quaternionToPack)
   {
      QuaternionConversion.convertRotationVectorToQuaternion(rotationVector, quaternionToPack);
   }

   /**
    * Performs a spherical linear interpolation (SLERP) from a quaternion between the start {@code q0} and end {@code q1} for a given alpha = [0, 1] using
    * SLERP. The results are stored in {@code qInterpolatedToPack}. This effectively applies a constant angular velocity to go from {@code q0} to {@code q1}.
    * <p>
    * At {@code alpha = 0}, the output matches {@code q0}. At {@code alpha = 1}, the output matches {@code q1}. {@code alpha} is not constrained to be between
    * [0, 1]
    * </p>
    * <a href="https://splines.readthedocs.io/en/latest/rotation/slerp.html">SLERP Descriptions.</a>
    * Computes: resultToPack = q0 (q0<sup>-1</sup> q1)<sup>alpha</sup>.
    *
    * @param alpha               fraction to interpolate between {@code q0} and {@code q1}. Should be between [0, 1].
    * @param q0                  quaternion value at the beginning of the interpolation (when {@code alpha} = 0). Not Modified.
    * @param q1                  quaternion value at the end of the interpolation (when {@code alpha} = 1). Not Modified.
    * @param qInterpolatedToPack interpolated quaternion to store the results. Modified.
    */
   public void interpolate(double alpha, QuaternionReadOnly q0, QuaternionReadOnly q1, QuaternionBasics qInterpolatedToPack)
   {
      interpolate(alpha, q0, q1, qInterpolatedToPack, true);
   }

   /**
    * Performs a spherical linear interpolation (SLERP) from a quaternion between the start {@code q0} and end {@code q1} for a given alpha = [0, 1] using
    * SLERP. The results are stored in {@code qInterpolatedToPack}. This effectively applies a constant angular velocity to go from {@code q0} to {@code q1}.
    * <p>
    * At {@code alpha = 0}, the output matches {@code q0}. At {@code alpha = 1}, the output matches {@code q1}. {@code alpha} is not constrained to be between
    * [0, 1]
    * </p>
    * <p>
    * When {@code preventExtraSpin} is true, this check ensures that the interpolation goes the minimum distance along the shortest path. As it's a spherical
    * interpolation, {@code q0} can rotate either direction to reach {@code q1}. When {@code preventExtraSpin} is true, the math checks to make that the
    * rotation
    * from {@code q0} to {@code q1} is less than {@link Math#PI}. If it is greater than this value, then it negates {@code q1} to reverse the directionality
    * of the rotation, ensuring that the interpolation is along the shortest path.
    * </p>
    * <a href="https://splines.readthedocs.io/en/latest/rotation/slerp.html">SLERP Descriptions.</a>
    * Computes: resultToPack = q0 (q0<sup>-1</sup> q1)<sup>alpha</sup>.
    *
    * @param alpha               fraction to interpolate between {@code q0} and {@code q1}. Should be between [0, 1].
    * @param q0                  quaternion value at the beginning of the interpolation (when {@code alpha} = 0). Not Modified.
    * @param q1                  quaternion value at the end of the interpolation (when {@code alpha} = 1). Not Modified.
    * @param qInterpolatedToPack interpolated quaternion to store the results. Modified.
    */
   public void interpolate(double alpha, QuaternionReadOnly q0, QuaternionReadOnly q1, QuaternionBasics qInterpolatedToPack, boolean preventExtraSpin)
   {
      tempQ1ForInterpolation.set(q1);

      // Checks if the interpolation is taking the shortest path.
      if (preventExtraSpin && q0.dot(tempQ1ForInterpolation) < 0.0)
      {
         tempQ1ForInterpolation.negate();
      }

      qInterpolatedToPack.difference(q0, tempQ1ForInterpolation);
      pow(qInterpolatedToPack, alpha, qInterpolatedToPack);
      qInterpolatedToPack.multiply(q0, qInterpolatedToPack);
   }

   /**
    * This computes: resultToPack = q^power.
    *
    * @param q            is a unit quaternion and describes an orientation. Not modifeid.
    * @param power        exponential to apply to the quaternion.
    * @param resultToPack is used to store the result. Modified.
    */
   public void pow(QuaternionReadOnly q, double power, QuaternionBasics resultToPack)
   {
      axisAngleForPow.set(q);
      axisAngleForPow.setAngle(power * axisAngleForPow.getAngle());
      resultToPack.set(axisAngleForPow);
   }

   /**
    * This computes the angular velocity expressed in the rotation frame of a body from the rotation of the body expressed as a quaternion and its quaternion
    * rate.
    *
    * <p>
    * The rotation from the parent frame, F<sub>parent</sub> to the current frame F<sub>child</sub> is defined by a quaternion {@code q}. This quaternion has a
    * corresponding velocity, which is defined by the rate of change of its quaternion coordinates, {@code qDot}.
    * </p>
    * <p>
    * This method computes the local 3-dimensional angular velocity, expressed in the child frame F<sub>child</sub>, and stores it in
    * {@code angularVelocityToPack}.
    * </p>
    *
    * @param q                     quaternion defining the rotation to the local frame. Not modified.
    * @param qDot                  quaternion rate of change of {@code q}. Not modified.
    * @param angularVelocityToPack angular velocity in the local frame. Modified.
    */
   public void computeAngularVelocityInRotatedFrame(QuaternionReadOnly q, Vector4DReadOnly qDot, Vector3DBasics angularVelocityToPack)
   {
      qConj.setAndConjugate(q);
      multiply(qConj, qDot, angularVelocityToPack);
      angularVelocityToPack.scale(2.0);
   }

   /**
    * This computes the angular velocity expressed in the parent frame of a body from the rotation of the body expressed as a quaternion and its quaternion
    * rate.
    *
    * <p>
    * The rotation from the parent frame, F<sub>parent</sub> to the current frame F<sub>child</sub> is defined by a quaternion {@code q}. This quaternion has a
    * corresponding velocity, which is defined by the rate of change of its quaternion coordinates, {@code qDot}.
    * </p>
    * <p>
    * This method computes the 3-dimensional angular velocity, expressed in the parent frame F<sub>parent</sub>, and stores it in
    * {@code angularVelocityToPack}.
    * </p>
    *
    * @param q                     quaternion defining the rotation to the local frame. Not modified.
    * @param qDot                  quaternion rate of change of {@code q}. Not modified.
    * @param angularVelocityToPack angular velocity in the parent frame. Modified.
    */
   public void computeAngularVelocityInParentFrame(QuaternionReadOnly q, Vector4DReadOnly qDot, Vector3DBasics angularVelocityToPack)
   {
      qConj.setAndConjugate(q);
      multiply(qDot, qConj, angularVelocityToPack);
      angularVelocityToPack.scale(2.0);
   }

   /**
    * This computes the quaternion rate from the rotation of a body expressed as a quaternion and its angular velocity expressed in the frame of its parent.
    *
    * <p>
    * The rotation from the parent frame, F<sub>parent</sub> to the current frame F<sub>child</sub> is defined by a quaternion {@code q}. This quaternion has a
    * corresponding velocity, which is defined by the rate of change of its quaternion coordinates, {@code qDot}.
    * </p>
    * <p>
    * This method computes the quaternion rate, and stores it in {@code qDotToPack}
    * </p>
    *
    * @param q                       quaternion defining the rotation to the local frame. Not Modified.
    * @param angularVelocityInParent angular velocity in the parent frame. Not Modified.
    * @param qDotToPack              quaternion rate of change of {@code q} to compute. Modified.
    */
   public static void computeQDotInParentFrame(QuaternionReadOnly q, Vector3DReadOnly angularVelocityInParent, Vector4DBasics qDotToPack)
   {
      multiply(angularVelocityInParent, q, qDotToPack);
      qDotToPack.scale(0.5);
   }

   /**
    * This computes the quaternion rate from the rotation of a body expressed as a quaternion and its angular velocity expressed in the rotated frame.
    *
    * <p>
    * The rotation from the parent frame, F<sub>parent</sub> to the current frame F<sub>child</sub> is defined by a quaternion {@code q}. This quaternion has a
    * corresponding velocity, which is defined by the rate of change of its quaternion coordinates, {@code qDot}.
    * </p>
    * <p>
    * This method computes the quaternion rate, and stores it in {@code qDotToPack}
    * </p>
    *
    * @param q                        quaternion defining the rotation to the local frame. Not Modified.
    * @param angularVelocityInRotated angular velocity in the rotated frame. Not Modified.
    * @param qDotToPack               quaternion rate of change of {@code q} to compute. Modified.
    */
   public static void computeQDotInRotatedFrame(QuaternionReadOnly q, Vector3DReadOnly angularVelocityInRotated, Vector4DBasics qDotToPack)
   {
      multiply(q, angularVelocityInRotated, qDotToPack);
      qDotToPack.scale(0.5);
   }

   /**
    * This computes the quaternion acceleration from the rotation of a body expressed as a quaternion, its quaternion rate, and its angular acceleration
    * expressed in the frame of its parent.
    *
    * <p>
    * The rotation from the parent frame, F<sub>parent</sub> to the current frame F<sub>child</sub> is defined by a quaternion {@code q}. This quaternion has a
    * corresponding velocity, which is defined by the rate of change of its quaternion coordinates, {@code qDot}, and acceleratoin, which is defined by the
    * acceleration of its quaternion coordinates, {@code qDDot}.
    * </p>
    * <p>
    * This method computes the quaternion acceleration, and stores it in {@code qDDotToPack}
    * </p>
    *
    * @param q                           quaternion defining the rotation to the local frame. Not Modified.
    * @param qDot                        quaternion rate of change of {@code q}. Not modified.
    * @param angularAccelerationInParent angular acceleration in the parent frame. Not Modified.
    * @param qDDotToPack                 quaternion acceleration of {@code q} to compute. Not modified.
    */
   public void computeQDDotInParentFrame(QuaternionReadOnly q, Vector4DReadOnly qDot, Vector3DReadOnly angularAccelerationInParent, Vector4DBasics qDDotToPack)
   {
      computeAngularVelocityInParentFrame(q, qDot, intermediateAngularVelocity);
      computeQDDotInParentFrame(q, qDot, intermediateAngularVelocity, angularAccelerationInParent, qDDotToPack);
   }

   /**
    * This computes the quaternion acceleration from the rotation of a body expressed as a quaternion and its angular velocity and acceleration
    * expressed in the frame of its parent.
    *
    * <p>
    * The rotation from the parent frame, F<sub>parent</sub> to the current frame F<sub>child</sub> is defined by a quaternion {@code q}. This quaternion has a
    * corresponding velocity, which is defined by the rate of change of its quaternion coordinates, {@code qDot}, and acceleratoin, which is defined by the
    * acceleration of its quaternion coordinates, {@code qDDot}.
    * </p>
    * <p>
    * This method computes the quaternion acceleration, and stores it in {@code qDDotToPack}
    * </p>
    *
    * @param q                           quaternion defining the rotation to the local frame. Not Modified.
    * @param angularVelocityInParent     angular velocity in the parent frame. Not modified.
    * @param angularAccelerationInParent angular acceleration in the parent frame. Not Modified.
    * @param qDDotToPack                 quaternion acceleration of {@code q} to compute. Not modified.
    */
   public void computeQDDotInParentFrame(QuaternionReadOnly q,
                                         Vector3DReadOnly angularVelocityInParent,
                                         Vector3DReadOnly angularAccelerationInParent,
                                         Vector4DBasics qDDotToPack)
   {
      computeQDotInParentFrame(q, angularVelocityInParent, intermediateQDot);
      computeQDDotInParentFrame(q, intermediateQDot, angularVelocityInParent, angularAccelerationInParent, qDDotToPack);
   }

   /**
    * This computes the angular acceleration expressed in the parent frame of a body from the rotation of the body expressed as a quaternion and its quaternion
    * rate and acceleration.
    *
    * <p>
    * The rotation from the parent frame, F<sub>parent</sub> to the current frame F<sub>child</sub> is defined by a quaternion {@code q}. This quaternion has a
    * corresponding velocity, which is defined by the rate of change and acceleration of its quaternion coordinates, {@code qDot} and {@code qDDot}.
    * </p>
    * <p>
    * This method computes the 3-dimensional angular acceleration, expressed in the parent frame F<sub>parent</sub>, and stores it in
    * {@code angularAccelerationToPack}.
    * </p>
    *
    * @param q                                 quaternion defining the rotation to the local frame. Not modified.
    * @param qDDot                             quaternion acceleration of {@code q}. Not modified.
    * @param angularVelocityInParent           angular velocity in the parent frame. Not modified.
    * @param angularAccelerationInParentToPack angular acceleration in the parent frame. Modified.
    */
   public void computeAngularAccelerationInParentFrame(QuaternionReadOnly q,
                                                       Vector4DReadOnly qDDot,
                                                       Vector3DReadOnly angularVelocityInParent,
                                                       Vector3DBasics angularAccelerationInParentToPack)
   {
      computeQDotInParentFrame(q, angularVelocityInParent, intermediateQDot);
      computeAngularAccelerationInRotatedFrame(q, intermediateQDot, qDDot, angularAccelerationInParentToPack);
   }

   /**
    * This computes the angular acceleration expressed in the child frame of a body from the rotation of the body expressed as a quaternion and its quaternion
    * rate and acceleration.
    *
    * <p>
    * The rotation from the parent frame, F<sub>parent</sub> to the current frame F<sub>child</sub> is defined by a quaternion {@code q}. This quaternion has a
    * corresponding velocity, which is defined by the rate of change and acceleration of its quaternion coordinates, {@code qDot} and {@code qDDot}.
    * </p>
    * <p>
    * This method computes the 3-dimensional angular acceleration, expressed in the parent frame F<sub>parent</sub>, and stores it in
    * {@code angularAccelerationToPack}.
    * </p>
    *
    * @param q                                  quaternion defining the rotation to the local frame. Not modified.
    * @param qDot                               quaternion rate of change of {@code q}. Not modified.
    * @param qDDot                              quaternion acceleration of {@code q}. Not modified.
    * @param angularAccelerationInRotatedToPack angular acceleration in the child frame. Modified.
    */
   public void computeAngularAccelerationInRotatedFrame(QuaternionReadOnly q,
                                                        Vector4DReadOnly qDot,
                                                        Vector4DReadOnly qDDot,
                                                        Vector3DBasics angularAccelerationInRotatedToPack)
   {
      qConj.setAndConjugate(q);
      qDotConj.set(-qDot.getX(), -qDot.getY(), -qDot.getZ(), qDot.getS());
      multiply(qDot, qDotConj, intermediateAngularAcceleration);
      multiply(qDDot, qConj, angularAccelerationInRotatedToPack);
      angularAccelerationInRotatedToPack.add(intermediateAngularAcceleration);
      angularAccelerationInRotatedToPack.scale(2.0);
   }

   /**
    * This is used to estimate the rate of change of the quaternion coordinates of a time series of data.
    *
    * <p>If the time series is represented by</p>
    * <p>[q<sub>prev</sub>, q<sub>current</sub>, q<sub>next</sub>]</p>
    * <p> where the data measurements are separated by a time discretisation of {@param dt}, this method is used to compute the quaternion rate of the
    * quaternion at the current time. It uses the previous {@param qPrevious} and next {@param qNext} values. The results are stored in {@param qDotToPack}.</p>
    *
    * @param qPrevious  previous quaternion estimate. Not modified.
    * @param qNext      next quaternion estimate. Not modified.
    * @param dt         time between quaternion estimates
    * @param qDotToPack estimated quaternion rate at the current value. Modified.
    */
   public static void computeQDotByFiniteDifferenceCentral(QuaternionReadOnly qPrevious, QuaternionReadOnly qNext, double dt, Vector4DBasics qDotToPack)
   {
      qDotToPack.sub(qNext, qPrevious);
      qDotToPack.scale(0.5 / dt);
   }

   /**
    * This is used to estimate the acceleration of the quaternion coordinates of a time series of data.
    *
    * <p>If the time series is represented by</p>
    * <p>[q<sub>prev</sub>, q<sub>current</sub>, q<sub>next</sub>]</p>
    * <p> where the data measurements are separated by a time discretisation of {@param dt}, this method is used to compute the quaternion acceleration
    * at the current time. It uses the previous {@param qPrevious}, current {@param q}, and next {@param qNext} values. The results are stored in
    * {@param qDotToPack}.</p>
    *
    * @param qPrevious   previous quaternion estimate. Not modified.
    * @param q           next quaternion estimate. Not modified.
    * @param qNext       next quaternion estimate. Not modified.
    * @param dt          time between quaternion estimates.
    * @param qDDotToPack estimated quaternion rate at the current value. Modified.
    */
   public static void computeQDDotByFiniteDifferenceCentral(QuaternionReadOnly qPrevious,
                                                            QuaternionReadOnly q,
                                                            QuaternionReadOnly qNext,
                                                            double dt,
                                                            Vector4DBasics qDDotToPack)
   {
      qDDotToPack.sub(qNext, q);
      qDDotToPack.sub(q);
      qDDotToPack.add(qPrevious);
      qDDotToPack.scale(1.0 / EuclidCoreTools.square(dt));
   }

   /**
    * This is an internal method used to compute the quaternion acceleration from the quaternion value, its rate, and the angular velocity and acceleration of
    * that frame relative to the parent expressed in the parent.
    */
   void computeQDDotInParentFrame(QuaternionReadOnly q,
                                  Vector4DReadOnly qDot,
                                  Vector3DReadOnly angularVelocityInParent,
                                  Vector3DReadOnly angularAccelerationInParent,
                                  Vector4DBasics qDDotToPack)
   {
      multiply(angularAccelerationInParent, q, intermediateQDDot);
      multiply(angularVelocityInParent, qDot, qDDotToPack);
      qDDotToPack.add(intermediateQDDot);
      qDDotToPack.scale(0.5);
   }

   private static void multiply(QuaternionReadOnly q, Vector3DReadOnly v, Vector4DBasics resultToPack)
   {
      setAsPureQuaternion(v, resultToPack);
      QuaternionTools.multiply(q, resultToPack, resultToPack);
   }

   private static void multiply(Vector3DReadOnly v, Vector4DReadOnly q, Vector4DBasics resultToPack)
   {
      setAsPureQuaternion(v, resultToPack);
      QuaternionTools.multiply(resultToPack, q, resultToPack);
   }

   private static void multiply(Vector3DReadOnly v, QuaternionReadOnly q, Vector4DBasics resultToPack)
   {
      setAsPureQuaternion(v, resultToPack);
      QuaternionTools.multiply(resultToPack, q, resultToPack);
   }

   private void multiply(Vector4DReadOnly q1, Vector4DReadOnly q2, Vector3DBasics resultToPack)
   {
      QuaternionTools.multiply(q1, q2, pureQuatForMultiply);
      setVectorFromPureQuaternion(pureQuatForMultiply, resultToPack);
   }

   private void multiply(Vector4DReadOnly q1, QuaternionReadOnly q2, Vector3DBasics resultToPack)
   {
      QuaternionTools.multiply(q1, q2, pureQuatForMultiply);
      setVectorFromPureQuaternion(pureQuatForMultiply, resultToPack);
   }

   private void multiply(QuaternionReadOnly q1, Vector4DReadOnly q2, Vector3DBasics resultToPack)
   {
      QuaternionTools.multiply(q1, q2, pureQuatForMultiply);
      setVectorFromPureQuaternion(pureQuatForMultiply, resultToPack);
   }

   /**
    * Provides a convenience function for setting a 4D pure quaternion vector from a 3D vector, where the 4D vector is assumed to represent a pure quaternion (w
    * = 0.0).
    *
    * @param vector              rotation data source. Not modified.
    * @param pureQuaternionToSet results to pack. Modified.
    */
   private static void setAsPureQuaternion(Vector3DReadOnly vector, Vector4DBasics pureQuaternionToSet)
   {
      pureQuaternionToSet.set(vector.getX(), vector.getY(), vector.getZ(), 0.0);
   }

   /**
    * Provides a convenience function for setting a 3D vector from a 4D vector, where the 4D vector is assumed to represent a pure quaternion (w = 0.0).
    *
    * @param pureQuaternion quaternion data source. Not modified.
    * @param vectorToPack   results to pack. Modified.
    */
   private static void setVectorFromPureQuaternion(Vector4DReadOnly pureQuaternion, Vector3DBasics vectorToPack)
   {
      vectorToPack.set(pureQuaternion.getX(), pureQuaternion.getY(), pureQuaternion.getZ());
   }
}
