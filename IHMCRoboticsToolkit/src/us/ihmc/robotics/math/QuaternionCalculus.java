package us.ihmc.robotics.math;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.rotationConversion.QuaternionConversion;
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
import us.ihmc.robotics.MathTools;

public class QuaternionCalculus
{
   public QuaternionCalculus()
   {
   }

   private final AxisAngle axisAngleForLog = new AxisAngle();

   /**
    * This computes: resultToPack = log(q)
    * @param q is a unit quaternion and describes a orientation.
    * @param resultToPack is used to store the result and is a pure quaternion (w = 0.0).
    */
   public void log(QuaternionReadOnly q, Vector4DBasics resultToPack)
   {
      axisAngleForLog.set(q);
      resultToPack.set(axisAngleForLog.getX(), axisAngleForLog.getY(), axisAngleForLog.getZ(), 0.0);
      resultToPack.scale(axisAngleForLog.getAngle());
   }

   /**
    * This computes: resultToPack = log(q)
    * @param q is a unit quaternion and describes a orientation.
    * @param resultToPack is used to store the result.
    */
   public void log(QuaternionReadOnly q, Vector3DBasics resultToPack)
   {
      axisAngleForLog.set(q);
      resultToPack.set(axisAngleForLog.getX(), axisAngleForLog.getY(), axisAngleForLog.getZ());
      resultToPack.scale(axisAngleForLog.getAngle());
   }

   public void exp(Vector3DReadOnly rotationVector, QuaternionBasics quaternionToPack)
   {
      QuaternionConversion.convertRotationVectorToQuaternion(rotationVector, quaternionToPack);
   }

   /**
    * Interpolation from q0 to q1 for a given alpha = [0, 1] using SLERP.
    * Computes: resultToPack = q0 (q0^-1 q1)^alpha.
    */
   public void interpolate(double alpha, QuaternionReadOnly q0, QuaternionReadOnly q1, QuaternionBasics qInterpolatedToPack)
   {
      interpolate(alpha, q0, q1, qInterpolatedToPack, true);
   }

   private final Quaternion tempQ1ForInterpolation = new Quaternion();

   public void interpolate(double alpha, QuaternionReadOnly q0, QuaternionReadOnly q1, QuaternionBasics qInterpolatedToPack, boolean preventExtraSpin)
   {
      tempQ1ForInterpolation.set(q1);

      if (preventExtraSpin && q0.dot(tempQ1ForInterpolation) < 0.0)
      {
         tempQ1ForInterpolation.negate();
      }

      computeQuaternionDifference(q0, tempQ1ForInterpolation, qInterpolatedToPack);
      pow(qInterpolatedToPack, alpha, qInterpolatedToPack);
      qInterpolatedToPack.multiply(q0, qInterpolatedToPack);
   }

   private final AxisAngle axisAngleForPow = new AxisAngle();

   /**
    * This computes: resultToPack = q^power.
    * @param q is a unit quaternion and describes a orientation.
    * @param resultToPack is used to store the result.
    */
   public void pow(QuaternionReadOnly q, double power, QuaternionBasics resultToPack)
   {
      axisAngleForPow.set(q);
      axisAngleForPow.setAngle(power * axisAngleForPow.getAngle());
      resultToPack.set(axisAngleForPow);
   }

   private final Quaternion qConj = new Quaternion();

   public void computeAngularVelocityInBodyFixedFrame(QuaternionReadOnly q, Vector4DReadOnly qDot, Vector3DBasics angularVelocityToPack)
   {
      qConj.setAndConjugate(q);
      multiply(qConj, qDot, angularVelocityToPack);
      angularVelocityToPack.scale(2.0);
   }

   public void computeAngularVelocityInWorldFrame(QuaternionReadOnly q, Vector4DReadOnly qDot, Vector3DBasics angularVelocityToPack)
   {
      qConj.setAndConjugate(q);
      multiply(qDot, qConj, angularVelocityToPack);
      angularVelocityToPack.scale(2.0);
   }

   public void computeQDot(QuaternionReadOnly q, Vector3DReadOnly angularVelocity, Vector4DBasics qDotToPack)
   {
      multiply(angularVelocity, q, qDotToPack);
      qDotToPack.scale(0.5);
   }

   private final Vector4D intermediateQDot = new Vector4D();
   private final Vector4D qDotConj = new Vector4D();
   private final Vector3D intermediateAngularAcceleration = new Vector3D();
   private final Vector3D intermediateAngularVelocity = new Vector3D();
   private final Vector4D intermediateQDDot = new Vector4D();

   public void computeQDDot(QuaternionReadOnly q, Vector4DReadOnly qDot, Vector3DReadOnly angularAcceleration, Vector4DBasics qDDotToPack)
   {
      computeAngularVelocityInWorldFrame(q, qDot, intermediateAngularVelocity);
      computeQDDot(q, qDot, intermediateAngularVelocity, angularAcceleration, qDDotToPack);
   }

   public void computeQDDot(QuaternionReadOnly q, Vector3DReadOnly angularVelocity, Vector3DReadOnly angularAcceleration, Vector4DBasics qDDotToPack)
   {
      computeQDot(q, angularVelocity, intermediateQDot);
      computeQDDot(q, intermediateQDot, angularVelocity, angularAcceleration, qDDotToPack);
   }

   public void computeQDDot(QuaternionReadOnly q, Vector4DReadOnly qDot, Vector3DReadOnly angularVelocity, Vector3DReadOnly angularAcceleration, Vector4DBasics qDDotToPack)
   {
      multiply(angularAcceleration, q, intermediateQDDot);
      multiply(angularVelocity, qDot, qDDotToPack);
      qDDotToPack.add(intermediateQDDot);
      qDDotToPack.scale(0.5);
   }

   public void computeAngularAcceleration(QuaternionReadOnly q, Vector4DReadOnly qDDot, Vector3DReadOnly angularVelocity, Vector3DBasics angularAccelerationToPack)
   {
      computeQDot(q, angularVelocity, intermediateQDot);
      computeAngularAcceleration(q, intermediateQDot, qDDot, angularAccelerationToPack);
   }

   public void computeAngularAcceleration(QuaternionReadOnly q, Vector4DReadOnly qDot, Vector4DReadOnly qDDot, Vector3DBasics angularAccelerationToPack)
   {
      qConj.setAndConjugate(q);
      qDotConj.set(-qDot.getX(), -qDot.getY(), -qDot.getZ(), qDot.getS());
      multiply(qDot, qDotConj, intermediateAngularAcceleration);
      multiply(qDDot, qConj, angularAccelerationToPack);
      angularAccelerationToPack.add(intermediateAngularAcceleration);
      angularAccelerationToPack.scale(2.0);
   }

   public void computeQDotByFiniteDifferenceCentral(QuaternionReadOnly qPrevious, QuaternionReadOnly qNext, double dt, Vector4DBasics qDotToPack)
   {
      qDotToPack.set(qNext);
      qDotToPack.sub(qPrevious);
      qDotToPack.scale(0.5 / dt);
   }

   public void computeQDDotByFiniteDifferenceCentral(QuaternionReadOnly qPrevious, QuaternionReadOnly q, QuaternionReadOnly qNext, double dt, Vector4DBasics qDDotToPack)
   {
      qDDotToPack.set(qNext);
      qDDotToPack.sub(q);
      qDDotToPack.sub(q);
      qDDotToPack.add(qPrevious);
      qDDotToPack.scale(1.0 / MathTools.square(dt));
   }

   private final Quaternion qInv = new Quaternion();

   /**
    * This computes the product: resultToPack = (q0^-1 q1)
    */
   public void computeQuaternionDifference(QuaternionReadOnly q0, QuaternionReadOnly q1, QuaternionBasics resultToPack)
   {
      resultToPack.setAndConjugate(q0);
      resultToPack.multiply(q1);
   }

   private final Vector4D pureQuatForMultiply = new Vector4D();

   public void multiply(QuaternionReadOnly q, Vector3DReadOnly v, Vector4DBasics resultToPack)
   {
      setAsPureQuaternion(v, resultToPack);
      QuaternionTools.multiply(q, resultToPack, resultToPack);
   }

   public void multiply(Vector3DReadOnly v, Vector4DReadOnly q, Vector4DBasics resultToPack)
   {
      setAsPureQuaternion(v, resultToPack);
      QuaternionTools.multiply(resultToPack, q, resultToPack);
   }

   public void multiply(Vector3DReadOnly v, QuaternionReadOnly q, Vector4DBasics resultToPack)
   {
      setAsPureQuaternion(v, resultToPack);
      QuaternionTools.multiply(resultToPack, q, resultToPack);
   }

   public void multiply(Vector4DReadOnly q1, Vector4DReadOnly q2, Vector3DBasics resultToPack)
   {
      QuaternionTools.multiply(q1, q2, pureQuatForMultiply);
      setVectorFromPureQuaternion(pureQuatForMultiply, resultToPack);
   }

   public void multiply(Vector4DReadOnly q1, QuaternionReadOnly q2, Vector3DBasics resultToPack)
   {
      QuaternionTools.multiply(q1, q2, pureQuatForMultiply);
      setVectorFromPureQuaternion(pureQuatForMultiply, resultToPack);
   }

   public void multiply(QuaternionReadOnly q1, Vector4DReadOnly q2, Vector3DBasics resultToPack)
   {
      QuaternionTools.multiply(q1, q2, pureQuatForMultiply);
      setVectorFromPureQuaternion(pureQuatForMultiply, resultToPack);
   }

   public void inverseMultiply(QuaternionReadOnly q1, QuaternionReadOnly q2, QuaternionBasics resultToPack)
   {
      qInv.setAndConjugate(q1);
      resultToPack.multiply(qInv, q2);
   }

   public void setAsPureQuaternion(Vector3DReadOnly vector, Vector4DBasics pureQuaternionToSet)
   {
      pureQuaternionToSet.set(vector.getX(), vector.getY(), vector.getZ(), 0.0);
   }

   public void setVectorFromPureQuaternion(Vector4DReadOnly pureQuaternion, Vector3DBasics vectorToPack)
   {
      vectorToPack.set(pureQuaternion.getX(), pureQuaternion.getY(), pureQuaternion.getZ());
   }
}
