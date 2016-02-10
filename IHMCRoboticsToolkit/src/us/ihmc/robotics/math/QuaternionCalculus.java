package us.ihmc.robotics.math;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.MathTools;

public class QuaternionCalculus
{
   public QuaternionCalculus()
   {
   }

   private final AxisAngle4d axisAngleForLog = new AxisAngle4d();

   /**
    * This computes: resultToPack = log(q)
    * @param q is a unit quaternion and describes a orientation.
    * @param resultToPack is used to store the result and is a pure quaternion (w = 0.0).
    */
   public void log(Quat4d q, Quat4d resultToPack)
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
   public void log(Quat4d q, Vector3d resultToPack)
   {
      axisAngleForLog.set(q);
      resultToPack.set(axisAngleForLog.getX(), axisAngleForLog.getY(), axisAngleForLog.getZ());
      resultToPack.scale(axisAngleForLog.getAngle());
   }

   private final Vector3d vectorForExp = new Vector3d();

   public void exp(Vector3d rotationVector, Quat4d quaternionToPack)
   {
      double length = rotationVector.length();
      if (length < 1.0e-7)
      {
         quaternionToPack.set(0.0, 0.0, 0.0, 1.0);
         return;
      }
      
      quaternionToPack.w = Math.cos(0.5 * length);
      double s = Math.sin(0.5 * length);
      vectorForExp.set(rotationVector);
      vectorForExp.scale(1.0 / length);
      quaternionToPack.x = s * vectorForExp.x;
      quaternionToPack.y = s * vectorForExp.y;
      quaternionToPack.z = s * vectorForExp.z;
   }

   /**
    * Interpolation from q0 to q1 for a given alpha = [0, 1] using SLERP.
    * Computes: resultToPack = q0 (q0^-1 q1)^alpha.
    */
   public void interpolate(double alpha, Quat4d q0, Quat4d q1, Quat4d qInterpolatedToPack)
   {
      interpolate(alpha, q0, q1, qInterpolatedToPack, true);
   }

   private final Quat4d tempQ1ForInterpolation = new Quat4d();

   public void interpolate(double alpha, Quat4d q0, Quat4d q1, Quat4d qInterpolatedToPack, boolean preventExtraSpin)
   {
      tempQ1ForInterpolation.set(q1);

      if (preventExtraSpin && dot(q0, tempQ1ForInterpolation) < 0.0)
      {
         tempQ1ForInterpolation.negate();
      }

      computeQuaternionDifference(q0, tempQ1ForInterpolation, qInterpolatedToPack);
      pow(qInterpolatedToPack, alpha, qInterpolatedToPack);
      qInterpolatedToPack.mul(q0, qInterpolatedToPack);
   }

   private final AxisAngle4d axisAngleForPow = new AxisAngle4d();

   /**
    * This computes: resultToPack = q^power.
    * @param q is a unit quaternion and describes a orientation.
    * @param resultToPack is used to store the result.
    */
   public void pow(Quat4d q, double power, Quat4d resultToPack)
   {
      axisAngleForPow.set(q);
      axisAngleForPow.setAngle(power * axisAngleForPow.getAngle());
      resultToPack.set(axisAngleForPow);
   }

   private final Quat4d qConj = new Quat4d();

   public void computeAngularVelocity(Quat4d q, Quat4d qDot, Vector3d angularVelocityToPack)
   {
      qConj.conjugate(q);
      multiply(qDot, qConj, angularVelocityToPack);
      angularVelocityToPack.scale(2.0);
   }

   public void computeQDot(Quat4d q, Vector3d angularVelocity, Quat4d qDotToPack)
   {
      multiply(angularVelocity, q, qDotToPack);
      qDotToPack.scale(0.5);
   }

   private final Quat4d intermediateQDot = new Quat4d();
   private final Quat4d qDotConj = new Quat4d();
   private final Vector3d intermediateAngularAcceleration = new Vector3d();
   private final Vector3d intermediateAngularVelocity = new Vector3d();
   private final Quat4d intermediateQDDot = new Quat4d();

   public void computeQDDot(Quat4d q, Quat4d qDot, Vector3d angularAcceleration, Quat4d qDDotToPack)
   {
      computeAngularVelocity(q, qDot, intermediateAngularVelocity);
      computeQDDot(q, qDot, intermediateAngularVelocity, angularAcceleration, qDDotToPack);
   }

   public void computeQDDot(Quat4d q, Vector3d angularVelocity, Vector3d angularAcceleration, Quat4d qDDotToPack)
   {
      computeQDot(q, angularVelocity, intermediateQDot);
      computeQDDot(q, intermediateQDot, angularVelocity, angularAcceleration, qDDotToPack);
   }

   public void computeQDDot(Quat4d q, Quat4d qDot, Vector3d angularVelocity, Vector3d angularAcceleration, Quat4d qDDotToPack)
   {
      multiply(angularAcceleration, q, intermediateQDDot);
      multiply(angularVelocity, qDot, qDDotToPack);
      qDDotToPack.add(intermediateQDDot);
      qDDotToPack.scale(0.5);
   }

   public void computeAngularAcceleration(Quat4d q, Quat4d qDDot, Vector3d angularVelocity, Vector3d angularAccelerationToPack)
   {
      computeQDot(q, angularVelocity, intermediateQDot);
      computeAngularAcceleration(q, intermediateQDot, qDDot, angularAccelerationToPack);
   }

   public void computeAngularAcceleration(Quat4d q, Quat4d qDot, Quat4d qDDot, Vector3d angularAccelerationToPack)
   {
      qConj.conjugate(q);
      qDotConj.conjugate(qDot);
      multiply(qDot, qDotConj, intermediateAngularAcceleration);
      multiply(qDDot, qConj, angularAccelerationToPack);
      angularAccelerationToPack.add(intermediateAngularAcceleration);
      angularAccelerationToPack.scale(2.0);
   }

   public void computeQDotByFiniteDifferenceCentral(Quat4d qPrevious, Quat4d qNext, double dt, Quat4d qDotToPack)
   {
      qDotToPack.set(qNext);
      qDotToPack.sub(qPrevious);
      qDotToPack.scale(0.5 / dt);
   }

   public void computeQDDotByFiniteDifferenceCentral(Quat4d qPrevious, Quat4d q, Quat4d qNext, double dt, Quat4d qDDotToPack)
   {
      qDDotToPack.set(qNext);
      qDDotToPack.sub(q);
      qDDotToPack.sub(q);
      qDDotToPack.add(qPrevious);
      qDDotToPack.scale(1.0 / MathTools.square(dt));
   }

   /**
    * Rotates qToTransform by the rotation described by q: qToTransform = q qToTransform q^-1
    * Assumes that q is a unit-quaternion and describes a orientation.
    * Assumes that qToTransform is a pure-quaternion.
    */
   public void transform(Quat4d q, Quat4d qToTransform)
   {
      transform(q, qToTransform, qToTransform);
   }

   public void transform(Quat4d q, Quat4d qToTransform, Quat4d resultToPack)
   {
      resultToPack.mul(q, qToTransform);
      resultToPack.mulInverse(q);
   }

   public void transform(Quat4d q, Vector3d vectorToTransform)
   {
      transform(q, vectorToTransform, vectorToTransform);
   }

   private final Quat4d pureQuatForTransform = new Quat4d();

   public void transform(Quat4d q, Vector3d vectorToTransform, Vector3d resultToPack)
   {
      setAsPureQuaternion(vectorToTransform, pureQuatForTransform);
      pureQuatForTransform.mul(q, pureQuatForTransform);
      qInv.conjugate(q);
      pureQuatForTransform.mul(qInv);
      setVectorFromPureQuaternion(pureQuatForTransform, resultToPack);
   }

   private final Quat4d qInv = new Quat4d();

   /**
    * Rotates qToTransform by the inverse of the rotation described by q: qToTransform = q^-1 qToTransform q
    * Assumes that q is a unit-quaternion and describes a orientation.
    * Assumes that qToTransform is a pure-quaternion.
    */
   public void invertTransform(Quat4d q, Quat4d qToTransform)
   {
      invertTransform(q, qToTransform, qToTransform);
   }

   public void invertTransform(Quat4d q, Quat4d qToTransform, Quat4d resultToPack)
   {
      qInv.conjugate(q);
      transform(qInv, qToTransform, resultToPack);
   }

   public void invertTransform(Quat4d q, Vector3d vectorToTransform)
   {
      invertTransform(q, vectorToTransform, vectorToTransform);
   }

   public void invertTransform(Quat4d q, Vector3d vectorToTransform, Vector3d resultToPack)
   {
      qInv.conjugate(q);
      transform(qInv, vectorToTransform, resultToPack);
   }

   public double dot(Quat4d q0, Quat4d q1)
   {
      return q0.getX() * q1.getX() + q0.getY() * q1.getY() + q0.getZ() * q1.getZ() + q0.getW() * q1.getW();
   }

   /**
    * This computes the product: resultToPack = (q0^-1 q1)
    */
   public void computeQuaternionDifference(Quat4d q0, Quat4d q1, Quat4d resultToPack)
   {
      resultToPack.conjugate(q0);
      resultToPack.mul(q1);
   }

   private final Quat4d pureQuatForMultiply = new Quat4d();

   public void multiply(Quat4d q, Vector3d v, Vector3d resultToPack)
   {
      setAsPureQuaternion(v, pureQuatForMultiply);
      pureQuatForMultiply.mul(q, pureQuatForMultiply);
      resultToPack.set(pureQuatForMultiply.x, pureQuatForMultiply.y, pureQuatForMultiply.z);
   }

   public void multiply(Quat4d q, Vector3d v, Quat4d resultToPack)
   {
      setAsPureQuaternion(v, resultToPack);
      resultToPack.mul(q, resultToPack);
   }

   public void multiply(Vector3d v, Quat4d q, Quat4d resultToPack)
   {
      setAsPureQuaternion(v, resultToPack);
      resultToPack.mul(q);
   }

   public void multiply(Quat4d q1, Quat4d q2, Vector3d resultToPack)
   {
      pureQuatForMultiply.mul(q1, q2);
      setVectorFromPureQuaternion(pureQuatForMultiply, resultToPack);
   }

   public void inverseMultiply(Quat4d q1, Quat4d q2, Quat4d resultToPack)
   {
      qInv.conjugate(q1);
      resultToPack.mul(qInv, q2);
   }

   public void setAsPureQuaternion(Vector3d vector, Quat4d pureQuaternionToSet)
   {
      pureQuaternionToSet.set(vector.getX(), vector.getY(), vector.getZ(), 0.0);
   }

   public void setVectorFromPureQuaternion(Quat4d pureQuaternion, Vector3d vectorToPack)
   {
      vectorToPack.set(pureQuaternion.getX(), pureQuaternion.getY(), pureQuaternion.getZ());
   }
}
