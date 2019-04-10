package us.ihmc.robotics.math.trajectories;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;

/**
 * This calculator is based on {@link HermiteCurvedBasedOrientationTrajectoryGenerator} without YoVariables.
 * And this is using {@code Riven.atan2(y, x)} which is faster than {@code Math.atan2(y, x)} defined in {@code jre 1.8}.
 * See test file {@link InverseOfTangentComputationTest.}
 */
public class SimpleHermiteCurvedBasedOrientationTrajectoryCalculator
{
   private double trajectoryTime;
   private int numberOfRevolutions;

   private double[] cumulativeBeziers;
   private double[] cumulativeBeziersDot;
   private double[] cumulativeBeziersDDot;

   private final Vector3DBasics[] controlRotations;

   private final QuaternionBasics currentOrientation;
   private final Vector3DBasics currentAngularVelocity;
   private final Vector3DBasics currentAngularAcceleration;

   private final QuaternionBasics initialOrientation;
   private final Vector3DBasics initialAngularVelocity;
   private final QuaternionBasics finalOrientation;
   private final Vector3DBasics finalAngularVelocity;

   private static final double EPS = 10E-12;

   private boolean convertAngularVelocity = false;
   private boolean convertAngularAcceleration = true;

   Quaternion qProduct = new Quaternion();

   Vector4D qDot = new Vector4D();
   Vector4D qDot1 = new Vector4D();
   Vector4D qDot2 = new Vector4D();
   Vector4D qDot3 = new Vector4D();
   Vector4D qProductDot = new Vector4D();

   Vector4D qDDot = new Vector4D();
   Vector4D qDDot1 = new Vector4D();
   Vector4D qDDot2 = new Vector4D();
   Vector4D qDDot3 = new Vector4D();
   Vector4D qDDotTemp = new Vector4D();
   Vector4D qProductDDot = new Vector4D();

   Vector3D d1 = new Vector3D();
   Vector3D d2 = new Vector3D();
   Vector3D d3 = new Vector3D();

   Quaternion expD1B1 = new Quaternion();
   Quaternion expD2B2 = new Quaternion();
   Quaternion expD1B1_expD2B2 = new Quaternion();
   Quaternion expD3B3 = new Quaternion();

   Vector4D d1B1Dot = new Vector4D();
   Vector4D d2B2Dot = new Vector4D();
   Vector4D d3B3Dot = new Vector4D();

   Vector4D d1B1DDot = new Vector4D();
   Vector4D d2B2DDot = new Vector4D();
   Vector4D d3B3DDot = new Vector4D();

   Quaternion qInterpolated = new Quaternion();
   Vector3D angularVelocityInterpolated = new Vector3D();
   Vector3D angularAccelerationInterpolated = new Vector3D();

   public SimpleHermiteCurvedBasedOrientationTrajectoryCalculator()
   {
      cumulativeBeziers = new double[4];
      cumulativeBeziersDot = new double[4];
      cumulativeBeziersDDot = new double[4];
      controlRotations = new Vector3D[4];
      for (int i = 0; i < 4; i++)
         controlRotations[i] = new Vector3D();

      currentOrientation = new Quaternion();
      currentAngularVelocity = new Vector3D();
      currentAngularAcceleration = new Vector3D();

      initialOrientation = new Quaternion();
      initialAngularVelocity = new Vector3D();
      finalOrientation = new Quaternion();
      finalAngularVelocity = new Vector3D();
   }

   public void setNumberOfRevolutions(int numberOfRevolutions)
   {
      this.numberOfRevolutions = numberOfRevolutions;
   }

   public void setTrajectoryTime(double duration)
   {
      MathTools.checkIntervalContains(duration, 0.0, Double.POSITIVE_INFINITY);
      trajectoryTime = duration;
   }

   public void setInitialConditions(QuaternionReadOnly initialOrientation, Vector3DReadOnly initialAngularVelocity)
   {
      this.initialOrientation.set(initialOrientation);
      this.initialAngularVelocity.set(initialAngularVelocity);
   }

   public void setFinalConditions(QuaternionReadOnly finalOrientation, Vector3DReadOnly finalAngularVelocity)
   {
      this.finalOrientation.set(finalOrientation);
      this.finalAngularVelocity.set(finalAngularVelocity);
   }

   public void initialize()
   {
      if (initialOrientation.dot(finalOrientation) < 0.0)
         finalOrientation.negate();

      updateControlQuaternions();
      compute(0.0);
   }

   Quaternion tempQuaternion = new Quaternion();
   Vector3D wa = new Vector3D();
   Vector3D wb = new Vector3D();
   Vector3D delta = new Vector3D();

   private void updateControlQuaternions()
   {
      double TOverThree = trajectoryTime / 3.0;

      QuaternionReadOnly qa = initialOrientation;
      QuaternionReadOnly qb = finalOrientation;
      wa.set(initialAngularVelocity);
      wb.set(finalAngularVelocity);
      qa.inverseTransform(wa);
      qb.inverseTransform(wb);

      // delta1 = wa * T / 3.0
      delta.setAndScale(TOverThree, wa);
      controlRotations[1].set(delta);

      // delta2 = log( exp(-wa*T/3) * qa^-1 * qb * exp(-wb*T/3) )
      tempQuaternion.difference(qa, qb);
      tempQuaternion.preMultiply(exp(-TOverThree, wa));
      tempQuaternion.multiply(exp(-TOverThree, wb));
      controlRotations[2].set(log(tempQuaternion));

      if (numberOfRevolutions != 0)
      {
         delta.set(controlRotations[2]);
         if (delta.lengthSquared() > 1.0e-10)
         {
            delta.normalize();
            delta.scale(numberOfRevolutions * 2.0 * Math.PI);
            controlRotations[2].add(delta);
         }
      }

      // delta3 = wb * T / 3.0
      delta.setAndScale(TOverThree, wb);
      controlRotations[3].set(delta);
   }

   public void setConvertingAngularVelocity(boolean use)
   {
      convertAngularVelocity = use;
   }

   public void setConvertingAngularAcceleration(boolean use)
   {
      convertAngularAcceleration = use;
   }

   public void compute(double time)
   {
      if (Double.isNaN(time))
      {
         throw new RuntimeException("Can not call compute on trajectory generator with time NaN.");
      }

      if (time < 0.0)
      {
         currentOrientation.set(initialOrientation);
         currentAngularVelocity.setToZero();
         currentAngularAcceleration.setToZero();
         return;
      }

      if (time > trajectoryTime)
      {
         currentOrientation.set(finalOrientation);
         currentAngularVelocity.setToZero();
         currentAngularAcceleration.setToZero();
         return;
      }

      if (trajectoryTime < EPS && trajectoryTime >= 0.0)
      {
         currentOrientation.set(initialOrientation);
         currentAngularVelocity.set(initialAngularVelocity);
         currentAngularAcceleration.setToZero();
         return;
      }

      time = MathTools.clamp(time, 0.0, trajectoryTime);

      computeBezierBasedCurve(time, qInterpolated, angularVelocityInterpolated, angularAccelerationInterpolated);

      currentOrientation.set(qInterpolated);
      currentAngularVelocity.set(angularVelocityInterpolated);
      currentAngularAcceleration.set(angularAccelerationInterpolated);
   }

   private void computeBezierBasedCurve(double time, QuaternionBasics q, Vector3D angularVelocity, Vector3D angularAcceleration)
   {
      updateBezierCoefficients(time);

      // Changing naming convention to make expressions smaller
      QuaternionReadOnly q0 = initialOrientation;
      d1.set(controlRotations[1]);
      d2.set(controlRotations[2]);
      d3.set(controlRotations[3]);

      // Update intermediate variables
      expD1B1.set(exp(cumulativeBeziers[1], d1));
      expD2B2.set(exp(cumulativeBeziers[2], d2));
      expD3B3.set(exp(cumulativeBeziers[3], d3));
      expD1B1_expD2B2.set(expD1B1);
      expD1B1_expD2B2.multiply(expD2B2);

      // In page 1, the authors say they use a specific type of quaternion for which
      // exp(theta * u) = (cos(theta), u * sin(theta)) instead of:
      // exp(theta * u) = (cos(theta/2), u * sin(theta/2)).
      // Because of the latter, the quaternion derivatives in the paper are actually wrong.
      // When derivating the exponent of an exponential term, it should be as follows:
      // d/dt(exp(alpha(t) * u)) = 0.5 * alphaDot(t) * u * exp(alpha(t) * u)
      d1B1Dot.set(d1);
      d2B2Dot.set(d2);
      d3B3Dot.set(d3);
      d1B1Dot.scale(0.5 * cumulativeBeziersDot[1]);
      d2B2Dot.scale(0.5 * cumulativeBeziersDot[2]);
      d3B3Dot.scale(0.5 * cumulativeBeziersDot[3]);

      d1B1DDot.set(d1);
      d2B2DDot.set(d2);
      d3B3DDot.set(d3);
      d1B1DDot.scale(0.5 * cumulativeBeziersDDot[1]);
      d2B2DDot.scale(0.5 * cumulativeBeziersDDot[2]);
      d3B3DDot.scale(0.5 * cumulativeBeziersDDot[3]);

      // Calculate qStar = exp(w1*B1) * exp(w2*B2) * exp(w3*B3)
      qProduct.set(expD1B1);
      qProduct.multiply(expD2B2);
      qProduct.multiply(expD3B3);

      // Calculate qStarDot = qDot1 + qDot2 + qDot3, with:
      // qDot1 = ((w1*BDot1) * qStar
      QuaternionTools.multiply(d1B1Dot, qProduct, qDot1);
      // qDot2 = exp(w1*B1) * exp(w2*B2) * (w2*BDot2) * exp(w3*B3)
      QuaternionTools.multiply(expD1B1_expD2B2, d2B2Dot, qDot2);
      QuaternionTools.multiply(qDot2, expD3B3, qDot2);
      // qDot3 = qStar * (w3*BDot3)
      QuaternionTools.multiply(qProduct, d3B3Dot, qDot3);
      // Now qStarDot
      qProductDot.add(qDot1, qDot2);
      qProductDot.add(qDot3);

      // Calculate qStarDDot:
      // qStarDDot = qDDot1 + qDDot2 + qDDot3
      // qDDot1 = (w1*BDDot1) * qStar + (w1*BDot1) * qStarDot
      QuaternionTools.multiply(d1B1DDot, qProduct, qDDotTemp);
      QuaternionTools.multiply(d1B1Dot, qProductDot, qDDot1);
      qDDot1.add(qDDotTemp);

      // qDDot2 = exp(w1*B1)*exp(w2*B2)*{ (w2*BDDot2) + (w2*BDot2)*(w2*BDot2) }*exp(w3*B3) + (w1*BDot1)*qDot2 + qDot2*(w3*BDot3)
      QuaternionTools.multiply(d2B2Dot, d2B2Dot, qDDot2);
      qDDot2.add(d2B2DDot);
      QuaternionTools.multiply(expD1B1_expD2B2, qDDot2, qDDotTemp);
      QuaternionTools.multiply(qDDotTemp, expD3B3, qDDotTemp);
      qDDot2.set(qDDotTemp);
      QuaternionTools.multiply(d1B1Dot, qDot2, qDDotTemp);
      qDDot2.add(qDDotTemp);
      QuaternionTools.multiply(qDot2, d3B3Dot, qDDotTemp);
      qDDot2.add(qDDotTemp);

      // qDDot3 = qStar * (w3*BDDot3) + qStarDot * (w3*BDot3)
      QuaternionTools.multiply(qProduct, d3B3DDot, qDDotTemp);
      QuaternionTools.multiply(qProductDot, d3B3Dot, qDDot3);
      qDDot3.add(qDDotTemp);

      // Now qStarDDot
      qProductDDot.add(qDDot1, qDDot2);
      qProductDDot.add(qDDot3);

      q.multiply(q0, qProduct);
      QuaternionTools.multiply(q0, qProductDot, qDot);
      QuaternionTools.multiply(q0, qProductDDot, qDDot);

      if (convertAngularVelocity)
         convertToAngularVelocity(q, qDot, angularVelocity);
      if (convertAngularAcceleration)
         convertToAngularAcceleration(q, qDot, qDDot, angularAcceleration);
   }

   private void updateBezierCoefficients(double t)
   {
      double oneOverT = 1.0 / trajectoryTime;
      double tOverT = t * oneOverT;

      cumulativeBeziers[1] = (1.0 - cube(1.0 - tOverT));
      cumulativeBeziers[2] = 3.0 * square(tOverT) - 2.0 * cube(tOverT);
      cumulativeBeziers[3] = cube(tOverT);

      cumulativeBeziersDot[1] = 3.0 * oneOverT * square(1.0 - tOverT);
      cumulativeBeziersDot[2] = 6.0 * tOverT * oneOverT * (1.0 - tOverT);
      cumulativeBeziersDot[3] = 3.0 * square(tOverT) * oneOverT;

      cumulativeBeziersDDot[1] = -6.0 * square(oneOverT) * (1.0 - tOverT);
      cumulativeBeziersDDot[2] = 6.0 * square(oneOverT) * (1.0 - 2.0 * tOverT);
      cumulativeBeziersDDot[3] = 6.0 * t * cube(oneOverT);
   }

   private static void convertToAngularVelocity(QuaternionReadOnly q, Vector4DReadOnly qDot, Vector3D angularVelocityToPack)
   {// w = qDot * q^-1
      Vector4D tempConvertVector4D = new Vector4D();

      QuaternionTools.multiplyConjugateRight(qDot, q, tempConvertVector4D);
      angularVelocityToPack.setX(tempConvertVector4D.getX());
      angularVelocityToPack.setY(tempConvertVector4D.getY());
      angularVelocityToPack.setZ(tempConvertVector4D.getZ());
      angularVelocityToPack.scale(2.0);
   }

   private static void convertToAngularAcceleration(QuaternionReadOnly q, Vector4DReadOnly qDot, Vector4DReadOnly qDDot, Vector3D angularAccelerationToPack)
   { // w = qDDot * q^-1 + qDot * qDot^-1
      Vector4D tempConvertVector4D = new Vector4D();

      QuaternionTools.multiplyConjugateRight(qDDot, q, tempConvertVector4D);
      angularAccelerationToPack.setX(tempConvertVector4D.getX());
      angularAccelerationToPack.setY(tempConvertVector4D.getY());
      angularAccelerationToPack.setZ(tempConvertVector4D.getZ());

      QuaternionTools.multiplyConjugateRight(qDot, qDot, tempConvertVector4D);
      angularAccelerationToPack.addX(tempConvertVector4D.getX());
      angularAccelerationToPack.addY(tempConvertVector4D.getY());
      angularAccelerationToPack.addZ(tempConvertVector4D.getZ());
      angularAccelerationToPack.scale(2.0);
   }

   private double square(double value)
   {
      return value * value;
   }

   private double cube(double value)
   {
      return value * value * value;
   }

   private QuaternionReadOnly exp(double alpha, Vector3DReadOnly rotation)
   {
      Quaternion tempLogExpQuaternion = new Quaternion();
      Vector3D tempLogExpVector3D = new Vector3D();
      tempLogExpVector3D.setAndScale(alpha, rotation);
      tempLogExpQuaternion.setRotationVector(tempLogExpVector3D);
      return tempLogExpQuaternion;
   }

   private Vector3DReadOnly log(QuaternionReadOnly q)
   {
      // Expensive guy.
      Vector3D tempLogExpVector3D = new Vector3D();
      q.getRotationVector(tempLogExpVector3D);

      return tempLogExpVector3D;
   }

   public void getOrientation(QuaternionBasics angularOrientationToPack)
   {
      angularOrientationToPack.set(currentOrientation);
   }

   public void getAngularVelocity(Vector3D angularVelocityToPack)
   {
      angularVelocityToPack.set(currentAngularVelocity);
   }

   public void getAngularAcceleration(Vector3D angularAccelerationToPack)
   {
      angularAccelerationToPack.set(currentAngularAcceleration);
   }
}
