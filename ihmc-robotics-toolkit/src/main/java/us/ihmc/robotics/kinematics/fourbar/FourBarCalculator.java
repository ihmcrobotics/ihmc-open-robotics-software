package us.ihmc.robotics.kinematics.fourbar;

import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.unknownTriangleSideLengthByLawOfCosine;

import us.ihmc.commons.MathTools;

public class FourBarCalculator implements FourbarCalculatorWithDerivatives
{
   /*
    * @formatter:off
    *  Representation of the four bar with name correspondences:
    *        a
    *    D--------A
    *    |\      /|
    *    | \e   / |
    *   d|  \  /  |b
    *    |   \/   |
    *    |   /\   |
    *    |  /  \  |
    *    | /f   \ |
    *    |/      \|
    *    C--------B
    *        c
    * @formatter:on
    */

   private double a, b, c, d;
   private double minA, maxA;

   private boolean invertedAtDAB, invertedAtABC, invertedAtBCD, invertedAtCDA;

   // Angles
   private double angleDAB, angleABC, angleBCD, angleCDA;

   // Angular velocities
   private double angleDtDAB, angleDtABC, angleDtBCD, angleDtCDA;

   // Angular accelerations
   private double angleDt2DAB, angleDt2ABC, angleDt2BCD, angleDt2CDA;

   public FourBarCalculator()
   {
   }

   public void setSideLengths(double length_DA, double length_AB, double length_BC, double length_CD)
   {
      a = Math.abs(length_DA);
      b = Math.abs(length_AB);
      c = Math.abs(length_BC);
      d = Math.abs(length_CD);

      double eMax = Math.min(a + b, d + c);
      if (eMax == a + b)
         maxA = Math.PI;
      else
         maxA = FourbarCalculatorTools.angleWithCosineLaw(a, b, eMax);

      double fMax = Math.min(a + d, b + c);
      if (fMax == a + d)
         minA = FourbarCalculatorTools.angleWithCosineLaw(fMax, b, c);
      else
         minA = FourbarCalculatorTools.angleWithCosineLaw(fMax, a, d);
   }

   public void setInversion(boolean invertedAtDAB, boolean invertedAtABC, boolean invertedAtBCD, boolean invertedAtCDA)
   {
      int inversionCount = invertedAtDAB ? 1 : 0;
      inversionCount += invertedAtABC ? 1 : 0;
      inversionCount += invertedAtBCD ? 1 : 0;
      inversionCount += invertedAtCDA ? 1 : 0;

      if (inversionCount > 1)
         throw new IllegalArgumentException("Four-bar can only be inverted at one vertex.");

      this.invertedAtDAB = invertedAtDAB;
      this.invertedAtABC = invertedAtABC;
      this.invertedAtBCD = invertedAtBCD;
      this.invertedAtCDA = invertedAtCDA;
   }

   /**
    * Compute every angle of the quadrilateral AND crop the range of the input angle so that the four
    * bar doesn't flip.
    *
    * @param angleDABInRadians is the angle formed by the sides a and b (see scheme in this class)
    * @return true if the angle DAB is out of range making the quadrilateral non-convex
    */
   @Override
   public boolean updateAnglesGivenAngleDAB(double angleDABInRadians)
   {
      // Solve angles
      double A = MathTools.clamp(angleDABInRadians, minA, maxA);
      double e = unknownTriangleSideLengthByLawOfCosine(a, b, A);
      double C = FourbarCalculatorTools.angleWithCosineLaw(c, d, e);
      double angleDBA = FourbarCalculatorTools.angleWithCosineLaw(b, e, a);
      double angleDBC = FourbarCalculatorTools.angleWithCosineLaw(c, e, d);
      double B = angleDBA + angleDBC;
      double D = 2.0 * Math.PI - A - B - C;
      angleDAB = A;
      angleABC = B;
      angleBCD = C;
      angleCDA = D;

      return !MathTools.intervalContains(angleDABInRadians, minA, maxA);
   }

   /**
    * Takes in angle B and computes the value of the master joint angle. Same notation used in the rest
    * of the class.
    */
   @Override
   public void computeMasterJointAngleGivenAngleABC(double angleABCInRadians)
   {
      double B = angleABCInRadians;
      double f = unknownTriangleSideLengthByLawOfCosine(b, c, B);
      double D = FourbarCalculatorTools.angleWithCosineLaw(d, a, f);
      double angleACB = FourbarCalculatorTools.angleWithCosineLaw(c, f, b);
      double angleACD = FourbarCalculatorTools.angleWithCosineLaw(d, f, a);
      double C = angleACB + angleACD;
      double A = 2 * Math.PI - D - B - C;
      angleDAB = A;
   }

   /**
    * Takes in angle C and computes the value of the master joint angle. Same notation used in the rest
    * of the class.
    */
   @Override
   public void computeMasterJointAngleGivenAngleBCD(double angleBCDInRadians)
   {
      double C = angleBCDInRadians;
      double e = unknownTriangleSideLengthByLawOfCosine(c, d, C);
      double A = FourbarCalculatorTools.angleWithCosineLaw(a, b, e);
      angleDAB = A;
   }

   /**
    * Takes in angle D and computes the value of the master joint angle. Same notation used in the rest
    * of the class.
    */
   @Override
   public void computeMasterJointAngleGivenAngleCDA(double angleCDAInRadians)
   {
      double D = angleCDAInRadians;
      double f = unknownTriangleSideLengthByLawOfCosine(d, a, D);
      double angleCAD = FourbarCalculatorTools.angleWithCosineLaw(a, f, d);
      double angleCAB = FourbarCalculatorTools.angleWithCosineLaw(b, f, c);
      double A = angleCAD + angleCAB;
      angleDAB = A;
   }

   /**
    * Compute every angle of the quadrilateral.
    *
    * @param angleDABInRadians  is the angle formed by the sides a and b (see scheme in this class)
    * @param angularVelocityDAB first time-derivative of the angle DAB
    * @return true if the angle DAB is out of range making the quadrilateral non-convex
    */
   @Override
   public boolean updateAnglesAndVelocitiesGivenAngleDAB(double angleDABInRadians, double angularVelocityDAB)
   {
      // Solve angles
      boolean isAHittingBounds = updateAnglesGivenAngleDAB(angleDABInRadians);

      // Solve angular velocity
      double A = MathTools.clamp(angleDABInRadians, minA, maxA);
      double dAdT = angularVelocityDAB;
      double e = unknownTriangleSideLengthByLawOfCosine(a, b, A);
      double eDot = a * b * Math.sin(A) * dAdT / e;
      double dCdT = FourbarCalculatorTools.angleDotWithCosineLaw(c, d, 0.0, e, eDot);
      double angleDotDBA = FourbarCalculatorTools.angleDotWithCosineLaw(b, e, eDot, a, 0.0);
      double angleDotDBC = FourbarCalculatorTools.angleDotWithCosineLaw(c, e, eDot, d, 0.0);
      double dBdT = angleDotDBA + angleDotDBC;
      double dDdT = -dAdT - dBdT - dCdT;
      angleDtDAB = dAdT;
      angleDtABC = dBdT;
      angleDtBCD = dCdT;
      angleDtCDA = dDdT;

      return isAHittingBounds;
   }

   /**
    * Compute every angle of the quadrilateral.
    *
    * @param angleDABInRadians      is the angle formed by the sides a and b (see scheme in this class)
    * @param angularVelocityDAB     first time-derivative of the angle DAB
    * @param angularAccelerationDAB second time-derivative of the angle DAB
    * @return true if the angle DAB is out of range making the quadrilateral non-convex
    */
   @Override
   public boolean updateAnglesVelocitiesAndAccelerationsGivenAngleDAB(double angleDABInRadians, double angularVelocityDAB, double angularAccelerationDAB)
   {
      // Solve angles and angular velocity
      boolean isAHittingBounds = updateAnglesAndVelocitiesGivenAngleDAB(angleDABInRadians, angularVelocityDAB);

      // Solve angular acceleration
      double A = MathTools.clamp(angleDABInRadians, minA, maxA);
      double dAdT = angularVelocityDAB;
      double dAdT2 = angularAccelerationDAB;
      double e = unknownTriangleSideLengthByLawOfCosine(a, b, A);
      double eDot = a * b * Math.sin(A) * dAdT / e;
      double eDDot = a * b / e * (Math.cos(A) * dAdT * dAdT + Math.sin(A) * (dAdT2 - eDot * dAdT / e));
      double dCdT2 = FourbarCalculatorTools.angleDDotWithCosineLaw(c, d, 0.0, 0.0, e, eDot, eDDot);
      double angleDDotDBA = FourbarCalculatorTools.angleDDotWithCosineLaw(b, e, eDot, eDDot, a, 0.0, 0.0);
      double angleDDotDBC = FourbarCalculatorTools.angleDDotWithCosineLaw(c, e, eDot, eDDot, d, 0.0, 0.0);
      double dBdT2 = angleDDotDBA + angleDDotDBC;
      double dDdT2 = -dAdT2 - dBdT2 - dCdT2;
      angleDt2DAB = dAdT2;
      angleDt2ABC = dBdT2;
      angleDt2BCD = dCdT2;
      angleDt2CDA = dDdT2;

      return isAHittingBounds;
   }

   @Override
   public double getAngleDAB()
   {
      return angleDAB;
   }

   @Override
   public double getAngleABC()
   {
      return angleABC;
   }

   @Override
   public double getAngleBCD()
   {
      return angleBCD;
   }

   @Override
   public double getAngleCDA()
   {
      return angleCDA;
   }

   @Override
   public double getAngleDtDAB()
   {
      return angleDtDAB;
   }

   @Override
   public double getAngleDtABC()
   {
      return angleDtABC;
   }

   @Override
   public double getAngleDtBCD()
   {
      return angleDtBCD;
   }

   @Override
   public double getAngleDtCDA()
   {
      return angleDtCDA;
   }

   @Override
   public double getAngleDt2DAB()
   {
      return angleDt2DAB;
   }

   @Override
   public double getAngleDt2ABC()
   {
      return angleDt2ABC;
   }

   @Override
   public double getAngleDt2BCD()
   {
      return angleDt2BCD;
   }

   @Override
   public double getAngleDt2CDA()
   {
      return angleDt2CDA;
   }

   @Override
   public double getMinDAB()
   {
      return minA;
   }

   @Override
   public double getMaxDAB()
   {
      return maxA;
   }

   @Override
   public double getAB()
   {
      return b;
   }

   @Override
   public double getBC()
   {
      return c;
   }

   @Override
   public double getCD()
   {
      return d;
   }

   @Override
   public double getDA()
   {
      return a;
   }
}
