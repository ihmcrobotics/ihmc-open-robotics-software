package us.ihmc.robotics.kinematics.fourbar;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.acos;
import static java.lang.Math.cos;
import static java.lang.Math.min;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static us.ihmc.robotics.MathTools.checkIfInRange;
import static us.ihmc.robotics.MathTools.clipToMinMax;
import static us.ihmc.robotics.MathTools.cube;
import static us.ihmc.robotics.MathTools.square;
import static us.ihmc.robotics.geometry.GeometryTools.getUnknownTriangleSideLengthByLawOfCosine;

import us.ihmc.robotics.MathTools;

public class FourBarCalculatorFromFastRunner
{
   /*
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
    */

   private final double a, b, c, d;
   private final double minA, maxA;
   

   // Angles
   private double angleDAB, angleABC, angleBCD, angleCDA;

   // Angular velocities
   private double angleDtDAB, angleDtABC, angleDtBCD, angleDtCDA;

   // Angular accelerations
   private double angleDt2DAB, angleDt2ABC, angleDt2BCD, angleDt2CDA;

   public FourBarCalculatorFromFastRunner(double length_DA, double length_AB, double length_BC, double length_CD)
   {
      this.a = abs(length_DA);
      this.b = abs(length_AB);
      this.c = abs(length_BC);
      this.d = abs(length_CD);

      double eMax = min(a + b, d + c);
      if (eMax == a + b)
         maxA = PI;
      else
         maxA = getAngleWithCosineLaw(a, b, eMax);

      double fMax = min(a + d, b + c);
      if (fMax == a + d)
         minA = getAngleWithCosineLaw(fMax, b, c);
      else
         minA = getAngleWithCosineLaw(fMax, a, d);
   }

   /**
    * Compute every angle of the quadrilateral AND crop the range of the input angle so that the four bar doesn't flip.
    * @param angleDABInRadians is the angle formed by the sides a and b (see scheme in this class)
    * @return true if the angle DAB is out of range making the quadrilateral non-convex
    */
   public boolean updateAnglesGivenAngleDAB(double angleDABInRadians)
   {
      // Solve angles
      double A = clipToMinMax(angleDABInRadians, minA, maxA);
      double e = getUnknownTriangleSideLengthByLawOfCosine(a, b, A);
      double C = getAngleWithCosineLaw(c, d, e);
      double angleDBA = getAngleWithCosineLaw(b, e, a);
      double angleDBC = getAngleWithCosineLaw(c, e, d);
      double B = angleDBA + angleDBC;
      double D = 2 * PI - A - B - C;
      this.angleDAB = A;
      this.angleABC = B;
      this.angleBCD = C;
      this.angleCDA = D;

      return !MathTools.isInsideBoundsInclusive(angleDABInRadians, minA, maxA);
   }
   
   /**
    * Takes in angle B and computes the value of the master joint angle. 
    * Kept the same notation used in the rest of the class, but rotated: 
    * in this case angle ABC is A, so angle DAB (master) is actually D
    */
   public void computeMasterJointAngleGivenAngleABC(double angleABCInRadians)
   {
      double A = angleABCInRadians;
      double e = getUnknownTriangleSideLengthByLawOfCosine(a, b, A);
      double C = getAngleWithCosineLaw(c, d, e);
      double angleDBA = getAngleWithCosineLaw(b, e, a);
      double angleDBC = getAngleWithCosineLaw(c, e, d);
      double B = angleDBA + angleDBC;
      double D = 2 * PI - A - B - C;
      this.angleDAB = D; 
   }
   
   /**
    * Takes in angle B and computes the value of the master joint angle. 
    * Kept the same notation used in the rest of the class, but rotated: 
    * in this case angle BCD is A, so angle DAB (master) is actually C
    */
   public void computeMasterJointAngleGivenAngleBCD(double angleBCDInRadians)
   {
      double A = angleBCDInRadians;
      double e = getUnknownTriangleSideLengthByLawOfCosine(a, b, A);
      double C = getAngleWithCosineLaw(c, d, e);
      this.angleDAB = C;
   }      
   
   /**
    * Takes in angle B and computes the value of the master joint angle. 
    * Kept the same notation used in the rest of the class, but rotated: 
    * in this case angle CDA is A, so angle DAB (master) is actually B
    */
   public void computeMasterJointAngleGivenAngleCDA(double angleCDAInRadians)
   {
      double A = angleCDAInRadians;
      double e = getUnknownTriangleSideLengthByLawOfCosine(a, b, A);
      double angleDBA = getAngleWithCosineLaw(b, e, a);
      double angleDBC = getAngleWithCosineLaw(c, e, d);
      double B = angleDBA + angleDBC;
      this.angleDAB = B;
   }

   /**
    * Compute every angle of the quadrilateral.
    * @param angleDABInRadians is the angle formed by the sides a and b (see scheme in this class)
    * @param angularVelocityDAB first time-derivative of the angle DAB
    * @return true if the angle DAB is out of range making the quadrilateral non-convex
    */
   public boolean updateAnglesAndVelocitiesGivenAngleDAB(double angleDABInRadians, double angularVelocityDAB)
   {
      // Solve angles
      boolean isAHittingBounds = updateAnglesGivenAngleDAB(angleDABInRadians);

      // Solve angular velocity
      double A = clipToMinMax(angleDABInRadians, minA, maxA);
      double dAdT = angularVelocityDAB;
      double e = getUnknownTriangleSideLengthByLawOfCosine(a, b, A);
      double eDot = a * b * sin(A) * dAdT / e;
      double dCdT = getAngleDotWithCosineLaw(c, d, 0.0, e, eDot);
      double angleDotDBA = getAngleDotWithCosineLaw(b, e, eDot, a, 0.0);
      double angleDotDBC = getAngleDotWithCosineLaw(c, e, eDot, d, 0.0);
      double dBdT = angleDotDBA + angleDotDBC;
      double dDdT = -dAdT - dBdT - dCdT;
      this.angleDtDAB = dAdT;
      this.angleDtABC = dBdT;
      this.angleDtBCD = dCdT;
      this.angleDtCDA = dDdT;

      return isAHittingBounds;
   }

   /**
    * Compute every angle of the quadrilateral.
    * @param angleDABInRadians is the angle formed by the sides a and b (see scheme in this class)
    * @param angularVelocityDAB first time-derivative of the angle DAB
    * @param angularAccelerationDAB second time-derivative of the angle DAB
    * @return true if the angle DAB is out of range making the quadrilateral non-convex
    */
   public boolean updateAnglesVelocitiesAndAccelerationsGivenAngleDAB(double angleDABInRadians, double angularVelocityDAB, double angularAccelerationDAB)
   {
      // Solve angles and angular velocity
      boolean isAHittingBounds = updateAnglesAndVelocitiesGivenAngleDAB(angleDABInRadians, angularVelocityDAB);

      // Solve angular acceleration
      double A = clipToMinMax(angleDABInRadians, minA, maxA);
      double dAdT = angularVelocityDAB;
      double dAdT2 = angularAccelerationDAB;
      double e = getUnknownTriangleSideLengthByLawOfCosine(a, b, A);
      double eDot = a * b * sin(A) * dAdT / e;
      double eDDot = a * b / e * (cos(A) * dAdT * dAdT + sin(A) * (dAdT2 - eDot * dAdT / e));
      double dCdT2 = getAngleDDotWithCosineLaw(c, d, 0.0, 0.0, e, eDot, eDDot);
      double angleDDotDBA = getAngleDDotWithCosineLaw(b, e, eDot, eDDot, a, 0.0, 0.0);
      double angleDDotDBC = getAngleDDotWithCosineLaw(c, e, eDot, eDDot, d, 0.0, 0.0);
      double dBdT2 = angleDDotDBA + angleDDotDBC;
      double dDdT2 = -dAdT2 - dBdT2 - dCdT2;
      this.angleDt2DAB = dAdT2;
      this.angleDt2ABC = dBdT2;
      this.angleDt2BCD = dCdT2;
      this.angleDt2CDA = dDdT2;

      return isAHittingBounds;
   }

   private double getCosineAngleWithCosineLaw(double l_neighbour1, double l_neighbour2, double l_opposite)
   {
      checkIfInRange(l_neighbour1, 0.0, Double.POSITIVE_INFINITY);
      checkIfInRange(l_neighbour2, 0.0, Double.POSITIVE_INFINITY);
      checkIfInRange(l_opposite, 0.0, Double.POSITIVE_INFINITY);

      double cosAngle = MathTools.clipToMinMax((square(l_neighbour1) + square(l_neighbour2) - square(l_opposite)) / (2.0 * l_neighbour1 * l_neighbour2), -1.0,
                           1.0);

      return cosAngle;
   }

   private double getCosineAngleDotWithCosineLaw(double l_neighbour1, double l_neighbour2, double lDot_neighbour2, double l_opposite, double lDot_opposite)
   {
      checkIfInRange(l_neighbour1, 0.0, Double.POSITIVE_INFINITY);
      checkIfInRange(l_neighbour2, 0.0, Double.POSITIVE_INFINITY);
      checkIfInRange(l_opposite, 0.0, Double.POSITIVE_INFINITY);

      double cosAngleDot = (square(l_neighbour2) * lDot_neighbour2 - 2.0 * l_neighbour2 * l_opposite * lDot_opposite - lDot_neighbour2 * square(l_neighbour1)
                            + lDot_neighbour2 * square(l_opposite)) / (2.0 * square(l_neighbour2) * l_neighbour1);

      return cosAngleDot;
   }

   private double getCosineAngleDDotWithCosineLaw(double l_neighbour1, double l_neighbour2, double lDot_neighbour2, double lDDot_neighbour2, double l_opposite,
           double lDot_opposite, double lDDot_opposite)
   {
      checkIfInRange(l_neighbour1, 0.0, Double.POSITIVE_INFINITY);
      checkIfInRange(l_neighbour2, 0.0, Double.POSITIVE_INFINITY);
      checkIfInRange(l_opposite, 0.0, Double.POSITIVE_INFINITY);

      double cosAngleDDot = (cube(l_neighbour2) * lDDot_neighbour2 - 2 * square(l_neighbour2 * lDot_opposite)
                             - 2 * square(l_neighbour2) * l_opposite * lDDot_opposite + 4 * l_neighbour2 * lDot_neighbour2 * l_opposite * lDot_opposite
                             + 2 * square(lDot_neighbour2 * l_neighbour1) - 2 * square(lDot_neighbour2 * l_opposite)
                             - l_neighbour2 * lDDot_neighbour2 * square(l_neighbour1)
                             + l_neighbour2 * lDDot_neighbour2 * square(l_opposite)) / (2.0 * cube(l_neighbour2) * l_neighbour1);

      return cosAngleDDot;
   }

   private double getAngleWithCosineLaw(double l_neighbour1, double l_neighbour2, double l_opposite)
   {
      double angle = acos(getCosineAngleWithCosineLaw(l_neighbour1, l_neighbour2, l_opposite));

      return angle;
   }

   private double getAngleDotWithCosineLaw(double l_neighbour1, double l_neighbour2, double lDot_neighbour2, double l_opposite, double lDot_opposite)
   {
      checkIfInRange(l_neighbour1, 0.0, Double.POSITIVE_INFINITY);
      checkIfInRange(l_neighbour2, 0.0, Double.POSITIVE_INFINITY);
      checkIfInRange(l_opposite, 0.0, Double.POSITIVE_INFINITY);

      double cosAngle = getCosineAngleWithCosineLaw(l_neighbour1, l_neighbour2, l_opposite);
      double cosAngleDot = getCosineAngleDotWithCosineLaw(l_neighbour1, l_neighbour2, lDot_neighbour2, l_opposite, lDot_opposite);
      double angleDot = -cosAngleDot / sqrt(1 - cosAngle * cosAngle);

      return angleDot;
   }

   private double getAngleDDotWithCosineLaw(double l_neighbour1, double l_neighbour2, double lDot_neighbour2, double lDDot_neighbour2, double l_opposite,
           double lDot_opposite, double lDDot_opposite)
   {
      checkIfInRange(l_neighbour1, 0.0, Double.POSITIVE_INFINITY);
      checkIfInRange(l_neighbour2, 0.0, Double.POSITIVE_INFINITY);
      checkIfInRange(l_opposite, 0.0, Double.POSITIVE_INFINITY);

      double cosAngle = getCosineAngleWithCosineLaw(l_neighbour1, l_neighbour2, l_opposite);
      double cosAngleDot = getCosineAngleDotWithCosineLaw(l_neighbour1, l_neighbour2, lDot_neighbour2, l_opposite, lDot_opposite);
      double cosAngleDDot = getCosineAngleDDotWithCosineLaw(l_neighbour1, l_neighbour2, lDot_neighbour2, lDDot_neighbour2, l_opposite, lDot_opposite,
                               lDDot_opposite);
      double angleDDot = (-cosAngleDDot + cosAngleDDot * square(cosAngle) - cosAngleDot * cosAngleDot * cosAngle) / pow(1 - square(cosAngle), 3.0 / 2.0);

      return angleDDot;
   }

   public double getAngleDAB()
   {
      return angleDAB;
   }

   public double getAngleABC()
   {
      return angleABC;
   }

   public double getAngleBCD()
   {
      return angleBCD;
   }

   public double getAngleCDA()
   {
      return angleCDA;
   }

   public double getAngleDtDAB()
   {
      return angleDtDAB;
   }

   public double getAngleDtABC()
   {
      return angleDtABC;
   }

   public double getAngleDtBCD()
   {
      return angleDtBCD;
   }

   public double getAngleDtCDA()
   {
      return angleDtCDA;
   }

   public double getAngleDt2DAB()
   {
      return angleDt2DAB;
   }

   public double getAngleDt2ABC()
   {
      return angleDt2ABC;
   }

   public double getAngleDt2BCD()
   {
      return angleDt2BCD;
   }

   public double getAngleDt2CDA()
   {
      return angleDt2CDA;
   }

   public double getMinDAB()
   {
      return minA;
   }

   public double getMaxDAB()
   {
      return maxA;
   }
}
