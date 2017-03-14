package us.ihmc.robotics.kinematics.fourbar;

import static java.lang.Math.*;
import static java.lang.Math.min;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.*;
import static us.ihmc.robotics.MathTools.*;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.robotics.MathTools;

public class ConstantSideFourBarCalculatorWithDerivatives implements FourbarCalculatorWithDerivatives
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

   public ConstantSideFourBarCalculatorWithDerivatives(ConstantSideFourBarCalculatorWithDerivatives constantSideFourBarCalculatorWithDerivatives)
   {
      this.a = constantSideFourBarCalculatorWithDerivatives.a;
      this.b = constantSideFourBarCalculatorWithDerivatives.b;
      this.c = constantSideFourBarCalculatorWithDerivatives.c;
      this.d = constantSideFourBarCalculatorWithDerivatives.d;

      this.minA = constantSideFourBarCalculatorWithDerivatives.minA;
      this.maxA = constantSideFourBarCalculatorWithDerivatives.maxA;
   }

   public ConstantSideFourBarCalculatorWithDerivatives(double length_DA, double length_AB, double length_BC, double length_CD)
   {
      this.a = abs(length_DA);
      this.b = abs(length_AB);
      this.c = abs(length_BC);
      this.d = abs(length_CD);

      double eMax = min(a + b, d + c);
      if (eMax == a + b)
         maxA = PI;
      else
         maxA = FourbarCalculatorTools.getAngleWithCosineLaw(a, b, eMax);

      double fMax = min(a + d, b + c);
      if (fMax == a + d)
         minA = FourbarCalculatorTools.getAngleWithCosineLaw(fMax, b, c);
      else
         minA = FourbarCalculatorTools.getAngleWithCosineLaw(fMax, a, d);
   }

   /**
    * Compute every angle of the quadrilateral AND crop the range of the input angle so that the four bar doesn't flip.
    * @param angleDABInRadians is the angle formed by the sides a and b (see scheme in this class)
    * @return true if the angle DAB is out of range making the quadrilateral non-convex
    */
   public boolean updateAnglesGivenAngleDAB(double angleDABInRadians)
   {
      // Solve angles
      double A = clamp(angleDABInRadians, minA, maxA);
      double e = unknownTriangleSideLengthByLawOfCosine(a, b, A);
      double C = FourbarCalculatorTools.getAngleWithCosineLaw(c, d, e);
      double angleDBA = FourbarCalculatorTools.getAngleWithCosineLaw(b, e, a);
      double angleDBC = FourbarCalculatorTools.getAngleWithCosineLaw(c, e, d);
      double B = angleDBA + angleDBC;
      double D = 2 * PI - A - B - C;
      this.angleDAB = A;
      this.angleABC = B;
      this.angleBCD = C;
      this.angleCDA = D;

      return !MathTools.intervalContains(angleDABInRadians, minA, maxA);
   }
   
   /**
    * Takes in angle B and computes the value of the master joint angle. 
    * Same notation used in the rest of the class. 
    */
   public void computeMasterJointAngleGivenAngleABC(double angleABCInRadians)
   {
      double B = angleABCInRadians;
      double f = unknownTriangleSideLengthByLawOfCosine(b, c, B);
      double D = FourbarCalculatorTools.getAngleWithCosineLaw(d, a, f);
      double angleACB = FourbarCalculatorTools.getAngleWithCosineLaw(c, f, b);
      double angleACD = FourbarCalculatorTools.getAngleWithCosineLaw(d, f, a);
      double C = angleACB + angleACD;
      double A = 2 * PI - D - B - C;
      this.angleDAB = A;
   }
   
   /**
    * Takes in angle C and computes the value of the master joint angle. 
    * Same notation used in the rest of the class. 
    */
   public void computeMasterJointAngleGivenAngleBCD(double angleBCDInRadians)
   {
      double C = angleBCDInRadians;
      double e = unknownTriangleSideLengthByLawOfCosine(c, d, C);
      double A = FourbarCalculatorTools.getAngleWithCosineLaw(a, b, e);
      this.angleDAB = A;
   }      
   
   /**
    * Takes in angle D and computes the value of the master joint angle. 
    * Same notation used in the rest of the class. 
    */
   public void computeMasterJointAngleGivenAngleCDA(double angleCDAInRadians)
   {
      double D = angleCDAInRadians;
      double f = unknownTriangleSideLengthByLawOfCosine(d, a, D);
      double angleCAD = FourbarCalculatorTools.getAngleWithCosineLaw(a, f, d);
      double angleCAB = FourbarCalculatorTools.getAngleWithCosineLaw(b, f, c);
      double A = angleCAD + angleCAB;
      this.angleDAB = A;
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
      double A = clamp(angleDABInRadians, minA, maxA);
      double dAdT = angularVelocityDAB;
      double e = unknownTriangleSideLengthByLawOfCosine(a, b, A);
      double eDot = a * b * sin(A) * dAdT / e;
      double dCdT = FourbarCalculatorTools.getAngleDotWithCosineLaw(c, d, 0.0, e, eDot);
      double angleDotDBA = FourbarCalculatorTools.getAngleDotWithCosineLaw(b, e, eDot, a, 0.0);
      double angleDotDBC = FourbarCalculatorTools.getAngleDotWithCosineLaw(c, e, eDot, d, 0.0);
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
      double A = clamp(angleDABInRadians, minA, maxA);
      double dAdT = angularVelocityDAB;
      double dAdT2 = angularAccelerationDAB;
      double e = unknownTriangleSideLengthByLawOfCosine(a, b, A);
      double eDot = a * b * sin(A) * dAdT / e;
      double eDDot = a * b / e * (cos(A) * dAdT * dAdT + sin(A) * (dAdT2 - eDot * dAdT / e));
      double dCdT2 = FourbarCalculatorTools.getAngleDDotWithCosineLaw(c, d, 0.0, 0.0, e, eDot, eDDot);
      double angleDDotDBA = FourbarCalculatorTools.getAngleDDotWithCosineLaw(b, e, eDot, eDDot, a, 0.0, 0.0);
      double angleDDotDBC = FourbarCalculatorTools.getAngleDDotWithCosineLaw(c, e, eDot, eDDot, d, 0.0, 0.0);
      double dBdT2 = angleDDotDBA + angleDDotDBC;
      double dDdT2 = -dAdT2 - dBdT2 - dCdT2;
      this.angleDt2DAB = dAdT2;
      this.angleDt2ABC = dBdT2;
      this.angleDt2BCD = dCdT2;
      this.angleDt2CDA = dDdT2;

      return isAHittingBounds;
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
   
   public double getAB()
   {
      return a;
   }
   
   public double getBC()
   {
      return b;
   }
   
   public double getCD()
   {
      return c;
   }
   
   public double getDA()
   {
      return d;
   }
}
