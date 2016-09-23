package us.ihmc.robotics.kinematics.fourbar;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import static java.lang.Math.*;
import static java.lang.Math.sin;
import static us.ihmc.robotics.MathTools.checkIfInRange;
import static us.ihmc.robotics.MathTools.clipToMinMax;
import static us.ihmc.robotics.MathTools.square;
import static us.ihmc.robotics.geometry.GeometryTools.getUnknownTriangleSideLengthByLawOfCosine;

public class YoVariableSideFourbarCalculatorWithDerivatives implements FourbarCalculatorWithDerivatives
{
   private final DoubleYoVariable a, b, c, d;
   private final DoubleYoVariable minA, maxA;

   // Angles
   private double angleDAB, angleABC, angleBCD, angleCDA;

   // Angular velocities
   private double angleDtDAB, angleDtABC, angleDtBCD, angleDtCDA;

   // Angular accelerations
   private double angleDt2DAB, angleDt2ABC, angleDt2BCD, angleDt2CDA;

   public YoVariableSideFourbarCalculatorWithDerivatives(String namePrefix, YoVariableRegistry registry)
   {
      this.a = new DoubleYoVariable(namePrefix + "_lenghtA", registry);
      this.b = new DoubleYoVariable(namePrefix + "_lenghtB", registry);
      this.c = new DoubleYoVariable(namePrefix + "_lenghtC", registry);
      this.d = new DoubleYoVariable(namePrefix + "_lenghtD", registry);

      this.minA = new DoubleYoVariable(namePrefix + "_minLengthA", registry);
      this.maxA = new DoubleYoVariable(namePrefix + "_maxLengthA", registry);
   }

   public void setSideLengths(double lengthA, double lengthB, double lengthC, double lengthD)
   {
      this.a.set(lengthA);
      this.b.set(lengthB);
      this.c.set(lengthC);
      this.d.set(lengthD);

      double eMax = min(lengthA + lengthB, lengthD + lengthC);
      if (eMax == lengthA + lengthB)
         maxA.set(PI);
      else
         maxA.set(getAngleWithCosineLaw(lengthA, lengthB, eMax));

      double fMax = min(lengthA + lengthD, lengthB + lengthC);
      if (fMax == lengthA + lengthD)
         minA.set(getAngleWithCosineLaw(fMax, lengthB, lengthC));
      else
         minA.set(getAngleWithCosineLaw(fMax, lengthA, lengthD));
   }

   private static double getAngleWithCosineLaw(double l_neighbour1, double l_neighbour2, double l_opposite)
   {
      double angle = Math.acos(getCosineAngleWithCosineLaw(l_neighbour1, l_neighbour2, l_opposite));

      return angle;
   }

   private static double getCosineAngleWithCosineLaw(double l_neighbour1, double l_neighbour2, double l_opposite)
   {
      checkIfInRange(l_neighbour1, 0.0, Double.POSITIVE_INFINITY);
      checkIfInRange(l_neighbour2, 0.0, Double.POSITIVE_INFINITY);
      checkIfInRange(l_opposite, 0.0, Double.POSITIVE_INFINITY);

      double cosAngle = MathTools.clipToMinMax((square(l_neighbour1) + square(l_neighbour2) - square(l_opposite)) / (2.0 * l_neighbour1 * l_neighbour2), -1.0,
                                               1.0);

      return cosAngle;
   }

   public boolean updateAnglesGivenAngleDAB(double angleDABInRadians)
   {
      // Solve angles
      double A = clipToMinMax(angleDABInRadians, minA.getDoubleValue(), maxA.getDoubleValue());
      double e = getUnknownTriangleSideLengthByLawOfCosine(a.getDoubleValue(), b.getDoubleValue(), A);
      double C = getAngleWithCosineLaw(c.getDoubleValue(), d.getDoubleValue(), e);
      double angleDBA = getAngleWithCosineLaw(b.getDoubleValue(), e, a.getDoubleValue());
      double angleDBC = getAngleWithCosineLaw(c.getDoubleValue(), e, d.getDoubleValue());
      double B = angleDBA + angleDBC;
      double D = 2 * PI - A - B - C;
      this.angleDAB = A;
      this.angleABC = B;
      this.angleBCD = C;
      this.angleCDA = D;

      return !MathTools.isInsideBoundsInclusive(angleDABInRadians, minA.getDoubleValue(), maxA.getDoubleValue());
   }

   public void computeMasterJointAngleGivenAngleABC(double angleABCInRadians)
   {
      double B = angleABCInRadians;
      double f = getUnknownTriangleSideLengthByLawOfCosine(b.getDoubleValue(), c.getDoubleValue(), B);
      double D = getAngleWithCosineLaw(d.getDoubleValue(), a.getDoubleValue(), f);
      double angleACB = getAngleWithCosineLaw(c.getDoubleValue(), f, b.getDoubleValue());
      double angleACD = getAngleWithCosineLaw(d.getDoubleValue(), f, a.getDoubleValue());
      double C = angleACB + angleACD;
      double A = 2 * PI - D - B - C;
      this.angleDAB = A;
   }

   public void computeMasterJointAngleGivenAngleBCD(double angleBCDInRadians)
   {
      double C = angleBCDInRadians;
      double e = getUnknownTriangleSideLengthByLawOfCosine(c.getDoubleValue(), d.getDoubleValue(), C);
      double A = getAngleWithCosineLaw(a.getDoubleValue(), b.getDoubleValue(), e);
      this.angleDAB = A;
   }

   /**
    * Takes in angle D and computes the value of the master joint angle.
    * Same notation used in the rest of the class.
    */
   public void computeMasterJointAngleGivenAngleCDA(double angleCDAInRadians)
   {
      double D = angleCDAInRadians;
      double f = getUnknownTriangleSideLengthByLawOfCosine(d.getDoubleValue(), a.getDoubleValue(), D);
      double angleCAD = getAngleWithCosineLaw(a.getDoubleValue(), f, d.getDoubleValue());
      double angleCAB = getAngleWithCosineLaw(b.getDoubleValue(), f, c.getDoubleValue());
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
      double A = clipToMinMax(angleDABInRadians, minA.getDoubleValue(), maxA.getDoubleValue());
      double dAdT = angularVelocityDAB;
      double e = getUnknownTriangleSideLengthByLawOfCosine(a.getDoubleValue(), b.getDoubleValue(), A);
      double eDot = a.getDoubleValue() * b.getDoubleValue() * sin(A) * dAdT / e;
      double dCdT = FourbarCalculatorTools.getAngleDotWithCosineLaw(c.getDoubleValue(), d.getDoubleValue(), 0.0, e, eDot);
      double angleDotDBA = FourbarCalculatorTools.getAngleDotWithCosineLaw(b.getDoubleValue(), e, eDot, a.getDoubleValue(), 0.0);
      double angleDotDBC = FourbarCalculatorTools.getAngleDotWithCosineLaw(c.getDoubleValue(), e, eDot, d.getDoubleValue(), 0.0);
      double dBdT = angleDotDBA + angleDotDBC;
      double dDdT = -dAdT - dBdT - dCdT;
      this.angleDtDAB = dAdT;
      this.angleDtABC = dBdT;
      this.angleDtBCD = dCdT;
      this.angleDtCDA = dDdT;

      return isAHittingBounds;
   }

   public boolean updateAnglesVelocitiesAndAccelerationsGivenAngleDAB(double angleDABInRadians, double angularVelocityDAB, double angularAccelerationDAB)
   {
      // Solve angles and angular velocity
      boolean isAHittingBounds = updateAnglesAndVelocitiesGivenAngleDAB(angleDABInRadians, angularVelocityDAB);

      // Solve angular acceleration
      double A = clipToMinMax(angleDABInRadians, minA.getDoubleValue(), maxA.getDoubleValue());
      double dAdT = angularVelocityDAB;
      double dAdT2 = angularAccelerationDAB;
      double e = getUnknownTriangleSideLengthByLawOfCosine(a.getDoubleValue(), b.getDoubleValue(), A);
      double eDot = a.getDoubleValue() * b.getDoubleValue() * sin(A) * dAdT / e;
      double eDDot = a.getDoubleValue() * b.getDoubleValue() / e * (cos(A) * dAdT * dAdT + sin(A) * (dAdT2 - eDot * dAdT / e));
      double dCdT2 = FourbarCalculatorTools.getAngleDDotWithCosineLaw(c.getDoubleValue(), d.getDoubleValue(), 0.0, 0.0, e, eDot, eDDot);
      double angleDDotDBA = FourbarCalculatorTools.getAngleDDotWithCosineLaw(b.getDoubleValue(), e, eDot, eDDot, a.getDoubleValue(), 0.0, 0.0);
      double angleDDotDBC = FourbarCalculatorTools.getAngleDDotWithCosineLaw(c.getDoubleValue(), e, eDot, eDDot, d.getDoubleValue(), 0.0, 0.0);
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
      return minA.getDoubleValue();
   }

   public double getMaxDAB()
   {
      return maxA.getDoubleValue();
   }

   public double getAB()
   {
      return a.getDoubleValue();
   }

   public double getBC()
   {
      return b.getDoubleValue();
   }

   public double getCD()
   {
      return c.getDoubleValue();
   }

   public double getDA()
   {
      return d.getDoubleValue();
   }
}
