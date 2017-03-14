package us.ihmc.robotics.kinematics.fourbar;

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.min;
import static java.lang.Math.sin;
import static us.ihmc.robotics.MathTools.checkIntervalContains;
import static us.ihmc.robotics.MathTools.clamp;
import static us.ihmc.robotics.MathTools.square;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.unknownTriangleSideLengthByLawOfCosine;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class YoVariableSideFourbarCalculatorWithDerivatives implements FourbarCalculatorWithDerivatives
{
   private final DoubleYoVariable lengthAD, lengthBA, lengthCB, lengthDC;
   private final DoubleYoVariable minA, maxA;

   // Angles
   private double angleDAB, angleABC, angleBCD, angleCDA;

   // Angular velocities
   private double angleDtDAB, angleDtABC, angleDtBCD, angleDtCDA;

   // Angular accelerations
   private double angleDt2DAB, angleDt2ABC, angleDt2BCD, angleDt2CDA;

   public YoVariableSideFourbarCalculatorWithDerivatives(String namePrefix, YoVariableRegistry registry)
   {
      this.lengthAD = new DoubleYoVariable(namePrefix + "_lenghtAD", registry);
      this.lengthBA = new DoubleYoVariable(namePrefix + "_lenghtBA", registry);
      this.lengthCB = new DoubleYoVariable(namePrefix + "_lenghtCB", registry);
      this.lengthDC = new DoubleYoVariable(namePrefix + "_lenghtDC", registry);

      this.minA = new DoubleYoVariable(namePrefix + "_minLengthA", registry);
      this.maxA = new DoubleYoVariable(namePrefix + "_maxLengthA", registry);
   }

   public void setSideLengths(double lengthAD, double lengthBA, double lengthCB, double lengthDC)
   {
      this.lengthAD.set(lengthAD);
      this.lengthBA.set(lengthBA);
      this.lengthCB.set(lengthCB);
      this.lengthDC.set(lengthDC);

      double eMax = min(lengthAD + lengthBA, lengthDC + lengthCB);
      if (eMax == lengthAD + lengthBA)
         maxA.set(PI);
      else
         maxA.set(getAngleWithCosineLaw(lengthAD, lengthBA, eMax));

      double fMax = min(lengthAD + lengthDC, lengthBA + lengthCB);
      if (fMax == lengthAD + lengthDC)
         minA.set(getAngleWithCosineLaw(fMax, lengthBA, lengthCB));
      else
         minA.set(getAngleWithCosineLaw(fMax, lengthAD, lengthDC));
   }

   private static double getAngleWithCosineLaw(double l_neighbour1, double l_neighbour2, double l_opposite)
   {
      double angle = Math.acos(getCosineAngleWithCosineLaw(l_neighbour1, l_neighbour2, l_opposite));

      return angle;
   }

   private static double getCosineAngleWithCosineLaw(double l_neighbour1, double l_neighbour2, double l_opposite)
   {
      checkIntervalContains(l_neighbour1, 0.0, Double.POSITIVE_INFINITY);
      checkIntervalContains(l_neighbour2, 0.0, Double.POSITIVE_INFINITY);
      checkIntervalContains(l_opposite, 0.0, Double.POSITIVE_INFINITY);

      double cosAngle = MathTools.clamp((square(l_neighbour1) + square(l_neighbour2) - square(l_opposite)) / (2.0 * l_neighbour1 * l_neighbour2), -1.0,
                                               1.0);

      return cosAngle;
   }

   public boolean updateAnglesGivenAngleDAB(double angleDABInRadians)
   {
      // Solve angles
      double A = clamp(angleDABInRadians, minA.getDoubleValue(), maxA.getDoubleValue());
      double e = unknownTriangleSideLengthByLawOfCosine(lengthAD.getDoubleValue(), lengthBA.getDoubleValue(), A);
      double C = getAngleWithCosineLaw(lengthCB.getDoubleValue(), lengthDC.getDoubleValue(), e);
      double angleDBA = getAngleWithCosineLaw(lengthBA.getDoubleValue(), e, lengthAD.getDoubleValue());
      double angleDBC = getAngleWithCosineLaw(lengthCB.getDoubleValue(), e, lengthDC.getDoubleValue());
      double B = angleDBA + angleDBC;
      double D = 2 * PI - A - B - C;
      this.angleDAB = A;
      this.angleABC = B;
      this.angleBCD = C;
      this.angleCDA = D;

      return !MathTools.intervalContains(angleDABInRadians, minA.getDoubleValue(), maxA.getDoubleValue());
   }

   public void computeMasterJointAngleGivenAngleABC(double angleABCInRadians)
   {
      double B = angleABCInRadians;
      double f = unknownTriangleSideLengthByLawOfCosine(lengthBA.getDoubleValue(), lengthCB.getDoubleValue(), B);
      double D = getAngleWithCosineLaw(lengthDC.getDoubleValue(), lengthAD.getDoubleValue(), f);
      double angleACB = getAngleWithCosineLaw(lengthCB.getDoubleValue(), f, lengthBA.getDoubleValue());
      double angleACD = getAngleWithCosineLaw(lengthDC.getDoubleValue(), f, lengthAD.getDoubleValue());
      double C = angleACB + angleACD;
      double A = 2 * PI - D - B - C;
      this.angleDAB = A;
   }

   public void computeMasterJointAngleGivenAngleBCD(double angleBCDInRadians)
   {
      double C = angleBCDInRadians;
      double e = unknownTriangleSideLengthByLawOfCosine(lengthCB.getDoubleValue(), lengthDC.getDoubleValue(), C);
      double A = getAngleWithCosineLaw(lengthAD.getDoubleValue(), lengthBA.getDoubleValue(), e);
      this.angleDAB = A;
   }

   /**
    * Takes in angle D and computes the value of the master joint angle.
    * Same notation used in the rest of the class.
    */
   public void computeMasterJointAngleGivenAngleCDA(double angleCDAInRadians)
   {
      double D = angleCDAInRadians;
      double f = unknownTriangleSideLengthByLawOfCosine(lengthDC.getDoubleValue(), lengthAD.getDoubleValue(), D);
      double angleCAD = getAngleWithCosineLaw(lengthAD.getDoubleValue(), f, lengthDC.getDoubleValue());
      double angleCAB = getAngleWithCosineLaw(lengthBA.getDoubleValue(), f, lengthCB.getDoubleValue());
      double A = angleCAD + angleCAB;
      this.angleDAB = A;
   }

   /**
    * Compute every angle of the quadrilateral.
    * @param angleDABInRadians is the angle formed by the sides lengthAD and lengthBA (see scheme in this class)
    * @param angularVelocityDAB first time-derivative of the angle DAB
    * @return true if the angle DAB is out of range making the quadrilateral non-convex
    */
   public boolean updateAnglesAndVelocitiesGivenAngleDAB(double angleDABInRadians, double angularVelocityDAB)
   {
      // Solve angles
      boolean isAHittingBounds = updateAnglesGivenAngleDAB(angleDABInRadians);

      // Solve angular velocity
      double A = clamp(angleDABInRadians, minA.getDoubleValue(), maxA.getDoubleValue());
      double dAdT = angularVelocityDAB;
      double e = unknownTriangleSideLengthByLawOfCosine(lengthAD.getDoubleValue(), lengthBA.getDoubleValue(), A);
      double eDot = lengthAD.getDoubleValue() * lengthBA.getDoubleValue() * sin(A) * dAdT / e;
      double dCdT = FourbarCalculatorTools.getAngleDotWithCosineLaw(lengthCB.getDoubleValue(), lengthDC.getDoubleValue(), 0.0, e, eDot);
      double angleDotDBA = FourbarCalculatorTools.getAngleDotWithCosineLaw(lengthBA.getDoubleValue(), e, eDot, lengthAD.getDoubleValue(), 0.0);
      double angleDotDBC = FourbarCalculatorTools.getAngleDotWithCosineLaw(lengthCB.getDoubleValue(), e, eDot, lengthDC.getDoubleValue(), 0.0);
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
      double A = clamp(angleDABInRadians, minA.getDoubleValue(), maxA.getDoubleValue());
      double dAdT = angularVelocityDAB;
      double dAdT2 = angularAccelerationDAB;
      double e = unknownTriangleSideLengthByLawOfCosine(lengthAD.getDoubleValue(), lengthBA.getDoubleValue(), A);
      double eDot = lengthAD.getDoubleValue() * lengthBA.getDoubleValue() * sin(A) * dAdT / e;
      double eDDot = lengthAD.getDoubleValue() * lengthBA.getDoubleValue() / e * (cos(A) * dAdT * dAdT + sin(A) * (dAdT2 - eDot * dAdT / e));
      double dCdT2 = FourbarCalculatorTools.getAngleDDotWithCosineLaw(lengthCB.getDoubleValue(), lengthDC.getDoubleValue(), 0.0, 0.0, e, eDot, eDDot);
      double angleDDotDBA = FourbarCalculatorTools.getAngleDDotWithCosineLaw(lengthBA.getDoubleValue(), e, eDot, eDDot, lengthAD.getDoubleValue(), 0.0, 0.0);
      double angleDDotDBC = FourbarCalculatorTools.getAngleDDotWithCosineLaw(lengthCB.getDoubleValue(), e, eDot, eDDot, lengthDC.getDoubleValue(), 0.0, 0.0);
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
      return lengthAD.getDoubleValue();
   }

   public double getBC()
   {
      return lengthBA.getDoubleValue();
   }

   public double getCD()
   {
      return lengthCB.getDoubleValue();
   }

   public double getDA()
   {
      return lengthDC.getDoubleValue();
   }
}
