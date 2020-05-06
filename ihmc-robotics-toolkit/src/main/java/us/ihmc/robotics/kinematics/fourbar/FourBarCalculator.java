package us.ihmc.robotics.kinematics.fourbar;

import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.unknownTriangleSideLengthByLawOfCosine;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Bound;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;

public class FourBarCalculator implements FourbarCalculatorWithDerivatives
{
   /*
    * @formatter:off
    *  Representation of the four bar with name correspondences:
    *        DA
    *    D--------A
    *    |\      /|
    *    | \DB  / |
    *    |  \  /  |
    * CD |   \/   | AB
    *    |   /\   |
    *    |  /  \  |
    *    | /AC  \ |
    *    |/      \|
    *    C--------B
    *        BC
    * Angle name convention:
    * - Inner angle at vertex A: DAB
    * - Inner angle at vertex B: ABC
    * - Inner angle at vertex C: BCD
    * - Inner angle at vertex D: CDA
    * @formatter:on
    */
   /**
    * Side length (input):
    * 
    * <pre>
    *        DA
    *    D--------A
    *    |\      /|
    *    | \DB  / |
    *    |  \  /  |
    * CD |   \/   | AB
    *    |   /\   |
    *    |  /  \  |
    *    | /AC  \ |
    *    |/      \|
    *    C--------B
    *        BC
    * </pre>
    */
   private double AB, BC, CD, DA;
   /**
    * Diagonal length (output):
    * 
    * <pre>
    *        DA
    *    D--------A
    *    |\      /|
    *    | \DB  / |
    *    |  \  /  |
    * CD |   \/   | AB
    *    |   /\   |
    *    |  /  \  |
    *    | /AC  \ |
    *    |/      \|
    *    C--------B
    *        BC
    * </pre>
    */
   private double AC, BD;

   /**
    * Rate of change of the diagonal length (output).
    * 
    * @see AC, BD
    */
   private double ACDt, BDDt;

   private double minDAB, maxDAB;
   private double minABC, maxABC;
   private double minBCD, maxBCD;
   private double minCDA, maxCDA;

   private boolean invertedAtDAB, invertedAtABC, invertedAtBCD, invertedAtCDA;

   // Angles
   private double angleDAB, angleABC, angleBCD, angleCDA;

   // Angular velocities
   private double angleDtDAB, angleDtABC, angleDtBCD, angleDtCDA;

   // Angular accelerations
   private double angleDt2DAB, angleDt2ABC, angleDt2BCD, angleDt2CDA;

   public enum Angle
   {
      ABC, BCD, CDA, DAB
   };

   public FourBarCalculator()
   {
   }

   public void setSideLengths(double AB, double BC, double CD, double DA)
   {
      this.AB = Math.abs(AB);
      this.BC = Math.abs(BC);
      this.CD = Math.abs(CD);
      this.DA = Math.abs(DA);

      clearState();
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

      clearState();
   }

   private void clearState()
   {
      AC = Double.NaN;
      BD = Double.NaN;
      minDAB = Double.NaN;
      maxDAB = Double.NaN;
      minABC = Double.NaN;
      maxABC = Double.NaN;
      minBCD = Double.NaN;
      maxBCD = Double.NaN;
      minCDA = Double.NaN;
      maxCDA = Double.NaN;

      setVelocityToZero();
      setAccelerationToZero();
   }

   public void setToMin(Angle source)
   {
      switch (source)
      {
         case ABC:
         case CDA:
            angleABC = getMinABC();
            angleBCD = getMaxBCD();
            angleCDA = getMinCDA();
            angleDAB = getMaxDAB();
            break;
         case BCD:
         case DAB:
            angleABC = getMaxABC();
            angleBCD = getMinBCD();
            angleCDA = getMaxCDA();
            angleDAB = getMinDAB();
            break;
      }

      setVelocityToZero();
      setAccelerationToZero();
   }

   public void setToMax(Angle source)
   {
      switch (source)
      {
         case ABC:
         case CDA:
            angleABC = getMaxABC();
            angleBCD = getMinBCD();
            angleCDA = getMaxCDA();
            angleDAB = getMinDAB();
            break;
         case BCD:
         case DAB:
            angleABC = getMinABC();
            angleBCD = getMaxBCD();
            angleCDA = getMinCDA();
            angleDAB = getMaxDAB();
            break;
      }

      setVelocityToZero();
      setAccelerationToZero();
   }

   public void setVelocityToZero()
   {
      angleDtABC = 0.0;
      angleDtBCD = 0.0;
      angleDtCDA = 0.0;
      angleDtDAB = 0.0;
   }

   public void setAccelerationToZero()
   {
      angleDt2ABC = 0.0;
      angleDt2BCD = 0.0;
      angleDt2CDA = 0.0;
      angleDt2DAB = 0.0;
   }

   /**
    * <p>
    * Updates internally: all the angles and diagonal lengths.
    * </p>
    * 
    * @param source
    * @param angle
    * @return
    */
   public Bound update(Angle source, double angle)
   {
      if (source == Angle.ABC || source == Angle.CDA)
      {
         if (source == Angle.ABC)
         {
            if (angle <= getMinABC())
            {
               setToMin(source);
               return Bound.MIN;
            }
            else if (angle > getMaxABC())
            {
               setToMax(source);
               return Bound.MAX;
            }
            else
            {
               angleABC = MathTools.clamp(angle, getMinABC(), getMaxABC());
               AC = EuclidGeometryTools.unknownTriangleSideLengthByLawOfCosine(AB, BC, angleABC);
               angleCDA = FourbarCalculatorTools.angleWithCosineLaw(CD, DA, AC);
            }
         }
         else
         {
            if (angle <= getMinCDA())
            {
               setToMin(source);
               return Bound.MIN;
            }
            else if (angle > getMaxCDA())
            {
               setToMax(source);
               return Bound.MAX;
            }
            else
            {
               angleCDA = MathTools.clamp(angle, getMinCDA(), getMaxCDA());
               AC = EuclidGeometryTools.unknownTriangleSideLengthByLawOfCosine(CD, DA, angleCDA);
               angleABC = FourbarCalculatorTools.angleWithCosineLaw(AB, BC, AC);
            }
         }
         double angleCAB = FourbarCalculatorTools.angleWithCosineLaw(AB, AC, BC);
         double angleDAC = FourbarCalculatorTools.angleWithCosineLaw(DA, AC, CD);
         angleDAB = angleDAC + angleCAB;
         angleBCD = 2.0 * Math.PI - angleABC - angleCDA - angleDAB;
      }
      else
      {
         if (source == Angle.BCD)
         {
            if (angle <= getMinBCD())
            {
               setToMin(source);
               return Bound.MIN;
            }
            else if (angle > getMaxBCD())
            {
               setToMax(source);
               return Bound.MAX;
            }
            else
            {
               angleBCD = MathTools.clamp(angle, getMinBCD(), getMaxBCD());
               BD = unknownTriangleSideLengthByLawOfCosine(BC, CD, angleBCD);
               angleDAB = FourbarCalculatorTools.angleWithCosineLaw(DA, AB, BD);
            }
         }
         else
         {
            if (angle <= getMinDAB())
            {
               setToMin(source);
               return Bound.MIN;
            }
            else if (angle > getMaxDAB())
            {
               setToMax(source);
               return Bound.MAX;
            }
            else
            {
               angleDAB = MathTools.clamp(angle, getMinDAB(), getMaxDAB());
               BD = unknownTriangleSideLengthByLawOfCosine(DA, AB, angleDAB);
               angleBCD = FourbarCalculatorTools.angleWithCosineLaw(BC, CD, BD);
            }
         }
         double angleDBC = FourbarCalculatorTools.angleWithCosineLaw(BC, BD, CD);
         double angleABD = FourbarCalculatorTools.angleWithCosineLaw(AB, BD, DA);
         angleABC = angleABD + angleDBC;
         angleCDA = 2.0 * Math.PI - angleBCD - angleDAB - angleABC;
      }
      return null;
   }

   public Bound update(Angle source, double angle, double angleDot)
   {
      Bound limit = update(source, angle);

      if (limit == Bound.MIN)
      {
         if (angleDot <= 0.0)
         {
            setVelocityToZero();
            return limit;
         }
      }
      else if (limit == Bound.MAX)
      {
         if (angleDot >= 0.0)
         {
            setVelocityToZero();
            return limit;
         }
      }

      if (source == Angle.ABC || source == Angle.CDA)
      {
         if (source == Angle.ABC)
         {
            angleDtABC = angleDot;
            ACDt = AB * BC * Math.sin(angleABC) * angleDtABC / AC;
            angleDtCDA = FourbarCalculatorTools.angleDotWithCosineLaw(CD, DA, 0.0, AC, ACDt);
         }
         else
         {
            angleDtCDA = angleDot;
            ACDt = AC * CD * Math.sin(angleCDA) * angleDtCDA / AC;
            angleDtABC = FourbarCalculatorTools.angleDotWithCosineLaw(AB, BC, 0.0, AC, ACDt);
         }

         double angleDtCAB = FourbarCalculatorTools.angleDotWithCosineLaw(AB, AC, ACDt, BC, 0.0);
         double angleDtDAC = FourbarCalculatorTools.angleDotWithCosineLaw(DA, AC, ACDt, CD, 0.0);
         angleDtDAB = angleDtDAC + angleDtCAB;
         angleDt2BCD = -angleDtABC - angleDtCDA - angleDtDAB;
      }
      else
      {
         if (source == Angle.CDA)
         {
            angleDtBCD = angleDot;
            BDDt = BC * CD * Math.sin(angleBCD) * angleDtBCD / BD;
            angleDtDAB = FourbarCalculatorTools.angleDotWithCosineLaw(DA, AB, 0.0, BD, BDDt);
         }
         else
         {
            angleDtDAB = angleDot;
            BDDt = DA * AB * Math.sin(angleDAB) * angleDtDAB / BD;
            angleDtBCD = FourbarCalculatorTools.angleDotWithCosineLaw(BC, CD, 0.0, BD, BDDt);
         }

         double angleDtDBA = FourbarCalculatorTools.angleDotWithCosineLaw(AB, BD, BDDt, DA, 0.0);
         double angleDtDBC = FourbarCalculatorTools.angleDotWithCosineLaw(BC, BD, BDDt, CD, 0.0);
         angleDtABC = angleDtDBA + angleDtDBC;
         angleDtCDA = -angleDtDAB - angleDtABC - angleDtBCD;
      }

      return null;
   }

   public Bound update(Angle source, double angle, double angleDot, double angleDDot)
   {
      Bound limit = update(source, angle, angleDot);

      if (limit == Bound.MIN)
      {
         if (angleDot <= 0.0)
         {
            setAccelerationToZero();
            return limit;
         }
      }
      else if (limit == Bound.MAX)
      {
         if (angleDot >= 0.0)
         {
            setAccelerationToZero();
            return limit;
         }
      }

      if (source == Angle.ABC || source == Angle.CDA)
      {
         double ACDt2;

         if (source == Angle.ABC)
         {
            angleDt2ABC = angleDDot;
            ACDt2 = AB * BC / AC * (Math.cos(angleABC) * angleDtABC * angleDtABC + Math.sin(angleABC) * (angleDt2ABC - ACDt * angleDtABC / AC));
            angleDt2CDA = FourbarCalculatorTools.angleDDotWithCosineLaw(CD, DA, 0.0, 0.0, AC, ACDt, ACDt2);
         }
         else
         {
            angleDt2CDA = angleDDot;
            ACDt2 = CD * DA / AC * (Math.cos(angleCDA) * angleDtCDA * angleDtCDA + Math.sin(angleCDA) * (angleDt2CDA - ACDt * angleDtCDA / AC));
            angleDt2ABC = FourbarCalculatorTools.angleDDotWithCosineLaw(CD, DA, 0.0, 0.0, AC, ACDt, ACDt2);
         }

         double angleDt2CAB = FourbarCalculatorTools.angleDDotWithCosineLaw(AB, AC, ACDt, ACDt2, BC, 0.0, 0.0);
         double angleDt2DAC = FourbarCalculatorTools.angleDDotWithCosineLaw(DA, AC, ACDt, ACDt2, CD, 0.0, 0.0);
         angleDt2DAB = angleDt2DAC + angleDt2CAB;
         angleDt2BCD = -angleDt2ABC - angleDt2CDA - angleDt2DAB;
      }
      else
      {
         double BDDt2;

         if (source == Angle.CDA)
         {
            angleDt2BCD = angleDDot;
            BDDt2 = BC * CD / BD * (Math.cos(angleBCD) * angleDtBCD * angleDtBCD + Math.sin(angleBCD) * (angleDt2BCD - BDDt * angleDtBCD / BD));
            angleDt2DAB = FourbarCalculatorTools.angleDDotWithCosineLaw(DA, AB, 0.0, 0.0, BD, BDDt, BDDt2);
         }
         else
         {
            angleDt2DAB = angleDDot;
            BDDt2 = DA * AB / BD * (Math.cos(angleDAB) * angleDtDAB * angleDtDAB + Math.sin(angleDAB) * (angleDt2DAB - BDDt * angleDtDAB / BD));
            angleDt2BCD = FourbarCalculatorTools.angleDDotWithCosineLaw(BC, CD, 0.0, 0.0, BD, BDDt, BDDt2);
         }

         double angleDt2DBA = FourbarCalculatorTools.angleDDotWithCosineLaw(AB, BD, BDDt, BDDt2, DA, 0.0, 0.0);
         double angleDt2DBC = FourbarCalculatorTools.angleDDotWithCosineLaw(BC, BD, BDDt, BDDt2, CD, 0.0, 0.0);
         angleDt2ABC = angleDt2DBA + angleDt2DBC;
         angleDt2CDA = -angleDt2DAB - angleDt2ABC - angleDt2BCD;
      }

      return null;
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
      return update(Angle.DAB, angleDABInRadians) != null;
   }

   /**
    * Takes in angle B and computes the value of the master joint angle. Same notation used in the rest
    * of the class.
    */
   @Override
   public void computeMasterJointAngleGivenAngleABC(double angleABCInRadians)
   {
      update(Angle.ABC, angleABCInRadians);
   }

   /**
    * Takes in angle C and computes the value of the master joint angle. Same notation used in the rest
    * of the class.
    */
   @Override
   public void computeMasterJointAngleGivenAngleBCD(double angleBCDInRadians)
   {
      update(Angle.BCD, angleBCDInRadians);
   }

   /**
    * Takes in angle D and computes the value of the master joint angle. Same notation used in the rest
    * of the class.
    */
   @Override
   public void computeMasterJointAngleGivenAngleCDA(double angleCDAInRadians)
   {
      update(Angle.CDA, angleCDAInRadians);
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
      return update(Angle.DAB, angleDABInRadians, angularVelocityDAB) != null;
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
      return update(Angle.DAB, angleDABInRadians, angularVelocityDAB, angularAccelerationDAB) != null;
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
      if (Double.isNaN(minDAB))
      {
         double ACMax = Math.min(CD + DA, AB + BC);

         if (ACMax == DA + CD)
            minDAB = FourbarCalculatorTools.angleWithCosineLaw(ACMax, AB, BC);
         else
            minDAB = FourbarCalculatorTools.angleWithCosineLaw(ACMax, DA, CD);
      }
      return minDAB;
   }

   @Override
   public double getMaxDAB()
   {
      if (Double.isNaN(maxDAB))
      {
         double DBMax = Math.min(DA + AB, BC + CD);

         if (DBMax == DA + AB)
            maxDAB = Math.PI;
         else
            maxDAB = FourbarCalculatorTools.angleWithCosineLaw(DA, AB, DBMax);
      }
      return maxDAB;
   }

   public double getMinABC()
   {
      if (Double.isNaN(minABC))
      {
         double DBMax = Math.min(DA + AB, BC + CD);

         if (DBMax == DA + AB)
            minABC = FourbarCalculatorTools.angleWithCosineLaw(DBMax, BC, CD);
         else
            minABC = FourbarCalculatorTools.angleWithCosineLaw(DBMax, AB, DA);
      }
      return minABC;
   }

   public double getMaxABC()
   {
      if (Double.isNaN(maxABC))
      {
         double ACMax = Math.min(CD + DA, AB + BC);

         if (ACMax == AB + BC)
            maxABC = Math.PI;
         else
            maxABC = FourbarCalculatorTools.angleWithCosineLaw(AB, BC, ACMax);
      }
      return maxABC;
   }

   public double getMinBCD()
   {
      if (Double.isNaN(minBCD))
      {
         double ACMax = Math.min(CD + DA, AB + BC);

         if (ACMax == DA + CD)
            minBCD = FourbarCalculatorTools.angleWithCosineLaw(ACMax, BC, AB);
         else
            minBCD = FourbarCalculatorTools.angleWithCosineLaw(ACMax, CD, DA);
      }
      return minBCD;
   }

   public double getMaxBCD()
   {
      if (Double.isNaN(maxBCD))
      {
         double DBMax = Math.min(DA + AB, BC + CD);

         if (DBMax == BC + CD)
            maxBCD = Math.PI;
         else
            maxBCD = FourbarCalculatorTools.angleWithCosineLaw(BC, CD, DBMax);
      }
      return maxBCD;
   }

   public double getMinCDA()
   {
      if (Double.isNaN(minCDA))
      {
         double DBMax = Math.min(DA + AB, BC + CD);

         if (DBMax == DA + AB)
            minCDA = FourbarCalculatorTools.angleWithCosineLaw(DBMax, CD, BC);
         else
            minCDA = FourbarCalculatorTools.angleWithCosineLaw(DBMax, DA, AB);
      }
      return minCDA;
   }

   public double getMaxCDA()
   {
      if (Double.isNaN(maxCDA))
      {
         double ACMax = Math.min(CD + DA, AB + BC);

         if (ACMax == CD + DA)
            maxCDA = Math.PI;
         else
            maxCDA = FourbarCalculatorTools.angleWithCosineLaw(CD, DA, ACMax);
      }
      return maxCDA;
   }

   @Override
   public double getAB()
   {
      return AB;
   }

   @Override
   public double getBC()
   {
      return BC;
   }

   @Override
   public double getCD()
   {
      return CD;
   }

   @Override
   public double getDA()
   {
      return DA;
   }
}
