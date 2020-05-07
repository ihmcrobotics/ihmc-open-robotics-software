package us.ihmc.robotics.kinematics.fourbar;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.kinematics.fourbar.FourBar.Angle;

public class FourBarTest
{
   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1e-7;
   private static final boolean PRINT = false;

   @Test
   public void testSquare()
   {
      FourBar fourBar = new FourBar();
      fourBar.setSideLengths(1.0, 1.0, 1.0, 1.0);
      fourBar.update(Angle.DAB, Math.PI / 2);
      assertEquals(Math.PI / 2, fourBar.getAngleDAB(), EPSILON);
      assertEquals(Math.PI / 2, fourBar.getAngleABC(), EPSILON);
      assertEquals(Math.PI / 2, fourBar.getAngleBCD(), EPSILON);
      assertEquals(Math.PI / 2, fourBar.getAngleCDA(), EPSILON);
   }

   @Test
   public void testConstruction()
   {
      Random random = new Random(1738);
      double DA = random.nextDouble();
      double AB = random.nextDouble();
      double BC = random.nextDouble();
      double CD = random.nextDouble();

      FourBar fourBar = new FourBar();
      fourBar.setSideLengths(AB, BC, CD, DA);
      assertEquals(AB, fourBar.getAB(), EPSILON);
      assertEquals(BC, fourBar.getBC(), EPSILON);
      assertEquals(CD, fourBar.getCD(), EPSILON);
      assertEquals(DA, fourBar.getDA(), EPSILON);
   }

   @Test
   public void testSquareDerivatives()
   {
      FourBar fourBar = new FourBar();
      fourBar.setSideLengths(1.0, 1.0, 1.0, 1.0);
      fourBar.update(Angle.DAB, Math.PI / 2, 1.0);
      assertEquals(Math.PI / 2, fourBar.getAngleDAB(), EPSILON);
      assertEquals(Math.PI / 2, fourBar.getAngleABC(), EPSILON);
      assertEquals(Math.PI / 2, fourBar.getAngleBCD(), EPSILON);
      assertEquals(Math.PI / 2, fourBar.getAngleCDA(), EPSILON);
      assertEquals(1, fourBar.getAngleDtDAB(), EPSILON);
      assertEquals(1, fourBar.getAngleDtBCD(), EPSILON);
      assertEquals(-1, fourBar.getAngleDtABC(), EPSILON);
      assertEquals(-1, fourBar.getAngleDtCDA(), EPSILON);
   }

   @Test
   public void testParallelogram()
   {
      FourBar fourBar = new FourBar();
      fourBar.setSideLengths(1.0, 1.0, 1.0, 1.0);
      fourBar.update(Angle.DAB, Math.PI / 3, 1.0);
      assertEquals(Math.PI / 3, fourBar.getAngleDAB(), EPSILON);
      assertEquals(2 * Math.PI / 3, fourBar.getAngleABC(), EPSILON);
      assertEquals(Math.PI / 3, fourBar.getAngleBCD(), EPSILON);
      assertEquals(2 * Math.PI / 3, fourBar.getAngleCDA(), EPSILON);
      assertEquals(1, fourBar.getAngleDtDAB(), EPSILON);
      assertEquals(1, fourBar.getAngleDtBCD(), EPSILON);
      assertEquals(-1, fourBar.getAngleDtABC(), EPSILON);
      assertEquals(-1, fourBar.getAngleDtCDA(), EPSILON);
   }

   @Test
   public void testRandomQuadrilateral()
   {
      Random rand = new Random(1986L);
      for (int i = 0; i < 10000; i++)
      {
         double e = 100 * (rand.nextDouble() + 0.001);
         double k1 = rand.nextDouble();
         double k2 = rand.nextDouble();
         double d1 = e * Math.abs(rand.nextGaussian());
         double d2 = e * Math.abs(rand.nextGaussian());
         double DE = e * k1, DF = e * k2, BE = e * (1 - k1), BF = e * (1 - k2);
         double AE = d1, CF = d2;
         double AD = Math.sqrt(DE * DE + AE * AE), DAE = Math.atan2(DE, AE), ADE = Math.atan2(AE, DE);
         double AB = Math.sqrt(AE * AE + BE * BE), BAE = Math.atan2(BE, AE), ABE = Math.atan2(AE, BE);
         double CD = Math.sqrt(CF * CF + DF * DF), CDF = Math.atan2(CF, DF), DCF = Math.atan2(DF, CF);
         double BC = Math.sqrt(BF * BF + CF * CF), CBF = Math.atan2(CF, BF), BCF = Math.atan2(BF, CF);
         double BAD = DAE + BAE, ABC = ABE + CBF, BCD = BCF + DCF, ADC = ADE + CDF;

         solveFourBar(AD, AB, BC, CD, BAD, ABC, BCD, ADC);
      }
   }

   private void solveFourBar(double a, double b, double c, double d, double A, double B, double C, double D)
   {
      if (PRINT)
      {
         String[] names = "a,b,c,d,A,B,C,D".split(",");
         double[] vals = new double[] {a, b, c, d, A, B, C, D};
         for (int i = 0; i < vals.length; i++)
         {
            System.out.println(names[i] + " = " + vals[i]);
         }
      }
      FourBar fourBar = new FourBar();
      fourBar.setSideLengths(b, c, d, a);
      fourBar.update(Angle.DAB, A, 0.0);
      assertEquals(A, fourBar.getAngleDAB(), EPSILON);
      assertEquals(B, fourBar.getAngleABC(), EPSILON);
      assertEquals(C, fourBar.getAngleBCD(), EPSILON);
      assertEquals(D, fourBar.getAngleCDA(), EPSILON);
   }

   @Test
   public void testRandomConvenientLinkageDerivatives()
   {
      Random rand = new Random(1986L);
      for (int i = 0; i < 10000; i++)
      {
         double s = 100 * (rand.nextDouble() + 0.001);
         double l = s * (rand.nextDouble() + 0.001);
         double k = s * (5 * rand.nextDouble() + 0.1);

         double AB = l;
         double BC = k;
         double CD = Math.sqrt(l * l + (k - s) * (k - s));
         double AD = s;
         double BAD = Math.PI / 2;
         double ABC = Math.PI / 2;
         double BCD = Math.atan2(l, k - s);
         double ADC = Math.PI - BCD;
         double dBAD = 1;
         double dABC = -dBAD * s / k;
         double dADC = -dBAD;
         double dBCD = -dABC;

         FourBar fourBar = new FourBar();
         fourBar.setSideLengths(AB, BC, CD, AD);
         fourBar.update(Angle.DAB, BAD, dBAD);
         assertEquals(BAD, fourBar.getAngleDAB(), EPSILON);
         assertEquals(ABC, fourBar.getAngleABC(), EPSILON);
         assertEquals(BCD, fourBar.getAngleBCD(), EPSILON);
         assertEquals(ADC, fourBar.getAngleCDA(), EPSILON);
         assertEquals(0, dBAD + dABC + dBCD + dADC, EPSILON);
         assertEquals(0, fourBar.getAngleDtDAB() + fourBar.getAngleDtABC() + fourBar.getAngleDtBCD() + fourBar.getAngleDtCDA(), EPSILON);
         assertEquals(dBAD, fourBar.getAngleDtDAB(), EPSILON);
         assertEquals(dABC, fourBar.getAngleDtABC(), EPSILON);
         assertEquals(dBCD, fourBar.getAngleDtBCD(), EPSILON);
         assertEquals(dADC, fourBar.getAngleDtCDA(), EPSILON);

      }
   }

   @Test
   public void testVelocitiesWithRandomQuadrilateral()
   {
      double eps = 1.0e-6;
      Random rand = new Random(1976L);
      double DA, AB, BC, CD;
      FourBar fourBar;
      boolean isQuadrilateralOK;
      int k;

      double DAB_t0 = 0.0, DAB_tf = 0.0, DAB_tj = 0.0;
      double dDAB = 0.0;

      double ABC_t0 = 0.0, CDA_t0 = 0.0, BCD_t0 = 0.0;
      double ABC_next_tj = 0.0, CDA_next_tj = 0.0, BCD_next_tj = 0.0;
      double ABC_numerical_tf = 0.0, CDA_numerical_tf = 0.0, BCD_numerical_tf = 0.0;
      double ABC_fourbar_tf = 0.0, CDA_fourbar_tf = 0.0, BCD_fourbar_tf = 0.0;

      int nSteps = 20000;
      double T = 0.001, deltaT = T / (nSteps - 1.0);
      boolean isDABOutOfRange = true;
      boolean DAB_t0_outOfRange, DAB_tf_outOfRange, goToNextIteration;

      for (int i = 0; i < 100000; i++)
      {
         AB = RandomNumbers.nextDouble(rand, 0.1, 2.0);
         BC = RandomNumbers.nextDouble(rand, 0.1, 2.0);
         CD = RandomNumbers.nextDouble(rand, 0.1, 2.0);
         DA = RandomNumbers.nextDouble(rand, 0.1, 2.0);
         fourBar = new FourBar();
         fourBar.setSideLengths(AB, BC, CD, DA);
         isQuadrilateralOK = fourBar.getMaxDAB() - fourBar.getMinDAB() > 1.0e-5;

         k = 0;

         while (!isQuadrilateralOK)
         {
            AB = RandomNumbers.nextDouble(rand, 0.1, 2.0);
            BC = RandomNumbers.nextDouble(rand, 0.1, 2.0);
            CD = RandomNumbers.nextDouble(rand, 0.1, 2.0);
            DA = RandomNumbers.nextDouble(rand, 0.1, 2.0);
            fourBar = new FourBar();
            fourBar.setSideLengths(AB, BC, CD, DA);
            isQuadrilateralOK = fourBar.getMaxDAB() - fourBar.getMinDAB() > 1.0e-5;

            k++;
            if (k > 10)
               throw new RuntimeException("Could not make a convex quadrilateral");
         }

         goToNextIteration = false;
         k = 0;

         while (isDABOutOfRange)
         {
            dDAB = RandomNumbers.nextDouble(rand, -30.0, 30.0);
            DAB_t0 = RandomNumbers.nextDouble(rand, 0.1, Math.PI - 0.1);
            DAB_tf = DAB_t0 + dDAB * T;

            DAB_t0_outOfRange = fourBar.update(Angle.DAB, DAB_t0) != null;
            ABC_t0 = fourBar.getAngleABC();
            CDA_t0 = fourBar.getAngleCDA();
            BCD_t0 = fourBar.getAngleBCD();

            DAB_tf_outOfRange = fourBar.update(Angle.DAB, DAB_tf) != null;
            ABC_fourbar_tf = fourBar.getAngleABC();
            CDA_fourbar_tf = fourBar.getAngleCDA();
            BCD_fourbar_tf = fourBar.getAngleBCD();

            k++;

            if (k > 10)
            {
               goToNextIteration = true;

               break;
            }

            isDABOutOfRange = DAB_t0_outOfRange || DAB_tf_outOfRange;

            if (isDABOutOfRange)
               continue;

            ABC_next_tj = ABC_t0;
            CDA_next_tj = CDA_t0;
            BCD_next_tj = BCD_t0;

            DAB_tj = DAB_t0;

            for (int j = 0; j < nSteps - 1; j++)
            {
               DAB_tj = DAB_t0 + dDAB * j * deltaT;
               fourBar.update(Angle.DAB, DAB_tj, dDAB);

               ABC_next_tj += fourBar.getAngleDtABC() * deltaT;
               CDA_next_tj += fourBar.getAngleDtCDA() * deltaT;
               BCD_next_tj += fourBar.getAngleDtBCD() * deltaT;
            }
         }

         if (goToNextIteration)
            break;

         ABC_numerical_tf = ABC_next_tj;
         CDA_numerical_tf = CDA_next_tj;
         BCD_numerical_tf = BCD_next_tj;

         assertEquals(ABC_numerical_tf, ABC_fourbar_tf, eps);
         assertEquals(CDA_numerical_tf, CDA_fourbar_tf, eps);
         assertEquals(BCD_numerical_tf, BCD_fourbar_tf, eps);
      }
   }

   @Test
   public void testAccelerationsWithRandomQuadrilateral()
   {
      double eps = 1.0e-5;
      Random random = new Random(1984L);
      double AD, BA, CB, DC;
      FourBar fourBar;
      boolean isQuadrilateralOK;
      int k;

      double DAB_t0 = 0.0, DAB_tf = 0.0, DAB_tj = 0.0;
      double dDAB_t0 = 0.0, dDAB_tf = 0.0, dDAB_tj = 0.0;
      double ddDAB = 0.0;

      double ABC_t0 = 0.0, CDA_t0 = 0.0, BCD_t0 = 0.0;
      double ABC_next_tj = 0.0, CDA_next_tj = 0.0, BCD_next_tj = 0.0;
      double ABC_numerical_tf = 0.0, CDA_numerical_tf = 0.0, BCD_numerical_tf = 0.0;
      double ABC_fourbar_tf = 0.0, CDA_fourbar_tf = 0.0, BCD_fourbar_tf = 0.0;

      int nSteps = 10000;
      double T = 0.001, deltaT = T / (nSteps - 1.0);

      for (int i = 0; i < 50; i++)
      {
         BA = RandomNumbers.nextDouble(random, 0.1, 2.0);
         CB = RandomNumbers.nextDouble(random, 0.1, 2.0);
         DC = RandomNumbers.nextDouble(random, 0.1, 2.0);
         AD = RandomNumbers.nextDouble(random, 0.1, 2.0);
         fourBar = new FourBar();
         fourBar.setSideLengths(BA, CB, DC, AD);
         isQuadrilateralOK = fourBar.getMaxDAB() - fourBar.getMinDAB() > 1.0e-5;

         k = 0;

         while (!isQuadrilateralOK)
         {
            BA = RandomNumbers.nextDouble(random, 0.1, 2.0);
            CB = RandomNumbers.nextDouble(random, 0.1, 2.0);
            DC = RandomNumbers.nextDouble(random, 0.1, 2.0);
            AD = RandomNumbers.nextDouble(random, 0.1, 2.0);
            fourBar = new FourBar();
            fourBar.setSideLengths(BA, CB, DC, AD);
            isQuadrilateralOK = fourBar.getMaxDAB() - fourBar.getMinDAB() > 1.0e-5;

            k++;
            if (k > 10)
               throw new RuntimeException("Could not make a convex quadrilateral");
         }

         DAB_t0 = RandomNumbers.nextDouble(random, fourBar.getMinDAB(), fourBar.getMaxDAB());
         DAB_tf = RandomNumbers.nextDouble(random, fourBar.getMinDAB(), fourBar.getMaxDAB());
         dDAB_t0 = 0.0;
         ddDAB = 2.0 * (DAB_tf - DAB_t0 - dDAB_t0 * T) / MathTools.square(T);
         dDAB_tf = dDAB_t0 + ddDAB * T;

         fourBar.update(Angle.DAB, DAB_t0, dDAB_t0);
         ABC_t0 = fourBar.getAngleABC();
         CDA_t0 = fourBar.getAngleCDA();
         BCD_t0 = fourBar.getAngleBCD();

         fourBar.update(Angle.DAB, DAB_tf, dDAB_tf);
         ABC_fourbar_tf = fourBar.getAngleABC();
         CDA_fourbar_tf = fourBar.getAngleCDA();
         BCD_fourbar_tf = fourBar.getAngleBCD();

         ABC_next_tj = ABC_t0;
         CDA_next_tj = CDA_t0;
         BCD_next_tj = BCD_t0;

         for (int j = 0; j < nSteps - 1; j++)
         {
            DAB_tj = ddDAB * MathTools.square(j * deltaT) / 2.0 + dDAB_t0 * j * deltaT + DAB_t0;
            dDAB_tj = ddDAB * j * deltaT + dDAB_t0;
            fourBar.update(Angle.DAB, DAB_tj, dDAB_tj, ddDAB);

            ABC_next_tj += fourBar.getAngleDtABC() * deltaT + fourBar.getAngleDt2ABC() * MathTools.square(deltaT) / 2.0;
            CDA_next_tj += fourBar.getAngleDtCDA() * deltaT + fourBar.getAngleDt2CDA() * MathTools.square(deltaT) / 2.0;
            BCD_next_tj += fourBar.getAngleDtBCD() * deltaT + fourBar.getAngleDt2BCD() * MathTools.square(deltaT) / 2.0;
         }

         ABC_numerical_tf = ABC_next_tj;
         CDA_numerical_tf = CDA_next_tj;
         BCD_numerical_tf = BCD_next_tj;

         assertEquals(ABC_numerical_tf, ABC_fourbar_tf, eps);
         assertEquals(CDA_numerical_tf, CDA_fourbar_tf, eps);
         assertEquals(BCD_numerical_tf, BCD_fourbar_tf, eps);
      }
   }

   @Test
   public void testMasterAngleComputations()
   {
      Random rand = new Random(1986L);
      for (int i = 0; i < 10000; i++)
      {
         double e = 100 * (rand.nextDouble() + 0.001);
         double k1 = rand.nextDouble();
         double k2 = rand.nextDouble();
         double d1 = e * Math.abs(rand.nextGaussian());
         double d2 = e * Math.abs(rand.nextGaussian());
         double DE = e * k1, DF = e * k2, BE = e * (1 - k1), BF = e * (1 - k2);
         double AE = d1, CF = d2;
         double AD = Math.sqrt(DE * DE + AE * AE), DAE = Math.atan2(DE, AE), ADE = Math.atan2(AE, DE);
         double AB = Math.sqrt(AE * AE + BE * BE), BAE = Math.atan2(BE, AE), ABE = Math.atan2(AE, BE);
         double CD = Math.sqrt(CF * CF + DF * DF), CDF = Math.atan2(CF, DF), DCF = Math.atan2(DF, CF);
         double BC = Math.sqrt(BF * BF + CF * CF), CBF = Math.atan2(CF, BF), BCF = Math.atan2(BF, CF);
         double DAB = DAE + BAE, ABC = ABE + CBF, BCD = BCF + DCF, CDA = ADE + CDF;

         FourBar calculator = new FourBar();
         calculator.setSideLengths(AB, BC, CD, AD);

         calculator.update(Angle.ABC, ABC);
         assertEquals(DAB, calculator.getAngleDAB(), EPSILON);
         assertEquals(ABC, calculator.getAngleABC(), EPSILON);
         assertEquals(BCD, calculator.getAngleBCD(), EPSILON);
         assertEquals(CDA, calculator.getAngleCDA(), EPSILON);

         calculator.update(Angle.BCD, BCD);
         assertEquals(DAB, calculator.getAngleDAB(), EPSILON);
         assertEquals(ABC, calculator.getAngleABC(), EPSILON);
         assertEquals(BCD, calculator.getAngleBCD(), EPSILON);
         assertEquals(CDA, calculator.getAngleCDA(), EPSILON);

         calculator.update(Angle.CDA, CDA);
         assertEquals(DAB, calculator.getAngleDAB(), EPSILON);
         assertEquals(ABC, calculator.getAngleABC(), EPSILON);
         assertEquals(BCD, calculator.getAngleBCD(), EPSILON);
         assertEquals(CDA, calculator.getAngleCDA(), EPSILON);
      }
   }

   @Test
   public void testGetMinMaxAngles()
   {
      Random random = new Random(45647);
      FourBar calculator = new FourBar();

      for (int i = 0; i < ITERATIONS; i++)
      { // Testing for a convex quadrilateral
         List<Point2D> vertices = EuclidGeometryRandomTools.nextCircleBasedConvexPolygon2D(random, 10.0, 10.0, 4);
         Point2D A = vertices.get(0);
         Point2D B = vertices.get(1);
         Point2D C = vertices.get(2);
         Point2D D = vertices.get(3);

         double ABLength = A.distance(B);
         double BCLength = B.distance(C);
         double CDLength = C.distance(D);
         double DALength = D.distance(A);
         calculator.setSideLengths(ABLength, BCLength, CDLength, DALength);

         // Check sum of angles is 2*pi.
         // when diagonal AC is max we have: DAB=minDAB, ABC=maxABC, BCD=minBCD, CDA=maxCDA. Let's verify this:
         double minDAB = calculator.getMinDAB();
         double maxABC = calculator.getMaxABC();
         double minBCD = calculator.getMinBCD();
         double maxCDA = calculator.getMaxCDA();
         assertEquals(2.0 * Math.PI, minDAB + maxABC + minBCD + maxCDA, EPSILON, "Iteration " + i);
         // when diagonal BD is max we have: DAB=maxDAB, ABC=minABC, BCD=maxBCD, CDA=minCDA. Let's verify this:
         double maxDAB = calculator.getMaxDAB();
         double minABC = calculator.getMinABC();
         double maxBCD = calculator.getMaxBCD();
         double minCDA = calculator.getMinCDA();
         assertEquals(2.0 * Math.PI, maxDAB + minABC + maxBCD + minCDA, EPSILON, "Iteration " + i);
      }
   }
}
