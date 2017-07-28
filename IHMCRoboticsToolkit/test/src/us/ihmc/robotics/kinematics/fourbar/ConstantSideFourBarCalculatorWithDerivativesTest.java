package us.ihmc.robotics.kinematics.fourbar;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.sqrt;
import static org.junit.Assert.assertEquals;
import static us.ihmc.robotics.MathTools.square;

import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class ConstantSideFourBarCalculatorWithDerivativesTest
{
   private static final double eps = 1e-7;
   private static final boolean PRINT = false;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSquare()
   {
      ConstantSideFourBarCalculatorWithDerivatives fourBar = new ConstantSideFourBarCalculatorWithDerivatives(1.0, 1.0, 1.0, 1.0);
      fourBar.updateAnglesGivenAngleDAB(PI / 2);
      assertEquals(PI / 2, fourBar.getAngleDAB(), eps);
      assertEquals(PI / 2, fourBar.getAngleABC(), eps);
      assertEquals(PI / 2, fourBar.getAngleBCD(), eps);
      assertEquals(PI / 2, fourBar.getAngleCDA(), eps);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSquareDer()
   {
      ConstantSideFourBarCalculatorWithDerivatives fourBar = new ConstantSideFourBarCalculatorWithDerivatives(1.0, 1.0, 1.0, 1.0);
      fourBar.updateAnglesAndVelocitiesGivenAngleDAB(PI / 2, 1);
      assertEquals(PI / 2, fourBar.getAngleDAB(), eps);
      assertEquals(PI / 2, fourBar.getAngleABC(), eps);
      assertEquals(PI / 2, fourBar.getAngleBCD(), eps);
      assertEquals(PI / 2, fourBar.getAngleCDA(), eps);
      assertEquals(1, fourBar.getAngleDtDAB(), eps);
      assertEquals(1, fourBar.getAngleDtBCD(), eps);
      assertEquals(-1, fourBar.getAngleDtABC(), eps);
      assertEquals(-1, fourBar.getAngleDtCDA(), eps);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testParallelogram()
   {
      ConstantSideFourBarCalculatorWithDerivatives fourBar = new ConstantSideFourBarCalculatorWithDerivatives(1.0, 1.0, 1.0, 1.0);
      fourBar.updateAnglesAndVelocitiesGivenAngleDAB(PI / 3, 1);
      assertEquals(PI / 3, fourBar.getAngleDAB(), eps);
      assertEquals(2 * PI / 3, fourBar.getAngleABC(), eps);
      assertEquals(PI / 3, fourBar.getAngleBCD(), eps);
      assertEquals(2 * PI / 3, fourBar.getAngleCDA(), eps);
      assertEquals(1, fourBar.getAngleDtDAB(), eps);
      assertEquals(1, fourBar.getAngleDtBCD(), eps);
      assertEquals(-1, fourBar.getAngleDtABC(), eps);
      assertEquals(-1, fourBar.getAngleDtCDA(), eps);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testRandomQuadrilatteral()
   {
      Random rand = new Random(1986L);
      for (int i = 0; i < 10000; i++)
      {
         double e = 100 * (rand.nextDouble() + 0.001);
         double k1 = rand.nextDouble();
         double k2 = rand.nextDouble();
         double d1 = e * abs(rand.nextGaussian());
         double d2 = e * abs(rand.nextGaussian());
         double DE = e * k1, DF = e * k2, BE = e * (1 - k1), BF = e * (1 - k2);
         double AE = d1, CF = d2;
         double AD = sqrt(DE * DE + AE * AE), DAE = atan2(DE, AE), ADE = atan2(AE, DE);
         double AB = sqrt(AE * AE + BE * BE), BAE = atan2(BE, AE), ABE = atan2(AE, BE);
         double CD = sqrt(CF * CF + DF * DF), CDF = atan2(CF, DF), DCF = atan2(DF, CF);
         double BC = sqrt(BF * BF + CF * CF), CBF = atan2(CF, BF), BCF = atan2(BF, CF);
         double BAD = DAE + BAE, ABC = ABE + CBF, BCD = BCF + DCF, ADC = ADE + CDF;

         solveFourBar(AD, AB, BC, CD, BAD, ABC, BCD, ADC);
      }
   }

   private void solveFourBar(double a, double b, double c, double d, double A, double B, double C, double D)
   {
      if (PRINT)
      {
         String[] names = "a,b,c,d,A,B,C,D".split(",");
         double[] vals = new double[] { a, b, c, d, A, B, C, D };
         for (int i = 0; i < vals.length; i++)
         {
            System.out.println(names[i] + " = " + vals[i]);
         }
      }
      ConstantSideFourBarCalculatorWithDerivatives fourBar = new ConstantSideFourBarCalculatorWithDerivatives(a, b, c, d);
      fourBar.updateAnglesAndVelocitiesGivenAngleDAB(A, 0.0);
      assertEquals(A, fourBar.getAngleDAB(), eps);
      assertEquals(B, fourBar.getAngleABC(), eps);
      assertEquals(C, fourBar.getAngleBCD(), eps);
      assertEquals(D, fourBar.getAngleCDA(), eps);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
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
         double CD = sqrt(l * l + (k - s) * (k - s));
         double AD = s;
         double BAD = PI / 2;
         double ABC = PI / 2;
         double BCD = atan2(l, k - s);
         double ADC = PI - BCD;
         double dBAD = 1;
         double dABC = -dBAD * s / k;
         double dADC = -dBAD;
         double dBCD = -dABC;

         ConstantSideFourBarCalculatorWithDerivatives fourBar = new ConstantSideFourBarCalculatorWithDerivatives(AD, AB, BC, CD);
         fourBar.updateAnglesAndVelocitiesGivenAngleDAB(BAD, dBAD);
         assertEquals(BAD, fourBar.getAngleDAB(), eps);
         assertEquals(ABC, fourBar.getAngleABC(), eps);
         assertEquals(BCD, fourBar.getAngleBCD(), eps);
         assertEquals(ADC, fourBar.getAngleCDA(), eps);
         assertEquals(0, dBAD + dABC + dBCD + dADC, eps);
         assertEquals(0, fourBar.getAngleDtDAB() + fourBar.getAngleDtABC() + fourBar.getAngleDtBCD() + fourBar.getAngleDtCDA(), eps);
         assertEquals(dBAD, fourBar.getAngleDtDAB(), eps);
         assertEquals(dABC, fourBar.getAngleDtABC(), eps);
         assertEquals(dBCD, fourBar.getAngleDtBCD(), eps);
         assertEquals(dADC, fourBar.getAngleDtCDA(), eps);

      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.2)
	@Test(timeout = 30000)
   public void testVelocitiesWithRandomQuadrilateral()
   {
      double eps = 1.0e-6;
      Random rand = new Random(1976L);
      double DA, AB, BC, CD;
      ConstantSideFourBarCalculatorWithDerivatives fourBar;
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
         fourBar = new ConstantSideFourBarCalculatorWithDerivatives(DA, AB, BC, CD);
         isQuadrilateralOK = fourBar.getMaxDAB() - fourBar.getMinDAB() > 1.0e-5;

         k = 0;

         while (!isQuadrilateralOK)
         {
            AB = RandomNumbers.nextDouble(rand, 0.1, 2.0);
            BC = RandomNumbers.nextDouble(rand, 0.1, 2.0);
            CD = RandomNumbers.nextDouble(rand, 0.1, 2.0);
            DA = RandomNumbers.nextDouble(rand, 0.1, 2.0);
            fourBar = new ConstantSideFourBarCalculatorWithDerivatives(DA, AB, BC, CD);
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

            DAB_t0_outOfRange = fourBar.updateAnglesGivenAngleDAB(DAB_t0);
            ABC_t0 = fourBar.getAngleABC();
            CDA_t0 = fourBar.getAngleCDA();
            BCD_t0 = fourBar.getAngleBCD();

            DAB_tf_outOfRange = fourBar.updateAnglesGivenAngleDAB(DAB_tf);
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
               fourBar.updateAnglesAndVelocitiesGivenAngleDAB(DAB_tj, dDAB);

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

	@ContinuousIntegrationTest(estimatedDuration = 1.3)
	@Test(timeout = 30000)
   public void testAccelerationsWithRandomQuadrilateral()
   {
      double eps = 1.0e-5;
      Random random = new Random(1984L);
      double AD, BA, CB, DC;
      ConstantSideFourBarCalculatorWithDerivatives fourBar;
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
         fourBar = new ConstantSideFourBarCalculatorWithDerivatives(AD, BA, CB, DC);
         isQuadrilateralOK = fourBar.getMaxDAB() - fourBar.getMinDAB() > 1.0e-5;

         k = 0;

         while (!isQuadrilateralOK)
         {
            BA = RandomNumbers.nextDouble(random, 0.1, 2.0);
            CB = RandomNumbers.nextDouble(random, 0.1, 2.0);
            DC = RandomNumbers.nextDouble(random, 0.1, 2.0);
            AD = RandomNumbers.nextDouble(random, 0.1, 2.0);
            fourBar = new ConstantSideFourBarCalculatorWithDerivatives(AD, BA, CB, DC);
            isQuadrilateralOK = fourBar.getMaxDAB() - fourBar.getMinDAB() > 1.0e-5;

            k++;
            if (k > 10)
               throw new RuntimeException("Could not make a convex quadrilateral");
         }

         DAB_t0 = RandomNumbers.nextDouble(random, fourBar.getMinDAB(), fourBar.getMaxDAB());
         DAB_tf = RandomNumbers.nextDouble(random, fourBar.getMinDAB(), fourBar.getMaxDAB());
         dDAB_t0 = 0.0;
         ddDAB = 2.0 * (DAB_tf - DAB_t0 - dDAB_t0 * T) / square(T);
         dDAB_tf = dDAB_t0 + ddDAB * T;

         fourBar.updateAnglesAndVelocitiesGivenAngleDAB(DAB_t0, dDAB_t0);
         ABC_t0 = fourBar.getAngleABC();
         CDA_t0 = fourBar.getAngleCDA();
         BCD_t0 = fourBar.getAngleBCD();

         fourBar.updateAnglesAndVelocitiesGivenAngleDAB(DAB_tf, dDAB_tf);
         ABC_fourbar_tf = fourBar.getAngleABC();
         CDA_fourbar_tf = fourBar.getAngleCDA();
         BCD_fourbar_tf = fourBar.getAngleBCD();

         ABC_next_tj = ABC_t0;
         CDA_next_tj = CDA_t0;
         BCD_next_tj = BCD_t0;

         for (int j = 0; j < nSteps - 1; j++)
         {
            DAB_tj = ddDAB * square(j * deltaT) / 2.0 + dDAB_t0 * j * deltaT + DAB_t0;
            dDAB_tj = ddDAB * j * deltaT + dDAB_t0;
            fourBar.updateAnglesVelocitiesAndAccelerationsGivenAngleDAB(DAB_tj, dDAB_tj, ddDAB);

            ABC_next_tj += fourBar.getAngleDtABC() * deltaT + fourBar.getAngleDt2ABC() * square(deltaT) / 2.0;
            CDA_next_tj += fourBar.getAngleDtCDA() * deltaT + fourBar.getAngleDt2CDA() * square(deltaT) / 2.0;
            BCD_next_tj += fourBar.getAngleDtBCD() * deltaT + fourBar.getAngleDt2BCD() * square(deltaT) / 2.0;
         }

         ABC_numerical_tf = ABC_next_tj;
         CDA_numerical_tf = CDA_next_tj;
         BCD_numerical_tf = BCD_next_tj;

         assertEquals(ABC_numerical_tf, ABC_fourbar_tf, eps);
         assertEquals(CDA_numerical_tf, CDA_fourbar_tf, eps);
         assertEquals(BCD_numerical_tf, BCD_fourbar_tf, eps);
      }
   }
	
	@ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testMasterAngleComputations()
   {
	   Random rand = new Random(1986L);
      for (int i = 0; i < 10000; i++)
      {
         double e = 100 * (rand.nextDouble() + 0.001);
         double k1 = rand.nextDouble();
         double k2 = rand.nextDouble();
         double d1 = e * abs(rand.nextGaussian());
         double d2 = e * abs(rand.nextGaussian());
         double DE = e * k1, DF = e * k2, BE = e * (1 - k1), BF = e * (1 - k2);
         double AE = d1, CF = d2;
         double AD = sqrt(DE * DE + AE * AE), DAE = atan2(DE, AE), ADE = atan2(AE, DE);
         double AB = sqrt(AE * AE + BE * BE), BAE = atan2(BE, AE), ABE = atan2(AE, BE);
         double CD = sqrt(CF * CF + DF * DF), CDF = atan2(CF, DF), DCF = atan2(DF, CF);
         double BC = sqrt(BF * BF + CF * CF), CBF = atan2(CF, BF), BCF = atan2(BF, CF);
         double BAD = DAE + BAE, ABC = ABE + CBF, BCD = BCF + DCF, ADC = ADE + CDF;
         
         ConstantSideFourBarCalculatorWithDerivatives calculator = new ConstantSideFourBarCalculatorWithDerivatives(AD, AB, BC, CD);
         
         calculator.computeMasterJointAngleGivenAngleABC(ABC);    
         assertEquals(BAD, calculator.getAngleDAB(), eps); 
         
         calculator.computeMasterJointAngleGivenAngleBCD(BCD);
         assertEquals(BAD, calculator.getAngleDAB(), eps); 
         
         calculator.computeMasterJointAngleGivenAngleCDA(ADC);
         assertEquals(BAD, calculator.getAngleDAB(), eps); 
      }
      
   }
}
