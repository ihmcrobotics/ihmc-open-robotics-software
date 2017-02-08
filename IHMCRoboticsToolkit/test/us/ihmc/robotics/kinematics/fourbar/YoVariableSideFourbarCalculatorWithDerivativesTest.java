package us.ihmc.robotics.kinematics.fourbar;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.sqrt;
import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class YoVariableSideFourbarCalculatorWithDerivativesTest
{
   private static final double eps = 1e-7;
   private static final boolean PRINT = false;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSquare()
   {
      YoVariableSideFourbarCalculatorWithDerivatives calculator = new YoVariableSideFourbarCalculatorWithDerivatives("testCalculator", registry);
      calculator.setSideLengths(1.0, 1.0, 1.0, 1.0);
      calculator.updateAnglesGivenAngleDAB(PI / 2);
      assertEquals(PI / 2, calculator.getAngleDAB(), eps);
      assertEquals(PI / 2, calculator.getAngleABC(), eps);
      assertEquals(PI / 2, calculator.getAngleBCD(), eps);
      assertEquals(PI / 2, calculator.getAngleCDA(), eps);
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSquareDer()
   {
      YoVariableSideFourbarCalculatorWithDerivatives calculator = new YoVariableSideFourbarCalculatorWithDerivatives("testCalculator", registry);
      calculator.setSideLengths(1.0, 1.0, 1.0, 1.0);
      calculator.updateAnglesAndVelocitiesGivenAngleDAB(PI / 2.0, 1.0);
      assertEquals(1, calculator.getAngleDtDAB(), eps);
      assertEquals(1, calculator.getAngleDtBCD(), eps);
      assertEquals(-1, calculator.getAngleDtABC(), eps);
      assertEquals(-1, calculator.getAngleDtCDA(), eps);
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testParallelogram()
   {
      YoVariableSideFourbarCalculatorWithDerivatives calculator = new YoVariableSideFourbarCalculatorWithDerivatives("testCalculator", registry);
      calculator.setSideLengths(1.0, 1.0, 1.0, 1.0);
      calculator.updateAnglesAndVelocitiesGivenAngleDAB(PI / 3, 1);
      assertEquals(PI / 3, calculator.getAngleDAB(), eps);
      assertEquals(2 * PI / 3, calculator.getAngleABC(), eps);
      assertEquals(PI / 3, calculator.getAngleBCD(), eps);
      assertEquals(2 * PI / 3, calculator.getAngleCDA(), eps);
      assertEquals(1, calculator.getAngleDtDAB(), eps);
      assertEquals(1, calculator.getAngleDtBCD(), eps);
      assertEquals(-1, calculator.getAngleDtABC(), eps);
      assertEquals(-1, calculator.getAngleDtCDA(), eps);
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRandomQuadrilatteral()
   {
      YoVariableSideFourbarCalculatorWithDerivatives calculator = new YoVariableSideFourbarCalculatorWithDerivatives("testCalculator", registry);

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

         solveFourBar(calculator, AD, AB, BC, CD, BAD, ABC, BCD, ADC);
      }
   }

   private void solveFourBar(YoVariableSideFourbarCalculatorWithDerivatives calculator, double lengthA, double lengthB, double lengthC, double lengthD,
         double A, double B, double C, double D)
   {
      if (PRINT)
      {
         String[] names = "a,b,c,d,A,B,C,D".split(",");
         double[] vals = new double[] {lengthA, lengthB, lengthC, lengthD, A, B, C, D};
         for (int i = 0; i < vals.length; i++)
         {
            System.out.println(names[i] + " = " + vals[i]);
         }
      }
      calculator.setSideLengths(lengthA, lengthB, lengthC, lengthD);
      calculator.updateAnglesAndVelocitiesGivenAngleDAB(A, 0.0);
      assertEquals(A, calculator.getAngleDAB(), eps);
      assertEquals(B, calculator.getAngleABC(), eps);
      assertEquals(C, calculator.getAngleBCD(), eps);
      assertEquals(D, calculator.getAngleCDA(), eps);
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testMasterAngleComputations()
   {
      YoVariableSideFourbarCalculatorWithDerivatives calculator = new YoVariableSideFourbarCalculatorWithDerivatives("testCalculator", registry);

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

         calculator.setSideLengths(AD, AB, BC, CD);

         calculator.computeMasterJointAngleGivenAngleABC(ABC);
         assertEquals(BAD, calculator.getAngleDAB(), eps);

         calculator.computeMasterJointAngleGivenAngleBCD(BCD);
         assertEquals(BAD, calculator.getAngleDAB(), eps);

         calculator.computeMasterJointAngleGivenAngleCDA(ADC);
         assertEquals(BAD, calculator.getAngleDAB(), eps);
      }

   }
}
