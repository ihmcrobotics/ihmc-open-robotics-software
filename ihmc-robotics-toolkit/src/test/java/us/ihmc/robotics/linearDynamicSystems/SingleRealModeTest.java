package us.ihmc.robotics.linearDynamicSystems;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.robotics.dataStructures.Polynomial;

public class SingleRealModeTest
{
   @BeforeEach
   public void setUp() throws Exception
   {
   }

   @AfterEach
   public void tearDown() throws Exception
   {
   }

	@Test
   public void testDifferentLengths()
   {
      double eigenvalue = -2.0;
      double[] leftEigenvectorV = new double[] {1.0, 2.0};
      double[] rightEigenvectorW = new double[] {1.0};

      try
      {
         @SuppressWarnings("unused") SingleRealMode singleRealMode = new SingleRealMode(eigenvalue, leftEigenvectorV, rightEigenvectorW);
         fail();
      }
      catch (Exception e)
      {
      }
   }

	@Test
   public void testDotEqualsOne()
   {
      double eigenvalue = -2.0;
      double[] leftEigenvectorV = new double[] {1.0, 2.0};
      double[] rightEigenvectorW = new double[] {1.0, 3.0};

      try
      {
         @SuppressWarnings("unused")
         SingleRealMode singleRealMode = new SingleRealMode(eigenvalue, leftEigenvectorV, rightEigenvectorW);
         fail();
      }
      catch (Exception e)
      {
      }
   }

	@Test
   public void testConstructTransferFunctionMatrixSISO()
   {
      double eigenvalue = -2.0;
      double[] leftEigenvectorV = new double[] {1.0};
      double[] rightEigenvectorW = new double[] {1.0};    // Constraint v dot w = 1.0

      SingleRealMode singleRealMode = new SingleRealMode(eigenvalue, leftEigenvectorV, rightEigenvectorW);
      assertEquals(eigenvalue, singleRealMode.getEigenvalue(), 1e-7);

      TransferFunctionMatrix transferFunctionMatrix = singleRealMode.constructTransferFunctionMatrix();

      assertEquals(1, transferFunctionMatrix.getColumns());
      assertEquals(1, transferFunctionMatrix.getRows());

      TransferFunction transferFunction = transferFunctionMatrix.get(0, 0);

      Polynomial numeratorPolynomial = transferFunction.getNumeratorPolynomial();
      Polynomial denominatorPolynomial = transferFunction.getDenominatorPolynomial();

      double[] numeratorCoefficients = numeratorPolynomial.getCoefficients();
      double[] denominatorCoefficients = denominatorPolynomial.getCoefficients();

      DynamicSystemsTestHelpers.assertEpsilonEquals(new double[] {1.0}, numeratorCoefficients, 1e-7);
      DynamicSystemsTestHelpers.assertEpsilonEquals(new double[] {1.0, -eigenvalue}, denominatorCoefficients, 1e-7);
   }

	@Test
   public void testConstructTransferFunctionMatrixMIMO()
   {
      double eigenvalue = -5.0;
      double[] leftEigenvectorV = new double[] {1.0, 2.0};
      double[] rightEigenvectorW = new double[] {0.5, 0.25};    // Constraint v dot w = 1.0

      SingleRealMode singleRealMode = new SingleRealMode(eigenvalue, leftEigenvectorV, rightEigenvectorW);
      assertEquals(eigenvalue, singleRealMode.getEigenvalue(), 1e-7);

      TransferFunctionMatrix transferFunctionMatrix = singleRealMode.constructTransferFunctionMatrix();

      assertEquals(2, transferFunctionMatrix.getColumns());
      assertEquals(2, transferFunctionMatrix.getRows());

      double[] expectedDenominator = new double[] {1.0, -eigenvalue};
      double[][] expectedNumeratorConstants = new double[][]
      {
         {0.5, 0.25}, {1.0, 0.5}
      };

      for (int i = 0; i < 2; i++)
      {
         for (int j = 0; j < 2; j++)
         {
            TransferFunction transferFunction = transferFunctionMatrix.get(i, j);

            Polynomial numeratorPolynomial = transferFunction.getNumeratorPolynomial();
            Polynomial denominatorPolynomial = transferFunction.getDenominatorPolynomial();

            double[] numeratorCoefficients = numeratorPolynomial.getCoefficients();
            double[] denominatorCoefficients = denominatorPolynomial.getCoefficients();

            // DynamicSystemsTestHelpers.printArray("numeratorCoefficients", numeratorCoefficients);
            // DynamicSystemsTestHelpers.printArray("denominatorCoefficients", denominatorCoefficients);

            DynamicSystemsTestHelpers.assertEpsilonEquals(new double[] {expectedNumeratorConstants[i][j]}, numeratorCoefficients, 1e-7);
            DynamicSystemsTestHelpers.assertEpsilonEquals(expectedDenominator, denominatorCoefficients, 1e-7);
         }
      }
   }



}
