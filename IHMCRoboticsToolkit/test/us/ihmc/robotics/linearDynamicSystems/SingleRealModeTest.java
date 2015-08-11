package us.ihmc.robotics.linearDynamicSystems;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.robotics.dataStructures.Polynomial;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

public class SingleRealModeTest
{
   @Before
   public void setUp() throws Exception
   {
   }

   @After
   public void tearDown() throws Exception
   {
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
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

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
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

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
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

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
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
