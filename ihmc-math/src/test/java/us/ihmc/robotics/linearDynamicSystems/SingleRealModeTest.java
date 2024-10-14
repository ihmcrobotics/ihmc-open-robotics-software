package us.ihmc.robotics.linearDynamicSystems;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.robotics.math.trajectories.interfaces.PolynomialReadOnly;

import static org.junit.jupiter.api.Assertions.*;

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

      PolynomialReadOnly numeratorPolynomial = transferFunction.getNumeratorPolynomial();
      PolynomialReadOnly denominatorPolynomial = transferFunction.getDenominatorPolynomial();

      double[] numeratorCoefficients = numeratorPolynomial.getCoefficients();
      double[] denominatorCoefficients = denominatorPolynomial.getCoefficients();

      assertArrayEquals(new double[] {1.0}, numeratorCoefficients, 1e-7);
      assertArrayEquals(new double[] {-eigenvalue, 1.0}, denominatorCoefficients, 1e-7);
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

      double[] expectedDenominator = new double[] {-eigenvalue, 1.0};
      double[][] expectedNumeratorConstants = new double[][]
      {
         {0.5, 0.25}, {1.0, 0.5}
      };

      for (int i = 0; i < 2; i++)
      {
         for (int j = 0; j < 2; j++)
         {
            TransferFunction transferFunction = transferFunctionMatrix.get(i, j);

            PolynomialReadOnly numeratorPolynomial = transferFunction.getNumeratorPolynomial();
            PolynomialReadOnly denominatorPolynomial = transferFunction.getDenominatorPolynomial();

            double[] numeratorCoefficients = numeratorPolynomial.getCoefficients();
            double[] denominatorCoefficients = denominatorPolynomial.getCoefficients();

            // DynamicSystemsTestHelpers.printArray("numeratorCoefficients", numeratorCoefficients);
            // DynamicSystemsTestHelpers.printArray("denominatorCoefficients", denominatorCoefficients);

            assertArrayEquals(expectedDenominator, denominatorCoefficients, 1e-7);
            assertArrayEquals(new double[] {expectedNumeratorConstants[i][j]}, numeratorCoefficients, 1e-7);
         }
      }
   }



}
