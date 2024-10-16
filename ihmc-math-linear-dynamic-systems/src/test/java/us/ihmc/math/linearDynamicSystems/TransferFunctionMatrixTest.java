package us.ihmc.math.linearDynamicSystems;

import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.math.ComplexConjugateMode;
import us.ihmc.math.ComplexNumber;
import us.ihmc.math.SingleRealMode;
import us.ihmc.robotics.math.trajectories.core.Polynomial;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialReadOnly;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class TransferFunctionMatrixTest
{
   private Random random;
   private TransferFunction simpleDecayOne, secondOrderResponseOne;
   private TransferFunction simpleDecayTwo, secondOrderResponseTwo;
   private TransferFunctionMatrix transferFunctionMatrix;

   @BeforeEach
   public void setUp() throws Exception
   {
      random = new Random(1738L);

      secondOrderResponseOne = TransferFunction.constructSecondOrderTransferFunction(1.0, 10.0, 0.3);
      simpleDecayOne = new TransferFunction(new double[] {1.0}, new double[] {1.0, -3.0});
      secondOrderResponseTwo = TransferFunction.constructSecondOrderTransferFunction(2.0, 20.0, 2.0);
      simpleDecayTwo = new TransferFunction(new double[] {1.0}, new double[] {1.0, -10.0});

      TransferFunction[][] transferFunctions = new TransferFunction[][] {{simpleDecayOne, secondOrderResponseOne}, {simpleDecayTwo, secondOrderResponseTwo}};

      transferFunctionMatrix = new TransferFunctionMatrix(transferFunctions);
   }

   @AfterEach
   public void tearDown() throws Exception
   {
      simpleDecayOne = null;
      secondOrderResponseOne = null;
      simpleDecayTwo = null;
      secondOrderResponseTwo = null;

      transferFunctionMatrix = null;
   }

   @Test
   public void testGet()
   {
      verifyEpsilonEqual(simpleDecayOne, transferFunctionMatrix.get(0, 0), 1e-7);
   }

   private void verifyEpsilonEqual(TransferFunction expectedTransferFunction, TransferFunction actualTransferFunction, double epsilon)
   {
      verifyEpsilonEqual(expectedTransferFunction.getNumeratorCoefficients(), actualTransferFunction.getNumeratorCoefficients(), epsilon);
      verifyEpsilonEqual(expectedTransferFunction.getDenominatorCoefficients(), actualTransferFunction.getDenominatorCoefficients(), epsilon);
   }

   private void verifyEpsilonEqual(double[] expected, double[] actual, double epsilon)
   {
      assertEquals(expected.length, actual.length);

      //    if (expected.length != actual.length)
      //       fail();

      for (int i = 0; i < expected.length; i++)
      {
         assertEquals(expected[i], actual[i], epsilon);
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

      TransferFunctionMatrix transferFunctionMatrix = TransferFunctionMatrix.constructTransferFunctionMatrix(singleRealMode);

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

      TransferFunctionMatrix transferFunctionMatrix = TransferFunctionMatrix.constructTransferFunctionMatrix(singleRealMode);

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


   @Test
   public void testPremultiply()
   {
      SimpleMatrix matrixC = new SimpleMatrix(new double[][] {{1.0, 0.0}, {0.0, 1.0}});
      TransferFunctionMatrix newTransferFunctionMatrix = transferFunctionMatrix.preMultiply(matrixC);
      assertTrue(newTransferFunctionMatrix.epsilonEquals(transferFunctionMatrix, 1e-7));

      matrixC = new SimpleMatrix(new double[][] {{1.0, 0.0}, {0.0, 0.0}});
      newTransferFunctionMatrix = transferFunctionMatrix.preMultiply(matrixC);

      TransferFunction[][] transferFunctions = new TransferFunction[][] {{simpleDecayOne, secondOrderResponseOne},
                                                                         {TransferFunction.constructZeroTransferFunction(),
                                                                          TransferFunction.constructZeroTransferFunction()}};

      transferFunctionMatrix = new TransferFunctionMatrix(transferFunctions);
      assertTrue(newTransferFunctionMatrix.epsilonEquals(transferFunctionMatrix, 1e-7));
   }

   @Test
   public void testPreMultiplyException()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         int rows = transferFunctionMatrix.getRows();
         int columns = transferFunctionMatrix.getColumns();
         SimpleMatrix testMatrix = SimpleMatrix.random_DDRM(rows, columns - 1, -1.0, 1.0, random);

         transferFunctionMatrix.preMultiply(testMatrix);
      });
   }

   @Test
   public void testTimes()
   {
      SimpleMatrix matrixC = new SimpleMatrix(new double[][] {{1.0, 0.0}, {0.0, 1.0}});
      TransferFunctionMatrix newTransferFunctionMatrix = transferFunctionMatrix.times(matrixC);
      assertTrue(newTransferFunctionMatrix.epsilonEquals(transferFunctionMatrix, 1e-7));

      matrixC = new SimpleMatrix(new double[][] {{1.0, 0.0}, {0.0, 0.0}});
      newTransferFunctionMatrix = transferFunctionMatrix.times(matrixC);

      TransferFunction[][] transferFunctions = new TransferFunction[][] {{simpleDecayOne, TransferFunction.constructZeroTransferFunction()},
                                                                         {simpleDecayTwo, TransferFunction.constructZeroTransferFunction()}};

      transferFunctionMatrix = new TransferFunctionMatrix(transferFunctions);
      assertTrue(newTransferFunctionMatrix.epsilonEquals(transferFunctionMatrix, 1e-7));
   }

   @Test
   public void testTimesException()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         int rows = transferFunctionMatrix.getRows();
         int columns = transferFunctionMatrix.getColumns();
         SimpleMatrix testMatrix = SimpleMatrix.random_DDRM(rows - 1, columns, -1.0, 1.0, random);

         transferFunctionMatrix.times(testMatrix);
      });
   }

   @Test
   public void testPlusDouble()
   {
      SimpleMatrix testMatrix = SimpleMatrix.random_DDRM(transferFunctionMatrix.getRows(), transferFunctionMatrix.getRows(), -1.0, 1.0, random);
      TransferFunctionMatrix result = transferFunctionMatrix.plus(testMatrix);

      for (int m = 0; m < testMatrix.numRows(); m++)
      {
         for (int n = 0; n < testMatrix.numCols(); n++)
         {
            TransferFunction resultTF = result.get(m, n);
            TransferFunction originalTF = transferFunctionMatrix.get(m, n);
            Polynomial originalNumeratorCopy = new Polynomial(originalTF.getNumeratorCoefficients());
            Polynomial originalDenominatorCopy = new Polynomial(originalTF.getDenominatorCoefficients());


            /*
             *  Different way of calculating the sum of the originalTF and the scalar a:
             * originalTF = p1/p2; resultTF = originalTF + a = p1/p2 + (a*p2)/p2 = (p1+a*p2)/p2
             * resultTF = productTF * originalTF = productTF * p1/p2 = (p1+a*p2)/p2
             * productTF = (p1+a*p2)/p1
             * Note that this is not valid when p1 == 0.0
             */
            double matrixValue = testMatrix.get(m, n);
            if (originalNumeratorCopy.equalsZero(1e-15))
            {    // TODO: Add a way such that this too is tested.
               assertTrue(resultTF.getNumeratorCoefficients()[0] == matrixValue);
               assertEquals(1, resultTF.getNumeratorCoefficients().length);
            }
            else
            {
               TransferFunction productTF = new TransferFunction(originalDenominatorCopy.times(matrixValue).plus(originalNumeratorCopy), originalNumeratorCopy);

               TransferFunction expectedTF = productTF.times(originalTF);
               assertTrue(expectedTF.epsilonEquals(resultTF, 1e-5));
            }
         }
      }
   }

   @Test
   public void testPlusDoubleException()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         SimpleMatrix testMatrix = new SimpleMatrix(new double[][] {{2.0, 5.0}});
         transferFunctionMatrix.plus(testMatrix);
      });
   }

   @Test
   public void testPlusTransferFunctionDouble()
   {
      TransferFunction[][] transferFunctions = new TransferFunction[][] {{secondOrderResponseTwo, simpleDecayTwo}, {secondOrderResponseOne, simpleDecayOne}};
      TransferFunctionMatrix testMatrix = new TransferFunctionMatrix(transferFunctions);

      TransferFunctionMatrix result = transferFunctionMatrix.plus(testMatrix);

      for (int m = 0; m < testMatrix.getRows(); m++)
      {
         for (int n = 0; n < testMatrix.getColumns(); n++)
         {
            TransferFunction resultTF = result.get(m, n);
            TransferFunction originalTFOne = transferFunctionMatrix.get(m, n);
            Polynomial originalNumeratorOneCopy = new Polynomial(originalTFOne.getNumeratorCoefficients());
            Polynomial originalDenominatorOneCopy = new Polynomial(originalTFOne.getDenominatorCoefficients());

            TransferFunction originalTFTwo = testMatrix.get(m, n);
            Polynomial originalNumeratorTwoCopy = new Polynomial(originalTFTwo.getNumeratorCoefficients());
            Polynomial originalDenominatorTwoCopy = new Polynomial(originalTFTwo.getDenominatorCoefficients());

            /*
             * Different way of calculating the sum of TF1 and TF2:
             *             TF1 = N1/D1; TF2 = N2/D2; result = N1/D1 + N2/D2 = (N1*D2 + N2*D1)/(D1*D2)
             *             result = productTF * TF1 => productTF = (N1*D2 + N2*D1)/(N1*D2) = 1 + (N2*D1)/(N1*D2)
             *             Note that this is not applicable when N1 == 0.0
             */
            if (originalNumeratorOneCopy.equalsZero(1e-15))
            {    // TODO: Add a way such that this too is tested.
               assertTrue(originalTFTwo.epsilonEquals(resultTF, 1e-5));
            }
            else
            {
               TransferFunction productTF = new TransferFunction(originalNumeratorTwoCopy.times(originalDenominatorOneCopy),
                                                                 originalNumeratorOneCopy.times(originalDenominatorTwoCopy));
               productTF = productTF.plus(1.0);

               TransferFunction expectedTF = productTF.times(originalTFOne);
               assertTrue(expectedTF.epsilonEquals(resultTF, 1e-5));
            }
         }
      }
   }


   @Test
   public void testSecondOrderMassSpringDamper()
   {
      double wn = 7.2;
      double zeta = 0.3;
      double P1 = zeta * wn;
      double P2 = Math.sqrt(1.0 - zeta * zeta) * wn;
      @SuppressWarnings("unused")
      double P3 = wn * wn;

      ComplexNumber[] leftEigenvectorV = new ComplexNumber[] {new ComplexNumber(-P2, -P1), new ComplexNumber(0.0, 1.0)};
      ComplexNumber[] rightEigenvectorW = new ComplexNumber[] {new ComplexNumber(-1.0 / (2.0 * P2), 0.0), new ComplexNumber(-P1 / (2.0 * P2), -0.5)};

      ComplexNumber eigenvalue = new ComplexNumber(-P1, P2);

      ComplexConjugateMode complexConjugateMode = new ComplexConjugateMode(eigenvalue, leftEigenvectorV, rightEigenvectorW);
      DynamicSystemsTestHelpers.assertEpsilonEquals(eigenvalue, complexConjugateMode.getEigenvalue(), 1e-7);

      TransferFunctionMatrix transferFunctionMatrix = TransferFunctionMatrix.constructTransferFunctionMatrix(complexConjugateMode);

      @SuppressWarnings("unused") TransferFunction transferFunction00 = transferFunctionMatrix.get(0, 0);
      @SuppressWarnings("unused")
      TransferFunction transferFunction01 = transferFunctionMatrix.get(0, 1);
      TransferFunction transferFunction10 = transferFunctionMatrix.get(1, 0);
      @SuppressWarnings("unused")
      TransferFunction transferFunction11 = transferFunctionMatrix.get(1, 1);
      TransferFunction expectedTransferFunction10 = new TransferFunction(new double[] {1.0}, new double[] {wn * wn,  2.0 * zeta * wn, 1.0 });

      // System.out.println("\nMassSpringDamper:");
      //
      // System.out.print("transferFunction00 = " + transferFunction00);
      // System.out.println("                 transferFunction01 = " + transferFunction01);
      // System.out.print("transferFunction10 = " + transferFunction10);
      // System.out.println("                          transferFunction11 = " + transferFunction11);
      //
      // System.out.println("expectedTransferFunction10 = " + expectedTransferFunction10);

      assertTrue(expectedTransferFunction10.epsilonEquals(transferFunction10, 1e-7));
   }

   @Test
   public void testCircleGenerator()
   {
      ComplexNumber[] leftEigenvectorV = new ComplexNumber[] {new ComplexNumber(1.0, 0.0), new ComplexNumber(0.0, -1.0)};
      ComplexNumber[] rightEigenvectorW = new ComplexNumber[] {new ComplexNumber(0.5, 0.0), new ComplexNumber(0.0, 0.5)};

      ComplexNumber eigenvalue = new ComplexNumber(0.0, 1.0);

      ComplexConjugateMode complexConjugateMode = new ComplexConjugateMode(eigenvalue, leftEigenvectorV, rightEigenvectorW);
      DynamicSystemsTestHelpers.assertEpsilonEquals(eigenvalue, complexConjugateMode.getEigenvalue(), 1e-7);

      TransferFunctionMatrix transferFunctionMatrix = TransferFunctionMatrix.constructTransferFunctionMatrix(complexConjugateMode);

      @SuppressWarnings("unused")
      TransferFunction transferFunction00 = transferFunctionMatrix.get(0, 0);
      TransferFunction transferFunction01 = transferFunctionMatrix.get(0, 1);
      @SuppressWarnings("unused")
      TransferFunction transferFunction10 = transferFunctionMatrix.get(1, 0);
      @SuppressWarnings("unused")
      TransferFunction transferFunction11 = transferFunctionMatrix.get(1, 1);
      TransferFunction expectedTransferFunction01 = new TransferFunction(new double[] {-1.0}, new double[] {1.0, 0.0, 1.0});

      // System.out.println("\nCircleGenerator:");
      //
      // System.out.print("transferFunction00 = " + transferFunction00);
      // System.out.println("        transferFunction01 = " + transferFunction01);
      // System.out.print("transferFunction10 = " + transferFunction10);
      // System.out.println("                   transferFunction11 = " + transferFunction11);
      //
      // System.out.println("expectedTransferFunction01 = " + expectedTransferFunction01);

      assertTrue(expectedTransferFunction01.epsilonEquals(transferFunction01, 1e-7));
   }


   @Test
   public void testPlusTransferFunctionException()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         TransferFunctionMatrix testMatrix = new TransferFunctionMatrix(new TransferFunction[][] {{secondOrderResponseOne, secondOrderResponseTwo}});
         transferFunctionMatrix.plus(testMatrix);
      });
   }
}
