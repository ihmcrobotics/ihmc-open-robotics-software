package us.ihmc.robotics.linearDynamicSystems;

import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.robotics.math.trajectories.core.Polynomial;

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
   public void testPlusTransferFunctionException()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         TransferFunctionMatrix testMatrix = new TransferFunctionMatrix(new TransferFunction[][] {{secondOrderResponseOne, secondOrderResponseTwo}});
         transferFunctionMatrix.plus(testMatrix);
      });
   }
}
