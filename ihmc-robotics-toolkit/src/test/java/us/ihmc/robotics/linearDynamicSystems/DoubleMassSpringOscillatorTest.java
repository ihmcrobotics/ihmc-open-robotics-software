package us.ihmc.robotics.linearDynamicSystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.Test;

import Jama.Matrix;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.ComplexNumber;
import us.ihmc.robotics.dataStructures.Polynomial;

public class DoubleMassSpringOscillatorTest
{
   private static final boolean VERBOSE = true;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDecomposerGotEigenvaluesRight()
   {
      Matrix matrixA = new Matrix(new double[][] { { 0.0, 1.0, 0.0, 0.0 }, { -2.0, 0.0, 1.0, 0.0 }, { 0.0, 0.0, 0.0, 1.0 }, { 1.0, 0.0, -2.0, 0.0 } });
      Matrix matrixB = new Matrix(new double[][] { { 0.0 }, { 1.0 }, { 0.0 }, { 0.0 } });
      if (VERBOSE)
      {
         DynamicSystemsTestHelpers.printMatrix("Double Mass Spring. \n\nA Matrix:", matrixA);
         DynamicSystemsTestHelpers.printMatrix("\nB Matrix:", matrixB);
      }

      EigenvalueDecomposer eigenvalueDecomposerA = new EigenvalueDecomposer(matrixA);

      ComplexNumber[] eigenvaluesFromDecomposer = eigenvalueDecomposerA.getEigenvalues();
      if (VERBOSE)
         DynamicSystemsTestHelpers.printComplexArray("\nEigenvalues:", eigenvaluesFromDecomposer);

      ComplexNumber[] expectedEigenvalues = new ComplexNumber[] { new ComplexNumber(0.0, Math.sqrt(3.0)), new ComplexNumber(0, -Math.sqrt(3.0)), new ComplexNumber(0.0, Math.sqrt(1.0)),
            new ComplexNumber(0, -Math.sqrt(1.0)) };
      if (VERBOSE)
         DynamicSystemsTestHelpers.assertEpsilonEquals(expectedEigenvalues, eigenvaluesFromDecomposer, 1e-7);

      ComplexNumber[][] leftEigenvectorV = eigenvalueDecomposerA.getLeftEigenvectors();
      if (VERBOSE)
         DynamicSystemsTestHelpers.printComplexArray("\nLeft Eigenvectors", leftEigenvectorV);

      ComplexNumber[][] rightEigenvectorW = eigenvalueDecomposerA.getRightEigenvectors();
      if (VERBOSE)
         DynamicSystemsTestHelpers.printComplexArray("\nRight Eigenvectors", rightEigenvectorW);

      ArrayList<SingleRealMode> realModes = eigenvalueDecomposerA.getRealModes();
      if (VERBOSE)
         DynamicSystemsTestHelpers.printRealModeArray("\nReal Modes:", realModes);

      ArrayList<ComplexConjugateMode> complexConjugateModes = eigenvalueDecomposerA.getComplexConjugateModes();
      if (VERBOSE)
         DynamicSystemsTestHelpers.printComplexConjugateModeArray("\nComplex Conjugate Modes:", complexConjugateModes);

      LinearDynamicSystem system = new LinearDynamicSystem(matrixA, matrixB, null, null);
      TransferFunctionMatrix transferFunctionMatrix = system.getTransferFunctionMatrix();

      if (VERBOSE)
      {
         System.out.println("\nTransfer Function Matrix:");
         System.out.println(transferFunctionMatrix);
      }

      DynamicSystemsTestHelpers.checkLeftAndRightEigenvectors(leftEigenvectorV, rightEigenvectorW);

      TransferFunction transferFunction = transferFunctionMatrix.get(0, 0);
      Polynomial denominatorPolynomial = transferFunction.getDenominatorPolynomial();
      Polynomial numeratorPolynomial = transferFunction.getNumeratorPolynomial();

      assertTrue(denominatorPolynomial.epsilonEquals(new Polynomial(new double[] { 1.0, 0.0, 4.0, 0.0, 3.0 }), 1e-7));
      assertTrue(numeratorPolynomial.epsilonEquals(new Polynomial(new double[] { 1.0, 0.0, 2.0 }), 1e-7));

      double stepSize = 0.001;
      int numTicks = (int) (2.0 / stepSize);

      // Mode One:
      double[] initialConditionsOne = new double[] { 0.1, 0.0, -0.1, 0.0 };
      double[][] simulatedDataOne = system.simulateInitialConditions(initialConditionsOne, stepSize, numTicks);
      for (double[] dataPoint : simulatedDataOne)
      {
         assertEquals(dataPoint[0], -dataPoint[2], 1e-7);
      }

      // Mode Two:
      double[] initialConditionsTwo = new double[] { 0.1, 0.0, 0.1, 0.0 };
      double[][] simulatedDataTwo = system.simulateInitialConditions(initialConditionsTwo, stepSize, numTicks);
      for (double[] dataPoint : simulatedDataTwo)
      {
         assertEquals(dataPoint[0], dataPoint[2], 1e-7);
      }

      // Combine Modes One and Two:
      double[] initialConditionsCombined = new double[] { 0.2, 0.0, 0.0, 0.0 };
      double[][] simulatedDataCombined = system.simulateInitialConditions(initialConditionsCombined, stepSize, numTicks);

      for (int i = 0; i < simulatedDataCombined.length; i++)
      {
         double[] dataPointCombined = simulatedDataCombined[i];
         double[] dataPointOne = simulatedDataOne[i];
         double[] dataPointTwo = simulatedDataTwo[i];

         for (int j = 0; j < dataPointCombined.length; j++)
         {
            assertEquals(dataPointCombined[j], dataPointOne[j] + dataPointTwo[j], 1e-7);
         }
      }
   }

}
