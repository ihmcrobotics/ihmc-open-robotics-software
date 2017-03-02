package us.ihmc.robotics.linearDynamicSystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import Jama.Matrix;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.ComplexNumber;
import us.ihmc.robotics.dataStructures.Polynomial;

public class LinearDynamicSystemTest
{
   private static final boolean VERBOSE = false;
   private static final double wn = 1.0;
   private static final double zeta = 0.5;
   private Matrix simpleDecayMatrixA, massSpringDamperMatrixA;
   private LinearDynamicSystem simpleDecaySystem, massSpringDamperSystem;
   private Matrix simpleNotSquareMatrix;
   private Matrix simpleSquareMatrix;

   @Before
   public void setUp() throws Exception
   {
      simpleDecayMatrixA = new Matrix(new double[][] { { -1.0 } });
      simpleDecaySystem = new LinearDynamicSystem(simpleDecayMatrixA, null, null, null);
      massSpringDamperMatrixA = new Matrix(new double[][] { { -2.0 * zeta * wn, -wn * wn }, { 1.0, 0.0 } });
      massSpringDamperSystem = new LinearDynamicSystem(massSpringDamperMatrixA, null, null, null);
      simpleNotSquareMatrix = new Matrix(1, 2, 0);
      simpleSquareMatrix = new Matrix(2, 2, 0);
   }

   @After
   public void tearDown() throws Exception
   {
      simpleDecaySystem = null;
      simpleDecayMatrixA = null;
      massSpringDamperMatrixA = null;
      massSpringDamperSystem = null;
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testException()
   {
      boolean thrown = false;

      // Constructor Tests
      try
      {
         LinearDynamicSystem failedCreation = new LinearDynamicSystem(simpleNotSquareMatrix, null, null, null);
      }
      catch (RuntimeException e)
      {
         if (e.getMessage() == "matrixA must be square!")
         {
            thrown = true;
         }
      }

      assertTrue(thrown);
      thrown = false;

      try
      {
         LinearDynamicSystem failedCreation = new LinearDynamicSystem(null, null, null, null);
      }
      catch (RuntimeException e)
      {
         if (e.getMessage() == "matrixA must be defined. B,C,D can be null")
         {
            thrown = true;
         }
      }

      assertTrue(thrown);

      // addFullStateFeedBack() Test
      thrown = false;

      try
      {
         LinearDynamicSystem LDS = new LinearDynamicSystem(simpleSquareMatrix, null, null, null);

         LDS.addFullStateFeedback(simpleSquareMatrix);
      }
      catch (RuntimeException e)
      {
         if (e.getMessage() == "Matrix B must not be null for addFullStateFeedback!")
         {
            thrown = true;
         }
      }

      assertTrue(thrown);

      // addOutputStateFeedback() Test
      thrown = false;

      try
      {
         LinearDynamicSystem LDS = new LinearDynamicSystem(simpleSquareMatrix, null, null, simpleSquareMatrix);

         LDS.addOutputStateFeedback(simpleSquareMatrix, simpleSquareMatrix);
      }
      catch (RuntimeException e)
      {
         if (e.getMessage() == "Matrix B must not be null for addOutputStateFeedback!")
         {
            thrown = true;
         }
      }

      assertTrue(thrown);
      thrown = false;

      try
      {
         LinearDynamicSystem LDS = new LinearDynamicSystem(simpleSquareMatrix, simpleSquareMatrix, simpleSquareMatrix, simpleSquareMatrix);

         LDS.addOutputStateFeedback(simpleSquareMatrix, simpleSquareMatrix);
      }
      catch (RuntimeException e)
      {
         if (e.getMessage() == "Matrix D must be null for addOutputStateFeedback!")
         {
            thrown = true;
         }
      }

      assertTrue(thrown);
      thrown = false;

      try
      {
         LinearDynamicSystem LDS = new LinearDynamicSystem(simpleSquareMatrix, simpleSquareMatrix, null, null);

         LDS.addOutputStateFeedback(simpleSquareMatrix, simpleSquareMatrix);
      }
      catch (RuntimeException e)
      {
         if (e.getMessage() == "Matrix C must not be null for addOutputStateFeedback!")
         {
            thrown = true;
         }
      }

      assertTrue(thrown);
      thrown = false;

      try
      {
         LinearDynamicSystem LDS = new LinearDynamicSystem(simpleSquareMatrix, simpleSquareMatrix, simpleSquareMatrix, simpleSquareMatrix);
         double[] initCond = new double[simpleSquareMatrix.getColumnDimension() + 1];
         Random r = new Random();

         LDS.simulateInitialConditions(initCond, r.nextDouble(), r.nextInt());
      }
      catch (RuntimeException e)
      {
         if (e.getMessage() == "initialConditions.length != order")
         {
            thrown = true;
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMCSExampleOne()
   {

      // From Multivariable Control Systems.
      // J. Pratt Problem Set 3.
      double[][] elementsA = new double[][] { { 2.0, -2.0, 3.0 }, { 1.0, 1.0, 1.0 }, { 1.0, 3.0, -1.0 } };
      Matrix matrixA = new Matrix(elementsA);
      double[][] elementsB = new double[][] { { 0.0, 1.0 }, { 1.0, 0.0 }, { 3.0, 2.0 } };
      Matrix matrixB = new Matrix(elementsB);
      double[][] elementsC = new double[][] { { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 } };
      Matrix matrixC = new Matrix(elementsC);
      LinearDynamicSystem linearDynamicSystem = new LinearDynamicSystem(matrixA, null, null, null);
      TransferFunctionMatrix sIMinusAInverseMatrix = linearDynamicSystem.getTransferFunctionMatrix();

      // Expected transfer functions:
      Polynomial sMinusOne = new Polynomial(new double[] { 1.0, -1.0 });
      Polynomial sMinusThree = new Polynomial(new double[] { 1.0, -3.0 });
      Polynomial sPlusTwo = new Polynomial(new double[] { 1.0, 2.0 });

      // Note: There is pole/zero cancellation. Using cofactors for inverse does not currently discover them!
      //      Polynomial denominatorOne = sMinusOne.times(sMinusThree);
      Polynomial denominatorTwo = sMinusOne.times(sMinusThree).times(sPlusTwo);
      Polynomial numerator00 = new Polynomial(1.0, 0.0, -4.0); // new Polynomial(new double[]{1.0, -2.0});
      Polynomial numerator01 = new Polynomial(-2.0, 7.0);
      Polynomial numerator02 = new Polynomial(3.0, -5.0);
      Polynomial numerator10 = new Polynomial(1.0, 2.0); // = new Polynomial(new double[]{1.0});
      Polynomial numerator11 = new Polynomial(1.0, -1.0, -5.0);
      Polynomial numerator12 = new Polynomial(1.0, 1.0);
      Polynomial numerator20 = new Polynomial(1.0, 2.0); // new Polynomial(new double[]{1.0});
      Polynomial numerator21 = new Polynomial(3.0, -8.0);
      Polynomial numerator22 = new Polynomial(1.0, -3.0, 4.0);
      TransferFunction t00 = new TransferFunction(numerator00, denominatorTwo); // denominatorOne);
      TransferFunction t01 = new TransferFunction(numerator01, denominatorTwo);
      TransferFunction t02 = new TransferFunction(numerator02, denominatorTwo);
      TransferFunction t10 = new TransferFunction(numerator10, denominatorTwo); // denominatorOne);
      TransferFunction t11 = new TransferFunction(numerator11, denominatorTwo);
      TransferFunction t12 = new TransferFunction(numerator12, denominatorTwo);
      TransferFunction t20 = new TransferFunction(numerator20, denominatorTwo); // denominatorOne);
      TransferFunction t21 = new TransferFunction(numerator21, denominatorTwo);
      TransferFunction t22 = new TransferFunction(numerator22, denominatorTwo);
      TransferFunction[][] expectedSIMinusATransferFunctions = new TransferFunction[][] { { t00, t01, t02 }, { t10, t11, t12 }, { t20, t21, t22 } };
      TransferFunctionMatrix expectedSIMinusAInverseMatrix = new TransferFunctionMatrix(expectedSIMinusATransferFunctions);
      ComplexNumber complexNumber = new ComplexNumber(0.678, 1.234);
      ComplexMatrix evaluate1 = expectedSIMinusAInverseMatrix.evaluate(complexNumber);
      ComplexMatrix evaluate2 = sIMinusAInverseMatrix.evaluate(complexNumber);

      // System.out.println("sIMinusAInverseMatrix = " + sIMinusAInverseMatrix);
      // System.out.println("expectedSIMinusAInverseMatrix = " + expectedSIMinusAInverseMatrix);
      assertTrue(evaluate1.epsilonEquals(evaluate2, 1e-7));

      boolean passed = sIMinusAInverseMatrix.epsilonEquals(expectedSIMinusAInverseMatrix, 1e-7);

      assertTrue(passed);

      // Test multiplying by Identity:
      Matrix identity = Matrix.identity(3, 3);

      linearDynamicSystem.setMatrixB(identity);
      linearDynamicSystem.setMatrixC(identity);

      TransferFunctionMatrix sIMinusAInverseMatrix2 = linearDynamicSystem.getTransferFunctionMatrix();

      passed = sIMinusAInverseMatrix.epsilonEquals(sIMinusAInverseMatrix2, 1e-7);

      // Test with B and C:
      assertTrue(passed);
      linearDynamicSystem.setMatrixB(matrixB);
      linearDynamicSystem.setMatrixC(matrixC);
      linearDynamicSystem.setMatrixB(matrixB);
      linearDynamicSystem.setMatrixC(matrixC);

      TransferFunctionMatrix matrixG = linearDynamicSystem.getTransferFunctionMatrix();
      Polynomial numeratorG00 = new Polynomial(new double[] { 7.0, -8.0 });
      Polynomial numeratorG01 = new Polynomial(new double[] { 1.0, 6.0, -14.0 });
      Polynomial numeratorG10 = new Polynomial(new double[] { 1.0, 2.0, -2.0 });
      Polynomial numeratorG11 = new Polynomial(new double[] { 3.0, 4.0 });
      TransferFunction tG00 = new TransferFunction(numeratorG00, denominatorTwo);
      TransferFunction tG01 = new TransferFunction(numeratorG01, denominatorTwo);
      TransferFunction tG10 = new TransferFunction(numeratorG10, denominatorTwo);
      TransferFunction tG11 = new TransferFunction(numeratorG11, denominatorTwo);
      TransferFunction[][] expectedTransferFunctionsG = new TransferFunction[][] { { tG00, tG01 }, { tG10, tG11 } };
      TransferFunctionMatrix expectedMatrixG = new TransferFunctionMatrix(expectedTransferFunctionsG);

      // System.out.println("expectedMatrixG = " + expectedMatrixG);
      // System.out.println("matrixG = " + matrixG);
      // The terms aren't reduced. Verify they equal by evaluating them:
      complexNumber = new ComplexNumber(0.0, 0.0);

      ComplexMatrix evaluateMatrixG = matrixG.evaluate(complexNumber);
      ComplexMatrix evaluateExpectedMatrixG = expectedMatrixG.evaluate(complexNumber);

      // System.out.println("evaluateExpectedMatrixG = " + evaluateExpectedMatrixG);
      // System.out.println("evaluateMatrixG = " + evaluateMatrixG);
      passed = evaluateMatrixG.epsilonEquals(evaluateExpectedMatrixG, 1e-1);
      assertTrue(passed);

      // @todo: Implement reduction and/or improve epsilonEquals for Transfer Functions and transfer function matrices.
      passed = matrixG.epsilonEquals(expectedMatrixG, 1e-7);

      // assertTrue(passed);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMCSExampleTwo()
   {

      // From Multivariable Control Systems.
      // J. Pratt Problem Set 5.
      double[][] elementsA = new double[][] { { 0.0, 0.0, 1.0, 0.0 }, { 0.0, 0.0, 0.0, 1.0 }, { -1.0, 1.0, 0.0, 0.0 }, { 1.0, -1.0, 0.0, 0.0 } };
      Matrix matrixA = new Matrix(elementsA);
      double[][] elementsB = new double[][] { { 0.0, 0.0 }, { 0.0, 0.0 }, { 1.0, 0.0 }, { 0.0, 1.0 } };
      Matrix matrixB = new Matrix(elementsB);
      double[][] elementsC = new double[][] { { 0.0, 1.0, 0.0, 0.0 } };
      Matrix matrixC = new Matrix(elementsC);
      LinearDynamicSystem linearDynamicSystem = new LinearDynamicSystem(matrixA, null, null, null);
      TransferFunctionMatrix transferFunctionMatrix = linearDynamicSystem.getTransferFunctionMatrix();

      //      System.out.println("transferFunctionMatrix = \n" + transferFunctionMatrix);
      //      System.out.println("\ntransferFunctionMatrix(1, 3) = \n" + transferFunctionMatrix.get(1,3));
      // Expected transfer functions:
      Polynomial denominator = new Polynomial(new double[] { 1.0, 0.0, 2.0, 0.0, 0.0 });
      TransferFunction[][] expectedTransferFunctions = new TransferFunction[4][4];

      expectedTransferFunctions[0][0] = new TransferFunction(new Polynomial(1.0, 0.0, 1.0, 0.0), denominator);
      expectedTransferFunctions[0][1] = new TransferFunction(new Polynomial(1.0, 0.0), denominator);
      expectedTransferFunctions[0][2] = new TransferFunction(new Polynomial(1.0, 0.0, 1.0), denominator);
      expectedTransferFunctions[0][3] = new TransferFunction(new Polynomial(1.0), denominator);
      expectedTransferFunctions[1][0] = new TransferFunction(new Polynomial(1.0, 0.0), denominator);
      expectedTransferFunctions[1][1] = new TransferFunction(new Polynomial(1.0, 0.0, 1.0, 0.0), denominator);
      expectedTransferFunctions[1][2] = new TransferFunction(new Polynomial(1.0), denominator);
      expectedTransferFunctions[1][3] = new TransferFunction(new Polynomial(1.0, 0.0, 1.0), denominator);
      expectedTransferFunctions[2][0] = new TransferFunction(new Polynomial(-1.0, 0.0, 0.0), denominator);
      expectedTransferFunctions[2][1] = new TransferFunction(new Polynomial(1.0, 0.0, 0.0), denominator);
      expectedTransferFunctions[2][2] = new TransferFunction(new Polynomial(1.0, 0.0, 1.0, 0.0), denominator);
      expectedTransferFunctions[2][3] = new TransferFunction(new Polynomial(1.0, 0.0), denominator);
      expectedTransferFunctions[3][0] = new TransferFunction(new Polynomial(1.0, 0.0, 0.0), denominator);
      expectedTransferFunctions[3][1] = new TransferFunction(new Polynomial(-1.0, 0.0, 0.0), denominator);
      expectedTransferFunctions[3][2] = new TransferFunction(new Polynomial(1.0, 0.0), denominator);
      expectedTransferFunctions[3][3] = new TransferFunction(new Polynomial(1.0, 0.0, 1.0, 0.0), denominator);

      TransferFunctionMatrix expectedTransferFunctionMatrix = new TransferFunctionMatrix(expectedTransferFunctions);
      boolean passed = expectedTransferFunctionMatrix.epsilonEquals(transferFunctionMatrix, 1e-7);

      assertTrue(passed);

      Polynomial numerator0 = new Polynomial(new double[] { 1.0 });
      Polynomial numerator1 = new Polynomial(new double[] { 1.0, 0.0, 1.0 });
      TransferFunction t0 = new TransferFunction(numerator0, denominator);
      TransferFunction t1 = new TransferFunction(numerator1, denominator);
      TransferFunction[][] expectedTransferFunctionsWithBAndC = new TransferFunction[][] { { t0, t1 } };

      linearDynamicSystem = new LinearDynamicSystem(matrixA, matrixB, matrixC, null);
      transferFunctionMatrix = linearDynamicSystem.getTransferFunctionMatrix();

      //      System.out.println("transferFunctionMatrix = \n" + transferFunctionMatrix);
      expectedTransferFunctionMatrix = new TransferFunctionMatrix(expectedTransferFunctionsWithBAndC);
      passed = expectedTransferFunctionMatrix.epsilonEquals(transferFunctionMatrix, 1e-7);
      assertTrue(passed);

      // Feedback with only one input.
      // Using pole placement to place poles at -1, -1, -5, -5
      elementsB = new double[][] { { 0.0 }, { 0.0 }, { 1.0 }, { 0.0 } };
      matrixB = new Matrix(elementsB);
      linearDynamicSystem = new LinearDynamicSystem(matrixA, matrixB, matrixC, null);

      Matrix matrixG = new Matrix(new double[][] { { 44.0, -19.0, 12.0, 48.0 } });
      LinearDynamicSystem closedLoopSystem = linearDynamicSystem.addFullStateFeedback(matrixG);

      closedLoopSystem.getMatrixA();

      TransferFunctionMatrix closedLoopMatrix = closedLoopSystem.getTransferFunctionMatrix();
      Polynomial characteristicEquation = closedLoopMatrix.get(0, 0).getDenominatorPolynomial();

      if (VERBOSE)
      {
         System.out.println("characteristicEquation = " + characteristicEquation);
      }

      // Verify the roots of the characteristic equation are where the poles where place at:
      Polynomial expectedCharacteristicEquation = new Polynomial(new double[] { 1.0, 12.0, 46.0, 60.0, 25.0 });

      assertTrue(characteristicEquation.epsilonEquals(expectedCharacteristicEquation, 1e-7));

      // How to automatically test simulateInitialConditions???
      //      int numTicks = 1000;
      //      double stepSize = 0.001;
      //      double[] initialConditions = new double[]{0.0, 1.0, 0.0, 0.0};
      //      double[][] simulation = closedLoopSystem.simulateInitialConditions(initialConditions, stepSize, numTicks);
      //
      //      for (int i=0; i<simulation.length; i++)
      //      {
      //       System.out.println(simulation[i][2]);
      //      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimulateInitialConditions()
   {

      // This test is based on the composition propriety of the transfer state function of a LTI System
      double[] xi = new double[] { 0.1, -0.01 }; // initial state
      int T1 = 1000; // first time interval
      int T2 = 1000; // second time interval
      int order = massSpringDamperMatrixA.getRowDimension();
      double[] xm = new double[order]; // final state of the first simulation
      double[] xf = new double[order]; // final state of the second simulation
      double[] expectedXf = new double[order]; // final state of the total simulation
      double[][] firstSimulation = massSpringDamperSystem.simulateInitialConditions(xi, 0.001, T1);

      for (int i = 0; i < order; i++)
      {
         xm[i] = firstSimulation[T1 - 1][i];
      }

      // System.out.println("Xm = [" + xm[0] + " " + xm[1] + "]");
      double[][] secondSimulation = massSpringDamperSystem.simulateInitialConditions(xm, 0.001, T2);

      for (int i = 0; i < order; i++)
      {
         xf[i] = secondSimulation[T2 - 1][i];
      }

      // System.out.println("Xf = [" + xf[0] + " " + xf[1] + "]");
      double[][] totalSimulation = massSpringDamperSystem.simulateInitialConditions(xi, 0.001, T1 + T2 - 1);

      for (int i = 0; i < order; i++)
      {
         expectedXf[i] = totalSimulation[T1 + T2 - 2][i];
      }

      // System.out.println("expected Xf = [" + expectedXf[0] + " " + expectedXf[1] + "]");
      for (int i = 0; i < order; i++)
      {
         assertEquals(expectedXf[i], xf[i], 1e-7);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetTransferFunctionMatrix()
   {
      TransferFunctionMatrix transferFunctions = simpleDecaySystem.getTransferFunctionMatrix();

      assertEquals(1, transferFunctions.getRows());
      assertEquals(1, transferFunctions.getColumns());

      TransferFunction transferFunction = transferFunctions.get(0, 0);
      TransferFunction expectedTransferFunction = new TransferFunction(new double[] { 1.0 }, new double[] { 1.0, 1.0 });

      assertTrue(transferFunction.epsilonEquals(expectedTransferFunction, 1e-7));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleDecaySystem()
   {
      verifyLinearDynamicSystem(simpleDecaySystem, simpleDecayMatrixA);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMassSpringDamperSystem()
   {
      verifyLinearDynamicSystem(massSpringDamperSystem, massSpringDamperMatrixA);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.6)
   @Test(timeout = 30000)
   public void testRandomLinearDynamicSystems()
   {

      // Given an A Matrix, need to be able to compute T(s) = C(sI-A)^(-1)W
      // Do it two ways and make sure they are the same.
      int numberOfRandomTests = 100;
      Random random = new Random(1776L);

      for (int i = 0; i < numberOfRandomTests; i++)
      {
         Matrix matrixA = generateRandomMatrix(random, 5);
         LinearDynamicSystem linearDynamicSystem = new LinearDynamicSystem(matrixA, null, null, null);

         verifyLinearDynamicSystem(linearDynamicSystem, matrixA);
      }
   }

   private Matrix generateRandomMatrix(Random random, int maxOrder)
   {
      int order = random.nextInt(maxOrder) + 1;
      Matrix ret = new Matrix(order, order);

      for (int i = 0; i < order; i++)
      {
         for (int j = 0; j < order; j++)
         {
            ret.set(i, j, -100.0 * 200.0 * random.nextDouble());
         }
      }

      return ret;
   }

   private void verifyLinearDynamicSystem(LinearDynamicSystem linearDynamicSystem, Matrix matrixA)
   {
      TransferFunctionMatrix transferFunctionMatrix = linearDynamicSystem.getTransferFunctionMatrix();
      int order = matrixA.getRowDimension();
      ComplexMatrix identity = ComplexMatrix.constructIdentity(order);

      // Sweep through a frequency range and make sure that T(jw) is equal to C(sI-A)^(-1)W
      for (double omega = 0.0; omega < 100.0; omega = omega + 0.1)
      {
         ComplexNumber jOmega = new ComplexNumber(0.0, omega);
         ComplexMatrix sIMinusA = identity.times(jOmega).minus(matrixA);
         ComplexMatrix sIMinusAInverse = sIMinusA.inverse();
         ComplexMatrix transferFunctionAtJOmega = transferFunctionMatrix.evaluate(jOmega);
         boolean testPassed = sIMinusAInverse.epsilonEquals(transferFunctionAtJOmega, 1.0e-1);

         if (!testPassed)
         {
            System.out.println("sIMinusAInverse = " + sIMinusAInverse);
            System.out.println("transferFunctionAtJOmega = " + transferFunctionAtJOmega);
         }

         assertTrue(testPassed);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testStateFeedbackMethods()
   {
      double[][] elementsA = new double[][] { { -10.0 } };
      Matrix matrixA = new Matrix(elementsA);
      double[][] elementsB = new double[][] { { 1.0 } };
      Matrix matrixB = new Matrix(elementsB);
      double[][] elementsC = new double[][] { { 1.0 } };
      Matrix matrixC = new Matrix(elementsC);
      double[][] elementsK = new double[][] { { -10.0 } };
      Matrix matrixK = new Matrix(elementsK);
      double[][] elementsIC = new double[][] { { 0.0 } };
      Matrix matrixIC = new Matrix(elementsIC);
      double u = 1.5; // system input
      double y; // system output
      int numTicks = 4000;
      double stepSize = 0.001;

      // sysA is a simple first order system TIPO 0  dX=-10*x + u : y=x
      LinearDynamicSystem sysA = new LinearDynamicSystem(matrixA, matrixB, matrixC, null);

      // closedLoopSysA is expected to be a pure integrator dX=u : y=x
      LinearDynamicSystem closedLoopSysA = sysA.addOutputStateFeedback(matrixK);

      y = verifySimulateOutput(closedLoopSysA, matrixIC, u, stepSize, numTicks);

      double expectedY = (numTicks * stepSize) * u; // integral between 0 and (numTicks*stepSize) of a constant u

      // System.out.println(expectedY);
      // System.out.println(y);
      assertEquals(expectedY, y, 1e-7);

      // sysB is a simple first order system TIPO 0  dX=-10*x + 2*u : y=x
      matrixA.set(0, 0, -10.0);
      matrixB.set(0, 0, 2.0);
      matrixC.set(0, 0, 1.0);

      LinearDynamicSystem sysB = new LinearDynamicSystem(matrixA, matrixB, matrixC, null);

      // closedLoopSysA is expected to be a pure integrator dX=u : y=x
      matrixK.set(0, 0, -5.0);

      double[][] elementsKr = new double[][] { { 0.5 } };
      Matrix matrixKr = new Matrix(elementsKr);
      LinearDynamicSystem closedLoopSysB = sysB.addOutputStateFeedback(matrixK, matrixKr);

      y = verifySimulateOutput(closedLoopSysB, matrixIC, u, stepSize, numTicks);
      expectedY = (numTicks * stepSize) * u; // integral between 0 and (numTicks*stepSize) of a constant u

      // System.out.println(expectedY);
      // System.out.println(y);
      assertEquals(expectedY, y, 1e-7);
   }

   // Simulate the Output at T=(stepSize*numTicks) to a constant input u
   private double verifySimulateOutput(LinearDynamicSystem Sys, Matrix InitialCondition, double input, double stepSize, int numTicks)
   {
      double y;
      Matrix closedLoopMatrixA = Sys.getMatrixA();
      Matrix closedLoopMatrixB = Sys.getMatrixB();
      Matrix closedLoopMatrixC = Sys.getMatrixC();

      // Use Euler integrations for estimate the output of the system
      Matrix state = new Matrix(1, 1);

      state = InitialCondition.copy();

      for (int i = 0; i < numTicks; i++)
      {
         Matrix aTimesX = closedLoopMatrixA.times(state);
         Matrix bTimesU = closedLoopMatrixB.times(input);
         Matrix dotX = aTimesX.plus(bTimesU);

         state = state.plus(dotX.times(stepSize));
      }

      y = closedLoopMatrixC.times(state).get(0, 0);

      return y;
   }
}
