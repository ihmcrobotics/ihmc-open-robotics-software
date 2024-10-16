package us.ihmc.robotics.linearDynamicSystems;

import java.util.Random;

import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.log.LogTools;
import us.ihmc.robotics.dataStructures.ComplexNumber;
import us.ihmc.robotics.math.trajectories.core.Polynomial;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialBasics;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialReadOnly;

import static org.junit.jupiter.api.Assertions.*;

public class LinearDynamicSystemTest
{
   private static final boolean VERBOSE = false;
   private static final double wn = 1.0;
   private static final double zeta = 0.5;
   private SimpleMatrix simpleDecayMatrixA, massSpringDamperMatrixA;
   private LinearDynamicSystem simpleDecaySystem, massSpringDamperSystem;
   private SimpleMatrix simpleNotSquareMatrix;
   private SimpleMatrix simpleSquareMatrix;

   @BeforeEach
   public void setUp() throws Exception
   {
      simpleDecayMatrixA = new SimpleMatrix(new double[][] {{-1.0}});
      simpleDecaySystem = new LinearDynamicSystem(simpleDecayMatrixA, null, null, null);
      massSpringDamperMatrixA = new SimpleMatrix(new double[][] {{-2.0 * zeta * wn, -wn * wn}, {1.0, 0.0}});
      massSpringDamperSystem = new LinearDynamicSystem(massSpringDamperMatrixA, null, null, null);
      simpleNotSquareMatrix = new SimpleMatrix(1, 2);
      simpleSquareMatrix = new SimpleMatrix(2, 2);
   }

   @AfterEach
   public void tearDown() throws Exception
   {
      simpleDecaySystem = null;
      simpleDecayMatrixA = null;
      massSpringDamperMatrixA = null;
      massSpringDamperSystem = null;
   }

   @Test
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
         else
         {
            LogTools.error("WRONG MESSAGE: " + e.getMessage());
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
         double[] initCond = new double[simpleSquareMatrix.numCols() + 1];
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

   @Test
   public void testMCSExampleOne()
   {

      // From Multivariable Control Systems.
      // J. Pratt Problem Set 3.
      double[][] elementsA = new double[][] {{2.0, -2.0, 3.0}, {1.0, 1.0, 1.0}, {1.0, 3.0, -1.0}};
      SimpleMatrix matrixA = new SimpleMatrix(elementsA);
      double[][] elementsB = new double[][] {{0.0, 1.0}, {1.0, 0.0}, {3.0, 2.0}};
      SimpleMatrix matrixB = new SimpleMatrix(elementsB);
      double[][] elementsC = new double[][] {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}};
      SimpleMatrix matrixC = new SimpleMatrix(elementsC);
      LinearDynamicSystem linearDynamicSystem = new LinearDynamicSystem(matrixA, null, null, null);
      TransferFunctionMatrix sIMinusAInverseMatrix = linearDynamicSystem.getTransferFunctionMatrix();

      // Expected transfer functions:
      Polynomial sMinusOne = new Polynomial(-1.0, 1.0);
      Polynomial sMinusThree = new Polynomial(-3.0, 1.0);
      Polynomial sPlusTwo = new Polynomial(2.0, 1.0);

      // Note: There is pole/zero cancellation. Using cofactors for inverse does not currently discover them!
      //      Polynomial denominatorOne = sMinusOne.times(sMinusThree);
      PolynomialBasics denominatorTwo = sMinusOne.times(sMinusThree).times(sPlusTwo);
      Polynomial numerator00 = new Polynomial(false, 1.0, 0.0, -4.0); // new Polynomial(new double[]{1.0, -2.0});
      Polynomial numerator01 = new Polynomial(false, -2.0, 7.0);
      Polynomial numerator02 = new Polynomial(false, 3.0, -5.0);
      Polynomial numerator10 = new Polynomial(false, 1.0, 2.0); // = new Polynomial(new double[]{1.0});
      Polynomial numerator11 = new Polynomial(false, 1.0, -1.0, -5.0);
      Polynomial numerator12 = new Polynomial(false, 1.0, 1.0);
      Polynomial numerator20 = new Polynomial(false, 1.0, 2.0); // new Polynomial(new double[]{1.0});
      Polynomial numerator21 = new Polynomial(false, 3.0, -8.0);
      Polynomial numerator22 = new Polynomial(false, 1.0, -3.0, 4.0);
      TransferFunction t00 = new TransferFunction(numerator00, denominatorTwo); // denominatorOne);
      TransferFunction t01 = new TransferFunction(numerator01, denominatorTwo);
      TransferFunction t02 = new TransferFunction(numerator02, denominatorTwo);
      TransferFunction t10 = new TransferFunction(numerator10, denominatorTwo); // denominatorOne);
      TransferFunction t11 = new TransferFunction(numerator11, denominatorTwo);
      TransferFunction t12 = new TransferFunction(numerator12, denominatorTwo);
      TransferFunction t20 = new TransferFunction(numerator20, denominatorTwo); // denominatorOne);
      TransferFunction t21 = new TransferFunction(numerator21, denominatorTwo);
      TransferFunction t22 = new TransferFunction(numerator22, denominatorTwo);
      TransferFunction[][] expectedSIMinusATransferFunctions = new TransferFunction[][] {{t00, t01, t02}, {t10, t11, t12}, {t20, t21, t22}};
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
      SimpleMatrix identity = SimpleMatrix.identity(3);

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
      Polynomial numeratorG00 = new Polynomial(false, 7.0, -8.0);
      Polynomial numeratorG01 = new Polynomial(false, 1.0, 6.0, -14.0);
      Polynomial numeratorG10 = new Polynomial(false, 1.0, 2.0, -2.0);
      Polynomial numeratorG11 = new Polynomial(false, 3.0, 4.0);
      TransferFunction tG00 = new TransferFunction(numeratorG00, denominatorTwo);
      TransferFunction tG01 = new TransferFunction(numeratorG01, denominatorTwo);
      TransferFunction tG10 = new TransferFunction(numeratorG10, denominatorTwo);
      TransferFunction tG11 = new TransferFunction(numeratorG11, denominatorTwo);
      TransferFunction[][] expectedTransferFunctionsG = new TransferFunction[][] {{tG00, tG01}, {tG10, tG11}};
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

   @Test
   public void testMCSExampleTwo()
   {

      // From Multivariable Control Systems.
      // J. Pratt Problem Set 5.
      double[][] elementsA = new double[][] {{0.0, 0.0, 1.0, 0.0}, {0.0, 0.0, 0.0, 1.0}, {-1.0, 1.0, 0.0, 0.0}, {1.0, -1.0, 0.0, 0.0}};
      SimpleMatrix matrixA = new SimpleMatrix(elementsA);
      double[][] elementsB = new double[][] {{0.0, 0.0}, {0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}};
      SimpleMatrix matrixB = new SimpleMatrix(elementsB);
      double[][] elementsC = new double[][] {{0.0, 1.0, 0.0, 0.0}};
      SimpleMatrix matrixC = new SimpleMatrix(elementsC);
      LinearDynamicSystem linearDynamicSystem = new LinearDynamicSystem(matrixA, null, null, null);
      TransferFunctionMatrix transferFunctionMatrix = linearDynamicSystem.getTransferFunctionMatrix();

      //      System.out.println("transferFunctionMatrix = \n" + transferFunctionMatrix);
      //      System.out.println("\ntransferFunctionMatrix(1, 3) = \n" + transferFunctionMatrix.get(1,3));
      // Expected transfer functions:
      Polynomial denominator = new Polynomial(false, 1.0, 0.0, 2.0, 0.0, 0.0);
      TransferFunction[][] expectedTransferFunctions = new TransferFunction[4][4];

      expectedTransferFunctions[0][0] = new TransferFunction(new Polynomial(false, 1.0, 0.0, 1.0, 0.0), denominator);
      expectedTransferFunctions[0][1] = new TransferFunction(new Polynomial(false, 1.0, 0.0), denominator);
      expectedTransferFunctions[0][2] = new TransferFunction(new Polynomial(false, 1.0, 0.0, 1.0), denominator);
      expectedTransferFunctions[0][3] = new TransferFunction(new Polynomial(false, 1.0), denominator);
      expectedTransferFunctions[1][0] = new TransferFunction(new Polynomial(false, 1.0, 0.0), denominator);
      expectedTransferFunctions[1][1] = new TransferFunction(new Polynomial(false, 1.0, 0.0, 1.0, 0.0), denominator);
      expectedTransferFunctions[1][2] = new TransferFunction(new Polynomial(false, 1.0), denominator);
      expectedTransferFunctions[1][3] = new TransferFunction(new Polynomial(false, 1.0, 0.0, 1.0), denominator);
      expectedTransferFunctions[2][0] = new TransferFunction(new Polynomial(false, -1.0, 0.0, 0.0), denominator);
      expectedTransferFunctions[2][1] = new TransferFunction(new Polynomial(false, 1.0, 0.0, 0.0), denominator);
      expectedTransferFunctions[2][2] = new TransferFunction(new Polynomial(false, 1.0, 0.0, 1.0, 0.0), denominator);
      expectedTransferFunctions[2][3] = new TransferFunction(new Polynomial(false, 1.0, 0.0), denominator);
      expectedTransferFunctions[3][0] = new TransferFunction(new Polynomial(false, 1.0, 0.0, 0.0), denominator);
      expectedTransferFunctions[3][1] = new TransferFunction(new Polynomial(false, -1.0, 0.0, 0.0), denominator);
      expectedTransferFunctions[3][2] = new TransferFunction(new Polynomial(false, 1.0, 0.0), denominator);
      expectedTransferFunctions[3][3] = new TransferFunction(new Polynomial(false, 1.0, 0.0, 1.0, 0.0), denominator);

      TransferFunctionMatrix expectedTransferFunctionMatrix = new TransferFunctionMatrix(expectedTransferFunctions);
      boolean passed = expectedTransferFunctionMatrix.epsilonEquals(transferFunctionMatrix, 1e-7);

      assertTrue(passed);

      Polynomial numerator0 = new Polynomial(false, 1.0);
      Polynomial numerator1 = new Polynomial(false, 1.0, 0.0, 1.0);
      TransferFunction t0 = new TransferFunction(numerator0, denominator);
      TransferFunction t1 = new TransferFunction(numerator1, denominator);
      TransferFunction[][] expectedTransferFunctionsWithBAndC = new TransferFunction[][] {{t0, t1}};

      linearDynamicSystem = new LinearDynamicSystem(matrixA, matrixB, matrixC, null);
      transferFunctionMatrix = linearDynamicSystem.getTransferFunctionMatrix();

      //      System.out.println("transferFunctionMatrix = \n" + transferFunctionMatrix);
      expectedTransferFunctionMatrix = new TransferFunctionMatrix(expectedTransferFunctionsWithBAndC);
      passed = expectedTransferFunctionMatrix.epsilonEquals(transferFunctionMatrix, 1e-7);
      assertTrue(passed);

      // Feedback with only one input.
      // Using pole placement to place poles at -1, -1, -5, -5
      elementsB = new double[][] {{0.0}, {0.0}, {1.0}, {0.0}};
      matrixB = new SimpleMatrix(elementsB);
      linearDynamicSystem = new LinearDynamicSystem(matrixA, matrixB, matrixC, null);

      SimpleMatrix matrixG = new SimpleMatrix(new double[][] {{44.0, -19.0, 12.0, 48.0}});
      LinearDynamicSystem closedLoopSystem = linearDynamicSystem.addFullStateFeedback(matrixG);

      closedLoopSystem.getMatrixA();

      TransferFunctionMatrix closedLoopMatrix = closedLoopSystem.getTransferFunctionMatrix();
      PolynomialReadOnly characteristicEquation = closedLoopMatrix.get(0, 0).getDenominatorPolynomial();

      if (VERBOSE)
      {
         System.out.println("characteristicEquation = " + characteristicEquation);
      }

      // Verify the roots of the characteristic equation are where the poles where place at:
      Polynomial expectedCharacteristicEquation = new Polynomial(false, 1.0, 12.0, 46.0, 60.0, 25.0);

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

   @Test
   public void testSimulateInitialConditions()
   {

      // This test is based on the composition propriety of the transfer state function of a LTI System
      double[] xi = new double[] {0.1, -0.01}; // initial state
      int T1 = 1000; // first time interval
      int T2 = 1000; // second time interval
      int order = massSpringDamperMatrixA.numRows();
      double[] xm = new double[order]; // final state of the first simulation
      double[] xf = new double[order]; // final state of the second simulation
      double[] expectedXf = new double[order]; // final state of the total simulation
      double[][] firstSimulation = massSpringDamperSystem.simulateInitialConditions(xi, 0.001, T1);

      System.arraycopy(firstSimulation[T1 - 1], 0, xm, 0, order);

      // System.out.println("Xm = [" + xm[0] + " " + xm[1] + "]");
      double[][] secondSimulation = massSpringDamperSystem.simulateInitialConditions(xm, 0.001, T2);

      System.arraycopy(secondSimulation[T2 - 1], 0, xf, 0, order);

      // System.out.println("Xf = [" + xf[0] + " " + xf[1] + "]");
      double[][] totalSimulation = massSpringDamperSystem.simulateInitialConditions(xi, 0.001, T1 + T2 - 1);

      System.arraycopy(totalSimulation[T1 + T2 - 2], 0, expectedXf, 0, order);

      // System.out.println("expected Xf = [" + expectedXf[0] + " " + expectedXf[1] + "]");
      assertArrayEquals(expectedXf, xf, 1e-7);
   }

   @Test
   public void testGetTransferFunctionMatrix()
   {
      TransferFunctionMatrix transferFunctions = simpleDecaySystem.getTransferFunctionMatrix();

      assertEquals(1, transferFunctions.getRows());
      assertEquals(1, transferFunctions.getColumns());

      TransferFunction transferFunction = transferFunctions.get(0, 0);
      TransferFunction expectedTransferFunction = new TransferFunction(new double[] {1.0}, new double[] {1.0, 1.0});

      assertTrue(transferFunction.epsilonEquals(expectedTransferFunction, 1e-7));
   }

   @Test
   public void testSimpleDecaySystem()
   {
      verifyLinearDynamicSystem(simpleDecaySystem, simpleDecayMatrixA);
   }

   @Test
   public void testMassSpringDamperSystem()
   {
      verifyLinearDynamicSystem(massSpringDamperSystem, massSpringDamperMatrixA);
   }

   @Test
   public void testRandomLinearDynamicSystems()
   {

      // Given an A Matrix, need to be able to compute T(s) = C(sI-A)^(-1)W
      // Do it two ways and make sure they are the same.
      int numberOfRandomTests = 100;
      Random random = new Random(1776L);

      for (int i = 0; i < numberOfRandomTests; i++)
      {
         SimpleMatrix matrixA = generateRandomMatrix(random, 5);
         LinearDynamicSystem linearDynamicSystem = new LinearDynamicSystem(matrixA, null, null, null);

         verifyLinearDynamicSystem(linearDynamicSystem, matrixA);
      }
   }

   private SimpleMatrix generateRandomMatrix(Random random, int maxOrder)
   {
      int order = random.nextInt(maxOrder) + 1;
      SimpleMatrix ret = new SimpleMatrix(order, order);

      for (int i = 0; i < order; i++)
      {
         for (int j = 0; j < order; j++)
         {
            ret.set(i, j, -100.0 * 200.0 * random.nextDouble());
         }
      }

      return ret;
   }

   private void verifyLinearDynamicSystem(LinearDynamicSystem linearDynamicSystem, SimpleMatrix matrixA)
   {
      TransferFunctionMatrix transferFunctionMatrix = linearDynamicSystem.getTransferFunctionMatrix();
      int order = matrixA.numRows();
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

   @Test
   public void testStateFeedbackMethods()
   {
      double[][] elementsA = new double[][] {{-10.0}};
      SimpleMatrix matrixA = new SimpleMatrix(elementsA);
      double[][] elementsB = new double[][] {{1.0}};
      SimpleMatrix matrixB = new SimpleMatrix(elementsB);
      double[][] elementsC = new double[][] {{1.0}};
      SimpleMatrix matrixC = new SimpleMatrix(elementsC);
      double[][] elementsK = new double[][] {{-10.0}};
      SimpleMatrix matrixK = new SimpleMatrix(elementsK);
      double[][] elementsIC = new double[][] {{0.0}};
      SimpleMatrix matrixIC = new SimpleMatrix(elementsIC);
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

      double[][] elementsKr = new double[][] {{0.5}};
      SimpleMatrix matrixKr = new SimpleMatrix(elementsKr);
      LinearDynamicSystem closedLoopSysB = sysB.addOutputStateFeedback(matrixK, matrixKr);

      y = verifySimulateOutput(closedLoopSysB, matrixIC, u, stepSize, numTicks);
      expectedY = (numTicks * stepSize) * u; // integral between 0 and (numTicks*stepSize) of a constant u

      // System.out.println(expectedY);
      // System.out.println(y);
      assertEquals(expectedY, y, 1e-7);
   }

   // Simulate the Output at T=(stepSize*numTicks) to a constant input u
   private double verifySimulateOutput(LinearDynamicSystem Sys, SimpleMatrix InitialCondition, double input, double stepSize, int numTicks)
   {
      double y;
      SimpleMatrix closedLoopMatrixA = Sys.getMatrixA();
      SimpleMatrix closedLoopMatrixB = Sys.getMatrixB();
      SimpleMatrix closedLoopMatrixC = Sys.getMatrixC();

      // Use Euler integrations for estimate the output of the system
      SimpleMatrix state = new SimpleMatrix(1, 1);

      state = InitialCondition.copy();

      for (int i = 0; i < numTicks; i++)
      {
         SimpleMatrix aTimesX = closedLoopMatrixA.mult(state);
         SimpleMatrix bTimesU = closedLoopMatrixB.scale(input);
         SimpleMatrix dotX = aTimesX.plus(bTimesU);

         state = state.plus(dotX.scale(stepSize));
      }

      y = closedLoopMatrixC.mult(state).get(0, 0);

      return y;
   }

   @Test
   public void testEulerIntegrateSpringDamper()
   {
      double epsilon = 1e-7;
      
      double k = 100.0;
      double b = 10.0;

      double[][] elementsA = new double[][] {{0.0, 1.0}, {-k, -b}};
      double[][] elementsB = new double[][] {{0.0}, {1.0}};
      double[][] elementsC = new double[][] {{1.0, 0.0}};
      double[][] elementsD = new double[][] {{0.0}};

      SimpleMatrix matrixA = new SimpleMatrix(elementsA);
      SimpleMatrix matrixB = new SimpleMatrix(elementsB);
      SimpleMatrix matrixC = new SimpleMatrix(elementsC);
      SimpleMatrix matrixD = new SimpleMatrix(elementsD);

      LinearDynamicSystem system = new LinearDynamicSystem(matrixA, matrixB, matrixC, matrixD);

      double stepSize = 0.001;
      double[] currentState = new double[] {1.0, 0.0};
      double[] input = new double[] {0.0};

      currentState = system.eulerIntegrateOneStep(currentState, input, stepSize);
      double[] output = system.getOutputFromState(currentState, input);

      assertEquals(1, output.length);
      assertEquals(1.0, output[0], epsilon);
      
      currentState = system.eulerIntegrateOneStep(currentState, input, stepSize);
      output = system.getOutputFromState(currentState, input);
      assertEquals(0.9999, output[0], epsilon);
      
      currentState = system.eulerIntegrateOneStep(currentState, input, stepSize);
      output = system.getOutputFromState(currentState, input);
      assertEquals(0.999701, output[0], epsilon);
   }
   
   @Test
   public void testEulerIntegrateMIMO()
   {
      // 4 state variables. 
      // 3 input variables.
      // 2 output variables.
      // A is 4x4
      // B is 4x3
      // C is 2 x 4
      // D is 2 x 3
      
      double epsilon = 1e-7;

      double[][] elementsA = new double[][] {{0.1, 0.2, 0.3, 0.123}, {0.4, 0.5, 0.6, -0.222}, {0.7, 0.8, 0.9, -1.734}, {0.11, 0.22, 0.33, 0.44}};
      double[][] elementsB = new double[][] {{0.17, 0.94, 0.25}, {0.33, 0.55, -0.66}, {-0.17, 0.45, 0.123}, {0.137, 0.15, 0.223}};
      double[][] elementsC = new double[][] {{1.0, 2.0, 3.0, -4.0}, {5.0, 6.0, 7.0, 8.0}};
      double[][] elementsD = new double[][] {{0.123, 0.456, 0.789}, {-0.123, 0.256, -0.189}};

      SimpleMatrix matrixA = new SimpleMatrix(elementsA);
      SimpleMatrix matrixB = new SimpleMatrix(elementsB);
      SimpleMatrix matrixC = new SimpleMatrix(elementsC);
      SimpleMatrix matrixD = new SimpleMatrix(elementsD);

      LinearDynamicSystem system = new LinearDynamicSystem(matrixA, matrixB, matrixC, matrixD);

      double stepSize = 0.001;
      double[] currentState = new double[] {1.0, 0.0, 0.0, 0.0};
      double[] input = new double[] {0.1, 0.2, 0.3};

      currentState = system.eulerIntegrateOneStep(currentState, input, stepSize);
      double[] output = system.getOutputFromState(currentState, input);

      assertEquals(2, output.length);
      assertEquals(1.3428173, output[0], epsilon);
      
      currentState = system.eulerIntegrateOneStep(currentState, input, stepSize);
      output = system.getOutputFromState(currentState, input);
      assertEquals(1.3454372370822003, output[0], epsilon);
      
      currentState = system.eulerIntegrateOneStep(currentState, input, stepSize);
      output = system.getOutputFromState(currentState, input);
      assertEquals(1.348059813331106, output[0], epsilon);
   }
}
