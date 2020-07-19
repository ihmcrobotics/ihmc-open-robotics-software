package us.ihmc.commonWalkingControlModules.capturePoint.optimization.qpInput;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ICPQPConstraintCalculatorTest
{
   private final static int iters = 100;
   private final static double epsilon = 1e-7;

   @Test
   public void testFeedbackMaxValueConstraint()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoRegistry("dummy"));
      ICPQPConstraintCalculator inputCalculator = new ICPQPConstraintCalculator(indexHandler);
      ICPInequalityInput inequalityConstraint = new ICPInequalityInput(10, 10);

      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         indexHandler.computeProblemSize();
         double maxXValue = RandomNumbers.nextDouble(random, 0.01, 10.0);
         double maxYValue = RandomNumbers.nextDouble(random, 0.01, 10.0);

         ICPInequalityInput expectedConstraint = new ICPInequalityInput(4, 2);

         inputCalculator.calculateMaxFeedbackMagnitudeConstraint(inequalityConstraint, maxXValue, maxYValue);

         testInequalityConstraint(expectedConstraint, inequalityConstraint, random, maxXValue, maxYValue, iter);
      }
   }

   @Test
   public void testFeedbackMaxValueConstraintWithCMP()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoRegistry("dummy"));
      ICPQPConstraintCalculator inputCalculator = new ICPQPConstraintCalculator(indexHandler);
      ICPInequalityInput inequalityConstraint = new ICPInequalityInput(10, 10);
      indexHandler.setHasCMPFeedbackTask(true);

      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         indexHandler.computeProblemSize();
         double maxXValue = RandomNumbers.nextDouble(random, 0.01, 10.0);
         double maxYValue = RandomNumbers.nextDouble(random, 0.01, 10.0);

         ICPInequalityInput expectedConstraint = new ICPInequalityInput(4, 4);

         inputCalculator.calculateMaxFeedbackMagnitudeConstraint(inequalityConstraint, maxXValue, maxYValue);

         testInequalityConstraintWithCMP(expectedConstraint, inequalityConstraint, random, maxXValue, maxYValue, iter);
      }
   }

   @Test
   public void testFeedbackMaxValueConstraintWithInfiniteLimits()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoRegistry("dummy"));
      ICPQPConstraintCalculator inputCalculator = new ICPQPConstraintCalculator(indexHandler);
      ICPInequalityInput inequalityConstraint = new ICPInequalityInput(10, 10);

      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         indexHandler.setHasCMPFeedbackTask(false);
         indexHandler.computeProblemSize();

         double maxXValue = Double.POSITIVE_INFINITY;
         double maxYValue = RandomNumbers.nextDouble(random, 0.01, 10.0);

         ICPInequalityInput expectedConstraint = new ICPInequalityInput(2, 2);

         inputCalculator.calculateMaxFeedbackMagnitudeConstraint(inequalityConstraint, maxXValue, maxYValue);

         expectedConstraint.Aineq.set(0, 1, 1);
         expectedConstraint.Aineq.set(1, 1, -1);

         expectedConstraint.bineq.set(0, 0, maxYValue);
         expectedConstraint.bineq.set(1, 0, maxYValue);

         assertConstraintsEqual("", expectedConstraint, inequalityConstraint);

         expectedConstraint.reset();

         maxXValue = RandomNumbers.nextDouble(random, 0.01, 10.0);
         maxYValue = Double.POSITIVE_INFINITY;

         inputCalculator.calculateMaxFeedbackMagnitudeConstraint(inequalityConstraint, maxXValue, maxYValue);

         expectedConstraint.Aineq.set(0, 0, 1);
         expectedConstraint.Aineq.set(1, 0, -1);

         expectedConstraint.bineq.set(0, 0, maxXValue);
         expectedConstraint.bineq.set(1, 0, maxXValue);

         assertConstraintsEqual("", expectedConstraint, inequalityConstraint);

         expectedConstraint.reset();
         expectedConstraint.reshape(0, 2);

         maxXValue = Double.POSITIVE_INFINITY;
         maxYValue = Double.POSITIVE_INFINITY;

         inputCalculator.calculateMaxFeedbackMagnitudeConstraint(inequalityConstraint, maxXValue, maxYValue);

         assertConstraintsEqual("", expectedConstraint, inequalityConstraint);

         indexHandler.setHasCMPFeedbackTask(true);
         indexHandler.computeProblemSize();

         maxXValue = Double.POSITIVE_INFINITY;
         maxYValue = RandomNumbers.nextDouble(random, 0.01, 10.0);

         expectedConstraint = new ICPInequalityInput(2, 4);

         inputCalculator.calculateMaxFeedbackMagnitudeConstraint(inequalityConstraint, maxXValue, maxYValue);

         expectedConstraint.Aineq.set(0, 1, 1);
         expectedConstraint.Aineq.set(0, 3, 1);
         expectedConstraint.Aineq.set(1, 1, -1);
         expectedConstraint.Aineq.set(1, 3, -1);

         expectedConstraint.bineq.set(0, 0, maxYValue);
         expectedConstraint.bineq.set(1, 0, maxYValue);

         assertConstraintsEqual("", expectedConstraint, inequalityConstraint);

         expectedConstraint.reset();

         maxXValue = RandomNumbers.nextDouble(random, 0.01, 10.0);
         maxYValue = Double.POSITIVE_INFINITY;

         inputCalculator.calculateMaxFeedbackMagnitudeConstraint(inequalityConstraint, maxXValue, maxYValue);

         expectedConstraint.Aineq.set(0, 0, 1);
         expectedConstraint.Aineq.set(0, 2, 1);
         expectedConstraint.Aineq.set(1, 0, -1);
         expectedConstraint.Aineq.set(1, 2, -1);

         expectedConstraint.bineq.set(0, 0, maxXValue);
         expectedConstraint.bineq.set(1, 0, maxXValue);

         assertConstraintsEqual("", expectedConstraint, inequalityConstraint);

         expectedConstraint.reset();
         expectedConstraint.reshape(0, 4);

         maxXValue = Double.POSITIVE_INFINITY;
         maxYValue = Double.POSITIVE_INFINITY;

         inputCalculator.calculateMaxFeedbackMagnitudeConstraint(inequalityConstraint, maxXValue, maxYValue);

         assertConstraintsEqual("", expectedConstraint, inequalityConstraint);
      }
   }

   @Test
   public void testFeedbackMaxRateConstraint()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoRegistry("dummy"));
      ICPQPConstraintCalculator inputCalculator = new ICPQPConstraintCalculator(indexHandler);
      ICPInequalityInput inequalityConstraint = new ICPInequalityInput(10, 10);

      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         indexHandler.computeProblemSize();
         double controlDT = RandomNumbers.nextDouble(random, 1e-6, 1e-2);

         double maxXRate = RandomNumbers.nextDouble(random, 0.01, 10.0);
         double maxYRate = RandomNumbers.nextDouble(random, 0.01, 10.0);

         double previousXValue = RandomNumbers.nextDouble(random, 0.01, 10.0);
         double previousYValue = RandomNumbers.nextDouble(random, 0.01, 10.0);

         ICPInequalityInput expectedConstraint = new ICPInequalityInput(4, 2);

         inputCalculator.calculateMaxFeedbackRateConstraint(inequalityConstraint, maxXRate, maxYRate, previousXValue, previousYValue, controlDT);

         testInequalityRateConstraint(expectedConstraint, inequalityConstraint, random, previousXValue,previousYValue, maxXRate, maxYRate, controlDT, iter);
      }
   }

   @Test
   public void testFeedbackMaxRateConstraintWithCMP()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoRegistry("dummy"));
      ICPQPConstraintCalculator inputCalculator = new ICPQPConstraintCalculator(indexHandler);
      ICPInequalityInput inequalityConstraint = new ICPInequalityInput(10, 10);
      indexHandler.setHasCMPFeedbackTask(true);

      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         indexHandler.computeProblemSize();
         double controlDT = RandomNumbers.nextDouble(random, 1e-6, 1e-2);

         double maxXRate = RandomNumbers.nextDouble(random, 0.01, 10.0);
         double maxYRate = RandomNumbers.nextDouble(random, 0.01, 10.0);

         double previousXValue = RandomNumbers.nextDouble(random, 0.01, 10.0);
         double previousYValue = RandomNumbers.nextDouble(random, 0.01, 10.0);

         ICPInequalityInput expectedConstraint = new ICPInequalityInput(4, 4);

         inputCalculator.calculateMaxFeedbackRateConstraint(inequalityConstraint, maxXRate, maxYRate, previousXValue, previousYValue, controlDT);

         testInequalityRateConstraintWithCMP(expectedConstraint, inequalityConstraint, random, previousXValue, previousYValue, maxXRate, maxYRate, controlDT, iter);
      }
   }

   @Test
   public void testFeedbackMaxRateConstraintWithInfiniteLimits()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler(new YoRegistry("dummy"));
      ICPQPConstraintCalculator inputCalculator = new ICPQPConstraintCalculator(indexHandler);
      ICPInequalityInput inequalityConstraint = new ICPInequalityInput(10, 10);

      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         indexHandler.setHasCMPFeedbackTask(false);
         indexHandler.computeProblemSize();

         double controlDT = RandomNumbers.nextDouble(random, 1e-6, 1e-2);

         double maxXRate = Double.POSITIVE_INFINITY;
         double maxYRate = RandomNumbers.nextDouble(random, 0.01, 10.0);

         double previousXValue = RandomNumbers.nextDouble(random, 0.01, 10.0);
         double previousYValue = RandomNumbers.nextDouble(random, 0.01, 10.0);

         double maxYValue = previousYValue + controlDT * maxYRate;
         double minYValue = previousYValue - controlDT * maxYRate;

         ICPInequalityInput expectedConstraint = new ICPInequalityInput(2, 2);

         inputCalculator.calculateMaxFeedbackRateConstraint(inequalityConstraint, maxXRate, maxYRate, previousXValue, previousYValue, controlDT);

         expectedConstraint.Aineq.set(0, 1, 1);
         expectedConstraint.Aineq.set(1, 1, -1);

         expectedConstraint.bineq.set(0, 0, maxYValue);
         expectedConstraint.bineq.set(1, 0, -minYValue);

         assertConstraintsEqual("", expectedConstraint, inequalityConstraint);

         controlDT = RandomNumbers.nextDouble(random, 1e-6, 1e-2);

         maxXRate = RandomNumbers.nextDouble(random, 0.01, 10.0);
         maxYRate = RandomNumbers.nextDouble(random, 0.01, 10.0);

         previousXValue = Double.POSITIVE_INFINITY;
         previousYValue = RandomNumbers.nextDouble(random, 0.01, 10.0);

         maxYValue = previousYValue + controlDT * maxYRate;
         minYValue = previousYValue - controlDT * maxYRate;

         expectedConstraint.reset();

         inputCalculator.calculateMaxFeedbackRateConstraint(inequalityConstraint, maxXRate, maxYRate, previousXValue, previousYValue, controlDT);

         expectedConstraint.Aineq.set(0, 1, 1);
         expectedConstraint.Aineq.set(1, 1, -1);

         expectedConstraint.bineq.set(0, 0, maxYValue);
         expectedConstraint.bineq.set(1, 0, -minYValue);

         assertConstraintsEqual("", expectedConstraint, inequalityConstraint);

         expectedConstraint.reset();

         controlDT = RandomNumbers.nextDouble(random, 1e-6, 1e-2);

         maxXRate = RandomNumbers.nextDouble(random, 0.01, 10.0);
         maxYRate = Double.POSITIVE_INFINITY;

         previousXValue = RandomNumbers.nextDouble(random, 0.01, 10.0);
         previousYValue = RandomNumbers.nextDouble(random, 0.01, 10.0);

         double maxXValue = previousXValue + controlDT * maxXRate;
         double minXValue = previousXValue - controlDT * maxXRate;

         inputCalculator.calculateMaxFeedbackRateConstraint(inequalityConstraint, maxXRate, maxYRate, previousXValue, previousYValue, controlDT);

         expectedConstraint.Aineq.set(0, 0, 1);
         expectedConstraint.Aineq.set(1, 0, -1);

         expectedConstraint.bineq.set(0, 0, maxXValue);
         expectedConstraint.bineq.set(1, 0, -minXValue);

         assertConstraintsEqual("", expectedConstraint, inequalityConstraint);

         expectedConstraint.reset();

         controlDT = RandomNumbers.nextDouble(random, 1e-6, 1e-2);

         maxXRate = RandomNumbers.nextDouble(random, 0.01, 10.0);
         maxYRate = RandomNumbers.nextDouble(random, 0.01, 10.0);

         previousXValue = RandomNumbers.nextDouble(random, 0.01, 10.0);
         previousYValue = Double.POSITIVE_INFINITY;

         maxXValue = previousXValue + controlDT * maxXRate;
         minXValue = previousXValue - controlDT * maxXRate;

         inputCalculator.calculateMaxFeedbackRateConstraint(inequalityConstraint, maxXRate, maxYRate, previousXValue, previousYValue, controlDT);

         expectedConstraint.Aineq.set(0, 0, 1);
         expectedConstraint.Aineq.set(1, 0, -1);

         expectedConstraint.bineq.set(0, 0, maxXValue);
         expectedConstraint.bineq.set(1, 0, -minXValue);

         assertConstraintsEqual("", expectedConstraint, inequalityConstraint);

         expectedConstraint.reset();
         expectedConstraint.reshape(0, 2);

         controlDT = RandomNumbers.nextDouble(random, 1e-6, 1e-2);

         maxXRate = Double.POSITIVE_INFINITY;
         maxYRate = Double.POSITIVE_INFINITY;

         previousXValue = RandomNumbers.nextDouble(random, 0.01, 10.0);
         previousYValue = RandomNumbers.nextDouble(random, 0.01, 10.0);

         inputCalculator.calculateMaxFeedbackRateConstraint(inequalityConstraint, maxXRate, maxYRate, previousXValue, previousYValue, controlDT);

         assertConstraintsEqual("", expectedConstraint, inequalityConstraint);

         expectedConstraint.reset();

         controlDT = RandomNumbers.nextDouble(random, 1e-6, 1e-2);

         maxXRate = RandomNumbers.nextDouble(random, 0.01, 10.0);
         maxYRate = RandomNumbers.nextDouble(random, 0.01, 10.0);

         previousXValue = Double.POSITIVE_INFINITY;
         previousYValue = Double.POSITIVE_INFINITY;

         inputCalculator.calculateMaxFeedbackRateConstraint(inequalityConstraint, maxXRate, maxYRate, previousXValue, previousYValue, controlDT);

         assertConstraintsEqual("", expectedConstraint, inequalityConstraint);

         indexHandler.setHasCMPFeedbackTask(true);
         indexHandler.computeProblemSize();

         controlDT = RandomNumbers.nextDouble(random, 1e-6, 1e-2);

         maxXRate = Double.POSITIVE_INFINITY;
         maxYRate = RandomNumbers.nextDouble(random, 0.01, 10.0);

         previousXValue = RandomNumbers.nextDouble(random, 0.01, 10.0);
         previousYValue = RandomNumbers.nextDouble(random, 0.01, 10.0);

         maxYValue = previousYValue + controlDT * maxYRate;
         minYValue = previousYValue - controlDT * maxYRate;

         expectedConstraint = new ICPInequalityInput(2, 4);

         inputCalculator.calculateMaxFeedbackRateConstraint(inequalityConstraint, maxXRate, maxYRate, previousXValue, previousYValue, controlDT);

         expectedConstraint.Aineq.set(0, 1, 1);
         expectedConstraint.Aineq.set(0, 3, 1);
         expectedConstraint.Aineq.set(1, 1, -1);
         expectedConstraint.Aineq.set(1, 3, -1);

         expectedConstraint.bineq.set(0, 0, maxYValue);
         expectedConstraint.bineq.set(1, 0, -minYValue);

         assertConstraintsEqual("", expectedConstraint, inequalityConstraint);

         expectedConstraint.reset();

         controlDT = RandomNumbers.nextDouble(random, 1e-6, 1e-2);

         maxXRate = RandomNumbers.nextDouble(random, 0.01, 10.0);
         maxYRate = RandomNumbers.nextDouble(random, 0.01, 10.0);

         previousXValue = Double.POSITIVE_INFINITY;
         previousYValue = RandomNumbers.nextDouble(random, 0.01, 10.0);

         maxYValue = previousYValue + controlDT * maxYRate;
         minYValue = previousYValue - controlDT * maxYRate;

         inputCalculator.calculateMaxFeedbackRateConstraint(inequalityConstraint, maxXRate, maxYRate, previousXValue, previousYValue, controlDT);

         expectedConstraint.Aineq.set(0, 1, 1);
         expectedConstraint.Aineq.set(0, 3, 1);
         expectedConstraint.Aineq.set(1, 1, -1);
         expectedConstraint.Aineq.set(1, 3, -1);

         expectedConstraint.bineq.set(0, 0, maxYValue);
         expectedConstraint.bineq.set(1, 0,-minYValue);

         assertConstraintsEqual("", expectedConstraint, inequalityConstraint);

         expectedConstraint.reset();

         controlDT = RandomNumbers.nextDouble(random, 1e-6, 1e-2);

         maxXRate = RandomNumbers.nextDouble(random, 0.01, 10.0);
         maxYRate = Double.POSITIVE_INFINITY;

         previousXValue = RandomNumbers.nextDouble(random, 0.01, 10.0);
         previousYValue = RandomNumbers.nextDouble(random, 0.01, 10.0);

         maxXValue = previousXValue + controlDT * maxXRate;
         minXValue = previousXValue - controlDT * maxXRate;

         inputCalculator.calculateMaxFeedbackRateConstraint(inequalityConstraint, maxXRate, maxYRate, previousXValue, previousYValue, controlDT);

         expectedConstraint.Aineq.set(0, 0, 1);
         expectedConstraint.Aineq.set(0, 2, 1);
         expectedConstraint.Aineq.set(1, 0, -1);
         expectedConstraint.Aineq.set(1, 2, -1);

         expectedConstraint.bineq.set(0, 0, maxXValue);
         expectedConstraint.bineq.set(1, 0,-minXValue);

         assertConstraintsEqual("", expectedConstraint, inequalityConstraint);

         expectedConstraint.reset();

         controlDT = RandomNumbers.nextDouble(random, 1e-6, 1e-2);

         maxXRate = RandomNumbers.nextDouble(random, 0.01, 10.0);
         maxYRate = RandomNumbers.nextDouble(random, 0.01, 10.0);

         previousXValue = RandomNumbers.nextDouble(random, 0.01, 10.0);
         previousYValue = Double.POSITIVE_INFINITY;

         maxXValue = previousXValue + controlDT * maxXRate;
         minXValue = previousXValue - controlDT * maxXRate;

         inputCalculator.calculateMaxFeedbackRateConstraint(inequalityConstraint, maxXRate, maxYRate, previousXValue, previousYValue, controlDT);

         expectedConstraint.Aineq.set(0, 0, 1);
         expectedConstraint.Aineq.set(0, 2, 1);
         expectedConstraint.Aineq.set(1, 0, -1);
         expectedConstraint.Aineq.set(1, 2, -1);

         expectedConstraint.bineq.set(0, 0, maxXValue);
         expectedConstraint.bineq.set(1, 0,-minXValue);

         assertConstraintsEqual("", expectedConstraint, inequalityConstraint);

         expectedConstraint.reset();
         expectedConstraint.reshape(0, 4);

         maxXRate = Double.POSITIVE_INFINITY;
         maxYRate = Double.POSITIVE_INFINITY;

         previousXValue = RandomNumbers.nextDouble(random, 0.01, 10.0);
         previousYValue = RandomNumbers.nextDouble(random, 0.01, 10.0);

         inputCalculator.calculateMaxFeedbackRateConstraint(inequalityConstraint, maxXRate, maxYRate, previousXValue, previousYValue, controlDT);

         assertConstraintsEqual("", expectedConstraint, inequalityConstraint);

         expectedConstraint.reset();

         maxXRate = RandomNumbers.nextDouble(random, 0.01, 10.0);
         maxYRate = RandomNumbers.nextDouble(random, 0.01, 10.0);

         previousXValue = Double.POSITIVE_INFINITY;
         previousYValue = Double.POSITIVE_INFINITY;

         inputCalculator.calculateMaxFeedbackRateConstraint(inequalityConstraint, maxXRate, maxYRate, previousXValue, previousYValue, controlDT);

         assertConstraintsEqual("", expectedConstraint, inequalityConstraint);
      }
   }

   private static void testInequalityConstraint(ICPInequalityInput expectedConstraint, ICPInequalityInput inequalityConstraint, Random random,
                                                double maxXValue, double maxYValue, int iter)
   {
      expectedConstraint.Aineq.set(0, 0, 1);
      expectedConstraint.Aineq.set(1, 0, -1);
      expectedConstraint.Aineq.set(2, 1, 1);
      expectedConstraint.Aineq.set(3, 1, -1);

      expectedConstraint.bineq.set(0, 0, maxXValue);
      expectedConstraint.bineq.set(1, 0, maxXValue);
      expectedConstraint.bineq.set(2, 0, maxYValue);
      expectedConstraint.bineq.set(3, 0, maxYValue);

      assertConstraintsEqual("", expectedConstraint, inequalityConstraint);

      double distanceToBoundEdge = 1e-4;
      double randomXValue = RandomNumbers.nextDouble(random, -maxXValue + distanceToBoundEdge, maxXValue - distanceToBoundEdge);
      double randomYValue = RandomNumbers.nextDouble(random, -maxYValue + distanceToBoundEdge, maxYValue - distanceToBoundEdge);
      double xValueNearBound = maxXValue - distanceToBoundEdge;
      double yValueNearBound = maxYValue - distanceToBoundEdge;
      double xValueAtBound = maxXValue;
      double yValueAtBound = maxYValue;
      double xValueJustPastBound = maxXValue + distanceToBoundEdge;
      double yValueJustPastBound = maxYValue + distanceToBoundEdge;
      double xValueWellPastBound = maxXValue + 3.0;
      double yValueWellPastBound = maxYValue + 3.0;

      // test inside
      DMatrixRMaj value = new DMatrixRMaj(2, 1);
      value.set(0, 0, randomXValue);
      value.set(1, 0, randomYValue);

      assertInequalityHolds("Iteration " + iter + ". Both positive inside ", inequalityConstraint, value);

      value.set(0, 0, -randomXValue);
      value.set(1, 0, -randomYValue);

      assertInequalityHolds("Iteration " + iter + ". Both negative inside ", inequalityConstraint, value);

      value.set(0, 0, randomXValue);
      value.set(1, 0, -randomYValue);

      assertInequalityHolds("Iteration " + iter + ". Y negative inside ", inequalityConstraint, value);

      value.set(0, 0, -randomXValue);
      value.set(1, 0, randomYValue);

      assertInequalityHolds("Iteration " + iter + ". X negative inside ", inequalityConstraint, value);

      // test near bound positive
      value.set(0, 0, xValueNearBound);
      value.set(1, 0, yValueNearBound);

      assertInequalityHolds("Iteration " + iter + ". Both positive inside ", inequalityConstraint, value);

      value.set(0, 0, -xValueNearBound);
      value.set(1, 0, -yValueNearBound);

      assertInequalityHolds("Iteration " + iter + ". Both negative inside ", inequalityConstraint, value);

      value.set(0, 0, xValueNearBound);
      value.set(1, 0, -yValueNearBound);

      assertInequalityHolds("Iteration " + iter + ". Y negative inside ", inequalityConstraint, value);

      value.set(0, 0, -xValueNearBound);
      value.set(1, 0, yValueNearBound);

      assertInequalityHolds("Iteration " + iter + ". X negative inside ", inequalityConstraint, value);

      // test at bound
      value.set(0, 0, xValueAtBound);
      value.set(1, 0, yValueAtBound);

      assertInequalityEquals("Iteration " + iter + ". Both positive equal ", inequalityConstraint, value);

      value.set(0, 0, -xValueAtBound);
      value.set(1, 0, -yValueAtBound);

      assertInequalityEquals("Iteration " + iter + ". Both negative equal ", inequalityConstraint, value);

      value.set(0, 0, xValueAtBound);
      value.set(1, 0, -yValueAtBound);

      assertInequalityEquals("Iteration " + iter + ". Y negative equal ", inequalityConstraint, value);

      value.set(0, 0, -xValueAtBound);
      value.set(1, 0, yValueAtBound);

      assertInequalityEquals("Iteration " + iter + ". X negative equal ", inequalityConstraint, value);

      value.set(0, 0, xValueAtBound);
      value.set(1, 0, yValueNearBound);

      assertInequalityEquals("Iteration " + iter + ". Y value near positive equal ", inequalityConstraint, value);

      value.set(0, 0, -xValueAtBound);
      value.set(1, 0, yValueNearBound);

      assertInequalityEquals("Iteration " + iter + ". Y value near negative equal ", inequalityConstraint, value);

      value.set(0, 0, xValueNearBound);
      value.set(1, 0, yValueAtBound);

      assertInequalityEquals("Iteration " + iter + ". X value near positive equal ", inequalityConstraint, value);

      value.set(0, 0, xValueNearBound);
      value.set(1, 0, -yValueAtBound);

      assertInequalityEquals("Iteration " + iter + ". X value near negative equal ", inequalityConstraint, value);

      // test just outside bound
      value.set(0, 0, xValueJustPastBound);
      value.set(1, 0, yValueJustPastBound);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, -xValueJustPastBound);
      value.set(1, 0, -yValueJustPastBound);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, xValueJustPastBound);
      value.set(1, 0, -yValueJustPastBound);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, -xValueJustPastBound);
      value.set(1, 0, yValueJustPastBound);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, xValueJustPastBound);
      value.set(1, 0, randomYValue);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, -xValueJustPastBound);
      value.set(1, 0, randomYValue);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, randomXValue);
      value.set(1, 0, yValueJustPastBound);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, randomXValue);
      value.set(1, 0, -yValueJustPastBound);
      assertInequalityFails("", inequalityConstraint, value);

      // test far outside bound
      value.set(0, 0, xValueWellPastBound);
      value.set(1, 0, yValueWellPastBound);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, -xValueWellPastBound);
      value.set(1, 0, -yValueWellPastBound);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, xValueWellPastBound);
      value.set(1, 0, -yValueWellPastBound);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, -xValueWellPastBound);
      value.set(1, 0, yValueWellPastBound);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, xValueWellPastBound);
      value.set(1, 0, randomYValue);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, -xValueWellPastBound);
      value.set(1, 0, randomYValue);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, randomXValue);
      value.set(1, 0, yValueWellPastBound);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, randomXValue);
      value.set(1, 0, -yValueWellPastBound);
      assertInequalityFails("", inequalityConstraint, value);
   }


   private static void testInequalityRateConstraint(ICPInequalityInput expectedConstraint, ICPInequalityInput inequalityConstraint, Random random, double previousXValue,
                                                double previousYValue, double maxXRate, double maxYRate, double controlDT, int iter)
   {
      double maxXValue = previousXValue + controlDT * maxXRate;
      double minXValue = previousXValue - controlDT * maxXRate;
      double maxYValue = previousYValue + controlDT * maxYRate;
      double minYValue = previousYValue - controlDT * maxYRate;

      expectedConstraint.Aineq.set(0, 0, 1);
      expectedConstraint.Aineq.set(1, 0, -1);
      expectedConstraint.Aineq.set(2, 1, 1);
      expectedConstraint.Aineq.set(3, 1, -1);

      expectedConstraint.bineq.set(0, 0, maxXValue);
      expectedConstraint.bineq.set(1, 0, -minXValue);
      expectedConstraint.bineq.set(2, 0, maxYValue);
      expectedConstraint.bineq.set(3, 0, -minYValue);

      assertConstraintsEqual("", expectedConstraint, inequalityConstraint);

      double distanceToBoundEdge = 1e-4;
      double randomXRate = RandomNumbers.nextDouble(random, 0.0, maxXRate);
      double randomYRate = RandomNumbers.nextDouble(random, 0.0, maxYRate);
      double xValueNearUpperBound = previousXValue + controlDT * maxXRate - distanceToBoundEdge;
      double xValueNearLowerBound = previousXValue - controlDT * maxXRate + distanceToBoundEdge;
      double yValueNearUpperBound = previousYValue + controlDT * maxYRate - distanceToBoundEdge;
      double yValueNearLowerBound = previousYValue - controlDT * maxYRate + distanceToBoundEdge;
      double xValueJustPastUpperBound = maxXValue + distanceToBoundEdge;
      double yValueJustPastUpperBound = maxYValue + distanceToBoundEdge;
      double xValueJustPastLowerBound = minXValue - distanceToBoundEdge;
      double yValueJustPastLowerBound = minYValue - distanceToBoundEdge;
      double xValueWellPastUpperBound = maxXValue + 3.0;
      double yValueWellPastUpperBound = maxYValue + 3.0;
      double xValueWellPastLowerBound = minXValue - 3.0;
      double yValueWellPastLowerBound = minYValue - 3.0;

      DMatrixRMaj value = new DMatrixRMaj(2, 1);
      value.set(0, 0, previousXValue);
      value.set(1, 0, previousYValue);

      assertInequalityHolds(inequalityConstraint, value);

      // test inside
      value.set(0, 0, previousXValue + controlDT * randomXRate);
      value.set(1, 0, previousYValue + controlDT * randomYRate);

      assertInequalityHolds(inequalityConstraint, value);

      value.set(0, 0, previousXValue - controlDT * randomXRate);
      value.set(1, 0, previousYValue - controlDT * randomYRate);

      assertInequalityHolds(inequalityConstraint, value);

      value.set(0, 0, previousXValue - controlDT * randomXRate);
      value.set(1, 0, previousYValue + controlDT * randomYRate);

      assertInequalityHolds(inequalityConstraint, value);

      value.set(0, 0, previousXValue + controlDT * randomXRate);
      value.set(1, 0, previousYValue - controlDT * randomYRate);

      assertInequalityHolds(inequalityConstraint, value);






      // test near bound positive
      value.set(0, 0, xValueNearUpperBound);
      value.set(1, 0, yValueNearUpperBound);

      assertInequalityHolds("Iteration " + iter + ". Both positive inside ", inequalityConstraint, value);

      value.set(0, 0, xValueNearLowerBound);
      value.set(1, 0, yValueNearLowerBound);

      assertInequalityHolds("Iteration " + iter + ". Both negative inside ", inequalityConstraint, value);

      value.set(0, 0, xValueNearUpperBound);
      value.set(1, 0, yValueNearLowerBound);

      assertInequalityHolds("Iteration " + iter + ". Y negative inside ", inequalityConstraint, value);

      value.set(0, 0, xValueNearLowerBound);
      value.set(1, 0, yValueNearUpperBound);

      assertInequalityHolds("Iteration " + iter + ". X negative inside ", inequalityConstraint, value);




      // test at bound
      value.set(0, 0, maxXValue);
      value.set(1, 0, maxYValue);

      assertInequalityEquals("Iteration " + iter + ". Both positive equal ", inequalityConstraint, value);

      value.set(0, 0, minXValue);
      value.set(1, 0, minYValue);

      assertInequalityEquals("Iteration " + iter + ". Both negative equal ", inequalityConstraint, value);

      value.set(0, 0, maxXValue);
      value.set(1, 0, minYValue);

      assertInequalityEquals("Iteration " + iter + ". Y negative equal ", inequalityConstraint, value);

      value.set(0, 0, minXValue);
      value.set(1, 0, maxYValue);

      assertInequalityEquals("Iteration " + iter + ". X negative equal ", inequalityConstraint, value);

      value.set(0, 0, maxXValue);
      value.set(1, 0, yValueNearUpperBound);

      assertInequalityEquals("Iteration " + iter + ". Y value near positive equal ", inequalityConstraint, value);

      value.set(0, 0, minXValue);
      value.set(1, 0, yValueNearUpperBound);

      assertInequalityEquals("Iteration " + iter + ". Y value near negative equal ", inequalityConstraint, value);

      value.set(0, 0, xValueNearUpperBound);
      value.set(1, 0, maxYValue);

      assertInequalityEquals("Iteration " + iter + ". X value near positive equal ", inequalityConstraint, value);

      value.set(0, 0, xValueNearUpperBound);
      value.set(1, 0, minYValue);

      assertInequalityEquals("Iteration " + iter + ". X value near negative equal ", inequalityConstraint, value);




      // test just outside bound
      value.set(0, 0, xValueJustPastUpperBound);
      value.set(1, 0, yValueJustPastUpperBound);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, xValueJustPastLowerBound);
      value.set(1, 0, yValueJustPastLowerBound);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, xValueJustPastUpperBound);
      value.set(1, 0, yValueJustPastLowerBound);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, xValueJustPastLowerBound);
      value.set(1, 0, yValueJustPastUpperBound);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, xValueJustPastUpperBound);
      value.set(1, 0, previousYValue);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, xValueJustPastLowerBound);
      value.set(1, 0, previousYValue);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, previousXValue);
      value.set(1, 0, yValueJustPastUpperBound);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, previousXValue);
      value.set(1, 0, yValueJustPastLowerBound);
      assertInequalityFails("", inequalityConstraint, value);



      // test far outside bound
      value.set(0, 0, xValueWellPastUpperBound);
      value.set(1, 0, yValueWellPastUpperBound);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, xValueWellPastLowerBound);
      value.set(1, 0, yValueWellPastLowerBound);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, xValueWellPastUpperBound);
      value.set(1, 0, yValueWellPastLowerBound);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, xValueWellPastLowerBound);
      value.set(1, 0, yValueWellPastUpperBound);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, xValueWellPastUpperBound);
      value.set(1, 0, previousYValue);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, xValueWellPastLowerBound);
      value.set(1, 0, previousYValue);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, previousXValue);
      value.set(1, 0, yValueWellPastUpperBound);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, previousXValue);
      value.set(1, 0, yValueWellPastLowerBound);
      assertInequalityFails("", inequalityConstraint, value);
   }

   private static void testInequalityConstraintWithCMP(ICPInequalityInput expectedConstraint, ICPInequalityInput inequalityConstraint, Random random,
                                                       double maxXValue, double maxYValue, int iter)
   {
      expectedConstraint.Aineq.set(0, 0, 1);
      expectedConstraint.Aineq.set(0, 2, 1);
      expectedConstraint.Aineq.set(1, 0, -1);
      expectedConstraint.Aineq.set(1, 2, -1);
      expectedConstraint.Aineq.set(2, 1, 1);
      expectedConstraint.Aineq.set(2, 3, 1);
      expectedConstraint.Aineq.set(3, 1, -1);
      expectedConstraint.Aineq.set(3, 3, -1);

      expectedConstraint.bineq.set(0, 0, maxXValue);
      expectedConstraint.bineq.set(1, 0, maxXValue);
      expectedConstraint.bineq.set(2, 0, maxYValue);
      expectedConstraint.bineq.set(3, 0, maxYValue);

      assertConstraintsEqual("", expectedConstraint, inequalityConstraint);

      double distanceToBoundEdge = 1e-4;
      double randomXValue = RandomNumbers.nextDouble(random, -maxXValue + distanceToBoundEdge, maxXValue - distanceToBoundEdge);
      double randomXValue1 = RandomNumbers.nextDouble(random, 0, 1.0) * randomXValue;
      double randomXValue2 = randomXValue - randomXValue1;
      double randomYValue = RandomNumbers.nextDouble(random, -maxYValue + distanceToBoundEdge, maxYValue - distanceToBoundEdge);
      double randomYValue1 = RandomNumbers.nextDouble(random, 0.0, 1.0) * randomYValue;
      double randomYValue2 = randomYValue - randomYValue1;
      double xValueNearBound1 = RandomNumbers.nextDouble(random, 0.0, 1.0) * (maxXValue - distanceToBoundEdge);
      double xValueNearBound2 = maxXValue - distanceToBoundEdge - xValueNearBound1;
      double yValueNearBound1 = RandomNumbers.nextDouble(random, 0.0, 1.0) * (maxYValue - distanceToBoundEdge);
      double yValueNearBound2 = maxYValue - distanceToBoundEdge - yValueNearBound1;
      double xValueAtBound1 = RandomNumbers.nextDouble(random, 0.0, 1.0) * maxXValue;
      double xValueAtBound2 = maxXValue - xValueAtBound1;
      double yValueAtBound1 = RandomNumbers.nextDouble(random, 0.0, 1.0) * maxYValue;
      double yValueAtBound2 = maxYValue - yValueAtBound1;
      double xValueJustPastBound1 = RandomNumbers.nextDouble(random, 0.0, 1.0) * (maxXValue + distanceToBoundEdge);
      double xValueJustPastBound2 = maxXValue + distanceToBoundEdge - xValueJustPastBound1;
      double yValueJustPastBound1 = RandomNumbers.nextDouble(random, 0.0, 1.0) * (maxYValue + distanceToBoundEdge);
      double yValueJustPastBound2 = maxYValue + distanceToBoundEdge - yValueJustPastBound1;
      double xValueWellPastBound1 = RandomNumbers.nextDouble(random, 0.0, 1.0) * (maxXValue + 3.0);
      double xValueWellPastBound2 = maxXValue + 3.0 - xValueWellPastBound1;
      double yValueWellPastBound1 = RandomNumbers.nextDouble(random, 0.0, 1.0) * (maxYValue + 3.0);
      double yValueWellPastBound2 = maxYValue + 3.0 - yValueWellPastBound1;

      // test inside
      DMatrixRMaj value = new DMatrixRMaj(4, 1);
      value.set(0, 0, randomXValue1);
      value.set(1, 0, randomYValue1);
      value.set(2, 0, randomXValue2);
      value.set(3, 0, randomYValue2);

      assertInequalityHolds("Iteration " + iter + ". Both positive inside ", inequalityConstraint, value);

      value.set(0, 0, -randomXValue1);
      value.set(1, 0, -randomYValue1);
      value.set(2, 0, -randomXValue2);
      value.set(3, 0, -randomYValue2);

      assertInequalityHolds("Iteration " + iter + ". Both negative inside ", inequalityConstraint, value);

      value.set(0, 0, randomXValue1);
      value.set(1, 0, -randomYValue1);
      value.set(2, 0, randomXValue2);
      value.set(3, 0, -randomYValue2);

      assertInequalityHolds("Iteration " + iter + ". Y negative inside ", inequalityConstraint, value);

      value.set(0, 0, -randomXValue1);
      value.set(1, 0, randomYValue1);
      value.set(2, 0, -randomXValue2);
      value.set(3, 0, randomYValue2);

      assertInequalityHolds("Iteration " + iter + ". X negative inside ", inequalityConstraint, value);

      // test near bound positive
      value.set(0, 0, xValueNearBound1);
      value.set(1, 0, yValueNearBound1);
      value.set(2, 0, xValueNearBound2);
      value.set(3, 0, yValueNearBound2);

      assertInequalityHolds("Iteration " + iter + ". Both positive inside ", inequalityConstraint, value);

      value.set(0, 0, -xValueNearBound1);
      value.set(1, 0, -yValueNearBound1);
      value.set(2, 0, -xValueNearBound2);
      value.set(3, 0, -yValueNearBound2);

      assertInequalityHolds("Iteration " + iter + ". Both negative inside ", inequalityConstraint, value);

      value.set(0, 0, xValueNearBound1);
      value.set(1, 0, -yValueNearBound1);
      value.set(2, 0, xValueNearBound2);
      value.set(3, 0, -yValueNearBound2);

      assertInequalityHolds("Iteration " + iter + ". Y negative inside ", inequalityConstraint, value);

      value.set(0, 0, -xValueNearBound1);
      value.set(1, 0, yValueNearBound1);
      value.set(2, 0, -xValueNearBound2);
      value.set(3, 0, yValueNearBound2);

      assertInequalityHolds("Iteration " + iter + ". X negative inside ", inequalityConstraint, value);

      // test at bound
      value.set(0, 0, xValueAtBound1);
      value.set(1, 0, yValueAtBound1);
      value.set(2, 0, xValueAtBound2);
      value.set(3, 0, yValueAtBound2);

      assertInequalityEquals("Iteration " + iter + ". Both positive equal ", inequalityConstraint, value);

      value.set(0, 0, -xValueAtBound1);
      value.set(1, 0, -yValueAtBound1);
      value.set(2, 0, -xValueAtBound2);
      value.set(3, 0, -yValueAtBound2);

      assertInequalityEquals("Iteration " + iter + ". Both negative equal ", inequalityConstraint, value);

      value.set(0, 0, xValueAtBound1);
      value.set(1, 0, -yValueAtBound1);
      value.set(2, 0, xValueAtBound2);
      value.set(3, 0, -yValueAtBound2);

      assertInequalityEquals("Iteration " + iter + ". Y negative equal ", inequalityConstraint, value);

      value.set(0, 0, -xValueAtBound1);
      value.set(1, 0, yValueAtBound1);
      value.set(2, 0, -xValueAtBound2);
      value.set(3, 0, yValueAtBound2);

      assertInequalityEquals("Iteration " + iter + ". X negative equal ", inequalityConstraint, value);

      value.set(0, 0, xValueAtBound1);
      value.set(1, 0, yValueNearBound1);
      value.set(2, 0, xValueAtBound2);
      value.set(3, 0, yValueNearBound2);

      assertInequalityEquals("Iteration " + iter + ". Y value near positive equal ", inequalityConstraint, value);

      value.set(0, 0, -xValueAtBound1);
      value.set(1, 0, yValueNearBound1);
      value.set(2, 0, -xValueAtBound2);
      value.set(3, 0, yValueNearBound2);

      assertInequalityEquals("Iteration " + iter + ". Y value near negative equal ", inequalityConstraint, value);

      value.set(0, 0, xValueNearBound1);
      value.set(1, 0, yValueAtBound1);
      value.set(2, 0, xValueNearBound2);
      value.set(3, 0, yValueAtBound2);

      assertInequalityEquals("Iteration " + iter + ". X value near positive equal ", inequalityConstraint, value);

      value.set(0, 0, xValueNearBound1);
      value.set(1, 0, -yValueAtBound1);
      value.set(2, 0, xValueNearBound2);
      value.set(3, 0, -yValueAtBound2);

      assertInequalityEquals("Iteration " + iter + ". X value near negative equal ", inequalityConstraint, value);

      // test just outside bound
      value.set(0, 0, xValueJustPastBound1);
      value.set(1, 0, yValueJustPastBound1);
      value.set(2, 0, xValueJustPastBound2);
      value.set(3, 0, yValueJustPastBound2);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, -xValueJustPastBound1);
      value.set(1, 0, -yValueJustPastBound1);
      value.set(2, 0, -xValueJustPastBound2);
      value.set(3, 0, -yValueJustPastBound2);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, xValueJustPastBound1);
      value.set(1, 0, -yValueJustPastBound1);
      value.set(2, 0, xValueJustPastBound2);
      value.set(3, 0, -yValueJustPastBound2);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, -xValueJustPastBound1);
      value.set(1, 0, yValueJustPastBound1);
      value.set(2, 0, -xValueJustPastBound2);
      value.set(3, 0, yValueJustPastBound2);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, xValueJustPastBound1);
      value.set(1, 0, randomYValue1);
      value.set(2, 0, xValueJustPastBound2);
      value.set(3, 0, randomYValue2);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, -xValueJustPastBound1);
      value.set(1, 0, randomYValue1);
      value.set(2, 0, -xValueJustPastBound2);
      value.set(3, 0, randomYValue2);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, randomXValue1);
      value.set(1, 0, yValueJustPastBound1);
      value.set(2, 0, randomXValue2);
      value.set(3, 0, yValueJustPastBound2);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, randomXValue1);
      value.set(1, 0, -yValueJustPastBound1);
      value.set(2, 0, randomXValue2);
      value.set(3, 0, -yValueJustPastBound2);
      assertInequalityFails("", inequalityConstraint, value);

      // test far outside bound
      value.set(0, 0, xValueWellPastBound1);
      value.set(1, 0, yValueWellPastBound1);
      value.set(2, 0, xValueWellPastBound2);
      value.set(3, 0, yValueWellPastBound2);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, -xValueWellPastBound1);
      value.set(1, 0, -yValueWellPastBound1);
      value.set(2, 0, -xValueWellPastBound2);
      value.set(3, 0, -yValueWellPastBound2);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, xValueWellPastBound1);
      value.set(1, 0, -yValueWellPastBound1);
      value.set(2, 0, xValueWellPastBound2);
      value.set(3, 0, -yValueWellPastBound2);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, -xValueWellPastBound1);
      value.set(1, 0, yValueWellPastBound1);
      value.set(2, 0, -xValueWellPastBound2);
      value.set(3, 0, yValueWellPastBound2);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, xValueWellPastBound1);
      value.set(1, 0, randomYValue1);
      value.set(2, 0, xValueWellPastBound2);
      value.set(3, 0, randomYValue2);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, -xValueWellPastBound1);
      value.set(1, 0, randomYValue1);
      value.set(2, 0, -xValueWellPastBound2);
      value.set(3, 0, randomYValue2);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, randomXValue1);
      value.set(1, 0, yValueWellPastBound1);
      value.set(2, 0, randomXValue2);
      value.set(3, 0, yValueWellPastBound2);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, randomXValue1);
      value.set(1, 0, -yValueWellPastBound1);
      value.set(2, 0, randomXValue2);
      value.set(3, 0, -yValueWellPastBound2);
      assertInequalityFails("", inequalityConstraint, value);
   }

   private static void testInequalityRateConstraintWithCMP(ICPInequalityInput expectedConstraint, ICPInequalityInput inequalityConstraint, Random random, double previousXValue,
                                                    double previousYValue, double maxXRate, double maxYRate, double controlDT, int iter)
   {
      double maxXValue = previousXValue + controlDT * maxXRate;
      double minXValue = previousXValue - controlDT * maxXRate;
      double maxYValue = previousYValue + controlDT * maxYRate;
      double minYValue = previousYValue - controlDT * maxYRate;



      expectedConstraint.Aineq.set(0, 0, 1);
      expectedConstraint.Aineq.set(0, 2, 1);
      expectedConstraint.Aineq.set(1, 0, -1);
      expectedConstraint.Aineq.set(1, 2, -1);
      expectedConstraint.Aineq.set(2, 1, 1);
      expectedConstraint.Aineq.set(2, 3, 1);
      expectedConstraint.Aineq.set(3, 1, -1);
      expectedConstraint.Aineq.set(3, 3, -1);

      expectedConstraint.bineq.set(0, 0, maxXValue);
      expectedConstraint.bineq.set(1, 0, -minXValue);
      expectedConstraint.bineq.set(2, 0, maxYValue);
      expectedConstraint.bineq.set(3, 0, -minYValue);


      assertConstraintsEqual("", expectedConstraint, inequalityConstraint);

      double distanceToBoundEdge = 1e-4;
      double previousXValue1 = RandomNumbers.nextDouble(random, 0.0, 1.0) * previousXValue;
      double previousXValue2 = previousXValue - previousXValue1;
      double previousYValue1 = RandomNumbers.nextDouble(random, 0.0, 1.0) * previousYValue;
      double previousYValue2 = previousYValue - previousYValue1;
      double randomXRate = RandomNumbers.nextDouble(random, 0.0, maxXRate);
      double randomYRate = RandomNumbers.nextDouble(random, 0.0, maxYRate);
      double xValueInsideUpper = previousXValue + controlDT * randomXRate;
      double xValueInsideUpper1 = RandomNumbers.nextDouble(random, 0.0, 1.0) * xValueInsideUpper;
      double xValueInsideUpper2 = xValueInsideUpper - xValueInsideUpper1;
      double xValueInsideLower = previousXValue - controlDT * randomXRate;
      double xValueInsideLower1 = RandomNumbers.nextDouble(random, 0.0, 1.0) * xValueInsideLower;
      double xValueInsideLower2 = xValueInsideLower - xValueInsideLower1;
      double yValueInsideUpper = previousYValue + controlDT * randomYRate;
      double yValueInsideUpper1 = RandomNumbers.nextDouble(random, 0.0, 1.0) * yValueInsideUpper;
      double yValueInsideUpper2 = yValueInsideUpper - yValueInsideUpper1;
      double yValueInsideLower = previousYValue - controlDT * randomYRate;
      double yValueInsideLower1 = RandomNumbers.nextDouble(random, 0.0, 1.0) * yValueInsideLower;
      double yValueInsideLower2 = yValueInsideLower - yValueInsideLower1;
      double xValueNearUpperBound = previousXValue + controlDT * maxXRate - distanceToBoundEdge;
      double xValueNearUpperBound1 = RandomNumbers.nextDouble(random, 0.0, 1.0) * xValueNearUpperBound;
      double xValueNearUpperBound2 = xValueNearUpperBound - xValueNearUpperBound1;
      double xValueNearLowerBound = previousXValue - controlDT * maxXRate + distanceToBoundEdge;
      double xValueNearLowerBound1 = RandomNumbers.nextDouble(random, 0.0, 1.0) * xValueNearLowerBound;
      double xValueNearLowerBound2 = xValueNearLowerBound - xValueNearLowerBound1;
      double yValueNearUpperBound = previousYValue + controlDT * maxYRate - distanceToBoundEdge;
      double yValueNearUpperBound1 = RandomNumbers.nextDouble(random, 0.0, 1.0) * yValueNearUpperBound;
      double yValueNearUpperBound2 = yValueNearUpperBound - yValueNearUpperBound1;
      double yValueNearLowerBound = previousYValue - controlDT * maxYRate + distanceToBoundEdge;
      double yValueNearLowerBound1 = RandomNumbers.nextDouble(random, 0.0, 1.0) * yValueNearLowerBound;
      double yValueNearLowerBound2 = yValueNearLowerBound - yValueNearLowerBound1;
      double xValueJustPastUpperBound = maxXValue + distanceToBoundEdge;
      double xValueJustPastUpperBound1 = RandomNumbers.nextDouble(random, 0.0, 1.0) * xValueJustPastUpperBound;
      double xValueJustPastUpperBound2 = xValueJustPastUpperBound - xValueJustPastUpperBound1;
      double yValueJustPastUpperBound = maxYValue + distanceToBoundEdge;
      double yValueJustPastUpperBound1 = RandomNumbers.nextDouble(random, 0.0, 1.0) * yValueJustPastUpperBound;
      double yValueJustPastUpperBound2 = yValueJustPastUpperBound - yValueJustPastUpperBound1;
      double xValueJustPastLowerBound = minXValue - distanceToBoundEdge;
      double xValueJustPastLowerBound1 = RandomNumbers.nextDouble(random, 0.0, 1.0) * xValueJustPastLowerBound;
      double xValueJustPastLowerBound2 = xValueJustPastLowerBound - xValueJustPastLowerBound1;
      double yValueJustPastLowerBound = minYValue - distanceToBoundEdge;
      double yValueJustPastLowerBound1 = RandomNumbers.nextDouble(random, 0.0, 1.0) * yValueJustPastLowerBound;
      double yValueJustPastLowerBound2 = yValueJustPastLowerBound - yValueJustPastLowerBound1;
      double xValueWellPastUpperBound = maxXValue + 3.0;
      double xValueWellPastUpperBound1 = RandomNumbers.nextDouble(random, 0.0, 1.0) * xValueWellPastUpperBound;
      double xValueWellPastUpperBound2 = xValueWellPastUpperBound - xValueWellPastUpperBound1;
      double yValueWellPastUpperBound = maxYValue + 3.0;
      double yValueWellPastUpperBound1 = RandomNumbers.nextDouble(random, 0.0, 1.0) * yValueWellPastUpperBound;
      double yValueWellPastUpperBound2 = yValueWellPastUpperBound - yValueWellPastUpperBound1;
      double xValueWellPastLowerBound = minXValue - 3.0;
      double xValueWellPastLowerBound1 = RandomNumbers.nextDouble(random, 0.0, 1.0) * xValueWellPastLowerBound;
      double xValueWellPastLowerBound2 = xValueWellPastLowerBound - xValueWellPastLowerBound1;
      double yValueWellPastLowerBound = minYValue - 3.0;
      double yValueWellPastLowerBound1 = RandomNumbers.nextDouble(random, 0.0, 1.0) * yValueWellPastLowerBound;
      double yValueWellPastLowerBound2 = yValueWellPastLowerBound - yValueWellPastLowerBound1;

      double maxXValue1 = RandomNumbers.nextDouble(random, 0.0, 1.0) * maxXValue;
      double maxXValue2 = maxXValue - maxXValue1;
      double maxYValue1 = RandomNumbers.nextDouble(random, 0.0, 1.0) * maxYValue;
      double maxYValue2 = maxYValue - maxYValue1;

      double minXValue1 = RandomNumbers.nextDouble(random, 0.0, 1.0) * minXValue;
      double minXValue2 = minXValue - minXValue1;
      double minYValue1 = RandomNumbers.nextDouble(random, 0.0, 1.0) * minYValue;
      double minYValue2 = minYValue - minYValue1;










      DMatrixRMaj value = new DMatrixRMaj(4, 1);
      value.set(0, 0, previousXValue1);
      value.set(1, 0, previousYValue1);
      value.set(2, 0, previousXValue2);
      value.set(3, 0, previousYValue2);

      assertInequalityHolds(inequalityConstraint, value);

      // test inside
      value.set(0, 0, xValueInsideUpper1);
      value.set(1, 0, yValueInsideUpper1);
      value.set(2, 0, xValueInsideUpper2);
      value.set(3, 0, yValueInsideUpper2);

      assertInequalityHolds(inequalityConstraint, value);

      value.set(0, 0, xValueInsideLower1);
      value.set(1, 0, yValueInsideLower1);
      value.set(2, 0, xValueInsideLower2);
      value.set(3, 0, yValueInsideLower2);

      assertInequalityHolds(inequalityConstraint, value);

      value.set(0, 0, xValueInsideLower1);
      value.set(1, 0, yValueInsideUpper1);
      value.set(2, 0, xValueInsideLower2);
      value.set(3, 0, yValueInsideUpper2);

      assertInequalityHolds(inequalityConstraint, value);

      value.set(0, 0, xValueInsideUpper1);
      value.set(1, 0, yValueInsideLower1);
      value.set(2, 0, xValueInsideUpper2);
      value.set(3, 0, yValueInsideLower2);

      assertInequalityHolds(inequalityConstraint, value);






      // test near bound positive
      value.set(0, 0, xValueNearUpperBound1);
      value.set(1, 0, yValueNearUpperBound1);
      value.set(2, 0, xValueNearUpperBound2);
      value.set(3, 0, yValueNearUpperBound2);

      assertInequalityHolds("Iteration " + iter + ". Both positive inside ", inequalityConstraint, value);

      value.set(0, 0, xValueNearLowerBound1);
      value.set(1, 0, yValueNearLowerBound1);
      value.set(2, 0, xValueNearLowerBound2);
      value.set(3, 0, yValueNearLowerBound2);

      assertInequalityHolds("Iteration " + iter + ". Both negative inside ", inequalityConstraint, value);

      value.set(0, 0, xValueNearUpperBound1);
      value.set(1, 0, yValueNearLowerBound1);
      value.set(2, 0, xValueNearUpperBound2);
      value.set(3, 0, yValueNearLowerBound2);

      assertInequalityHolds("Iteration " + iter + ". Y negative inside ", inequalityConstraint, value);

      value.set(0, 0, xValueNearLowerBound1);
      value.set(1, 0, yValueNearUpperBound1);
      value.set(2, 0, xValueNearLowerBound2);
      value.set(3, 0, yValueNearUpperBound2);

      assertInequalityHolds("Iteration " + iter + ". X negative inside ", inequalityConstraint, value);




      // test at bound
      value.set(0, 0, maxXValue1);
      value.set(1, 0, maxYValue1);
      value.set(2, 0, maxXValue2);
      value.set(3, 0, maxYValue2);

      assertInequalityEquals("Iteration " + iter + ". Both positive equal ", inequalityConstraint, value);

      value.set(0, 0, minXValue1);
      value.set(1, 0, minYValue1);
      value.set(2, 0, minXValue2);
      value.set(3, 0, minYValue2);

      assertInequalityEquals("Iteration " + iter + ". Both negative equal ", inequalityConstraint, value);

      value.set(0, 0, maxXValue1);
      value.set(1, 0, minYValue1);
      value.set(2, 0, maxXValue2);
      value.set(3, 0, minYValue2);

      assertInequalityEquals("Iteration " + iter + ". Y negative equal ", inequalityConstraint, value);

      value.set(0, 0, minXValue1);
      value.set(1, 0, maxYValue1);
      value.set(2, 0, minXValue2);
      value.set(3, 0, maxYValue2);

      assertInequalityEquals("Iteration " + iter + ". X negative equal ", inequalityConstraint, value);

      value.set(0, 0, maxXValue1);
      value.set(1, 0, yValueNearUpperBound1);
      value.set(2, 0, maxXValue2);
      value.set(3, 0, yValueNearUpperBound2);

      assertInequalityEquals("Iteration " + iter + ". Y value near positive equal ", inequalityConstraint, value);

      value.set(0, 0, minXValue1);
      value.set(1, 0, yValueNearUpperBound1);
      value.set(2, 0, minXValue2);
      value.set(3, 0, yValueNearUpperBound2);

      assertInequalityEquals("Iteration " + iter + ". Y value near negative equal ", inequalityConstraint, value);

      value.set(0, 0, xValueNearUpperBound1);
      value.set(1, 0, maxYValue1);
      value.set(2, 0, xValueNearUpperBound2);
      value.set(3, 0, maxYValue2);

      assertInequalityEquals("Iteration " + iter + ". X value near positive equal ", inequalityConstraint, value);

      value.set(0, 0, xValueNearUpperBound1);
      value.set(1, 0, minYValue1);
      value.set(2, 0, xValueNearUpperBound2);
      value.set(3, 0, minYValue2);

      assertInequalityEquals("Iteration " + iter + ". X value near negative equal ", inequalityConstraint, value);




      // test just outside bound
      value.set(0, 0, xValueJustPastUpperBound1);
      value.set(1, 0, yValueJustPastUpperBound1);
      value.set(2, 0, xValueJustPastUpperBound2);
      value.set(3, 0, yValueJustPastUpperBound2);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, xValueJustPastLowerBound1);
      value.set(1, 0, yValueJustPastLowerBound1);
      value.set(2, 0, xValueJustPastLowerBound2);
      value.set(3, 0, yValueJustPastLowerBound2);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, xValueJustPastUpperBound1);
      value.set(1, 0, yValueJustPastLowerBound1);
      value.set(2, 0, xValueJustPastUpperBound2);
      value.set(3, 0, yValueJustPastLowerBound2);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, xValueJustPastLowerBound1);
      value.set(1, 0, yValueJustPastUpperBound1);
      value.set(2, 0, xValueJustPastLowerBound2);
      value.set(3, 0, yValueJustPastUpperBound2);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, xValueJustPastUpperBound1);
      value.set(1, 0, previousYValue1);
      value.set(2, 0, xValueJustPastUpperBound2);
      value.set(3, 0, previousYValue2);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, xValueJustPastLowerBound1);
      value.set(1, 0, previousYValue1);
      value.set(2, 0, xValueJustPastLowerBound2);
      value.set(3, 0, previousYValue2);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, previousXValue1);
      value.set(1, 0, yValueJustPastUpperBound1);
      value.set(2, 0, previousXValue2);
      value.set(3, 0, yValueJustPastUpperBound2);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, previousXValue1);
      value.set(1, 0, yValueJustPastLowerBound1);
      value.set(2, 0, previousXValue2);
      value.set(3, 0, yValueJustPastLowerBound2);
      assertInequalityFails("", inequalityConstraint, value);



      // test far outside bound
      value.set(0, 0, xValueWellPastUpperBound1);
      value.set(1, 0, yValueWellPastUpperBound1);
      value.set(2, 0, xValueWellPastUpperBound2);
      value.set(3, 0, yValueWellPastUpperBound2);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, xValueWellPastLowerBound1);
      value.set(1, 0, yValueWellPastLowerBound1);
      value.set(2, 0, xValueWellPastLowerBound2);
      value.set(3, 0, yValueWellPastLowerBound2);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, xValueWellPastUpperBound1);
      value.set(1, 0, yValueWellPastLowerBound1);
      value.set(2, 0, xValueWellPastUpperBound2);
      value.set(3, 0, yValueWellPastLowerBound2);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, xValueWellPastLowerBound1);
      value.set(1, 0, yValueWellPastUpperBound1);
      value.set(2, 0, xValueWellPastLowerBound2);
      value.set(3, 0, yValueWellPastUpperBound2);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, xValueWellPastUpperBound1);
      value.set(1, 0, previousYValue1);
      value.set(2, 0, xValueWellPastUpperBound2);
      value.set(3, 0, previousYValue2);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, xValueWellPastLowerBound1);
      value.set(1, 0, previousYValue1);
      value.set(2, 0, xValueWellPastLowerBound2);
      value.set(3, 0, previousYValue2);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, previousXValue1);
      value.set(1, 0, yValueWellPastUpperBound1);
      value.set(2, 0, previousXValue2);
      value.set(3, 0, yValueWellPastUpperBound2);
      assertInequalityFails("", inequalityConstraint, value);

      value.set(0, 0, previousXValue1);
      value.set(1, 0, yValueWellPastLowerBound1);
      value.set(2, 0, previousXValue2);
      value.set(3, 0, yValueWellPastLowerBound2);
      assertInequalityFails("", inequalityConstraint, value);
   }

   private static void assertConstraintsEqual(String prefix, ICPInequalityInput expected, ICPInequalityInput actual)
   {
      assertEquals(prefix + " the number of constraints are not equal.", expected.getNumberOfConstraints(), actual.getNumberOfConstraints());
      assertEquals(prefix + " the number of variables are not equal.", expected.getNumberOfVariables(), actual.getNumberOfVariables());
      MatrixTestTools.assertMatrixEquals(prefix + " Aineq is not equal.", expected.Aineq, actual.Aineq, epsilon);
      MatrixTestTools.assertMatrixEquals(prefix + " bineq is not equal.", expected.bineq, actual.bineq, epsilon);
   }

   private static void assertInequalityHolds(ICPInequalityInput inequalityToTest, DMatrixRMaj variables)
   {
      assertInequalityHolds("", inequalityToTest, variables);
   }

   private static void assertInequalityHolds(String prefix, ICPInequalityInput inequalityToTest, DMatrixRMaj variables)
   {
      DMatrixRMaj constraintValue = new DMatrixRMaj(inequalityToTest.getNumberOfConstraints(), 1);

      CommonOps_DDRM.mult(inequalityToTest.Aineq, variables, constraintValue);
      assertVectorLessThan(prefix, constraintValue, inequalityToTest.bineq);
   }

   private static void assertInequalityFails(String prefix, ICPInequalityInput inequalityToTest, DMatrixRMaj variables)
   {
      DMatrixRMaj constraintValue = new DMatrixRMaj(inequalityToTest.getNumberOfConstraints(), 1);

      CommonOps_DDRM.mult(inequalityToTest.Aineq, variables, constraintValue);
      assertVectorNotLessThan(prefix, constraintValue, inequalityToTest.bineq);
   }

   private static void assertInequalityEquals(String prefix, ICPInequalityInput inequalityToTest, DMatrixRMaj variables)
   {
      DMatrixRMaj constraintValue = new DMatrixRMaj(inequalityToTest.getNumberOfConstraints(), 1);

      CommonOps_DDRM.mult(inequalityToTest.Aineq, variables, constraintValue);

      boolean hasValueAtBound = false;
      for (int i = 0; i < constraintValue.numRows; i++)
      {
         if (MathTools.epsilonEquals(constraintValue.get(i), inequalityToTest.bineq.get(i), epsilon))
            hasValueAtBound = true;
      }

      assertTrue(prefix, hasValueAtBound);
   }

   private static void assertVectorLessThan(String prefix, DMatrixRMaj vector, DMatrixRMaj upperBound)
   {
      for (int i = 0; i < vector.numRows; i++)
      {
         assertTrue(prefix + " row " + i + "'s value " + vector.get(i) + " is not less than it's upper bound " + upperBound.get(i) + ".",
                    vector.get(i) < upperBound.get(i));
      }
   }

   private static void assertVectorNotLessThan(String prefix, DMatrixRMaj vector, DMatrixRMaj upperBound)
   {
      boolean allLessThan = true;
      for (int i = 0; i < vector.numRows; i++)
      {
         if (vector.get(i) > upperBound.get(i))
            allLessThan = false;
      }

      assertFalse(prefix + " the inequality is completely satisfied", allLessThan);
   }
}
