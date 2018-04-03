package us.ihmc.robotics.trajectories;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;

public class TrapezoidalVelocityTrajectoryTest
{
   private static final boolean VERBOSE = false;

   @Before
   public void setUp() throws Exception
   {
   }

   @After
   public void tearDown() throws Exception
   {
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetState()
   {
      double t0 = 3.0;
      double x0 = 0.0;
      double xF = 1.0;
      double v0 = 0.0;
      double vF = 10.0;
      double vMax = 10.01;
      double aMax = 1.01;

      TrapezoidalVelocityTrajectory trap = new TrapezoidalVelocityTrajectory(t0, x0, xF, v0, vF, vMax, aMax);
      
      if (VERBOSE)
      {
         System.out.println("t1 = " + trap.getT1());
         System.out.println("t2 = " + trap.getT2());
         System.out.println("tF = " + trap.getFinalTime());
      }
      
      double dT = 2.5e-2;
      double tMax = 60.0;
      int numberOfPoints = (int) (tMax / dT);

      double[][] position = new double[2][numberOfPoints];
      double[][] velocity = new double[2][numberOfPoints];

      for (int i = 0; i < numberOfPoints; i++)
      {
         double t = t0 + i * dT;
         double[] state = trap.getState(t);

         // Save trajectory:
         position[0][i] = t;
         position[1][i] = state[0];

         velocity[0][i] = t;
         velocity[1][i] = state[1];
      }


      // Position plot:
      ArrayList<double[][]> listOfCurves1 = new ArrayList<double[][]>();
      listOfCurves1.add(position);

//    PlotGraph2d pg1 = PlotGraph2d.createPlotGraph2dMultipleCurves(listOfCurves1);
//    pg1.plot();
//    pg1.setCloseChoice(PlotGraph2d.CLOSE_WINDOW_ONLY);
//    pg1.setGraphTitle("Position profile");

      // Velocity plot:
      ArrayList<double[][]> listOfCurves2 = new ArrayList<double[][]>();
      listOfCurves2.add(velocity);

//    PlotGraph2d pg2 = PlotGraph2d.createPlotGraph2dMultipleCurves(listOfCurves2);
//    pg2.setGraphTitle("Velocity profile");
//    pg2.setCloseChoice(PlotGraph2d.CLOSE_WINDOW_ONLY);
//    pg2.plot();

//    sleepForever();
   }



	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void DONTtestOne()
   {
      double t0 = 1.0;
      double x0 = 0.0;
      double xF = 4.0;
      double v0 = -0.2;
      double vF = -0.2;
      double vMax = 0.2;
      double aMax = 1.0;

      TrapezoidalVelocityTrajectory trap = new TrapezoidalVelocityTrajectory(t0, x0, xF, v0, vF, vMax, aMax);
      
      if (VERBOSE)
      {
         System.out.println("t1 = " + trap.getT1());
         System.out.println("t2 = " + trap.getT2());
         System.out.println("tF = " + trap.getFinalTime());
      }
      
      plotTrapezoid(trap, t0);

      performTests(trap, t0, x0, v0, xF, vF, vMax, aMax);

//    sleepForever();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTwo()
   {
      // Should we overshoot in this case, or just brake fast and violate acceleration limits...

      double t0 = 0.0;
      double x0 = 0.0;
      double xF = 1.0;
      double v0 = 10.0;
      double vF = 0.0;
      double vMax = 10.0;
      double aMax = 1.0;

      TrapezoidalVelocityTrajectory trap = new TrapezoidalVelocityTrajectory(t0, x0, xF, v0, vF, vMax, aMax);
      
      if (VERBOSE)
      {
         System.out.println("t1 = " + trap.getT1());
         System.out.println("t2 = " + trap.getT2());
         System.out.println("tF = " + trap.getFinalTime());
      }
      
//    plotTrapezoid(trap, t0);

      performTests(trap, t0, x0, v0, xF, vF, vMax, aMax);

//    sleepForever();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testWithoutEnforcingFinalVelocity()
   {
      double t0 = 0.0;
      double x0 = 0.0;
      double xF = 1.0;
      double v0 = 0.0;
      double vF = 10.0;
      double vMax = 10.0;
      double aMax = 1.0;

      TrapezoidalVelocityTrajectory trap = new TrapezoidalVelocityTrajectory(t0, x0, xF, v0, vF, vMax, aMax, false);
      
      if (VERBOSE)
      {
         System.out.println("t1 = " + trap.getT1());
         System.out.println("t2 = " + trap.getT2());
         System.out.println("tF = " + trap.getFinalTime());
      }
      
      verifyTimesInOrder(trap);
      verifyMaxVelocityNotExceeded(trap);
      verifyInitialConditionsAndFinalPosition(trap, t0, x0, v0, xF);

//    plotTrapezoid(trap, t0);
//    sleepForever();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testNotEnforcingFinalVelocityBadCase()
   {
      // +++tk 090219 added this test.

      double t0 = 0.0;
      double x0 = 0.0;
      double moveDistance = 0.011;
      double initialVelocity = 0.1430000000000001;
      double endVelocity = 0.0;
      double maxVelocity = 0.2;
      double acceleration = 0.8999999999999999;
      boolean enforceFinalVelocity = false;

      TrapezoidalVelocityTrajectory trap = new TrapezoidalVelocityTrajectory(t0, x0, moveDistance, initialVelocity, endVelocity, maxVelocity, acceleration,
                                              enforceFinalVelocity);

      double endTime = trap.getFinalTime();

      String errorMessage = "endTime < 0.0: t0 = " + 0.0 + ", x0 = " + 0.0 + ", xF = " + moveDistance + ", v0 = " + initialVelocity + ", vF = " + endVelocity
                            + ", vMax = " + maxVelocity + " aMax = " + acceleration + ", enforceFinalVelocity = " + enforceFinalVelocity;

      assertTrue("endTime < 0.0: " + errorMessage, endTime >= 0.0);

      double endPosition = trap.getPosition(endTime);
      double moveDistanceError = Math.abs(endPosition - moveDistance);
      assertTrue("moveDistanceError = " + moveDistanceError + " : " + errorMessage, Math.abs(endPosition - moveDistance) < 0.001 * moveDistance);

      double endSpeed = trap.getVelocity(endTime);

      assertTrue("endSpeed=" + endSpeed + ", : " + errorMessage, endSpeed > -1e-6);

      assertTrue("endSpeed=" + endSpeed + ", : " + errorMessage, endSpeed <= maxVelocity);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testRandomStuff()
   {
      Random random = new Random(1458L);

      double t0 = 0.0;
      double x0 = 0.0;

      int numTests = 1000000;
      boolean passedOK = true;

      for (int i = 0; i < numTests; i++)
      {
         double xF = random.nextDouble() * 1.0;
         double v0 = random.nextDouble() * 1.0;
         double vF = random.nextDouble() * 1.0;
         double vMax = random.nextDouble() * 1.0;
         double aMax = random.nextDouble() * 1.0;
         boolean enforceFinalVelocity = false;

         try
         {
            if (Math.abs(v0) > vMax)
               vMax = Math.abs(v0) + 1e-7;
            if (Math.abs(vF) > vMax)
               vF = (vMax - 1e-7) * Math.signum(vF);

            @SuppressWarnings("unused")
            TrapezoidalVelocityTrajectory trap = new TrapezoidalVelocityTrajectory(t0, x0, xF, v0, vF, vMax, aMax, enforceFinalVelocity);

         }

         catch (Exception e)
         {
            passedOK = false;
            System.err.println("Exception! t0 = " + t0 + "; x0 = " + x0 + "; v0 = " + v0 + "; xF = " + xF + "; vF = " + vF + "; vMax = " + vMax + "; aMax = "
                               + aMax);
         }

//       performTests(trap, t0, x0, v0, xF, vF, vMax, aMax);

      }

      if (!passedOK)
         throw new RuntimeException("testRandomStuff Failed. Check output.");
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testProblemOnDog()
   {
      double t0 = 0.0;
      double x0 = 0.0;
      double xF = 0.1281005782972713;
      double v0 = 0.14653270950092306;
      double vF = 0.1;
      double vMax = 0.14;
      double aMax = 0.28;
      boolean enforceFinalVelocity = false;

      TrapezoidalVelocityTrajectory trap = new TrapezoidalVelocityTrajectory(t0, x0, xF, v0, vF, vMax, aMax, enforceFinalVelocity);
      performTests(trap, t0, x0, v0, xF, vF, vMax, aMax);
   }

	@ContinuousIntegrationTest(estimatedDuration = 1.1)
   @Test(timeout = 30000)
   public void testNotEnforcingFinalVelocity()
   {
      double maxInitialVelocity = 0.2;
      double velocityStep = 0.01;
      double maxEndVelocity = 0.15;
      double maxAcceleration = maxInitialVelocity * 5.0;
      double accelerationStep = maxAcceleration / 20.0;
      double maxMoveDistance = 0.2;
      double moveDistanceStep = 0.005;

      int counter = 0;
      for (double initialVelocity = 0.0; initialVelocity < maxInitialVelocity; initialVelocity += velocityStep)
      {
         for (double endVelocity = 0.0; endVelocity < maxEndVelocity; endVelocity += velocityStep)
         {
            for (double acceleration = maxAcceleration; acceleration > 0.1 * maxAcceleration; acceleration -= accelerationStep)
            {
               for (double moveDistance = 0.001; moveDistance < maxMoveDistance; moveDistance += moveDistanceStep)
               {
                  TrapezoidalVelocityTrajectory trapezoidalVelocityTrajectory;

                  try
                  {
                     trapezoidalVelocityTrajectory = new TrapezoidalVelocityTrajectory(0.0, 0.0, moveDistance, initialVelocity, endVelocity,
                             maxInitialVelocity, acceleration, false);
                  }
                  catch (RuntimeException e)
                  {
                     String errorMessage = "Failed constructing a TrapezoidalVelocityTrajectory with: \n" + "moveDistance = " + moveDistance + "\n"
                                           + "initialVelocity = " + initialVelocity + "\n" + "endVelocity = " + endVelocity + "\n" + "maxInitialVelocity = "
                                           + maxInitialVelocity + "\n" + "acceleration = " + acceleration + "\n";
                     System.out.println(errorMessage);

                     System.out.println(e.getMessage());

                     throw e;
                  }

                  double endTime = trapezoidalVelocityTrajectory.getFinalTime();

                  String errorMessage = "endTime < 0.0: t0 = " + 0.0 + ", x0 = " + 0.0 + ", xF = " + moveDistance + ", v0 = " + initialVelocity + ", vF = "
                                        + endVelocity + ", vMax = " + maxInitialVelocity + " aMax = " + acceleration + ", enforceFinalVelocity=" + false;

                  assertTrue("endTime < 0.0: " + errorMessage, endTime >= 0.0);

                  double endPosition = trapezoidalVelocityTrajectory.getPosition(endTime);
                  double moveDistanceError = Math.abs(endPosition - moveDistance);
                  assertTrue("moveDistanceError = " + moveDistanceError + " : " + errorMessage, Math.abs(endPosition - moveDistance) < 0.001 * moveDistance);

                  double endSpeed = trapezoidalVelocityTrajectory.getVelocity(endTime);


                  // +++tk 090219 split this into two asserts
                  assertTrue("endSpeed=" + endSpeed + ", : " + errorMessage, endSpeed > -1e-6);

                  assertTrue("endSpeed=" + endSpeed + ", : " + errorMessage, endSpeed <= maxInitialVelocity);    // +++tk 090219 changed < to <=

                  counter++;
                  int counterDivisor = 100000;
                  if (counter % counterDivisor == 0)
                  {
                     if (VERBOSE) System.out.println("counter = " + counter);
                  }
               }
            }
         }
      }

   }


	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testProblemOne()
   {
      // Sometimes have a problem when vF == vMax.

      double t0 = 0.0;
      double x0 = 0.0;
      double v0 = 0.0;
      double xF = 1.0;
      double vF = 0.8770323306495469;
      double vMax = 0.8770323306495469;
      double aMax = 5.585490187816721;

      TrapezoidalVelocityTrajectory trap = new TrapezoidalVelocityTrajectory(t0, x0, xF, v0, vF, vMax, aMax);
      performTests(trap, t0, x0, v0, xF, vF, vMax, aMax);

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testProblemTwo()
   {
      double t0 = -6.454075981539176;
      double x0 = -4.591728723884476;
      double v0 = 0.933477719991699;
      double xF = 3.9492869787051283;
      double vF = -3.899870123248803;
      double vMax = 8.207283948162281;
      double aMax = 0.9784794691659375;

      TrapezoidalVelocityTrajectory trap = new TrapezoidalVelocityTrajectory(t0, x0, xF, v0, vF, vMax, aMax);

//    plotTrapezoid(trap, t0);
//    sleepForever();

      performTests(trap, t0, x0, v0, xF, vF, vMax, aMax);

   }

	/**
	 * Never really worked yet. Some day maybe.
	 */
   @ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 300000)
   public void DONTtestRandomSamples()
   {
      //TODO: This test binds at test number 110 or so!!!

      Random random = new Random(1972L);

      int numTests = 1000;

      for (int i = 0; i < numTests; i++)
      {
         double t0 = nextDouble(random, -10.0, 10.0);
         double x0 = nextDouble(random, -5.0, 5.0);
         double v0 = nextDouble(random, -5.0, 5.0);
         double xF = nextDouble(random, -5.0, 5.0);
         double vF = nextDouble(random, -5.0, 5.0);

         double vMax = nextDouble(random, 0.01, 10.0);
         double aMax = nextDouble(random, 0.01, 10.0);

//       t0 = 0.0;
//       x0 = 0.0;
//       v0 = 0.0;
//       xF = 1.0;
//       vF = 0.8770323306495469;
//       vMax = 0.8770323306495469;
//       aMax = 5.585490187816721;

         // Limit things:
         double epsilon = 1e-7;

         if (v0 > vMax)
            v0 = vMax - epsilon;
         if (v0 < -vMax)
            v0 = -vMax + epsilon;

         if (vF > vMax)
            vF = vMax - epsilon;
         if (vF < -vMax)
            vF = -vMax + epsilon;

         TrapezoidalVelocityTrajectory trap = null;

         try
         {
            System.out.println("x0 = " + x0 + ", v0 = " + v0 + ", xF = " + xF + ", vF = " + vF + ", vMax = " + vMax + ", aMax = "
                  + aMax);
            trap = new TrapezoidalVelocityTrajectory(t0, x0, xF, v0, vF, vMax, aMax);
            performTests(trap, t0, x0, v0, xF, vF, vMax, aMax);
         }
         catch (RuntimeException e)
         {
            System.err.println("Exception! t0 = " + t0 + ", x0 = " + x0 + ", v0 = " + v0 + ", xF = " + xF + ", vF = " + vF + ", vMax = " + vMax + ", aMax = "
                               + aMax);

            throw e;
         }

         if (i % 10 == 0)
         {
            System.out.println("Test number " + i);
         }
      }
   }

   private double nextDouble(Random random, double min, double max)
   {
      return min + random.nextDouble() * (max - min);
   }

   private void performTests(TrapezoidalVelocityTrajectory trap, double t0, double x0, double v0, double xF, double vF, double vMax, double aMax)
   {
      verifyTimesInOrder(trap);

      verifyInitialAndFinalConditions(trap, t0, x0, v0, xF, vF, vMax, aMax);

      verifyCanStartPartWayThrough(trap, xF, vF, 0.0);
      verifyCanStartPartWayThrough(trap, xF, vF, 0.2);
      verifyCanStartPartWayThrough(trap, xF, vF, 0.5);
      verifyCanStartPartWayThrough(trap, xF, vF, 0.7);

//    verifyCanStartPartWayThrough(trap, xF, vF, 1.0);

      verifyMaxVelocityNotExceeded(trap);

   }

   private void verifyMaxVelocityNotExceeded(TrapezoidalVelocityTrajectory trap)
   {
      double t0 = trap.getT0();
      @SuppressWarnings("unused")
      double x0 = trap.getX0();
      @SuppressWarnings("unused")
      double v0 = trap.getV0();
      @SuppressWarnings("unused")
      double t1 = trap.getT1();
      @SuppressWarnings("unused")
      double t2 = trap.getT2();
      double tF = trap.getFinalTime();

      double vMax = trap.getVMax();
      @SuppressWarnings("unused")
      double aMax = trap.getAMax();

      for (double t = t0; t < tF; t = t + 0.001)
      {
         double v = trap.getVelocity(t);

         if (Math.abs(v) > vMax + 1e-7)
            throw new RuntimeException("Math.abs(v) > vMax + 1e-7. v = " + v + ", vMax = " + vMax);
      }

   }

   private void verifyCanStartPartWayThrough(TrapezoidalVelocityTrajectory trap, double xF, double vF, double percentThrough)
   {
      double t0 = trap.getT0();
      double x0 = trap.getX0();
      double v0 = trap.getV0();
      double t1 = trap.getT1();
      double t2 = trap.getT2();
      double tF = trap.getFinalTime();

      double vMax = trap.getVMax();
      double aMax = trap.getAMax();

      double tPartWay = t0 + percentThrough * (tF - t0);

      double xPartWay = trap.getPosition(tPartWay);
      double vPartWay = trap.getVelocity(tPartWay);

      try
      {
         TrapezoidalVelocityTrajectory newTrapezoid = new TrapezoidalVelocityTrajectory(tPartWay, xPartWay, xF, vPartWay, vF, vMax, aMax);

         double newT1 = newTrapezoid.getT1();
         double newT2 = newTrapezoid.getT2();
         double newTF = newTrapezoid.getFinalTime();

         if (t1 > tPartWay)
         {
            assertEpsilonEquals(t1, newT1, 1e-7);
         }

         if (t2 > tPartWay)
         {
//          assertEpsilonEquals(t2, newT2, 1e-7);
            if (!(Math.abs(t2 - newT2) < 1e-7))
            {
               plotTrapezoid(trap, t0);
               plotTrapezoid(newTrapezoid, tPartWay);
               //sleepForever();
               throw new RuntimeException("Something needs to be debugged in TrapezoidalVelocityTrajectory!! t0 = " + t0 + ", x0 = " + x0 + ", v0 = " + v0 + ", xF = " + xF + ", vF = " + vF + ", vMax = " + vMax + ", aMax = "
                     + aMax);
            }
         }

         assertEpsilonEquals(tF, newTF, 1e-7);

      }
      catch (RuntimeException e)
      {
         System.err.println("\nException in verifyCanStartPartWayThrough! t0 = " + t0 + ", x0 = " + x0 + ", v0 = " + v0 + ", xF = " + xF + ", vF = " + vF
                            + ", vMax = " + vMax + ", aMax = " + aMax);
         System.err.println("                tPartWay = " + tPartWay + ", xPartWay = " + xPartWay + ", vPartWay = " + vPartWay + ", xF = " + xF + ", vF = "
                            + vF + ", vMax = " + vMax + ", aMax = " + aMax);
         System.err.println();

         throw e;
      }

   }


   private void verifyInitialAndFinalConditions(TrapezoidalVelocityTrajectory trap, double t0, double x0, double v0, double xF, double vF, double vMax,
           double aMax)
   {
      assertTrue(t0 == trap.getT0());
      assertTrue(x0 == trap.getX0());
      assertTrue(v0 == trap.getV0());

      double tF = trap.getFinalTime();
      double xFFromTrap = trap.getPosition(tF);
      double vFFromTrap = trap.getVelocity(tF);

      assertEpsilonEquals(xF, xFFromTrap, 1e-7);
      assertEpsilonEquals(vF, vFFromTrap, 1e-7);
   }

   private void verifyInitialConditionsAndFinalPosition(TrapezoidalVelocityTrajectory trap, double t0, double x0, double v0, double xF)
   {
      assertTrue(t0 == trap.getT0());
      assertTrue(x0 == trap.getX0());
      assertTrue(v0 == trap.getV0());

      double tF = trap.getFinalTime();
      double xFFromTrap = trap.getPosition(tF);

      assertEpsilonEquals(xF, xFFromTrap, 1e-7);
   }

   private void assertEpsilonEquals(double v1, double v2, double epsilon)
   {
      assertTrue("v1 does not epsilonEqual v2. v1 = " + v1 + ", v2 = " + v2, Math.abs(v1 - v2) <= epsilon);
   }

   private void assertEpsilonLessThan(String message, double v1, double v2, double epsilon)
   {
      assertTrue(message, v1 <= v2 + epsilon);
   }

   @SuppressWarnings("unused")
   private void assertEpsilonLessThan(String message, double v1, double v2)
   {
      assertEpsilonLessThan(message, v1, v2, 1e-7);
   }

   private void verifyTimesInOrder(TrapezoidalVelocityTrajectory trap)
   {
      double t0 = trap.getT0();
      double t1 = trap.getT1();
      double t2 = trap.getT2();
      double tF = trap.getFinalTime();

      String message = "t0 = " + t0 + ", t1 = " + t1 + ", t2 = " + t2 + ", tF = " + tF;
      assertEpsilonLessThan(message, t0, t1, trap.getEpsilon());
      assertEpsilonLessThan(message, t1, t2, trap.getEpsilon());
      assertEpsilonLessThan(message, t2, tF, trap.getEpsilon());
   }

   private void plotTrapezoid(TrapezoidalVelocityTrajectory trap, double t0)
   {
      plotTrapezoid(trap, t0, trap.getFinalTime());
   }

   private void plotTrapezoid(TrapezoidalVelocityTrajectory trap, double t0, double finalTime)
   {
      double dT = 2.5e-2;
      int numberOfPoints = (int) (finalTime / dT);

      double[][] position = new double[2][numberOfPoints];
      double[][] velocity = new double[2][numberOfPoints];

      for (int i = 0; i < numberOfPoints; i++)
      {
         double t = t0 + i * dT;
         double[] state = trap.getState(t);

         // Save trajectory:
         position[0][i] = t;
         position[1][i] = state[0];

         velocity[0][i] = t;
         velocity[1][i] = state[1];

//       assertTrue(velocity[1][i] <= vMax);
      }

      // Position plot:
      ArrayList<double[][]> listOfCurves1 = new ArrayList<double[][]>();
      listOfCurves1.add(position);

//    PlotGraph2d pg1 = PlotGraph2d.createPlotGraph2dMultipleCurves(listOfCurves1);
//    pg1.plot();
//    pg1.setCloseChoice(PlotGraph2d.CLOSE_WINDOW_ONLY);
//    pg1.setGraphTitle("Position profile");

      // Velocity plot:
      ArrayList<double[][]> listOfCurves2 = new ArrayList<double[][]>();
      listOfCurves2.add(velocity);

//    PlotGraph2d pg2 = PlotGraph2d.createPlotGraph2dMultipleCurves(listOfCurves2);
//    pg2.setGraphTitle("Velocity profile");
//    pg2.setCloseChoice(PlotGraph2d.CLOSE_WINDOW_ONLY);
//    pg2.plot();



   }




   public void DONTtestMaxSpeed()
   {
      double t0 = 1.0;
      double x0 = 0.0;
      double xF = 4.0;
      double v0 = -.2;
      double vF = -0.2;
      double vMax = 0.2;
      double aMax = 1.0;

      TrapezoidalVelocityTrajectory trap = new TrapezoidalVelocityTrajectory(t0, x0, xF, v0, vF, vMax, aMax);
      System.out.println("t1 = " + trap.getT1());
      System.out.println("t2 = " + trap.getT2());
      System.out.println("tF = " + trap.getFinalTime());

      double dT = 2.5e-2;
      double tMax = 60.0;
      int numberOfPoints = (int) (tMax / dT);

      double[][] position = new double[2][numberOfPoints];
      double[][] velocity = new double[2][numberOfPoints];

      for (int i = 0; i < numberOfPoints; i++)
      {
         double t = t0 + i * dT;
         double[] state = trap.getState(t);

         // Save trajectory:
         position[0][i] = t;
         position[1][i] = state[0];

         velocity[0][i] = t;
         velocity[1][i] = state[1];
         assertTrue(velocity[1][i] <= vMax);
      }

      // Position plot:
      ArrayList<double[][]> listOfCurves1 = new ArrayList<double[][]>();
      listOfCurves1.add(position);

//    PlotGraph2d pg1 = PlotGraph2d.createPlotGraph2dMultipleCurves(listOfCurves1);
//    pg1.plot();
//    pg1.setCloseChoice(PlotGraph2d.CLOSE_WINDOW_ONLY);
//    pg1.setGraphTitle("Position profile");

      // Velocity plot:
      ArrayList<double[][]> listOfCurves2 = new ArrayList<double[][]>();
      listOfCurves2.add(velocity);

//    PlotGraph2d pg2 = PlotGraph2d.createPlotGraph2dMultipleCurves(listOfCurves2);
//    pg2.setGraphTitle("Velocity profile");
//    pg2.setCloseChoice(PlotGraph2d.CLOSE_WINDOW_ONLY);
//    pg2.plot();

      sleepForever();
   }

   public void DONTtestTiming()
   {
      // set up
      int numberOfTests = 1000000;
      Random random = new Random(100l);

      double t0;
      double x0;
      double xF;
      double v0;
      double vF;
      double vMax;
      double aMax;

      long startTime = System.currentTimeMillis();
      for (int i = 0; i < numberOfTests; i++)
      {
         // do test
         t0 = random.nextDouble();
         vMax = random.nextDouble();
         aMax = random.nextDouble();

         x0 = random.nextDouble() - 0.5;
         xF = random.nextDouble() - 0.5;
         v0 = 2.0 * vMax * (random.nextDouble() - 0.5);
         vF = 2.0 * vMax * (random.nextDouble() - 0.5);

         TrapezoidalVelocityTrajectory trap = new TrapezoidalVelocityTrajectory(t0, x0, xF, v0, vF, vMax, aMax);
         if (Double.isNaN(trap.getDT1()) || Double.isNaN(trap.getDT2()) || Double.isNaN(trap.getMoveDuration()))
         {
            throw new RuntimeException("Double.isNaN(trap.getT1()) || Double.isNaN(trap.getT2()) || Double.isNaN(trap.getTF())");
         }
      }

      long endTime = System.currentTimeMillis();

      double totalTime = (endTime - startTime) * 0.001;

      double timePerTest = totalTime / ((double) numberOfTests);

      System.out.println("Finding a random trapezoidal velocity profile trajectory took " + timePerTest * 1000.0 + " milliseconds per test.");

      double maxTimeAllowed = 0.1 * 0.001;
      if (timePerTest > maxTimeAllowed)
         throw new RuntimeException();
   }


   private void sleepForever()
   {
      while (true)
      {
         try
         {
            Thread.sleep(1000);
         }
         catch (InterruptedException ex)
         {
         }
      }
   }
}
