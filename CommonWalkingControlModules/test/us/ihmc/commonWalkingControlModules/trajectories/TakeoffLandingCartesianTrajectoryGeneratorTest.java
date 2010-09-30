package us.ihmc.commonWalkingControlModules.trajectories;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.utilities.math.Differentiator;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class TakeoffLandingCartesianTrajectoryGeneratorTest
{
   @Test
   public void testOne()
   {
      boolean pause = false;
      System.out.println("Pausing...");
      while(pause)
      {
         try
         {
            Thread.sleep(1000);
         }
         catch (InterruptedException e)
         {
         }
      }

      // TODO: Finish this JUnit test. It tests start, end, and zClearance, but needs to test takeOffSlope, landingSlope, maxVel, maxAccel, some of which are difficult
      double maxAccel = 20.0;
      double maxVel = 2.0;
      double zClearance = 0.1;    // 0.15;
      double takeOffSlope = 0.4;    // 0.3;
      double landingSlope = 0.4;


      runTests(maxAccel, maxVel, zClearance, takeOffSlope, landingSlope);
   }

   private void runTests(double maxAccel, double maxVel, double zClearance, double takeOffSlope, double landingSlope)
   {
      YoVariableRegistry registry = new YoVariableRegistry("R2Sim02CartesianTrajectoryGeneratorTest");

      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
      TakeoffLandingCartesianTrajectoryGenerator generator = new TakeoffLandingCartesianTrajectoryGenerator(maxAccel, maxVel, zClearance, takeOffSlope, landingSlope,
                                                         referenceFrame, registry);

      double[][] initialPositions = new double[][]
      {
         {0.0, 0.0, 0.0}, {0.3, 0.4, 0.0}
      };
      double[][] initialVelocities = new double[][]
      {
         {0.0, 0.0, 0.0}, {0.0, 0.0, 0.4}
      };
      double[][] finalDesiredPositions = new double[][]
      {
         {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {1.0, 1.0, 0.0}
      };

      for (double[] initialPositionArray : initialPositions)
      {
         FramePoint initialPosition = new FramePoint(ReferenceFrame.getWorldFrame(), initialPositionArray);
         for (double[] initialVelocityArray : initialVelocities)
         {
            FrameVector initialVelocity = new FrameVector(ReferenceFrame.getWorldFrame(), initialVelocityArray);
            for (double[] finalDesiredPositionArray : finalDesiredPositions)
            {
               FramePoint finalDesiredPosition = new FramePoint(ReferenceFrame.getWorldFrame(), finalDesiredPositionArray);

               double groundZ = 0.0;

               runOneTest(generator, groundZ, initialPosition, initialVelocity, finalDesiredPosition);

            }
         }
      }


   }

   private void runOneTest(TakeoffLandingCartesianTrajectoryGenerator generator, double groundZ, FramePoint initialPosition, FrameVector initialVelocity,
                           FramePoint finalDesiredPosition)
   {
      generator.initialize(groundZ, initialPosition, initialVelocity, finalDesiredPosition);
      TrajectoryToTest trajectoryToTest = generateTrajectory(generator);

      trajectoryToTest.verifyTrajectoryOnlyCrossesZMaxTwice(generator.getZClearance());
      trajectoryToTest.verifyTrajectoryStartsAtStart(initialPosition);
      trajectoryToTest.verifyInitialVelocity(initialVelocity);
      trajectoryToTest.verifyTrajectoryEndsAtEnd(finalDesiredPosition);
      trajectoryToTest.verifyVelocitiesCorrect();
      trajectoryToTest.verifyAccelerationsCorrect();
   }

   private TrajectoryToTest generateTrajectory(TakeoffLandingCartesianTrajectoryGenerator generator)
   {
      double deltaT = 0.001;

      TrajectoryToTest trajectoryToTest = new TrajectoryToTest(deltaT);

      final int nTrajectoryPoints = 1000;
      for (int i = 0; i < nTrajectoryPoints; i++)
      {
         FramePoint position = new FramePoint(ReferenceFrame.getWorldFrame());
         FrameVector velocity = new FrameVector(ReferenceFrame.getWorldFrame());
         FrameVector acceleration = new FrameVector(ReferenceFrame.getWorldFrame());

         generator.computeNextTick(position, velocity, acceleration, deltaT);

         trajectoryToTest.addPoint(position, velocity, acceleration);
      }

      return trajectoryToTest;
   }

   /**
    * Class represents a trajectory, stored as arraylists of positions, velocities and accelerations.
    */
   private static class TrajectoryToTest
   {
      private final double deltaT;

      ArrayList<FramePoint> positions = new ArrayList<FramePoint>();
      ArrayList<FrameVector> velocities = new ArrayList<FrameVector>();
      ArrayList<FrameVector> accelerations = new ArrayList<FrameVector>();

      public TrajectoryToTest(double deltaT)
      {
         this.deltaT = deltaT;
      }

      public void addPoint(FramePoint position, FrameVector velocity, FrameVector acceleration)
      {
         positions.add(position);
         velocities.add(velocity);
         accelerations.add(acceleration);
      }

      public void verifyTrajectoryOnlyCrossesZMaxTwice(double zMax)
      {
         int numberCrossings = 0;

         boolean below = true;

         for (FramePoint position : positions)
         {
            if (below)
            {
               if (position.getZ() > zMax)
               {
                  numberCrossings++;
                  below = false;
               }
            }
            else
            {
               if (position.getZ() < zMax)
               {
                  numberCrossings++;
                  below = true;
               }
            }
         }

         boolean numberCrossingsEqualsTwo = numberCrossings == 2;
         assertTrue(numberCrossingsEqualsTwo);
      }

      public void verifyTrajectoryStartsAtStart(FramePoint desiredStartPosition)
      {
         FramePoint trajectoryStart = positions.get(0);
         double epsilon = 1e-3;
         boolean startsEqual = trajectoryStart.epsilonEquals(desiredStartPosition, epsilon);
         assertTrue("Start position incorrect" + "\n" + "desiredStartPosition = " + desiredStartPosition + "\n" + "trajectoryStart = " + trajectoryStart,
                    startsEqual);
      }

      public void verifyInitialVelocity(FrameVector desiredInitialVelocity)
      {
         FrameVector initialVelocity = velocities.get(0);

         double epsilon = 1e-1;    // TODO: improve precision!
         boolean startVelocitiesEqual = initialVelocity.epsilonEquals(desiredInitialVelocity, epsilon);
         assertTrue("Start velocity incorrect" + "\n" + "desiredInitialVelocity = " + desiredInitialVelocity + "\n" + "initialVelocity = " + initialVelocity,
                    startVelocitiesEqual);
      }

      public void verifyTrajectoryEndsAtEnd(FramePoint desiredEndPosition)
      {
         FramePoint trajectoryEnd = positions.get(positions.size() - 1);

         double epsilon = 1e-2;
         boolean endsEqual = trajectoryEnd.epsilonEquals(desiredEndPosition, epsilon);
         assertTrue("End position incorrect" + "\n" + "desiredEndPosition = " + desiredEndPosition + "\n" + "trajectoryEnd = " + trajectoryEnd, endsEqual);
      }

      public void verifyVelocitiesCorrect()
      {
         // numerically differentiate to obtain velocities
         ArrayList<FrameVector> numericallyDifferentiatedVelocities = numericallyDifferentiatePoints(this.positions, this.deltaT);

         // compare numerically differentiated velocities and accelerations to velocities and accelerations from trajectory
         double epsilon = 1e-5;
         int nStart = 1;
         for (int i = nStart; i < positions.size() - 1; i++)
         {
            FrameVector numericallyDifferentiatedVelocity = numericallyDifferentiatedVelocities.get(i);
            FrameVector velocityFromTrajectory = velocities.get(i);

            for (int j = 0; j < 3; j++)
            {
               assertEquals(numericallyDifferentiatedVelocity.get(j), velocityFromTrajectory.get(j), epsilon);
            }
         }
      }

      public void verifyAccelerationsCorrect()
      {
         // numerically differentiate to obtain accelerations
         ArrayList<FrameVector> numericallyDifferentiatedAccelerations = numericallyDifferentiateVectors(this.velocities, this.deltaT);

         // compare numerically differentiated velocities and accelerations to velocities and accelerations from trajectory
         double epsilon = 1e-5;
         int nStart = 1;
         int incorrectAccelerations = 0;
         for (int i = nStart; i < positions.size() - 1; i++)
         {
            FrameVector numericallyDifferentiatedAcceleration = numericallyDifferentiatedAccelerations.get(i);
            FrameVector accelerationFromTrajectory = accelerations.get(i);

            for (int j = 0; j < 3; j++)
            {
               if (!MathTools.epsilonEquals(numericallyDifferentiatedAcceleration.get(j), accelerationFromTrajectory.get(j), epsilon))
               {
                  incorrectAccelerations++;

                  break;
               }
            }
         }

         if (incorrectAccelerations > 1)
         {
            fail("Too many incorrect accelerations. Only one allowed (at the state transition from takeoff to cruise)");
         }
      }

      private static ArrayList<FrameVector> numericallyDifferentiatePoints(ArrayList<FramePoint> points, double deltaT)
      {
         ReferenceFrame referenceFrame = points.get(0).getReferenceFrame();

         Differentiator[] differentiators = {new Differentiator(deltaT), new Differentiator(deltaT), new Differentiator(deltaT)};

         ArrayList<FrameVector> ret = new ArrayList<FrameVector>();
         int size = points.size();
         for (int i = 0; i < size; i++)
         {
            FramePoint position = points.get(i);
            for (int j = 0; j < 3; j++)
            {
               differentiators[j].update(position.get(j));
            }

            FrameVector derivative = new FrameVector(referenceFrame, differentiators[0].val(), differentiators[1].val(), differentiators[2].val());
            ret.add(derivative);
         }

         return ret;
      }

      private static ArrayList<FrameVector> numericallyDifferentiateVectors(ArrayList<FrameVector> vectors, double deltaT)
      {
         ReferenceFrame referenceFrame = vectors.get(0).getReferenceFrame();

         Differentiator[] differentiators = {new Differentiator(deltaT), new Differentiator(deltaT), new Differentiator(deltaT)};

         ArrayList<FrameVector> ret = new ArrayList<FrameVector>();
         int size = vectors.size();
         for (int i = 0; i < size; i++)
         {
            FrameVector vector = vectors.get(i);
            for (int j = 0; j < 3; j++)
            {
               differentiators[j].update(vector.get(j));
            }

            FrameVector derivative = new FrameVector(referenceFrame, differentiators[0].val(), differentiators[1].val(), differentiators[2].val());
            ret.add(derivative);
         }

         return ret;
      }
   }    // TrajectoryToTest

}
