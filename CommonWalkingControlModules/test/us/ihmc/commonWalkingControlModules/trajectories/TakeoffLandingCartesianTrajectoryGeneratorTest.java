package us.ihmc.commonWalkingControlModules.trajectories;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.ArrayList;
import java.util.EnumMap;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.robotics.numericalMethods.Differentiator;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class TakeoffLandingCartesianTrajectoryGeneratorTest
{
   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }
   
   @After
   public void showMemoryUsageAfterTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

	@EstimatedDuration(duration = 0.5)
	@Test(timeout = 30000)
   public void testOne()
   {
      boolean pause = false;
      if (pause)
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
      generator.initialize(initialPosition, initialVelocity, null, finalDesiredPosition, null);
      TrajectoryToTest trajectoryToTest = generateTrajectory(generator);

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

            for (Direction direction : Direction.values())
            {
               assertEquals(numericallyDifferentiatedVelocity.get(direction), velocityFromTrajectory.get(direction), epsilon);
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

            for (Direction direction : Direction.values())
            {
               if (!MathTools.epsilonEquals(numericallyDifferentiatedAcceleration.get(direction), accelerationFromTrajectory.get(direction), epsilon))
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

         EnumMap<Direction, Differentiator> differentiators = new EnumMap<Direction, Differentiator>(Direction.class);
         for (Direction direction : Direction.values())
         {
            differentiators.put(direction, new Differentiator(deltaT));
         }

         ArrayList<FrameVector> ret = new ArrayList<FrameVector>();
         int size = points.size();
         for (int i = 0; i < size; i++)
         {
            FramePoint position = points.get(i);
            for (Direction direction : Direction.values())
            {
               differentiators.get(direction).update(position.get(direction));
            }

            FrameVector derivative = new FrameVector(referenceFrame, differentiators.get(Direction.X).val(), differentiators.get(Direction.Y).val(), differentiators.get(Direction.Z).val());
            ret.add(derivative);
         }

         return ret;
      }

      private static ArrayList<FrameVector> numericallyDifferentiateVectors(ArrayList<FrameVector> vectors, double deltaT)
      {
         ReferenceFrame referenceFrame = vectors.get(0).getReferenceFrame();

         EnumMap<Direction, Differentiator> differentiators = new EnumMap<Direction, Differentiator>(Direction.class);
         for (Direction direction : Direction.values())
         {
            differentiators.put(direction, new Differentiator(deltaT));
         }

         ArrayList<FrameVector> ret = new ArrayList<FrameVector>();
         int size = vectors.size();
         for (int i = 0; i < size; i++)
         {
            FrameVector vector = vectors.get(i);
            for (Direction direction : Direction.values())
            {
               differentiators.get(direction).update(vector.get(direction));
            }

            FrameVector derivative = new FrameVector(referenceFrame, differentiators.get(Direction.X).val(), differentiators.get(Direction.Y).val(), differentiators.get(Direction.Z).val());
            ret.add(derivative);
         }

         return ret;
      }
   }    // TrajectoryToTest

}
