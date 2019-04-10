package us.ihmc.manipulation.planning.gradientDescent;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.robotics.math.trajectories.generators.SO3TrajectoryPointCalculator;
import us.ihmc.robotics.random.RandomGeometry;

public class SO3TrajectoryPointCalculatorTest
{
   private final Random random = new Random(0612L);

   private final List<Quaternion> orientations = new ArrayList<>();
   private final TDoubleArrayList times = new TDoubleArrayList();

   private void generateRandomOrientations(double trajectoryTime, int numberOfPoints, boolean useGentleRandomGeneration)
   {
      orientations.add(new Quaternion());
      times.add(0.0);
      for (int i = 1; i < numberOfPoints; i++)
      {
         Quaternion qt;
         if (useGentleRandomGeneration)
            qt = createRandomOrientationFromPrevious(orientations.get(i - 1), Math.PI * 0.25);
         else
            qt = RandomGeometry.nextQuaternion(new Random());
         orientations.add(qt);
         times.add(trajectoryTime / (numberOfPoints - 1) * (i));
      }
   }

   private Quaternion createRandomOrientationFromPrevious(QuaternionBasics previousOrientation, double maximumRotation)
   {
      Quaternion randomOrientation = new Quaternion(previousOrientation);

      int axisIndex = random.nextInt(3);
      double appendingAngle = random.nextDouble() * maximumRotation;

      if (axisIndex == 0)
         previousOrientation.appendRollRotation(appendingAngle);
      else if (axisIndex == 1)
         previousOrientation.appendPitchRotation(appendingAngle);
      else if (axisIndex == 2)
         previousOrientation.appendYawRotation(appendingAngle);
      else
         ;

      return randomOrientation;
   }

   @Test
   public void testBestWayToCalculateInitialGuess()
   {
      System.out.println("\n## testBestWayToCalculateInitialGuess");
      int numberOfPoints = 100;
      double trajectoryTime = 5.0;
      generateRandomOrientations(trajectoryTime, numberOfPoints, true);

      SO3TrajectoryPointCalculator trajectoryPointCalculator = new SO3TrajectoryPointCalculator();

      System.out.println("without initial guess");
      trajectoryPointCalculator.clear();
      for (int i = 0; i < numberOfPoints; i++)
         trajectoryPointCalculator.appendTrajectoryPoint(times.get(i), orientations.get(i));
      trajectoryPointCalculator.compute();

      System.out.println("with first order initial guess");
      trajectoryPointCalculator.clear();
      for (int i = 0; i < numberOfPoints; i++)
         trajectoryPointCalculator.appendTrajectoryPoint(times.get(i), orientations.get(i));

      trajectoryPointCalculator.useFirstOrderInitialGuess();
      trajectoryPointCalculator.compute();

      System.out.println("with second order initial guess");
      trajectoryPointCalculator.clear();
      for (int i = 0; i < numberOfPoints; i++)
         trajectoryPointCalculator.appendTrajectoryPoint(times.get(i), orientations.get(i));

      trajectoryPointCalculator.useSecondOrderInitialGuess();
      trajectoryPointCalculator.compute();
   }

   @Test
   public void testSO3TrajectoryPointCalculator()
   {
      System.out.println("\n## testSO3TrajectoryPointCalculator");
      SO3TrajectoryPointCalculator trajectoryPointCalculator = new SO3TrajectoryPointCalculator();

      int numberOfPoints = 100;
      double trajectoryTime = 5.0;
      generateRandomOrientations(trajectoryTime, numberOfPoints, true);

      for (int i = 0; i < numberOfPoints; i++)
      {
         trajectoryPointCalculator.appendTrajectoryPoint(times.get(i), orientations.get(i));
      }

      trajectoryPointCalculator.useSecondOrderInitialGuess();
      trajectoryPointCalculator.compute();
   }

   @Test
   public void testFastComputation()
   {
      System.out.println("\n## testFastComputation");
      SO3TrajectoryPointCalculator trajectoryPointCalculator = new SO3TrajectoryPointCalculator();

      int numberOfPoints = 100;
      double trajectoryTime = 5.0;
      generateRandomOrientations(trajectoryTime, numberOfPoints, true);

      System.out.println("fastComputation");

      trajectoryPointCalculator.clear();
      for (int i = 0; i < numberOfPoints; i++)
         trajectoryPointCalculator.appendTrajectoryPoint(times.get(i), orientations.get(i));

      long start = System.nanoTime();
      trajectoryPointCalculator.useSecondOrderInitialGuess();
      trajectoryPointCalculator.compute();
      long computingTime = System.nanoTime() - start;
      System.out.println("computation time for the " + numberOfPoints + " way points, " + Conversions.nanosecondsToSeconds(computingTime));
   }
}
