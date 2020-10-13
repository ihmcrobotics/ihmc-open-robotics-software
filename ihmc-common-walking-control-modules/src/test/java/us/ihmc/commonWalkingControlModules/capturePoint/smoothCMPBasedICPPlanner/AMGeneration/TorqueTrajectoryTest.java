package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.AMGeneration;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.RandomNumbers;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.Trajectory;
import us.ihmc.robotics.math.trajectories.TrajectoryMathTools;

import java.util.Random;

public class TorqueTrajectoryTest
{
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double epsilon = 1e-6;
   private static final int iters = 100;

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testSetter()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < iters; iter++)
      {
         int numberOfSegments = RandomNumbers.nextInt(random, 2, 10);
         int numberOfCoefficients = RandomNumbers.nextInt(random, 3, 14);
         double verticalScalar = RandomNumbers.nextDouble(random, 0.1, 1000.0);

         AngularMomentumTrajectory angularMomentumTrajectory = generateRandomAngularMomentumTrajectory(random, numberOfSegments, numberOfCoefficients);
         TorqueTrajectory torqueTrajectory = new TorqueTrajectory(numberOfSegments, numberOfCoefficients);
         FrameTrajectory3D calculatedTrajectory = new FrameTrajectory3D(numberOfCoefficients, worldFrame);

         torqueTrajectory.setFromAngularMomentumTrajectory(angularMomentumTrajectory, verticalScalar);

         assertTrue("Got incorrect number of segments, got: " + torqueTrajectory.getNumberOfSegments() + " should have been: " + angularMomentumTrajectory
               .getNumberOfSegments(), torqueTrajectory.getNumberOfSegments() == angularMomentumTrajectory.getNumberOfSegments());

         for (int i = 0; i < torqueTrajectory.getNumberOfSegments(); i++)
         {
            TrajectoryMathTools.getDerivative(calculatedTrajectory.getTrajectoryX(), angularMomentumTrajectory.getSegment(i).getTrajectoryY());
            TrajectoryMathTools.getDerivative(calculatedTrajectory.getTrajectoryY(), angularMomentumTrajectory.getSegment(i).getTrajectoryX());
            calculatedTrajectory.getTrajectoryZ().setConstant(angularMomentumTrajectory.getSegment(i).getInitialTime(Axis3D.X),
                                                              angularMomentumTrajectory.getSegment(i).getFinalTime(Axis3D.X), 0.0);
            TrajectoryMathTools.scale(calculatedTrajectory.getTrajectoryY(), -1.0);
            TrajectoryMathTools.scale(1.0 / verticalScalar, calculatedTrajectory);

            assertTrue("Failed for segment " + i + " wanted: \n" + calculatedTrajectory.toString() + " got: \n" + torqueTrajectory.getSegment(i).toString()
                             + " from: \n" + angularMomentumTrajectory.getSegment(i).toString(),
                       TrajectoryMathTools.epsilonEquals(torqueTrajectory.getSegment(i), calculatedTrajectory, epsilon));
         }
      }
   }

   @Test
   public void testSetter2()
   {
      Random random = new Random(1738L);

      int numberOfSegments = 1;
      int numberOfCoefficients = 4;

      double verticalScalar = RandomNumbers.nextDouble(random, 0.1, 1000.0);

      AngularMomentumTrajectory angularMomentumTrajectory = generateRandomAngularMomentumTrajectory(random, numberOfSegments, numberOfCoefficients);
      TorqueTrajectory torqueTrajectory = new TorqueTrajectory(numberOfSegments, numberOfCoefficients);
      FrameTrajectory3D calculatedTrajectory = new FrameTrajectory3D(numberOfCoefficients, worldFrame);

      torqueTrajectory.setFromAngularMomentumTrajectory(angularMomentumTrajectory, verticalScalar);

      assertTrue("Got incorrect number of segments, got: " + torqueTrajectory.getNumberOfSegments() + " should have been: " + angularMomentumTrajectory
            .getNumberOfSegments(), torqueTrajectory.getNumberOfSegments() == angularMomentumTrajectory.getNumberOfSegments());

      for (int i = 0; i < torqueTrajectory.getNumberOfSegments(); i++)
      {
         TrajectoryMathTools.getDerivative(calculatedTrajectory.getTrajectoryX(), angularMomentumTrajectory.getSegment(i).getTrajectoryY());
         TrajectoryMathTools.getDerivative(calculatedTrajectory.getTrajectoryY(), angularMomentumTrajectory.getSegment(i).getTrajectoryX());
         calculatedTrajectory.getTrajectoryZ().setConstant(angularMomentumTrajectory.getSegment(i).getInitialTime(Axis3D.X),
                                                           angularMomentumTrajectory.getSegment(i).getFinalTime(Axis3D.X), 0.0);
         TrajectoryMathTools.scale(calculatedTrajectory.getTrajectoryY(), -1.0);
         TrajectoryMathTools.scale(1.0 / verticalScalar, calculatedTrajectory);

         assertTrue("Failed for segment " + i + " wanted: \n" + calculatedTrajectory.toString() + " got: \n" + torqueTrajectory.getSegment(i).toString()
                          + " from: \n" + angularMomentumTrajectory.getSegment(i).toString(),
                    TrajectoryMathTools.epsilonEquals(torqueTrajectory.getSegment(i), calculatedTrajectory, epsilon));
      }
   }

   private AngularMomentumTrajectory generateRandomAngularMomentumTrajectory(Random random, int numberOfSegments, int numberOfCoefficients)
   {
      AngularMomentumTrajectory trajectoryToSet = new AngularMomentumTrajectory(numberOfSegments, numberOfCoefficients);

      for (int i = 0; i < trajectoryToSet.getMaxNumberOfSegments(); i++)
      {
         FrameTrajectory3D randomTrajectory = trajectoryToSet.add();

         randomTrajectory.getTrajectoryX().setDirectly(RandomNumbers.nextDoubleArray(random, numberOfCoefficients, 10.0));
         randomTrajectory.getTrajectoryY().setDirectly(RandomNumbers.nextDoubleArray(random, numberOfCoefficients, 10.0));
         randomTrajectory.getTrajectoryZ().setDirectly(RandomNumbers.nextDoubleArray(random, numberOfCoefficients, 10.0));

         double startTime = RandomNumbers.nextDouble(random, 10.0);
         double duration = RandomNumbers.nextDouble(random, 0.0, 100.0);
         randomTrajectory.setInitialTime(startTime);
         randomTrajectory.setFinalTime(startTime + duration);
      }

      return trajectoryToSet;
   }
}
