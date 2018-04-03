package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.AMGeneration;

import static org.junit.Assert.assertTrue;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.commons.Epsilons;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.Trajectory;
import us.ihmc.robotics.math.trajectories.TrajectoryMathTools;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class TorqueTrajectoryTest
{
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final int minSegments = 2;
   private final int maxSegments = 10;
   private final int minCoefficients = 3;
   private final int maxCoefficients = 14;
   private AngularMomentumTrajectory angularMomentumTrajectory;
   private TorqueTrajectory torqueTrajectory;
   private FrameTrajectory3D calculatedTrajectory;
   private static final double epsilon = Epsilons.ONE_HUNDRED_BILLIONTH;

   @Before
   public void setup()
   {
      int numberOfSegments = minSegments + (int) Math.floor(Math.random() * (maxSegments - minSegments));
      int numberOfCoefficients = minCoefficients + (int) Math.floor(Math.random() * (maxCoefficients - minCoefficients));
      angularMomentumTrajectory = new AngularMomentumTrajectory(numberOfSegments, numberOfCoefficients);
      torqueTrajectory = new TorqueTrajectory(numberOfSegments, numberOfCoefficients);
      generateRandomAngularMomentumTrajectory(angularMomentumTrajectory);
      torqueTrajectory.setNext(angularMomentumTrajectory);
      calculatedTrajectory = new FrameTrajectory3D(numberOfCoefficients, worldFrame);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.7)
   @Test(timeout = 30000)
   public void testSetter()
   {
      assertTrue("Got incorrect number of segments, got: " + torqueTrajectory.getNumberOfSegments() + " should have been: "
            + angularMomentumTrajectory.getNumberOfSegments(), torqueTrajectory.getNumberOfSegments() == angularMomentumTrajectory.getNumberOfSegments());
      for (int i = 0; i < torqueTrajectory.getNumberOfSegments(); i++)
      {
         TrajectoryMathTools.getDerivative(calculatedTrajectory.getTrajectoryX(), angularMomentumTrajectory.getSegment(i).getTrajectoryY());
         TrajectoryMathTools.getDerivative(calculatedTrajectory.getTrajectoryY(), angularMomentumTrajectory.getSegment(i).getTrajectoryX());
         TrajectoryMathTools.scale(calculatedTrajectory.getTrajectoryY(), -1.0);
         calculatedTrajectory.getTrajectoryZ().setConstant(angularMomentumTrajectory.getSegment(i).getInitialTime(Axis.X),
                                                           angularMomentumTrajectory.getSegment(i).getFinalTime(Axis.X), 0.0);
         assertTrue("Failed for segment " + i + " wanted: \n" + calculatedTrajectory.toString() + " got: \n" + torqueTrajectory.getSegment(i).toString()
               + " from: \n" + angularMomentumTrajectory.getSegment(i).toString(),
                    TrajectoryMathTools.epsilonEquals(torqueTrajectory.getSegment(i), calculatedTrajectory, epsilon));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.7)
   @Test(timeout = 30000)
   public void testScaling()
   {
      double scalar = Math.random();
      torqueTrajectory.scale(scalar);
      assertTrue("Got incorrect number of segments, got: " + torqueTrajectory.getNumberOfSegments() + " should have been: "
            + angularMomentumTrajectory.getNumberOfSegments(), torqueTrajectory.getNumberOfSegments() == angularMomentumTrajectory.getNumberOfSegments());
      for (int i = 0; i < angularMomentumTrajectory.getNumberOfSegments(); i++)
      {
         TrajectoryMathTools.getDerivative(calculatedTrajectory, angularMomentumTrajectory.getSegment(i));
         Trajectory testTrajectory = torqueTrajectory.getSegment(i).getTrajectoryX();
         Trajectory controlTrajectory = calculatedTrajectory.getTrajectoryY();
         assertTrue("Got incorrect number of coefficients for segment " + i + "got: " + testTrajectory.getNumberOfCoefficients() + "  should have been: "
               + controlTrajectory.getNumberOfCoefficients(), testTrajectory.getNumberOfCoefficients() == controlTrajectory.getNumberOfCoefficients());
         for (int j = 0; j < controlTrajectory.getNumberOfCoefficients(); j++)
         {
            assertTrue(MathTools.epsilonEquals(testTrajectory.getCoefficient(j) / controlTrajectory.getCoefficient(j), scalar, epsilon));
         }
         testTrajectory = torqueTrajectory.getSegment(i).getTrajectoryY();
         controlTrajectory = calculatedTrajectory.getTrajectoryX();
         assertTrue("Got incorrect number of coefficients for segment " + i + "got: " + testTrajectory.getNumberOfCoefficients() + "  should have been: "
               + controlTrajectory.getNumberOfCoefficients(), testTrajectory.getNumberOfCoefficients() == controlTrajectory.getNumberOfCoefficients());
         for (int j = 0; j < controlTrajectory.getNumberOfCoefficients(); j++)
         {
            assertTrue(MathTools.epsilonEquals(testTrajectory.getCoefficient(j) / controlTrajectory.getCoefficient(j), -scalar, epsilon));
         }

         testTrajectory = torqueTrajectory.getSegment(i).getTrajectoryZ();
         assertTrue("Got incorrect number of coefficients for segment " + i + "got: " + testTrajectory.getNumberOfCoefficients() + "  should have been: 1",
                    1 == testTrajectory.getNumberOfCoefficients());
         assertTrue(MathTools.epsilonEquals(testTrajectory.getCoefficient(0), 0.0, epsilon));
      }
   }

   private void generateRandomAngularMomentumTrajectory(AngularMomentumTrajectory trajectoryToSet)
   {
      double coefficients[];
      trajectoryToSet.reset();
      for (int i = 0; i < trajectoryToSet.getMaxNumberOfSegments(); i++)
      {
         FrameTrajectory3D randomTrajectory = trajectoryToSet.add();
         coefficients = new double[randomTrajectory.getTrajectoryX().getMaximumNumberOfCoefficients()];
         for (int j = 0; j < coefficients.length; j++)
         {
            coefficients[j] = Math.random();
         }
         randomTrajectory.getTrajectoryX().setDirectly(coefficients);
         coefficients = new double[randomTrajectory.getTrajectoryY().getMaximumNumberOfCoefficients()];
         for (int j = 0; j < coefficients.length; j++)
         {
            coefficients[j] = Math.random();
         }
         randomTrajectory.getTrajectoryY().setDirectly(coefficients);

         coefficients = new double[randomTrajectory.getTrajectoryZ().getMaximumNumberOfCoefficients()];
         for (int j = 0; j < coefficients.length; j++)
         {
            coefficients[j] = Math.random();
         }
         randomTrajectory.getTrajectoryZ().setDirectly(coefficients);
         randomTrajectory.setInitialTime(Math.random());
         randomTrajectory.setFinalTime(Math.random());
      }
   }
}
