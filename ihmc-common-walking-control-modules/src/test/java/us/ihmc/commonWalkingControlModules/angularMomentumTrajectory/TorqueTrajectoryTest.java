package us.ihmc.commonWalkingControlModules.angularMomentumTrajectory;

import static org.junit.Assert.assertTrue;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.AngularMomentumTrajectory;
import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.TorqueTrajectory;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.TrajectoryMathTools;

public class TorqueTrajectoryTest
{
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   @Test
   public void test()
   {
      int numberOfSegments = 10;
      int numberOfCoefficients = 10;
      AngularMomentumTrajectory angularMomentumTrajectory = new AngularMomentumTrajectory(worldFrame, numberOfSegments, numberOfCoefficients);
      TorqueTrajectory torqueTrajectory = new TorqueTrajectory(numberOfSegments, numberOfCoefficients);
      FrameTrajectory3D calculatedTrajectory = new FrameTrajectory3D(numberOfCoefficients, worldFrame);
      generateRandomAngularMomentumTrajectory(angularMomentumTrajectory);
      torqueTrajectory.setNext(angularMomentumTrajectory);
      for (int i = 0; i < torqueTrajectory.getNumberOfSegments(); i++)
      {
         TrajectoryMathTools.getDerivative(calculatedTrajectory.getTrajectoryX(), angularMomentumTrajectory.getSegment(i).getTrajectoryY());
         TrajectoryMathTools.getDerivative(calculatedTrajectory.getTrajectoryY(), angularMomentumTrajectory.getSegment(i).getTrajectoryX());
         TrajectoryMathTools.scale(calculatedTrajectory.getTrajectoryY(), -1.0);
         calculatedTrajectory.getTrajectoryZ().setConstant(angularMomentumTrajectory.getSegment(i).getInitialTime(Direction.X),
                                                           angularMomentumTrajectory.getSegment(i).getFinalTime(Direction.X), 0.0);
         assertTrue("Failed for segment " + i + " wanted: \n" + calculatedTrajectory.toString() + " got: \n"
               + torqueTrajectory.getSegment(i).toString() + " from: \n" + angularMomentumTrajectory.getSegment(i).toString(), TrajectoryMathTools.epsilonEquals(torqueTrajectory.getSegment(i), calculatedTrajectory, Epsilons.ONE_HUNDRED_BILLIONTH));
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
