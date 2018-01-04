package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.configurations.CoPSplineType;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.WalkingTrajectoryType;
import us.ihmc.commons.Epsilons;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.commons.MathTools;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class CoPTrajectoryTest
{
   private static int maxNumberOfSegments = 2;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double epsilon = Epsilons.ONE_BILLIONTH;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLinear()
   {
      CoPTrajectory testTrajectory = new CoPTrajectory(CoPSplineType.LINEAR, maxNumberOfSegments, WalkingTrajectoryType.TRANSFER);
      assertTrue(testTrajectory.getTrajectoryType() == WalkingTrajectoryType.TRANSFER);
      assertTrue(testTrajectory.getMaxNumberOfSegments() == maxNumberOfSegments);
      testTrajectory.reset();
      testTrajectory.setNextSegment(0.0, 1.0, new FramePoint3D(worldFrame, 1.5, 2.6, 10.15), new FramePoint3D(worldFrame, 3.1, -1.5, 0.9));
      assertTrue(testTrajectory.getNumberOfSegments() == 1);
      assertTrue(testTrajectory.getSegment(0).getNumberOfCoefficients() == 2);
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryX().getCoefficient(0), 1.5, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryX().getCoefficient(1), 1.6, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryY().getCoefficient(0), 2.6, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryY().getCoefficient(1), -4.1, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryZ().getCoefficient(0), 10.15, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryZ().getCoefficient(1), -9.25, epsilon));
      testTrajectory.setNextSegment(1.0, 2.0, new FramePoint3D(worldFrame, -2.5, -0.6, -0.8), new FramePoint3D(worldFrame, 1.1, -0.6, 20.5));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryX().getCoefficient(0), 1.5, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryX().getCoefficient(1), 1.6, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryY().getCoefficient(0), 2.6, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryY().getCoefficient(1), -4.1, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryZ().getCoefficient(0), 10.15, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryZ().getCoefficient(1), -9.25, epsilon));
      assertTrue(testTrajectory.getNumberOfSegments() == 2);
      assertTrue(testTrajectory.getSegment(1).getNumberOfCoefficients() == 2);
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(1).getTrajectoryX().getCoefficient(0), -6.1, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(1).getTrajectoryX().getCoefficient(1), 3.6, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(1).getTrajectoryY().getCoefficient(0), -0.6, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(1).getTrajectoryY().getCoefficient(1), 0.0, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(1).getTrajectoryZ().getCoefficient(0), -22.1, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(1).getTrajectoryZ().getCoefficient(1), 21.3, epsilon));
   }


   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCubic()
   {
      CoPTrajectory testTrajectory = new CoPTrajectory(CoPSplineType.CUBIC, maxNumberOfSegments, WalkingTrajectoryType.SWING);
      assertTrue(testTrajectory.getTrajectoryType() == WalkingTrajectoryType.SWING);
      assertTrue(testTrajectory.getMaxNumberOfSegments() == maxNumberOfSegments);
      testTrajectory.reset();
      testTrajectory.setNextSegment(0.0, 1.0, new FramePoint3D(worldFrame, 1.5, 2.6, 10.15), new FramePoint3D(worldFrame, 3.1, -1.5, 0.9));
      assertTrue(testTrajectory.getNumberOfSegments() == 1);
      assertTrue(testTrajectory.getSegment(0).getNumberOfCoefficients() == 4);
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryX().getCoefficient(0), 1.5, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryX().getCoefficient(1), 0.0, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryX().getCoefficient(2), 4.8, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryX().getCoefficient(3), -3.2, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryY().getCoefficient(0), 2.6, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryY().getCoefficient(1), 0.0, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryY().getCoefficient(2), -12.3, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryY().getCoefficient(3), 8.2, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryZ().getCoefficient(0), 10.15, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryZ().getCoefficient(1), 0.0, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryZ().getCoefficient(2), -27.75, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryZ().getCoefficient(3), 18.5, epsilon));
      testTrajectory.setNextSegment(1.0, 2.0, new FramePoint3D(worldFrame, -2.5, -0.6, -0.8), new FramePoint3D(worldFrame, 1.1, -0.6, 20.5));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryX().getCoefficient(0), 1.5, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryX().getCoefficient(1), 0.0, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryX().getCoefficient(2), 4.8, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryX().getCoefficient(3), -3.2, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryY().getCoefficient(0), 2.6, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryY().getCoefficient(1), 0.0, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryY().getCoefficient(2), -12.3, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryY().getCoefficient(3), 8.2, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryZ().getCoefficient(0), 10.15, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryZ().getCoefficient(1), 0.0, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryZ().getCoefficient(2), -27.75, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(0).getTrajectoryZ().getCoefficient(3), 18.5, epsilon));
      assertTrue(testTrajectory.getNumberOfSegments() == 2);
      assertTrue(testTrajectory.getSegment(1).getNumberOfCoefficients() == 4);
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(1).getTrajectoryX().getCoefficient(0), 15.5, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(1).getTrajectoryX().getCoefficient(1), -43.2, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(1).getTrajectoryX().getCoefficient(2), 32.4, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(1).getTrajectoryX().getCoefficient(3), -7.2, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(1).getTrajectoryY().getCoefficient(0), -0.6, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(1).getTrajectoryY().getCoefficient(1), 0.0, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(1).getTrajectoryY().getCoefficient(2), 0.0, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(1).getTrajectoryY().getCoefficient(3), 0.0, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(1).getTrajectoryZ().getCoefficient(0), 105.7, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(1).getTrajectoryZ().getCoefficient(1), -255.6, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(1).getTrajectoryZ().getCoefficient(2), 191.7, epsilon));
      assertTrue(MathTools.epsilonEquals(testTrajectory.getSegment(1).getTrajectoryZ().getCoefficient(3), -42.6, epsilon));
   }

}
