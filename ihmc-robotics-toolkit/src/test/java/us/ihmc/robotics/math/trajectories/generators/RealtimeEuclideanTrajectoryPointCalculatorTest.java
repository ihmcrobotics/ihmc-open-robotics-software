package us.ihmc.robotics.math.trajectories.generators;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class RealtimeEuclideanTrajectoryPointCalculatorTest
{
   @Test
   public void testChangeReferenceFrame()
   {
      RealtimeEuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new RealtimeEuclideanTrajectoryPointCalculator();

      Random random = new Random(65444);
      ReferenceFrame referenceFrameNotWorld = EuclidFrameRandomTools.nextReferenceFrame(random);

      euclideanTrajectoryPointCalculator.changeFrame(referenceFrameNotWorld);

      assertEquals(referenceFrameNotWorld, euclideanTrajectoryPointCalculator.getTrajectoryPoints().getReferenceFrame());
   }
}
