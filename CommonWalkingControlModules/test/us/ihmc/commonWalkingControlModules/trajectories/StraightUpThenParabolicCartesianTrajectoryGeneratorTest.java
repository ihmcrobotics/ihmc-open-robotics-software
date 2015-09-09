package us.ihmc.commonWalkingControlModules.trajectories;

import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.random.RandomTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class StraightUpThenParabolicCartesianTrajectoryGeneratorTest
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

	@DeployableTestMethod(duration = 0.7)
	@Test(timeout = 30000)
   public void testMaxHeight()
   {
      Random random = new Random(176L);

      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();

      FramePoint positionToPack = new FramePoint(referenceFrame);
      FrameVector velocityToPack = new FrameVector(referenceFrame);
      FrameVector accelerationToPack = new FrameVector(referenceFrame);

      int nTests = 100;
      double epsilon = 1e-3;
      for (int i = 0; i < nTests; i++)
      {
         double straightUpVelocity = random.nextDouble();
         double parabolicTime = straightUpVelocity + random.nextDouble();
         double groundClearance = random.nextDouble();
         YoVariableRegistry registry = new YoVariableRegistry("test");
         StraightUpThenParabolicCartesianTrajectoryGenerator trajectoryGenerator = new StraightUpThenParabolicCartesianTrajectoryGenerator("test",
                                                                                      referenceFrame, straightUpVelocity, parabolicTime, groundClearance, registry);

         FramePoint initialPosition = new FramePoint(referenceFrame, RandomTools.generateRandomVector(random));
         FrameVector initialVelocity = new FrameVector(referenceFrame, RandomTools.generateRandomVector(random));
         FramePoint finalDesiredPosition = new FramePoint(initialPosition);
         finalDesiredPosition.add(new FrameVector(referenceFrame, random.nextDouble(), random.nextDouble(), random.nextDouble()));

         trajectoryGenerator.initialize(initialPosition, initialVelocity, null, finalDesiredPosition, null);

         double zMax = finalDesiredPosition.getZ() + groundClearance;
         double minZDifference = Double.POSITIVE_INFINITY;

         double dt = parabolicTime / 2000.0;
         while (!trajectoryGenerator.isDone())
         {
            trajectoryGenerator.computeNextTick(positionToPack, velocityToPack, accelerationToPack, dt);
            double z = positionToPack.getZ();
            if (z > zMax + epsilon)
               fail("z = " + z + ", zMax = " + zMax);

            double zDifference = Math.abs(z - zMax);
            if (zDifference < minZDifference)
               minZDifference = zDifference;
         }

         if (minZDifference > epsilon)
            fail("minZDifference = " + minZDifference);
      }
   }

}
