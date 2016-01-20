package us.ihmc.quadrupedRobotics.stateEstimator.kinematicsBased;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.AxisAngle4d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.OrientationFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.time.GlobalTimer;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.testing.TestPlanTarget;

@DeployableTestClass(targets = TestPlanTarget.Fast)
public class RootJointOrientationCorrectorHelperTest
{

   private final YoVariableRegistry registry = new YoVariableRegistry("test");

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      GlobalTimer.clearTimers();
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @DeployableTestMethod(estimatedDuration = 0.4)
   @Test (timeout = 60000)
   public void testGoalIsAchievedWhenAlphaIsSetToOne()
   {
      Random random = new Random();

      FrameOrientation orientationToBeCorrected = new FrameOrientation(ReferenceFrame.getWorldFrame());
      orientationToBeCorrected.set(RandomTools.generateRandomQuaternion(random, 2.0 * Math.PI));
      //      orientationToBeCorrected.setYawPitchRoll(0.0, 0.0, 0.0);
      OrientationFrame startOrientationFrame = new OrientationFrame(orientationToBeCorrected);

      FrameOrientation goalOrientation = new FrameOrientation(startOrientationFrame);
      goalOrientation.set(RandomTools.generateRandomQuaternion(random, 2.0 * Math.PI));
      //      goalOrientation.setYawPitchRoll(0.1, 0.0, 0.0);
      goalOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      OrientationFrame goalOrientationFrame = new OrientationFrame(goalOrientation);

      FrameOrientation orientationError = new FrameOrientation();
      orientationError.setToZero(goalOrientationFrame);
      orientationError.changeFrame(startOrientationFrame);

      RootJointOrientationCorrectorHelper helper = new RootJointOrientationCorrectorHelper("test", registry);
      helper.setCorrectionAlpha(1.0);

      helper.compensateOrientationError(orientationToBeCorrected, orientationError);

      assertTrue(orientationToBeCorrected.epsilonEquals(goalOrientation, 1e-6));

   }

   @DeployableTestMethod(estimatedDuration = 0.4)
   @Test (timeout = 60000)
   public void testErrorDecreasesAtEveryTick()
   {
      Random random = new Random();

      FrameOrientation orientationError = new FrameOrientation();
      AxisAngle4d errorAxisAngle = new AxisAngle4d();
      double errorAmplitude = 0.0;
      double previousErrorAmplitude = 0.0;

      FrameOrientation orientationToBeCorrected = new FrameOrientation(ReferenceFrame.getWorldFrame());
      orientationToBeCorrected.set(RandomTools.generateRandomQuaternion(random, 2.0 * Math.PI));
      //      orientationToBeCorrected.setYawPitchRoll(0.0, 0.0, 0.0);
      OrientationFrame startOrientationFrame = new OrientationFrame(orientationToBeCorrected);

      FrameOrientation goalOrientation = new FrameOrientation(startOrientationFrame);
      goalOrientation.set(RandomTools.generateRandomQuaternion(random, 2.0 * Math.PI));
      //      goalOrientation.setYawPitchRoll(0.1, 0.0, 0.0);
      goalOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      OrientationFrame goalOrientationFrame = new OrientationFrame(goalOrientation);

      orientationError.setToZero(goalOrientationFrame);
      orientationError.changeFrame(startOrientationFrame);
      orientationError.getAxisAngle(errorAxisAngle);
      previousErrorAmplitude = errorAxisAngle.getAngle();

      RootJointOrientationCorrectorHelper helper = new RootJointOrientationCorrectorHelper("test", registry);
      helper.setCorrectionAlpha(0.1);
      for (int i = 0; i < 500; i++)
      {
         helper.compensateOrientationError(orientationToBeCorrected, orientationError);

         startOrientationFrame.setOrientationAndUpdate(orientationToBeCorrected);
         orientationError.setToZero(goalOrientationFrame);
         orientationError.changeFrame(startOrientationFrame);

         orientationError.getAxisAngle(errorAxisAngle);
         errorAmplitude = errorAxisAngle.getAngle();

         assertTrue(errorAmplitude < previousErrorAmplitude || errorAmplitude < 1e-6);

         previousErrorAmplitude = errorAmplitude;
      }

      assertTrue(orientationToBeCorrected.epsilonEquals(goalOrientation, 1e-6));

   }
}
