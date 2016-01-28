package us.ihmc.quadrupedRobotics.stateEstimator.kinematicsBased;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.time.GlobalTimer;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.testing.TestPlanTarget;

@DeployableTestClass(targets = TestPlanTarget.Fast)
public class RootJointPoseCorrectorHelperTest
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

      FramePose poseToBeCorrected = new FramePose(ReferenceFrame.getWorldFrame());
      poseToBeCorrected.setPose(RandomTools.generateRandomPoint(random, 2.0, 2.0, 2.0), RandomTools.generateRandomQuaternion(random, 2.0 * Math.PI));
      PoseReferenceFrame startPoseFrame = new PoseReferenceFrame("poseToBeCorrectedFrame", poseToBeCorrected);

      FramePose goalPose = new FramePose(startPoseFrame);
      goalPose.setPose(RandomTools.generateRandomPoint(random, 2.0, 2.0, 2.0), RandomTools.generateRandomQuaternion(random, 2.0 * Math.PI));
      goalPose.changeFrame(ReferenceFrame.getWorldFrame());
      PoseReferenceFrame goalPoseFrame = new PoseReferenceFrame("goalPoseReferenceFrame", goalPose);

      FramePose poseError = new FramePose();
      poseError.setToZero(goalPoseFrame);
      poseError.changeFrame(startPoseFrame);

      RootJointPoseCorrectorHelper helper = new RootJointPoseCorrectorHelper("test", registry);
      helper.setCorrectionAlpha(1.0, 1.0);

      helper.compensatePoseError(poseToBeCorrected, poseError);

      assertTrue(poseToBeCorrected.epsilonEquals(goalPose, 1e-6));

   }

   @DeployableTestMethod(estimatedDuration = 0.4)
   @Test (timeout = 60000)
   public void testErrorDecreasesAtEveryTick()
   {
      Random random = new Random();

      FramePose poseError = new FramePose();

      AxisAngle4d orientationErrorAxisAngle = new AxisAngle4d();
      double orientationErrorAmplitude = 0.0;
      double previousOrientationErrorAmplitude = 0.0;

      Vector3d positionErrorVector = new Vector3d();
      double positionErrorAmplitude = 0.0;
      double previousPositionErrorAmplitude = 0.0;

      FramePose poseToBeCorrected = new FramePose(ReferenceFrame.getWorldFrame());
      poseToBeCorrected.setPose(RandomTools.generateRandomPoint(random, 2.0, 2.0, 2.0), RandomTools.generateRandomQuaternion(random, 2.0 * Math.PI));
      PoseReferenceFrame startPoseFrame = new PoseReferenceFrame("startPoseReferenceFrame", poseToBeCorrected);

      FramePose goalPose = new FramePose(startPoseFrame);
      goalPose.setPose(RandomTools.generateRandomPoint(random, 2.0, 2.0, 2.0), RandomTools.generateRandomQuaternion(random, 2.0 * Math.PI));
      goalPose.changeFrame(ReferenceFrame.getWorldFrame());
      PoseReferenceFrame goalPoseFrame = new PoseReferenceFrame("goalPoseReferenceFrame", goalPose);

      poseError.setToZero(goalPoseFrame);
      poseError.changeFrame(startPoseFrame);

      poseError.getPose(positionErrorVector, orientationErrorAxisAngle);
      previousOrientationErrorAmplitude = orientationErrorAxisAngle.getAngle();
      previousPositionErrorAmplitude = positionErrorVector.lengthSquared();

      RootJointPoseCorrectorHelper helper = new RootJointPoseCorrectorHelper("test", registry);
      helper.setCorrectionAlpha(0.1, 0.1);
      for (int i = 0; i < 500; i++)
      {
         helper.compensatePoseError(poseToBeCorrected, poseError);

         startPoseFrame.setPoseAndUpdate(poseToBeCorrected);
         poseError.setToZero(goalPoseFrame);
         poseError.changeFrame(startPoseFrame);

         poseError.getPose(positionErrorVector, orientationErrorAxisAngle);
         orientationErrorAmplitude = orientationErrorAxisAngle.getAngle();
         positionErrorAmplitude = positionErrorVector.lengthSquared();

         assertTrue(orientationErrorAmplitude < previousOrientationErrorAmplitude || orientationErrorAmplitude < 1e-6);
         assertTrue(positionErrorAmplitude < previousPositionErrorAmplitude || positionErrorAmplitude < 1e-6);

         previousOrientationErrorAmplitude = orientationErrorAmplitude;
         previousPositionErrorAmplitude = positionErrorAmplitude;
      }

      assertTrue(poseToBeCorrected.epsilonEquals(goalPose, 1e-6));

   }
}
