package us.ihmc.darpaRoboticsChallenge.posePlayback;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.commonWalkingControlModules.posePlayback.PlaybackPose;
import us.ihmc.commonWalkingControlModules.posePlayback.PlaybackPoseSequence;
import us.ihmc.commonWalkingControlModules.posePlayback.PlaybackPoseSequenceReader;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;


//TODO: update this test class to access poses via resource directory and undelete old pose files from svn
public abstract class PlaybackPoseInterpolatorDRCTest implements MultiRobotTestInterface
{
   private static final boolean SHOW_GUI = false;

	@DeployableTestMethod(estimatedDuration = 4.0)
	@Test(timeout = 30000)
   public void testMoveElbowExample()
   {
      DRCRobotModel robotModel = getRobotModel();

      SDFFullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      SDFHumanoidRobot sdfRobot = robotModel.createSdfRobot(false);

      double delay = 0.3;
      double trajectoryTime = 1.0;
      
      PlaybackPoseSequence sequence = PosePlaybackExampleSequence.createExamplePoseSequenceMoveArm(fullRobotModel, delay, trajectoryTime);
      
      PoseInterpolatorPlaybacker.playASequence(sdfRobot, sequence, SHOW_GUI, new PoseCheckerCallback()
      {
         @Override
         public void checkPose(PlaybackPose pose, PlaybackPose previousPose)
         {
            assertSmallPoseDifference(pose, previousPose);
         }
      });
   }

	@DeployableTestMethod(estimatedDuration = 1.2)
	@Test(timeout = 30000)
   public void testRandomExample()
   {
      DRCRobotModel robotModel = getRobotModel();
      SDFFullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      SDFHumanoidRobot sdfRobot = robotModel.createSdfRobot(false);
      
      int numberOfPoses = 5;
      double delay = 0.3;
      double trajectoryTime = 1.0;
      
      Random random = new Random(1776L);
      PlaybackPoseSequence sequence = PosePlaybackExampleSequence.createRandomPlaybackPoseSequence(random, fullRobotModel, numberOfPoses, delay, trajectoryTime);

      //sequence.writeToOutputStream(fullRobotModel, System.out);

      PoseInterpolatorPlaybacker.playASequence(sdfRobot, sequence, SHOW_GUI, new PoseCheckerCallback()
      {
         @Override
         public void checkPose(PlaybackPose pose, PlaybackPose previousPose)
         {
            assertSmallPoseDifference(pose, previousPose);
         }
      });
   }
   
//   @Test(timeout=300000)
//   public void testExampleOne()
//   {
//      DRCRobotJointMap jointMap = robotModel.getJointMap();
//      JaxbSDFLoader sdfLoader = DRCRobotSDFLoader.loadDRCRobot(jointMap, false);
//      
//      SDFFullRobotModel fullRobotModel = sdfLoader.createFullRobotModel(jointMap);
//      SDFRobot sdfRobot = sdfLoader.createRobot(jointMap, false);
//      
//      PosePlaybackRobotPoseSequence sequence = PosePlaybackExampleSequence.createExampleSequenceFourPoses(fullRobotModel);
//      playASequence(sdfRobot, sequence);
//   }

	@DeployableTestMethod(estimatedDuration = 3.8)
	@Test(timeout = 30000)
   public void testLoadingAndPlayingASequence()
   {
      DRCRobotModel robotModel = getRobotModel();
      SDFFullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      SDFHumanoidRobot sdfRobot = robotModel.createSdfRobot(false);
      
      PlaybackPoseSequence sequence = new PlaybackPoseSequence(fullRobotModel);
      PlaybackPoseSequenceReader.appendFromFile(sequence, getClass().getClassLoader().getResourceAsStream("testSequence2.poseSequence"));
      
      PoseInterpolatorPlaybacker.playASequence(sdfRobot, sequence, SHOW_GUI, new PoseCheckerCallback()
      {
         @Override
         public void checkPose(PlaybackPose pose, PlaybackPose previousPose)
         {
            assertSmallPoseDifference(pose, previousPose);
         }
      });
   }

	@DeployableTestMethod(estimatedDuration = 4.6)
	@Test(timeout = 30000)
   public void testLoadingAndPlayingAnotherSequence()
   {
      DRCRobotModel robotModel = getRobotModel();
      SDFFullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      SDFHumanoidRobot sdfRobot = robotModel.createSdfRobot(false);
      
      PlaybackPoseSequence sequence = new PlaybackPoseSequence(fullRobotModel);
      PlaybackPoseSequenceReader.appendFromFile(sequence, getClass().getClassLoader().getResourceAsStream("tenPoses.poseSequence"));

      System.out.println(sequence);
      
      PoseInterpolatorPlaybacker.playASequence(sdfRobot, sequence, SHOW_GUI, new PoseCheckerCallback()
      {
         @Override
         public void checkPose(PlaybackPose pose, PlaybackPose previousPose)
         {
            assertSmallPoseDifference(pose, previousPose);
         }
      });
   }

   private void assertSmallPoseDifference(PlaybackPose pose, PlaybackPose previousPose)
   {
      if (pose == null)
         return;
      if (previousPose == null)
         return;

      boolean smallDifference = pose.epsilonEquals(previousPose, 100.0, 100.0); //1e-2, 1.0);
      if (!smallDifference)
      {
         System.out.println("pose = " + pose);
         System.out.println("previousPose = " + previousPose);
      }
      assertTrue(smallDifference);
   }
}
