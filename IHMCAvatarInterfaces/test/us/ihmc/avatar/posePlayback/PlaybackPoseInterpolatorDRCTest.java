package us.ihmc.avatar.posePlayback;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;


//TODO: update this test class to access poses via resource directory and undelete old pose files from svn
public abstract class PlaybackPoseInterpolatorDRCTest implements MultiRobotTestInterface
{
   private static final boolean SHOW_GUI = false;

	@ContinuousIntegrationTest(estimatedDuration = 4.0)
	@Test(timeout = 30000)
   public void testMoveElbowExample()
   {
      DRCRobotModel robotModel = getRobotModel();

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot sdfRobot = robotModel.createHumanoidFloatingRootJointRobot(false);

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

	@ContinuousIntegrationTest(estimatedDuration = 1.2)
	@Test(timeout = 30000)
   public void testRandomExample()
   {
      DRCRobotModel robotModel = getRobotModel();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot sdfRobot = robotModel.createHumanoidFloatingRootJointRobot(false);
      
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
//      FullRobotModel fullRobotModel = sdfLoader.createFullRobotModel(jointMap);
//      SDFRobot sdfRobot = sdfLoader.createRobot(jointMap, false);
//      
//      PosePlaybackRobotPoseSequence sequence = PosePlaybackExampleSequence.createExampleSequenceFourPoses(fullRobotModel);
//      playASequence(sdfRobot, sequence);
//   }

	@ContinuousIntegrationTest(estimatedDuration = 3.8)
	@Test(timeout = 30000)
   public void testLoadingAndPlayingASequence()
   {
      DRCRobotModel robotModel = getRobotModel();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot sdfRobot = robotModel.createHumanoidFloatingRootJointRobot(false);
      
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

	@ContinuousIntegrationTest(estimatedDuration = 4.6)
	@Test(timeout = 30000)
   public void testLoadingAndPlayingAnotherSequence()
   {
      DRCRobotModel robotModel = getRobotModel();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot sdfRobot = robotModel.createHumanoidFloatingRootJointRobot(false);
      
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
