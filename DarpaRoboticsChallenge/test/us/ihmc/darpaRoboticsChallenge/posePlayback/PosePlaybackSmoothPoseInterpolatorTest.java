package us.ihmc.darpaRoboticsChallenge.posePlayback;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.darpaRoboticsChallenge.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.environment.DRCTask;
import us.ihmc.darpaRoboticsChallenge.environment.DRCTaskName;
import us.ihmc.utilities.ThreadTools;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class PosePlaybackSmoothPoseInterpolatorTest
{
   private static final boolean SHOW_GUI = true;
   private static final DRCRobotModel robotModel = DRCRobotModel.ATLAS_NO_HANDS_ADDED_MASS;


   @Test
   public void testExampleOne()
   {
      PosePlaybackRobotPoseSequence sequence = PosePlaybackExampleSequence.createExampleSequenceFourPoses();
      playASequence(sequence);
   }

   @Test
   public void testLoadingAndPlayingASequence()
   {
      PosePlaybackRobotPoseSequence sequence = new PosePlaybackRobotPoseSequence();
      sequence.appendFromFile("testSequence2.poseSequence");
      playASequence(sequence);
   }

   @Test
   public void testLoadingAndPlayingAnotherSequence()
   {
      PosePlaybackRobotPoseSequence sequence = new PosePlaybackRobotPoseSequence();
      sequence.appendFromFile("tenPoses.poseSequence");

      System.out.println(sequence);
      playASequence(sequence);
   }


   private void playASequence(PosePlaybackRobotPoseSequence sequence)
   {
      YoVariableRegistry registry = new YoVariableRegistry("PosePlaybackSmoothPoseInterpolatorTest");
      PosePlaybackSmoothPoseInterpolator interpolator = new PosePlaybackSmoothPoseInterpolator(registry);

      double simulateDT = 0.005;

      SimulationConstructionSet scs = null;
      SDFRobot sdfRobot = null;
      if (SHOW_GUI)
      {
         DRCTask vrcTask = new DRCTask(DRCTaskName.ONLY_VEHICLE, robotModel);
         sdfRobot = vrcTask.getRobot();

         scs = new SimulationConstructionSet(sdfRobot);
         int recordFrequency = 1;
         scs.setDT(simulateDT, recordFrequency);
         scs.addYoVariableRegistry(registry);
         scs.startOnAThread();
      }

      double startTime = 1.7;
      double time = startTime;

      interpolator.startSequencePlayback(sequence, startTime);


      PosePlaybackRobotPose previousPose = null;
      while (!interpolator.isDone())
      {
         time = time + simulateDT;
         scs.setTime(time);

         PosePlaybackRobotPose pose = interpolator.getPose(time);

         System.out.println(pose);

         if (SHOW_GUI)
         {
            pose.setRobotAtPose(sdfRobot);
            scs.tickAndUpdate();
         }

         assertSmallPoseDifference(pose, previousPose);
         previousPose = pose;
      }


      if (SHOW_GUI)
      {
         ThreadTools.sleepForever();
      }
   }


   private void assertSmallPoseDifference(PosePlaybackRobotPose pose, PosePlaybackRobotPose previousPose)
   {
      if (pose == null)
         return;
      if (previousPose == null)
         return;

      boolean smallDifference = pose.epsilonEquals(previousPose, 1e-2, 1.0);
      assertTrue(smallDifference);
   }

}
