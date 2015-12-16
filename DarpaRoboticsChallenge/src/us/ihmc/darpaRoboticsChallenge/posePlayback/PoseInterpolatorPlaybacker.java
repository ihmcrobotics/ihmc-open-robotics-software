package us.ihmc.darpaRoboticsChallenge.posePlayback;

import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.commonWalkingControlModules.posePlayback.PlaybackPose;
import us.ihmc.commonWalkingControlModules.posePlayback.PlaybackPoseInterpolator;
import us.ihmc.commonWalkingControlModules.posePlayback.PlaybackPoseSequence;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.thread.ThreadTools;

public class PoseInterpolatorPlaybacker
{
   public static void playASequence(SDFHumanoidRobot sdfRobot, PlaybackPoseSequence sequence, boolean showGui, PoseCheckerCallback poseCheckerCallback)
   {
      YoVariableRegistry registry = new YoVariableRegistry("PoseInterpolatorPlaybacker");
      PlaybackPoseInterpolator interpolator = new PlaybackPoseInterpolator(registry);

      double simulateDT = 0.005;
      
      SimulationConstructionSet scs = null;
      scs = new SimulationConstructionSet(sdfRobot);
      if (showGui)
      {
         int recordFrequency = 1;
         scs.setDT(simulateDT, recordFrequency);
         scs.addYoVariableRegistry(registry);
         scs.startOnAThread();
      }

      double startTime = 1.7;
      double time = startTime;

      interpolator.startSequencePlayback(sequence, startTime);


      PlaybackPose previousPose = null;
      while (!interpolator.isDone())
      {
         time = time + simulateDT;
         scs.setTime(time);

         PlaybackPose pose = interpolator.getPose(time);

//         System.out.println(pose);

         if (showGui)
         {
            pose.setRobotAtPose(sdfRobot);
            scs.tickAndUpdate();
         }

         poseCheckerCallback.checkPose(pose, previousPose);
         previousPose = pose;
      }

      if (showGui)
      {
         ThreadTools.sleepForever();
      }
   }
}
