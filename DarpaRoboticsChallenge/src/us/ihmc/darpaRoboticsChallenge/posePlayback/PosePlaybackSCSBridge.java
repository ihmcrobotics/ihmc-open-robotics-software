package us.ihmc.darpaRoboticsChallenge.posePlayback;

import java.io.IOException;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.environment.VRCTask;
import us.ihmc.darpaRoboticsChallenge.environment.VRCTaskName;
import us.ihmc.utilities.ThreadTools;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class PosePlaybackSCSBridge
{
   private static final String ipAddress = DRCConfigParameters.CLOUD_MINION5_IP;
   
   public static void main(String[] args) throws IOException
   {
      YoVariableRegistry registry = new YoVariableRegistry("PlaybackPoseSCSBridge");
      PosePlaybackAllJointsController posePlaybackController = new PosePlaybackAllJointsController(registry);
      PosePlaybackSender posePlaybackSender = new PosePlaybackSender(posePlaybackController, ipAddress);
      
      posePlaybackSender.connect();
      posePlaybackSender.waitUntilConnected();
      
      VRCTask vrcTask = new VRCTask(VRCTaskName.ONLY_VEHICLE);
      SDFRobot sdfRobot = vrcTask.getRobot();
      
      SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot);
      scs.addYoVariableRegistry(registry);
      
      
      DRCRobotMidiSliderBoardPositionManipulation sliderBoard = new DRCRobotMidiSliderBoardPositionManipulation(scs);

      
      scs.startOnAThread();
      
      while(true)
      {
         PosePlaybackRobotPose pose = new PosePlaybackRobotPose(sdfRobot);
         posePlaybackController.setPlaybackPose(pose);

         posePlaybackSender.writeData();
         
         ThreadTools.sleep(1000);
      }
   }

}
