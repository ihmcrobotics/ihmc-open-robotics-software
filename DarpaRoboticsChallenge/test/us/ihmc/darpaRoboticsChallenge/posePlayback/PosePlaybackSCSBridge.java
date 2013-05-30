package us.ihmc.darpaRoboticsChallenge.posePlayback;

import java.io.IOException;

import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.utilities.ThreadTools;

import com.yobotics.simulationconstructionset.Robot;
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
      
      
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("PlaybackPoseSCSBridgeRobot"));
      scs.addYoVariableRegistry(registry);
      scs.startOnAThread();
      
      while(true)
      {
         posePlaybackSender.writeData();

         ThreadTools.sleep(1000);
      }
   }

}
