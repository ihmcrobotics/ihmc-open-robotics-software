package us.ihmc.valkyrie.simulation;

import java.io.IOException;

import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.darpaRoboticsChallenge.DRCRobotSDFLoader;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.posePlayback.PosePlaybackSCSBridge;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyriePosePlaybackSCSBridge
{

   public static void main(String[] args) throws IOException
   {
      DRCRobotModel robotModel = new ValkyrieRobotModel(false);
      
      DRCRobotJointMap jointMap = robotModel.getJointMap();
      JaxbSDFLoader loader = robotModel.getJaxbSDFLoader(false);
      SDFRobot sdfRobot = loader.createRobot(jointMap, false);
      FullRobotModel fullRobotModel = loader.createFullRobotModel(jointMap);
      SDFFullRobotModel fullRobotModelForSlider = loader.createFullRobotModel(jointMap);
   
      new PosePlaybackSCSBridge(sdfRobot, fullRobotModel, fullRobotModelForSlider);
   
   }
}
