package us.ihmc.valkyrie.simulation;

import java.io.IOException;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.posePlayback.PosePlaybackSCSBridge;
import us.ihmc.humanoidRobotics.model.FullHumanoidRobotModel;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class ValkyriePosePlaybackSCSBridge
{

   public static void main(String[] args) throws IOException
   {
      DRCRobotModel robotModel = new ValkyrieRobotModel(false, false);
      
      DRCRobotJointMap jointMap = robotModel.getJointMap();
      SDFHumanoidRobot sdfRobot = robotModel.createSdfRobot(false);
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      SDFFullRobotModel fullRobotModelForSlider = robotModel.createFullRobotModel();
   
      new PosePlaybackSCSBridge(sdfRobot, fullRobotModel, fullRobotModelForSlider, robotModel.getControllerDT());
   
   }
}
