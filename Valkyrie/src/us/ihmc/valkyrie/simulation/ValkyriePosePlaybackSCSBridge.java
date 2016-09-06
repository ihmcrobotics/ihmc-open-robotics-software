package us.ihmc.valkyrie.simulation;

import java.io.IOException;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.SdfLoader.HumanoidFloatingRootJointRobot;
import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.posePlayback.PosePlaybackSCSBridge;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class ValkyriePosePlaybackSCSBridge
{

   public static void main(String[] args) throws IOException
   {
      DRCRobotModel robotModel = new ValkyrieRobotModel(DRCRobotModel.RobotTarget.SCS, false);
      
      DRCRobotJointMap jointMap = robotModel.getJointMap();
      HumanoidFloatingRootJointRobot sdfRobot = robotModel.createSdfRobot(false);
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      FullHumanoidRobotModel fullRobotModelForSlider = robotModel.createFullRobotModel();
   
      new PosePlaybackSCSBridge(sdfRobot, fullRobotModel, fullRobotModelForSlider, robotModel.getControllerDT());
   
   }
}
