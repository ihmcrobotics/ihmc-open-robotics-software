package us.ihmc.atlas;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.fiducialDetectorToolBox.FiducialDetectorToolboxModule;

public class AtlasFiducialModulalStarter
{
   public static void main(String[] args)
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.REAL_ROBOT);
      new FiducialDetectorToolboxModule(robotModel.getSimpleRobotName(),
                                        robotModel.getTarget(),
                                        robotModel.createFullRobotModel(),
                                        robotModel.getLogModelProvider());
   }
}
