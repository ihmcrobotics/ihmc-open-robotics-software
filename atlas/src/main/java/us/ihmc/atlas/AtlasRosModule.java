package us.ihmc.atlas;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.modules.RosModule;
import us.ihmc.communication.configuration.NetworkParameters;

import java.net.URI;

public class AtlasRosModule
{
   public AtlasRosModule()
   {
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.REAL_ROBOT);
      URI rosCoreURI = NetworkParameters.getROSURI();
      new RosModule(atlasRobotModel, rosCoreURI, null);
   }

   public static void main(String[] args)
   {
      new AtlasRosModule();
   }
}
