package us.ihmc.atlas.roughTerrainWalking;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.roughTerrainWalking.AvatarSwingOverPlanarRegionsVisualizer;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;

public class AtlasSwingOverPlanarRegionsVisualizer
{
   public static void main(String[] args)
   {
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.SCS, false);
      WalkingControllerParameters walkingControllerParameters = atlasRobotModel.getWalkingControllerParameters();
      AtlasContactPointParameters contactPointParameters = atlasRobotModel.getContactPointParameters();
      new AvatarSwingOverPlanarRegionsVisualizer(walkingControllerParameters, contactPointParameters);
   }
}
