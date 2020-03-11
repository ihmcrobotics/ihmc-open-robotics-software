package us.ihmc.atlas.roughTerrainWalking;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsStandaloneVisualizer;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.robotics.robotSide.RobotSide;

public class AtlasSwingOverPlanarRegionsVisualizer
{
   public static void main(String[] args)
   {
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
      WalkingControllerParameters walkingControllerParameters = atlasRobotModel.getWalkingControllerParameters();
      AtlasContactPointParameters contactPointParameters = atlasRobotModel.getContactPointParameters();
      ConvexPolygon2DReadOnly footPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(contactPointParameters.getFootContactPoints().get(RobotSide.LEFT)));
      new SwingOverPlanarRegionsStandaloneVisualizer(walkingControllerParameters, footPolygon);
   }
}
