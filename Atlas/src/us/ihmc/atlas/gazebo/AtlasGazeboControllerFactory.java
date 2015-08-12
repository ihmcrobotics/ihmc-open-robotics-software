package us.ihmc.atlas.gazebo;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.gazebo.GazeboControllerFactory;

import java.io.IOException;
import java.net.URISyntaxException;

/**
 *
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class AtlasGazeboControllerFactory
{
   public static void main(String[] args) throws IOException, URISyntaxException
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, AtlasRobotModel.AtlasTarget.GAZEBO, false);

      AtlasContactPointParameters contactPointParameters = robotModel.getContactPointParameters();

      contactPointParameters.createHandKnobContactPoints();

      new GazeboControllerFactory(robotModel, "/ihmc_ros", "atlas", "NONE");
   }
}
