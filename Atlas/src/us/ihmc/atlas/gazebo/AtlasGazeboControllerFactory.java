package us.ihmc.atlas.gazebo;

import java.io.IOException;
import java.net.URISyntaxException;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.gazebo.GazeboControllerFactory;

/**
 *
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class AtlasGazeboControllerFactory
{
   public static void main(String[] args) throws IOException, URISyntaxException
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.GAZEBO, false);

      AtlasContactPointParameters contactPointParameters = robotModel.getContactPointParameters();

      contactPointParameters.createHandKnobContactPoints();

      new GazeboControllerFactory(robotModel, "/ihmc_ros", "atlas", "NONE");
   }
}
