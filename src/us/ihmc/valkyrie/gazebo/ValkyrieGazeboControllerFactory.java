package us.ihmc.valkyrie.gazebo;

import java.io.IOException;
import java.net.URISyntaxException;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.gazebo.GazeboControllerFactory;
import us.ihmc.valkyrie.ValkyrieRobotModel;

/**
 *
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */

public class ValkyrieGazeboControllerFactory
{
   public static void main(String[] args) throws IOException, URISyntaxException
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(DRCRobotModel.RobotTarget.GAZEBO, true);

      new GazeboControllerFactory(robotModel, "/ihmc_ros", "valkyrie", "NONE");
   }
}
