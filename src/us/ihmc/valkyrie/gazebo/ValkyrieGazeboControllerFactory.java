package us.ihmc.valkyrie.gazebo;

import us.ihmc.gazebo.GazeboControllerFactory;
import us.ihmc.valkyrie.ValkyrieRobotModel;

import java.io.IOException;
import java.net.URISyntaxException;

/**
 *
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class ValkyrieGazeboControllerFactory
{
   public static void main(String[] args) throws IOException, URISyntaxException
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(false, false);

      new GazeboControllerFactory(robotModel, "/ihmc_ros", "valkyrie", "NONE");
   }
}
