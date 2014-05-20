package us.ihmc.valkyrie;

import java.net.URI;
import java.net.URISyntaxException;

import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkProcessor;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;

public class ValkyrieNetworkProcessor
{
   private static String rosMasterURI = DRCConfigParameters.ROS_MASTER_URI;
   private static final DRCRobotModel model = new ValkyrieRobotModel(true, true);
   
   public static void main(String[] args) throws URISyntaxException, JSAPException
   {
      JSAP jsap = new JSAP();
      FlaggedOption rosURIFlag =
         new FlaggedOption("ros-uri").setLongFlag("ros-uri").setShortFlag(JSAP.NO_SHORTFLAG).setRequired(false).setStringParser(JSAP.STRING_PARSER);

      jsap.registerParameter(rosURIFlag);
      JSAPResult config = jsap.parse(args);

      if (config.success())
      {
         if (config.getString(rosURIFlag.getID()) != null)
         {
            rosMasterURI = config.getString(rosURIFlag.getID());
         }
         new DRCNetworkProcessor(new URI(rosMasterURI), model);
      }
      else
      {
         System.err.println("Invalid parameters");
         System.out.println(jsap.getHelp());
         return;
      }
   }
}
