package us.ihmc.darpaRoboticsChallenge.remote;

import us.ihmc.GazeboStateCommunicator.yoVariableCommunicator.YoVariableClient;
import us.ihmc.GazeboStateCommunicator.yoVariableCommunicator.visualizer.SCSYoVariablesUpdatedListener;
import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.darpaRoboticsChallenge.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.DRCRobotSDFLoader;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;

public class RemoteAtlasVisualizer
{
   public static final String defaultHost = "localhost";
   public static final int defaultPort = 5555;
   
   public RemoteAtlasVisualizer(String host, int port)
   {
      System.out.println("Connecting to host " + host);
      
      DRCRobotJointMap jointMap = new DRCRobotJointMap(DRCRobotModel.ATLAS_NO_HANDS, false);
      JaxbSDFLoader robotLoader = DRCRobotSDFLoader.loadDRCRobot(jointMap);
      SDFRobot robot = robotLoader.createRobot(jointMap, false);
      SCSYoVariablesUpdatedListener scsYoVariablesUpdatedListener = new SCSYoVariablesUpdatedListener(robot);

      YoVariableClient client = new YoVariableClient(host, port, scsYoVariablesUpdatedListener, "remote");
      client.start();
   }

   public static void main(String[] args) throws JSAPException
   {
      JSAP jsap = new JSAP();
      
      FlaggedOption hostOption = new FlaggedOption("host").setStringParser(JSAP.STRING_PARSER).setRequired(false).setLongFlag("host").setShortFlag('L').setDefault(
            defaultHost);
      FlaggedOption portOption = new FlaggedOption("port").setStringParser(JSAP.INTEGER_PARSER).setRequired(false).setLongFlag("port").setShortFlag('p')
            .setDefault(String.valueOf(defaultPort));
      
      jsap.registerParameter(hostOption);
      jsap.registerParameter(portOption);
      
      JSAPResult config = jsap.parse(args);
      
      if (config.success())
      {
         String host = config.getString("host");
         int port = config.getInt("port");
         
         new RemoteAtlasVisualizer(host, port);         
      }
      else
      {
         System.err.println();
         System.err.println("Usage: java " + SCSYoVariablesUpdatedListener.class.getName());
         System.err.println("                " + jsap.getUsage());
         System.err.println();
         System.exit(1);
      }
      
      
      
   }
}
