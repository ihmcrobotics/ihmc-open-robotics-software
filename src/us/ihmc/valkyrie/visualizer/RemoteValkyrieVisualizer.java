package us.ihmc.valkyrie.visualizer;


import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.atlas.visualization.SliderBoardControllerListener;
import us.ihmc.atlas.visualization.SliderBoardFactory;
import us.ihmc.darpaRoboticsChallenge.valkyrie.ValkyrieJointMap;
import us.ihmc.darpaRoboticsChallenge.valkyrie.ValkyrieSDFLoader;
import us.ihmc.robotDataCommunication.YoVariableClient;
import us.ihmc.valkyrie.ValkyrieNetworkParameters;
import us.ihmc.valkyrie.controllers.ValkyrieSliderBoard;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;

public class RemoteValkyrieVisualizer
{
   public static final String defaultHost = ValkyrieNetworkParameters.CONTROL_COMPUTER_HOST;
   public static final int defaultPort = ValkyrieNetworkParameters.VARIABLE_SERVER_PORT;

   private static final boolean USE_FORCE_CONTROL = false;

   public RemoteValkyrieVisualizer(String host, int port, int bufferSize)
   {
      System.out.println("Connecting to host " + host);
      ValkyrieJointMap jointMap = new ValkyrieJointMap();
      JaxbSDFLoader robotLoader = ValkyrieSDFLoader.loadValkyrieRobot(false);


      SliderBoardFactory sliderBoardFactory;
      if (USE_FORCE_CONTROL)
      {
         sliderBoardFactory = ValkyrieSliderBoard.getForceControlFactory();
      }
      else
      {
         sliderBoardFactory = ValkyrieSliderBoard.getTurboDriverPositionControlFactory();
      }

      SliderBoardControllerListener scsYoVariablesUpdatedListener = new ValkyrieSliderBoardControllerListener(robotLoader, jointMap, bufferSize,
                                                                       sliderBoardFactory);

//    SCSYoVariablesUpdatedListener scsYoVariablesUpdatedListener = new SCSYoVariablesUpdatedListener(robot, 16000);

      YoVariableClient client = new YoVariableClient(host, port, scsYoVariablesUpdatedListener, "remote", false);
      client.start();
   }

   public static void main(String[] args) throws JSAPException
   {
      int bufferSize = 16384;
      JSAP jsap = new JSAP();

      FlaggedOption hostOption =
         new FlaggedOption("host").setStringParser(JSAP.STRING_PARSER).setRequired(false).setLongFlag("host").setShortFlag('L').setDefault(defaultHost);
      FlaggedOption portOption = new FlaggedOption("port").setStringParser(JSAP.INTEGER_PARSER).setRequired(false).setLongFlag("port").setShortFlag(
                                     'p').setDefault(String.valueOf(defaultPort));

      jsap.registerParameter(hostOption);
      jsap.registerParameter(portOption);

      JSAPResult config = jsap.parse(args);

      if (config.success())
      {
         String host = config.getString("host");
         int port = config.getInt("port");

         new RemoteValkyrieVisualizer(host, port, bufferSize);
      }
      else
      {
         System.err.println();
         System.err.println("Usage: java " + RemoteValkyrieVisualizer.class.getName());
         System.err.println("                " + jsap.getUsage());
         System.err.println();
         System.exit(1);
      }



   }
}
