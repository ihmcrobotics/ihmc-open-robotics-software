package us.ihmc.atlas;

import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.visualization.SliderBoardControllerListener;
import us.ihmc.darpaRoboticsChallenge.visualization.SliderBoardFactory;
import us.ihmc.darpaRoboticsChallenge.visualization.WalkControllerSliderBoard;
import us.ihmc.robotDataCommunication.YoVariableClient;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;

public class RemoteAtlasVisualizer
{
   public static final int defaultPort = NetworkConfigParameters.DEFAULT_YOVARIABLE_SERVER_PORT;
   private final boolean showOverheadView = true;
   
   public RemoteAtlasVisualizer(String host, int port, int bufferSize, DRCRobotModel robotModel)
   {
      System.out.println("Connecting to host " + host);
      
//      SDFRobot robot = robotLoader.createRobot(jointMap, false);
//      SliderBoardFactory sliderBoardFactory = GainControllerSliderBoard.getFactory();
      SliderBoardFactory sliderBoardFactory = WalkControllerSliderBoard.getFactory();
//      SliderBoardFactory sliderBoardFactory = JointAngleOffsetSliderBoard.getFactory();

      SliderBoardControllerListener scsYoVariablesUpdatedListener = new SliderBoardControllerListener(robotModel, bufferSize, sliderBoardFactory);
      scsYoVariablesUpdatedListener.addButton("requestStop", 1.0);
      scsYoVariablesUpdatedListener.addButton("setWristForceSensorsToZero", 1.0);
      
      
      YoVariableClient client = new YoVariableClient(host, port, scsYoVariablesUpdatedListener, "remote", showOverheadView);
      client.start();
   }

   public static void main(String[] args) throws JSAPException
   {
      int bufferSize = 16384;
      JSAP jsap = new JSAP();
      
      FlaggedOption hostOption = new FlaggedOption("host").setStringParser(JSAP.STRING_PARSER).setRequired(false).setLongFlag("host").setShortFlag('L');
      FlaggedOption portOption = new FlaggedOption("port").setStringParser(JSAP.INTEGER_PARSER).setRequired(false).setLongFlag("port").setShortFlag('p')
            .setDefault(String.valueOf(defaultPort));
      FlaggedOption robotModel = new FlaggedOption("robotModel").setLongFlag("model").setShortFlag('m').setRequired(true).setStringParser(JSAP.STRING_PARSER);
      robotModel.setHelp("Robot models: " + AtlasRobotModelFactory.robotModelsToString());
      
      jsap.registerParameter(hostOption);
      jsap.registerParameter(portOption);
      jsap.registerParameter(robotModel);
      
      JSAPResult config = jsap.parse(args);
      
      if (config.success())
      {
    	  DRCRobotModel model = AtlasRobotModelFactory.createDRCRobotModel(config.getString("robotModel"), true, false);
    	  
    	  String host;
    	  if (config.getString("host") != null)
    		  host = config.getString("host");
    	  else
    		  host = model.getNetworkParameters().getRobotControlComputerIP();
    	  
         int port = config.getInt("port");
         
         new RemoteAtlasVisualizer(host, port, bufferSize, model);         
      }
      else
      {
         System.err.println();
         System.err.println("Usage: java " + RemoteAtlasVisualizer.class.getName());
         System.err.println("                " + jsap.getUsage());
         System.err.println();
         System.exit(1);
      }
      
      
      
   }
}
