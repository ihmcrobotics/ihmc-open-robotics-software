package us.ihmc.atlas;

import us.ihmc.atlas.AtlasRobotModel.AtlasTarget;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.visualization.GainControllerSliderBoard;
import us.ihmc.darpaRoboticsChallenge.visualization.WalkControllerSliderBoard;
import us.ihmc.robotDataCommunication.YoVariableClient;
import us.ihmc.robotDataCommunication.visualizer.SCSVisualizer;
import us.ihmc.robotDataCommunication.visualizer.SCSVisualizerStateListener;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;
import com.martiansoftware.jsap.Switch;

public class RemoteAtlasVisualizer implements SCSVisualizerStateListener
{
   public static final int defaultPort = NetworkConfigParameters.DEFAULT_YOVARIABLE_SERVER_PORT;
   private final boolean showOverheadView = true;
   private static final AtlasSliderBoardType defaultSliderBoardType = AtlasSliderBoardType.WALK_CONTROLLER; 
   
   public enum AtlasSliderBoardType {GAIN_CONTROLLER, JOINT_ANGLE_OFFSET, WALK_CONTROLLER}
   
   private DRCRobotModel drcRobotModel;
   private SCSVisualizer scsVisualizer;
   
   public RemoteAtlasVisualizer(String host, int port, int bufferSize, DRCRobotModel drcRobotModel)
   {
      this.drcRobotModel = drcRobotModel;
      
      System.out.println("Connecting to host " + host);

      scsVisualizer = new SCSVisualizer(drcRobotModel.createSdfRobot(false), bufferSize);
      scsVisualizer.addSCSVisualizerStateListener(this);
      scsVisualizer.addButton("requestStop", 1.0);
      scsVisualizer.addButton("setWristForceSensorsToZero", 1.0);
      
      YoVariableClient client = new YoVariableClient(host, port, scsVisualizer, "remote", showOverheadView);
      client.start();
   }

   @Override
   public void starting()
   {
      switch (defaultSliderBoardType)
      {
         case WALK_CONTROLLER :
            new WalkControllerSliderBoard(scsVisualizer.getSCS(), scsVisualizer.getRegistry(), drcRobotModel.getGeneralizedRobotModel());

            break;

         case GAIN_CONTROLLER :
            new GainControllerSliderBoard(scsVisualizer.getSCS(), scsVisualizer.getRegistry(), drcRobotModel.getGeneralizedRobotModel());

            break;

         case JOINT_ANGLE_OFFSET :
            new JointAngleOffsetSliderBoard(scsVisualizer.getSCS(), scsVisualizer.getRegistry(), drcRobotModel.getGeneralizedRobotModel());

            break;
      }
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
      
      Switch runningOnRealRobot = new Switch("runningOnRealRobot").setLongFlag("realRobot");

      
      jsap.registerParameter(hostOption);
      jsap.registerParameter(portOption);
      jsap.registerParameter(robotModel);
      jsap.registerParameter(runningOnRealRobot);

      JSAPResult config = jsap.parse(args);
      
      if (config.success())
      {
        AtlasTarget target = config.getBoolean(runningOnRealRobot.getID()) ? AtlasTarget.REAL_ROBOT : AtlasTarget.SIM;
        DRCRobotModel model = AtlasRobotModelFactory.createDRCRobotModel(config.getString("robotModel"), target, false);

    	  
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
