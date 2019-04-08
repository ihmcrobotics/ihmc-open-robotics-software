package us.ihmc.atlas;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPResult;

import us.ihmc.log.LogTools;
import us.ihmc.parameterTuner.guiElements.main.ParameterGuiInterface;
import us.ihmc.parameterTuner.guiElements.main.ParameterTuningApplication;
import us.ihmc.parameterTuner.remote.RemoteInputManager;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizer;

public class RemoteAtlasVisualizer extends ParameterTuningApplication
{
   /** The IP address of the control computer. If you want to run this locally change this to "localhost" */
   private static final String CONTROLLER_IP = "10.7.4.104";
   /** The update rate of the visualizer in milliseconds. If you experience delay build-up increase this value. */
   private static final int DEFAULT_UPDATE_RATE = 6;
   /** Determined whether to automatically start the parameter tuner with the remote visualizer */
   private static final boolean LAUNCH_TUNER = true;

   private static void startVisualizer(int bufferSize, int updateRateInMs)
   {
      SCSVisualizer scsVisualizer = new SCSVisualizer(bufferSize);
      scsVisualizer.setVariableUpdateRate(updateRateInMs);
      scsVisualizer.addButton("requestStop", 1.0);
      scsVisualizer.addButton("calibrateWristForceSensors", 1.0);
      scsVisualizer.setShowOverheadView(true);

      YoVariableClient client = new YoVariableClient(scsVisualizer);
      client.start(CONTROLLER_IP, DataServerSettings.DEFAULT_PORT);
   }

   @Override
   protected ParameterGuiInterface createInputManager()
   {
      return new RemoteInputManager(CONTROLLER_IP);
   }

   public static void main(String[] args) throws Exception
   {
      LogTools.info("Starting the remote visualizer for controller at " + CONTROLLER_IP);

      int bufferSize = 16384;
      JSAP jsap = new JSAP();

      FlaggedOption updateRate = new FlaggedOption("updateRate").setLongFlag("rate").setShortFlag('r').setStringParser(JSAP.INTEGER_PARSER).setRequired(false);
      updateRate.setHelp("Determines how opten the GUI will update its parameters in milliseconds.");
      updateRate.setDefault(Integer.toString(DEFAULT_UPDATE_RATE));
      jsap.registerParameter(updateRate);

      JSAPResult config = jsap.parse(args);
      int updateRateInMs = DEFAULT_UPDATE_RATE;
      if (config.contains("displayOneInNPackets"))
      {
         updateRateInMs = config.getInt("displayOneInNPackets");
      }
      startVisualizer(bufferSize, updateRateInMs);

      if (LAUNCH_TUNER)
      {
         launch(args);
      }
   }
}
