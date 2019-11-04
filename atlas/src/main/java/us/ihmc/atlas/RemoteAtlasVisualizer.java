package us.ihmc.atlas;

import java.io.IOException;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizer;

public class RemoteAtlasVisualizer
{
   /** The update rate of the visualizer in milliseconds. If you experience delay build-up increase this value. */
   private static final int DEFAULT_BUFFER_SIZE = 16384;
   private static final int DEFAULT_UPDATE_RATE = 6;

   public static void startVisualizer()
   {
      ExceptionTools.handle(() -> startVisualizer(DEFAULT_BUFFER_SIZE, DEFAULT_UPDATE_RATE), DefaultExceptionHandler.RUNTIME_EXCEPTION);
   }

   public static void startVisualizer(int bufferSize, int updateRateInMs) throws IOException
   {
      SCSVisualizer scsVisualizer = new SCSVisualizer(bufferSize);
      scsVisualizer.setVariableUpdateRate(updateRateInMs);
      scsVisualizer.addButton("requestStop", 1.0);
      scsVisualizer.addButton("calibrateWristForceSensors", 1.0);
      scsVisualizer.setShowOverheadView(true);

      YoVariableClient client = new YoVariableClient(scsVisualizer);
      client.startWithHostSelector();
   }

   public static void main(String[] args) throws JSAPException, IOException
   {
      int bufferSize = DEFAULT_BUFFER_SIZE;
      JSAP jsap = new JSAP();

      FlaggedOption updateRate = new FlaggedOption("updateRate").setLongFlag("rate").setShortFlag('r').setStringParser(JSAP.INTEGER_PARSER).setRequired(false);
      updateRate.setHelp("Determines how often the GUI will update its parameters in milliseconds.");
      updateRate.setDefault(Integer.toString(DEFAULT_UPDATE_RATE));
      jsap.registerParameter(updateRate);

      JSAPResult config = jsap.parse(args);
      int updateRateInMs = DEFAULT_UPDATE_RATE;
      if (config.contains("displayOneInNPackets"))
      {
         updateRateInMs = config.getInt("displayOneInNPackets");
      }
      startVisualizer(bufferSize, updateRateInMs);
   }
}
