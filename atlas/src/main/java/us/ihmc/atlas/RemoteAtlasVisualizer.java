package us.ihmc.atlas;

import java.io.IOException;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;

import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizer;

public class RemoteAtlasVisualizer
{
   /** The update rate of the visualizer in milliseconds. If you experience delay build-up increase this value. */
   private static final int DEFAULT_UPDATE_RATE = 6;

   private static void startVisualizer(int bufferSize, int updateRateInMs) throws IOException
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
      int bufferSize = 16384;
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
