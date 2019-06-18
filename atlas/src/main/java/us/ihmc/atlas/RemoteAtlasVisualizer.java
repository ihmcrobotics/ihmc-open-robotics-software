package us.ihmc.atlas;

import java.io.IOException;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;

import us.ihmc.commons.Conversions;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPDataServerConnection;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizer;

public class RemoteAtlasVisualizer
{
   /** Timeout for starting the YoVariable client. If the server does not come online within this time the remote visualizer will crash. */
   private static final int TIMEOUT = (int) Conversions.secondsToMilliseconds(600);
   /** The IP address of the control computer. If you want to run this locally change this to "localhost". */
   private static final String CONTROLLER_IP = "10.7.4.104";
   /** The update rate of the visualizer in milliseconds. If you experience delay build-up increase this value. */
   private static final int DEFAULT_UPDATE_RATE = 6;

   private static void startVisualizer(int bufferSize, int updateRateInMs) throws IOException
   {
      SCSVisualizer scsVisualizer = new SCSVisualizer(bufferSize);
      scsVisualizer.setVariableUpdateRate(updateRateInMs);
      scsVisualizer.addButton("requestStop", 1.0);
      scsVisualizer.addButton("calibrateWristForceSensors", 1.0);
      scsVisualizer.setShowOverheadView(true);

      HTTPDataServerConnection connection = HTTPDataServerConnection.connect(CONTROLLER_IP, DataServerSettings.DEFAULT_PORT);
      YoVariableClient client = new YoVariableClient(scsVisualizer);
      client.start(TIMEOUT, connection);
   }

   public static void main(String[] args) throws JSAPException, IOException
   {
      LogTools.info("Starting the remote visualizer for controller at " + CONTROLLER_IP);

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
