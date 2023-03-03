package us.ihmc.robotDataCommunication;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.Conversions;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizer;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.yoVariables.buffer.YoBuffer;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoLong;

import static us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools.random;
import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;

public class YoVariableConnectionBurstTest
{
   enum TestEnum
   {
      A, B, C, D
   }

   private static final double dt = 0.001;
   private long timestamp = 0;
   private static final DataServerSettings logSettings = new DataServerSettings(true);

   private final YoRegistry registry = new YoRegistry("Main");
   private final YoLong seq_id = new YoLong("seq_id", registry);
   private final YoEnum<TestEnum> var3 = new YoEnum<>("var3", "", registry, TestEnum.class, true);

   private void ensureServerClientConnection(YoVariableServer server, long jitteryTimestamp)
   {
      //This amount of updates and sleep prevents any loss in data being lost in the initial connection
      ThreadTools.sleepSeconds(10);

      for (int i = 0; i < 6; i++)
      {
         server.update(jitteryTimestamp);
      }

      ThreadTools.sleepSeconds(10);
   }

   private void updateVariables(YoVariableServer server, long jitteryTimestamp)
   {
      for (int i = 0; i < 6; i++)
      {
         server.update(jitteryTimestamp);
      }

      ThreadTools.sleepSeconds(5);
   }

   @Disabled
   @Test
   public void TestYoVariableConnectionBurst()
   {
      // Creates server, adds the main registry, and starts the server
      final YoVariableServer server = new YoVariableServer("TestServer", null, logSettings, dt);
      server.setMainRegistry(registry, null);
      server.start();

      //Creates the listener for the client and starts the client with the localhost
      int bufferSize = 256;
      SCSVisualizer scsYoVariablesUpdatedListener = new SCSVisualizer(bufferSize);
      scsYoVariablesUpdatedListener.setVariableUpdateRate(0);
      scsYoVariablesUpdatedListener.setShowOverheadView(false);

      YoVariableClient client = new YoVariableClient(scsYoVariablesUpdatedListener);
      client.start("localhost", 8008);

      // timestamp and dtFactor are used to generate the jitteryTimestamp that will be sent to the server as the time when the update method was called
      timestamp += Conversions.secondsToNanoseconds(dt);
      long dtFactor = Conversions.secondsToNanoseconds(dt) / 2;
      long jitteryTimestamp = timestamp + (long) ((random.nextDouble() - 0.5) * dtFactor);

      ensureServerClientConnection(server, jitteryTimestamp);

      // Start a producer/consumer test
      seq_id.set(0L);

      TestEnum[] values = {TestEnum.A, TestEnum.B, TestEnum.C, TestEnum.D};

      for (int i = 0; i < 16; i++)
      {
         seq_id.increment();
         var3.set(values[i % 4]);

         updateVariables(server, jitteryTimestamp);

         YoBuffer buffer = scsYoVariablesUpdatedListener.getDataBuffer();
         YoLong seq = (YoLong) buffer.findVariable("seq_id");

         double[] filledBuffer = buffer.getEntry(seq).getBuffer();
         int lastIndex = bufferSize - 1;

         while (buffer.getEntry(seq).getBuffer()[lastIndex] == 0.0)
         {
            lastIndex = lastIndex - 1;
         }

         //Check if last 8 values of the YoVariable match the buffer data
         for (int j = lastIndex; j > (lastIndex - 8); j--)
         {
            assertEquals("Buffer at j: " + filledBuffer[j] + " but buffer at j + 1: " + filledBuffer[j + 1], filledBuffer[j], (double) j);
            assertFalse(filledBuffer[j] == filledBuffer[j - 1]);
         }

         scsYoVariablesUpdatedListener.closeAndDispose();
         server.close();
      }
   }
}
