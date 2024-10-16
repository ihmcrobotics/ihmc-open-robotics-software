package us.ihmc.behaviors.tools.yo;

import org.apache.commons.lang3.mutable.MutableLong;
import us.ihmc.commons.Conversions;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.commons.UnitConversions;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoLong;

public class YoVariableServerHelper
{
   private final YoVariableServer yoVariableServer;
   private PausablePeriodicThread yoServerUpdateThread;
   private final YoLong updateIndex;
   private final String name;
   private int port;
   private final double updatePeriod;

   public YoVariableServerHelper(Class<?> clazz, YoRegistry registry, int port, double updateFrequency)
   {
      this.name = clazz.getSimpleName();
      this.port = port;
      this.updatePeriod = UnitConversions.hertzToSeconds(updateFrequency);
      DataServerSettings dataServerSettings = new DataServerSettings(false, true, port, null);
      updateIndex = new YoLong("updateIndex", registry);
      yoVariableServer = new YoVariableServer(getClass().getSimpleName(), null, dataServerSettings, 0.01);
      yoVariableServer.setMainRegistry(registry, null);
   }

   public void start()
   {
      LogTools.info("Starting YoVariableServer on {}...", port);
      yoVariableServer.start();
      LogTools.info("Starting YoVariableServer update thread for {}...", port);
      MutableLong timestamp = new MutableLong();
      yoServerUpdateThread = new PausablePeriodicThread("YoServerUpdate" + name, updatePeriod, () ->
      {
         updateIndex.add(1);
         yoVariableServer.update(timestamp.getAndAdd(Conversions.secondsToNanoseconds(updatePeriod)));
      });
      yoServerUpdateThread.start();
   }

   public void destroy()
   {
      LogTools.info("Shutting down...");
      yoServerUpdateThread.destroy();
      yoVariableServer.close();
   }
}
