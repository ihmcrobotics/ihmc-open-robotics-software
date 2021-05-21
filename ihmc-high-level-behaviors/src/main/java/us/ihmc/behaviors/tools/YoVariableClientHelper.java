package us.ihmc.behaviors.tools;

import org.apache.commons.lang3.mutable.MutableBoolean;
import us.ihmc.behaviors.tools.interfaces.YoVariableClientPublishSubscribeAPI;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataLogger.YoVariablesUpdatedListener;
import us.ihmc.robotDataVisualizer.BasicYoVariablesUpdatedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.function.DoubleSupplier;

public class YoVariableClientHelper implements YoVariableClientPublishSubscribeAPI
{
   private final YoRegistry yoRegistry;
   private final YoVariablesUpdatedListener listener;
   private final YoVariableClient yoVariableClient;

   public YoVariableClientHelper(String registryName)
   {
      yoRegistry = new YoRegistry(registryName);
      listener = new BasicYoVariablesUpdatedListener(yoRegistry);
      yoVariableClient = new YoVariableClient(listener);
   }

   public void start(String hostname, int port)
   {
      MutableBoolean connecting = new MutableBoolean(true);
      ThreadTools.startAThread(() ->
      {
         while (connecting.getValue())
         {
            try
            {
               LogTools.info("Connecting to {}:{}", hostname, port);
               getClient().start(hostname, port);
               connecting.setValue(false);
               LogTools.info("Connected to {}:{}", hostname, port);
            }
            catch (RuntimeException e)
            {
               LogTools.warn("Couldn't connect to {}:{}. Trying again...", hostname, port);
               ThreadTools.sleepSeconds(1.0);
            }
         }
      }, "YoVariableClientHelperConnection");
   }

   public YoVariableClient getClient()
   {
      return yoVariableClient;
   }

   public DoubleSupplier subscribeViaYoDouble(String variableName)
   {
      return () ->
      {
         YoVariable variable;
         if (yoVariableClient.isConnected() && (variable = yoRegistry.findVariable(variableName)) != null)
         {
            return variable.getValueAsDouble();
         }
         return Double.NaN;
      };
   }
}
