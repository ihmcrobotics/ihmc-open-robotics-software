package us.ihmc.behaviors.tools.yo;

import org.apache.commons.lang3.mutable.MutableBoolean;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataLogger.YoVariablesUpdatedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.function.DoubleSupplier;

public class YoVariableClientHelper implements YoVariableClientPublishSubscribeAPI
{
   private final String registryName;
   private YoRegistry yoRegistry;
   private YoVariablesUpdatedListener listener;
   private YoVariableClient yoVariableClient;

   public YoVariableClientHelper(String registryName)
   {
      this.registryName = registryName;
   }

   public void start(String hostname, int port)
   {
      yoRegistry = new YoRegistry(registryName);
      listener = new YoVariableClientHelperUpdatedListener(yoRegistry);
      yoVariableClient = new YoVariableClient(listener);
      MutableBoolean connecting = new MutableBoolean(true);
      ThreadTools.startAThread(() ->
      {
         while (connecting.getValue())
         {
            try
            {
               LogTools.info("Connecting to {}:{}", hostname, port);
               yoVariableClient.start(hostname, port);
               connecting.setValue(false);
               LogTools.info("Connected to {}:{}", hostname, port);
            }
            catch (RuntimeException e)
            {
               LogTools.warn("Couldn't connect to {}:{}. {} Trying again...", hostname, port, e.getMessage());
               ThreadTools.sleepSeconds(1.0);
            }
         }
      }, "YoVariableClientHelperConnection");
   }

   public boolean isConnected()
   {
      return yoVariableClient != null && yoVariableClient.isConnected();
   }

   public void disconnect()
   {
      if (yoVariableClient != null)
         yoVariableClient.disconnect();
   }

   public String getServerName()
   {
      return yoVariableClient == null ? "null" : yoVariableClient.getServerName();
   }

   @Override
   public DoubleSupplier subscribeToYoVariableDoubleValue(String variableName)
   {
      return () ->
      {
         YoVariable variable = null;
         if (yoVariableClient != null && yoVariableClient.isConnected() && (variable = yoRegistry.findVariable(variableName)) != null)
         {
            return variable.getValueAsDouble();
         }
         return Double.NaN;
      };
   }

   @Override
   public void publishDoubleValueToYoVariable(String variableName, double value)
   {
      YoVariable variable;
      if (yoVariableClient != null && yoVariableClient.isConnected() && (variable = yoRegistry.findVariable(variableName)) != null)
      {
         variable.setValueFromDouble(value);
      }
   }

   @Override
   public YoBooleanClientHelper subscribeToYoBoolean(String variableName)
   {
      return new YoBooleanClientHelper()
      {
         private final DoubleSupplier getter = subscribeToYoVariableDoubleValue(variableName);

         @Override
         public boolean get()
         {
            return getter.getAsDouble() >= 0.5;
         }

         @Override
         public void set(boolean set)
         {
            publishDoubleValueToYoVariable(variableName, set ? 1.0 : 0.0);
         }
      };
   }

   @Override
   public YoDoubleClientHelper subscribeToYoDouble(String variableName)
   {
      return new YoDoubleClientHelper()
      {
         private final DoubleSupplier getter = subscribeToYoVariableDoubleValue(variableName);

         @Override
         public double get()
         {
            return getter.getAsDouble();
         }

         @Override
         public void set(double set)
         {
            publishDoubleValueToYoVariable(variableName, set);
         }
      };
   }
}
