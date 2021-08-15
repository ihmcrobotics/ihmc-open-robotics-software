package us.ihmc.behaviors.tools.yo;

import org.apache.commons.lang3.mutable.MutableBoolean;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataLogger.YoVariablesUpdatedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

public class YoVariableClientHelper implements YoVariableClientPublishSubscribeAPI
{
   private final String registryName;
   private YoRegistry yoRegistry;
   private YoVariablesUpdatedListener listener;
   private YoVariableClient yoVariableClient;
   private final MutableBoolean connecting = new MutableBoolean(false);

   public YoVariableClientHelper(String registryName)
   {
      this.registryName = registryName;
   }

   public void start(String hostname, int port)
   {
      yoRegistry = new YoRegistry(registryName);
      listener = new YoVariableClientHelperUpdatedListener(yoRegistry);
      yoVariableClient = new YoVariableClient(listener);
      connecting.setValue(true);
      ThreadTools.startAThread(() ->
      {
         int tries = 5;
         while (!isConnected() && tries > 0)
         {
            try
            {
               LogTools.info("Connecting to {}:{}", hostname, port);
               yoVariableClient.start(hostname, port);
               LogTools.info("Connected to {}:{}", hostname, port);
            }
            catch (RuntimeException e)
            {
               LogTools.warn("Couldn't connect to {}:{}. {} Trying again...", hostname, port, e.getMessage());
               ThreadTools.sleep(1000);
               --tries;
            }
         }
         connecting.setValue(false);
      }, "YoVariableClientHelperConnection");
   }

   public boolean isConnected()
   {
      return yoVariableClient != null && yoVariableClient.isConnected();
   }

   public boolean isConnecting()
   {
      return connecting.getValue();
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

   public ArrayList<String> getVariableNames()
   {
      ArrayList<String> variableNames = new ArrayList<>();
      if (isConnected())
      {
         getAllVariableNamesRecursively(yoRegistry, variableNames);
      }
      return variableNames;
   }

   private void getAllVariableNamesRecursively(YoRegistry registry, ArrayList<String> variableNames)
   {
      for (YoVariable variable : registry.getVariables())
      {
         variableNames.add(variable.getName());
      }

      for (YoRegistry child : registry.getChildren())
      {
         getAllVariableNamesRecursively(child, variableNames);
      }
   }

   private YoVariable tryToGetVariable(String variableName)
   {
      YoVariable variable = null;
      if (yoVariableClient != null && yoVariableClient.isConnected())
      {
         variable = yoRegistry.findVariable(variableName);
      }
      return variable;
   }

   @Override
   public DoubleSupplier subscribeToYoVariableDoubleValue(String variableName)
   {
      return () ->
      {
         YoVariable variable = tryToGetVariable(variableName);
         if (variable != null)
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

         @Override
         public String getName()
         {
            YoVariable variable = tryToGetVariable(variableName);
            return variable == null ? variableName.substring(variableName.lastIndexOf(".")) : variable.getName();
         }

         @Override
         public String getFullName()
         {
            YoVariable variable = tryToGetVariable(variableName);
            return variable == null ? "" : variable.getFullNameString();
         }

         @Override
         public String getDescription()
         {
            YoVariable variable = tryToGetVariable(variableName);
            return variable == null ? "" : variable.getDescription();
         }
      };
   }
}
