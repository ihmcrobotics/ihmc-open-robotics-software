package us.ihmc.behaviors.tools.yo;

import org.apache.commons.lang3.mutable.MutableBoolean;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataLogger.YoVariablesUpdatedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

/**
 * Tries to make it seamless to connect to a YoVariable server, observe and change
 * YoVariables.
 */
public class YoVariableClientHelper implements YoVariableClientPublishSubscribeAPI
{
   private final String registryName;
   private YoRegistry yoRegistry;
   private YoVariablesUpdatedListener listener;
   private YoVariableClient yoVariableClient;
   private final MutableBoolean connecting = new MutableBoolean(false);
   private final Notification connectedNotification = new Notification();

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
      StackTraceElement callerStackElement = new Throwable().getStackTrace()[1];
      String caller = callerStackElement.getFileName() + ":" + callerStackElement.getLineNumber();
      ThreadTools.startAThread(() ->
      {
         int numberOfTriesLeft = 5;
         while (!isConnected() && numberOfTriesLeft > 0 && connecting.getValue())
         {
            --numberOfTriesLeft;
            try
            {
               LogTools.info(caller + ": Connecting to {}:{}", hostname, port);
               yoVariableClient.start(hostname, port);
               LogTools.info(caller + ": Connected to {}:{}", hostname, port);
               connectedNotification.set();
            }
            catch (RuntimeException e)
            {
               if (numberOfTriesLeft > 0)
               {
                  LogTools.warn("%s: Couldn't connect to %s:%d. Trying again %d more time(s).\n\t%s".formatted(caller, hostname, port, numberOfTriesLeft, e.getMessage()));
                  ThreadTools.sleep(1000);
               }
               else
               {
                  LogTools.warn("%s: Couldn't connect to %s:%d. Giving up.\n\t%s".formatted(caller, hostname, port, e.getMessage()));
               }
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
      {
         yoVariableClient.disconnect();
         yoVariableClient = null;
         connecting.setValue(false);
      }
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
   
   public YoVariable getYoVariable(String variableName)
   {
      return yoRegistry.findVariable(variableName);
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

   /**
    * Allows the user to get notified when we are connected, so they can
    * set YoVariables and stuff.
    */
   public Notification getConnectedNotification()
   {
      return connectedNotification;
   }
}
