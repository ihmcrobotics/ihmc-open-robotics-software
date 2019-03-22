package us.ihmc.parameterTuner.remote;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicReference;

import gnu.trove.map.TObjectIntMap;
import gnu.trove.map.hash.TObjectIntHashMap;
import us.ihmc.parameterTuner.guiElements.GuiParameter;
import us.ihmc.parameterTuner.guiElements.GuiRegistry;
import us.ihmc.robotDataLogger.YoVariableClientInterface;
import us.ihmc.robotDataLogger.YoVariablesUpdatedListener;
import us.ihmc.robotDataLogger.handshake.LogHandshake;
import us.ihmc.robotDataLogger.handshake.YoVariableHandshakeParser;
import us.ihmc.robotDataLogger.websocket.command.DataServerCommand;
import us.ihmc.yoVariables.listener.ParameterChangedListener;
import us.ihmc.yoVariables.parameters.YoParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.yoVariables.variable.YoVariableType;

public class ParameterUpdateListener implements YoVariablesUpdatedListener
{
   private YoVariableClientInterface yoVariableClientInterface;
   private YoVariableRegistry yoRootRegistry;

   private final AtomicReference<List<GuiRegistry>> guiRegistriesCopy = new AtomicReference<>(null);

   private final Map<String, GuiParameter> guiParametersByYoName = new HashMap<>();
   private final Map<String, YoVariable<?>> yoVariablesByGuiName = new HashMap<>();

   private final ConcurrentLinkedQueue<GuiParameter> serverChangedParameters = new ConcurrentLinkedQueue<>();
   private final ConcurrentLinkedQueue<GuiParameter> userChangedParameters = new ConcurrentLinkedQueue<>();

   private boolean isUpdatingYoVariables = false;

   private final List<ConnectionListener> connectionListeners = new ArrayList<>();

   @Override
   public void receivedTimestampOnly(long timestamp)
   {
   }


   public void exitActionPerformed()
   {
      if (yoVariableClientInterface != null)
      {
         yoVariableClientInterface.stop();
      }
   }

   @Override
   public boolean updateYoVariables()
   {
      return true;
   }

   @Override
   public boolean changesVariables()
   {
      return true;
   }

   @Override
   public void setShowOverheadView(boolean showOverheadView)
   {
   }

   @Override
   public void start(YoVariableClientInterface yoVariableClientInterface, LogHandshake handshake, YoVariableHandshakeParser handshakeParser)
   {
      this.yoVariableClientInterface = yoVariableClientInterface;
      yoRootRegistry = handshakeParser.getRootRegistry();

      yoRootRegistry.getAllParameters().stream().forEach(parameter -> {
         parameter.addParameterChangedListener(new ParameterChangedListener()
         {
            @Override
            public void notifyOfParameterChange(YoParameter<?> changedParameter)
            {
               if (isUpdatingYoVariables)
               {
                  return;
               }

               String yoName = getUniqueName(changedParameter);
               GuiParameter newGuiParameter = new GuiParameter(guiParametersByYoName.get(yoName));
               newGuiParameter.setValue(changedParameter.getValueAsString());
               serverChangedParameters.add(newGuiParameter);
            }
         });
      });

      guiParametersByYoName.clear();
      yoVariablesByGuiName.clear();
   }

   @Override
   public void disconnected()
   {
      connectionListeners.forEach(l -> l.conectionStatusChanged(false));
   }

   @Override
   public void receivedTimestampAndData(long timestamp)
   {
      GuiParameter userChangedParameter;
      Map<String, GuiParameter> parametersToUpdate = new HashMap<>();
      while ((userChangedParameter = userChangedParameters.poll()) != null)
      {
         parametersToUpdate.put(userChangedParameter.getUniqueName(), userChangedParameter);
      }

      isUpdatingYoVariables = true;
      parametersToUpdate.values().forEach(guiParameter -> {
         String uniqueName = guiParameter.getUniqueName();
         YoVariable<?> yoVariable = yoVariablesByGuiName.get(uniqueName);
         setYoVariableFromGuiParameter(yoVariable, guiParameter);
      });
      isUpdatingYoVariables = false;
   }

   @Override
   public void connected()
   {
      List<GuiRegistry> registriesCopy = new ArrayList<>();
      ArrayList<YoVariableRegistry> yoRegistries = yoRootRegistry.getChildren();
      yoRegistries.stream().forEach(yoRegistry -> {
         GuiRegistry guiRegistry = new GuiRegistry(yoRegistry.getName(), null);
         createGuiRegistryRecursive(guiRegistry, yoRegistry);
         registriesCopy.add(guiRegistry.createFullCopy());
      });
      guiRegistriesCopy.set(registriesCopy);

      userChangedParameters.clear();

      connectionListeners.forEach(l -> l.conectionStatusChanged(true));
   }

   private void createGuiRegistryRecursive(GuiRegistry guiRegistry, YoVariableRegistry yoRegistry)
   {
      yoRegistry.getAllVariablesInThisListOnly().stream().forEach(yoVariable -> {
         if (yoVariable.isParameter())
         {
            TObjectIntMap<String> valueOptions = createValueOptions(yoVariable);
            YoParameter<?> yoParameter = yoVariable.getParameter();
            String guiParameterType = yoParameter.getClass().getSimpleName();
            GuiParameter guiParameter = new GuiParameter(yoParameter.getName(), guiParameterType, valueOptions, guiRegistry);
            setGuiParameterFromYoVariable(guiParameter, yoVariable);
            guiRegistry.addParameter(guiParameter);

            guiParametersByYoName.put(getUniqueName(yoParameter), guiParameter);
            yoVariablesByGuiName.put(guiParameter.getUniqueName(), yoVariable);
         }
      });
      yoRegistry.getChildren().stream().forEach(yoChild -> {
         if (!yoChild.getAllParameters().isEmpty())
         {
            GuiRegistry guiChild = new GuiRegistry(yoChild.getName(), guiRegistry);
            createGuiRegistryRecursive(guiChild, yoChild);
            guiRegistry.addRegistry(guiChild);
         }
      });
   }

   public boolean hasNewGuiRegistry()
   {
      return guiRegistriesCopy.get() != null;
   }

   public List<GuiRegistry> pollGuiRegistries()
   {
      return guiRegistriesCopy.getAndSet(null);
   }

   public List<GuiParameter> getChangedParametersAndClear()
   {
      GuiParameter serverChangedParameter;
      Map<String, GuiParameter> parametersToUpdate = new HashMap<>();
      while ((serverChangedParameter = serverChangedParameters.poll()) != null)
      {
         parametersToUpdate.put(serverChangedParameter.getUniqueName(), serverChangedParameter);
      }
      return new ArrayList<>(parametersToUpdate.values());
   }

   public void changeVariables(List<GuiParameter> changedParameters)
   {
      changedParameters.stream().forEach(guiParameter -> {
         userChangedParameters.add(guiParameter);
      });
   }

   /**
    * Note, that here the YoVariables are set twice:</br>
    * First to the new value triggering the change listeners that will send the value to the server.</br>
    * Second to the old value without sending it such that the value of the YoVariable will be updated properly once
    * it received the change back from the server.
    */
   private static void setYoVariableFromGuiParameter(YoVariable<?> yoVariable, GuiParameter guiParameter)
   {
      doChecks(guiParameter, yoVariable);
      long oldValue = yoVariable.getValueAsLongBits();

      switch (yoVariable.getYoVariableType())
      {
      case DOUBLE:
         double doubleValue = Double.parseDouble(guiParameter.getCurrentValue());
         YoDouble yoDouble = (YoDouble) yoVariable;
         yoDouble.set(doubleValue);
         break;
      case INTEGER:
         int integerValue = Integer.parseInt(guiParameter.getCurrentValue());
         YoInteger yoInteger = (YoInteger) yoVariable;
         yoInteger.set(integerValue);
         break;
      case BOOLEAN:
         boolean booleanValue = Boolean.parseBoolean(guiParameter.getCurrentValue());
         YoBoolean yoBoolean = (YoBoolean) yoVariable;
         yoBoolean.set(booleanValue);
         break;
      case ENUM:
         TObjectIntMap<String> valueOptions = guiParameter.getValueOptions();
         int ordinalValue = valueOptions.get(guiParameter.getCurrentValue());
         YoEnum<?> yoEnum = (YoEnum<?>) yoVariable;
         yoEnum.set(ordinalValue);
         break;
      default:
         throw new RuntimeException("Unhandled parameter type: " + yoVariable.getYoVariableType());
      }

      yoVariable.setValueFromLongBits(oldValue, false);
   }

   private static void setGuiParameterFromYoVariable(GuiParameter guiParameter, YoVariable<?> yoVariable)
   {
      doChecks(guiParameter, yoVariable);

      YoParameter<?> yoParameter = yoVariable.getParameter();
      guiParameter.setValue(yoParameter.getValueAsString());
      yoVariable.getManualScalingMin();
      guiParameter.setDescription(yoParameter.getDescription());
      guiParameter.setLoadStatus(yoParameter.getLoadStatus());

      switch (yoVariable.getYoVariableType())
      {
      case DOUBLE:
         guiParameter.setMin(Double.toString(yoVariable.getManualScalingMin()));
         guiParameter.setMax(Double.toString(yoVariable.getManualScalingMax()));
         break;
      case INTEGER:
         guiParameter.setMin(Integer.toString((int) yoVariable.getManualScalingMin()));
         guiParameter.setMax(Integer.toString((int) yoVariable.getManualScalingMax()));
         break;
      case BOOLEAN:
         break;
      case ENUM:
         break;
      default:
         throw new RuntimeException("Unhandled parameter type: " + yoVariable.getYoVariableType());
      }
   }

   private static TObjectIntMap<String> createValueOptions(YoVariable<?> yoVariable)
   {
      if (yoVariable.getYoVariableType() != YoVariableType.ENUM)
      {
         return null;
      }

      YoEnum<?> yoEnum = (YoEnum<?>) yoVariable;
      TObjectIntMap<String> valueOptions = new TObjectIntHashMap<>();
      if (yoEnum.getAllowNullValue())
      {
         valueOptions.put("null", YoEnum.NULL_VALUE);
      }
      String[] options = yoEnum.getEnumValuesAsString();
      for (int ordinal = 0; ordinal < options.length; ordinal++)
      {
         valueOptions.put(options[ordinal], ordinal);
      }

      return valueOptions;
   }

   private static void doChecks(GuiParameter guiParameter, YoVariable<?> yoVariable)
   {
      if (!yoVariable.isParameter())
      {
         throw new RuntimeException("Expecting parameter.");
      }

      String guiParameterType = guiParameter.getType();
      String yoParameterType = yoVariable.getParameter().getClass().getSimpleName();
      if (!guiParameterType.equals(yoParameterType))
      {
         throw new RuntimeException("Incompatible data types: " + guiParameterType + " and " + yoParameterType);
      }
   }

   private static String getUniqueName(YoParameter<?> yoParameter)
   {
      return yoParameter.getNameSpace() + "." + yoParameter.getName();
   }

   @Override
   public void receivedCommand(DataServerCommand command, int argument)
   {
   }

   public void addConnectionListener(ConnectionListener connectionListener)
   {
      connectionListeners.add(connectionListener);
   }
}
