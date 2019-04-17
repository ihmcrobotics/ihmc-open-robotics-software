package us.ihmc.parameterTuner.local;

import gnu.trove.map.TObjectIntMap;
import gnu.trove.map.hash.TObjectIntHashMap;
import javafx.geometry.Pos;
import javafx.scene.Node;
import javafx.scene.layout.VBox;
import us.ihmc.commons.thread.Notification;
import us.ihmc.parameterTuner.guiElements.GuiParameter;
import us.ihmc.parameterTuner.guiElements.GuiRegistry;
import us.ihmc.parameterTuner.guiElements.main.ChangeCollector;
import us.ihmc.parameterTuner.guiElements.main.ParameterGuiInterface;
import us.ihmc.parameterTuner.guiElements.main.ParameterSavingNode;
import us.ihmc.yoVariables.parameters.YoParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.yoVariables.variable.YoVariableType;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentLinkedQueue;

public class RegistryInputManager implements ParameterGuiInterface
{

   private final HashMap<String, GuiParameter> parameterMap = new HashMap<>();
   private final ParameterSavingNode savingNode = new ParameterSavingNode(false, false);

   private final Notification needsReloading = new Notification();

   private final YoVariableRegistry registry;
   private final Map<String, GuiParameter> guiParametersByYoName = new HashMap<>();
   private final Map<String, YoVariable<?>> yoVariablesByGuiName = new HashMap<>();


   private final ConcurrentLinkedQueue<GuiParameter> serverChangedParameters = new ConcurrentLinkedQueue<>();
   private final ConcurrentLinkedQueue<GuiParameter> userChangedParameters = new ConcurrentLinkedQueue<>();


   private ChangeCollector changeCollector = new ChangeCollector();


   public RegistryInputManager(YoVariableRegistry registry)
   {
      this.registry = registry;

      registry.getAllParameters()

      needsReloading.set();
   }

   /** @inheritDoc */
   @Override
   public boolean pollReloadAll()
   {
      return needsReloading.poll(); // return true once at the beginning then never again
   }

   /** @inheritDoc */
   @Override
   public List<GuiRegistry> getRegistriesCopy()
   {
      List<GuiRegistry> registriesCopy = new ArrayList<>();
      ArrayList<YoVariableRegistry> yoRegistries = registry.getChildren();
      yoRegistries.stream().forEach(yoRegistry -> {
         GuiRegistry guiRegistry = new GuiRegistry(yoRegistry.getName(), null);
         createGuiRegistryRecursive(guiRegistry, yoRegistry);
         registriesCopy.add(guiRegistry.createFullCopy());
      });

      // Maintain a local copy for saving that is only modified from the GUI thread.
      parameterMap.clear();
      List<GuiRegistry> localRegistries = new ArrayList<>();
      List<GuiParameter> allParameters = new ArrayList<>();
      registriesCopy.stream().forEach(fullGuiRegistry -> {
         GuiRegistry localRegistry = fullGuiRegistry.createFullCopy();
         allParameters.addAll(localRegistry.getAllParameters());
         localRegistries.add(localRegistry);
      });
      allParameters.stream().forEach(parameter -> {
         parameterMap.put(parameter.getUniqueName(), parameter);
      });
      savingNode.setRegistries(localRegistries);

      changeCollector = new ChangeCollector();

      return registriesCopy;
   }

   /** @inheritDoc */
   @Override
   public void submitChangedParameters(List<GuiParameter> changedParameters)
   {
      // For changes from the GUI set all properties of the parameter.
      changedParameters.stream().forEach(parameter -> {
         parameterMap.get(parameter.getUniqueName()).set(parameter);
         changeCollector.changed(parameter);
      });

      List<GuiParameter> changedParametersAndClear = changeCollector.getChangedParametersAndClear();
      changedParametersAndClear.stream().forEach(guiParameter -> {
         userChangedParameters.add(guiParameter);
      });
   }

   /** @inheritDoc */
   @Override
   public List<GuiParameter> pollUpdatedParameters()
   {
      GuiParameter serverChangedParameter;
      Map<String, GuiParameter> parametersToUpdate = new HashMap<>();
      while ((serverChangedParameter = serverChangedParameters.poll()) != null)
      {
         parametersToUpdate.put(serverChangedParameter.getUniqueName(), serverChangedParameter);
      }
      List<GuiParameter> changedParameters = new ArrayList<>(parametersToUpdate.values());

      // For changes from the server only update if the value changes to avoid loosing the status.
      changedParameters.stream().forEach(externalParameter -> {
         GuiParameter localParameter = parameterMap.get(externalParameter.getUniqueName());

         String uniqueName = externalParameter.getUniqueName();
         if (changeCollector.isPending(uniqueName))
         {
            changeCollector.parameterWasUpdated(uniqueName, externalParameter.getCurrentValue());
         }
         else if (!localParameter.getCurrentValue().equals(externalParameter.getCurrentValue()))
         {
            localParameter.setValueAndStatus(externalParameter);
         }
      });

      return changedParameters;
   }

   /** @inheritDoc */
   @Override
   public Node getInputManagerNode()
   {
      VBox node = new VBox();
      node.setMaxHeight(Double.NEGATIVE_INFINITY);
      node.setMaxWidth(Double.NEGATIVE_INFINITY);
      node.setSpacing(10.0);
      node.setAlignment(Pos.CENTER_LEFT);
      node.getChildren().add(savingNode);
      return node;
   }

   /** @inheritDoc */
   @Override
   public void shutdown()
   {
      // do nothing
   }

   /** @inheritDoc */
   @Override
   public void changeRootRegistries(List<String> rootRegistryNames)
   {
      savingNode.setRootRegistries(rootRegistryNames);
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
}
