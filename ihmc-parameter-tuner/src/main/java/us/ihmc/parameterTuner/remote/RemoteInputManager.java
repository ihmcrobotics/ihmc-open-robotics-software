package us.ihmc.parameterTuner.remote;

import java.util.HashMap;
import java.util.List;

import javafx.scene.Node;
import us.ihmc.parameterTuner.guiElements.GuiParameter;
import us.ihmc.parameterTuner.guiElements.GuiRegistry;
import us.ihmc.parameterTuner.guiElements.main.ParameterGuiInterface;
import us.ihmc.parameterTuner.guiElements.main.ParameterSavingNode;
import us.ihmc.robotDataLogger.YoVariableClient;

public class RemoteInputManager implements ParameterGuiInterface
{
   private final HashMap<String, GuiParameter> parameterMap = new HashMap<>();
   private final ParameterSavingNode savingNode = new ParameterSavingNode(false, false);

   private final ParameterUpdateListener updateListener;

   public RemoteInputManager()
   {
      updateListener = new ParameterUpdateListener();
      YoVariableClient client = new YoVariableClient(updateListener);
      client.start();
   }

   @Override
   public boolean pollReloadAll()
   {
      return updateListener.hasNewGuiRegistry();
   }

   @Override
   public GuiRegistry getFullRegistryCopy()
   {
      GuiRegistry fullGuiRegistry = updateListener.pollGuiRegistry();

      // Maintain a local copy for saving that is only modified from the GUI thread.
      GuiRegistry localRegistry = fullGuiRegistry.createFullCopy();
      parameterMap.clear();
      localRegistry.getAllParameters().stream().forEach(parameter -> {
         parameterMap.put(parameter.getUniqueName(), parameter);
      });
      savingNode.setRegistry(localRegistry);

      return fullGuiRegistry;
   }

   @Override
   public void submitChangedParameters(List<GuiParameter> changedParameters)
   {
      // For changes from the GUI set all properties of the parameter.
      changedParameters.stream().forEach(parameter -> {
         parameterMap.get(parameter.getUniqueName()).set(parameter);
      });

      updateListener.changeVariables(changedParameters);
   }

   @Override
   public List<GuiParameter> pollUpdatedParameters()
   {
      List<GuiParameter> changedParameters = updateListener.getChangedParametersAndClear();

      // For changes from the server only update if the value changes to avoid loosing the status.
      changedParameters.stream().forEach(externalParameter -> {
         GuiParameter localParameter = parameterMap.get(externalParameter.getUniqueName());
         if (!localParameter.getCurrentValue().equals(externalParameter.getCurrentValue()))
         {
            localParameter.setValueAndStatus(externalParameter);
         }
      });

      return changedParameters;
   }

   @Override
   public Node getInputManagerNode()
   {
      return savingNode;
   }

   @Override
   public void shutdown()
   {
      updateListener.exitActionPerformed();
   }
}
