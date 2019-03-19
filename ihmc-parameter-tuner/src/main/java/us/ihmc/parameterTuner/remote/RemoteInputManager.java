package us.ihmc.parameterTuner.remote;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import javafx.application.Platform;
import javafx.geometry.Pos;
import javafx.scene.Node;
import javafx.scene.control.Button;
import javafx.scene.layout.VBox;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.parameterTuner.guiElements.GuiParameter;
import us.ihmc.parameterTuner.guiElements.GuiRegistry;
import us.ihmc.parameterTuner.guiElements.main.ParameterGuiInterface;
import us.ihmc.parameterTuner.guiElements.main.ParameterSavingNode;
import us.ihmc.robotDataLogger.YoVariableClient;

public class RemoteInputManager implements ParameterGuiInterface
{
   private static final double TIMEOUT = 2.5;

   private final HashMap<String, GuiParameter> parameterMap = new HashMap<>();
   private final ParameterSavingNode savingNode = new ParameterSavingNode(false, false);

   private final Button reconnect = new Button("Reconnect");

   private final AtomicBoolean connected = new AtomicBoolean(false);
   private final ParameterUpdateListener updateListener;

   public RemoteInputManager()
   {
      updateListener = new ParameterUpdateListener();
      YoVariableClient client = new YoVariableClient(updateListener);

      updateListener.addConnectionListener(connected -> {
         this.connected.set(connected);
         Platform.runLater(() -> reconnect.setDisable(connected));
      });

      client.startWithHostSelector();
      waitForConnection();

      reconnect.setOnAction(event -> {
         if (connected.get())
         {
            return;
         }
         LogTools.info("Reconnecting to server.");
         try
         {
            client.reconnect();
            waitForConnection();
         }
         catch (IOException e)
         {
            throw new RuntimeException("Unable to reconnect.");
         }
      });
   }

   private void waitForConnection()
   {
      long startTime = System.nanoTime();
      while (!connected.get())
      {
         ThreadTools.sleep(10);
         if (Conversions.nanosecondsToSeconds(System.nanoTime() - startTime) > TIMEOUT)
         {
            throw new RuntimeException("Connection failed.");
         }
      }
   }

   @Override
   public boolean pollReloadAll()
   {
      return updateListener.hasNewGuiRegistry();
   }

   @Override
   public List<GuiRegistry> getRegistriesCopy()
   {
      List<GuiRegistry> guiRegistries = updateListener.pollGuiRegistries();

      // Maintain a local copy for saving that is only modified from the GUI thread.
      parameterMap.clear();
      List<GuiRegistry> localRegistries = new ArrayList<>();
      List<GuiParameter> allParameters = new ArrayList<>();
      guiRegistries.stream().forEach(fullGuiRegistry -> {
         GuiRegistry localRegistry = fullGuiRegistry.createFullCopy();
         allParameters.addAll(localRegistry.getAllParameters());
         localRegistries.add(localRegistry);
      });
      allParameters.stream().forEach(parameter -> {
         parameterMap.put(parameter.getUniqueName(), parameter);
      });
      savingNode.setRegistries(localRegistries);

      return guiRegistries;
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
      VBox node = new VBox();
      node.setMaxHeight(Double.NEGATIVE_INFINITY);
      node.setMaxWidth(Double.NEGATIVE_INFINITY);
      node.setSpacing(10.0);
      node.setAlignment(Pos.CENTER_LEFT);
      node.getChildren().add(reconnect);
      node.getChildren().add(savingNode);
      return node;
   }

   @Override
   public void shutdown()
   {
      updateListener.exitActionPerformed();
   }

   @Override
   public void changeRootRegistries(List<String> rootRegistryNames)
   {
      savingNode.setRootRegistries(rootRegistryNames);
   }
}
