package us.ihmc.parameterTuner.remote;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import javafx.application.Platform;
import javafx.geometry.Pos;
import javafx.scene.Node;
import javafx.scene.control.Alert;
import javafx.scene.control.Alert.AlertType;
import javafx.scene.control.Button;
import javafx.scene.control.ButtonType;
import javafx.scene.layout.Region;
import javafx.scene.layout.VBox;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.parameterTuner.guiElements.GuiParameter;
import us.ihmc.parameterTuner.guiElements.GuiRegistry;
import us.ihmc.parameterTuner.guiElements.main.ChangeCollector;
import us.ihmc.parameterTuner.guiElements.main.ParameterGuiInterface;
import us.ihmc.parameterTuner.guiElements.main.ParameterSavingNode;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataLogger.logger.DataServerSettings;

public class RemoteInputManager implements ParameterGuiInterface
{
   private static final double TIMEOUT = 2.5;

   private final HashMap<String, GuiParameter> parameterMap = new HashMap<>();
   private final ParameterSavingNode savingNode = new ParameterSavingNode(false, false);

   private final Button reconnect = new Button("Reconnect");

   private final AtomicBoolean connected = new AtomicBoolean(false);
   private final ParameterUpdateListener updateListener;

   private ChangeCollector changeCollector = new ChangeCollector();

   public RemoteInputManager()
   {
      this(null);
   }

   public RemoteInputManager(String serverAddress)
   {
      updateListener = new ParameterUpdateListener();
      updateListener.addConnectionListener(connected -> {
         this.connected.set(connected);
         Platform.runLater(() -> reconnect.setDisable(connected));
      });

      YoVariableClient client = new YoVariableClient(updateListener);
      if (serverAddress == null)
      {
         client.startWithHostSelector();
      }
      else
      {
         client.start(serverAddress, DataServerSettings.DEFAULT_PORT);
      }
      waitForConnection();

      reconnect.setOnAction(event -> {
         if (connected.get())
         {
            return;
         }
         if (!checkReconnectWithUser())
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

   private boolean checkReconnectWithUser()
   {
      Alert alert = new Alert(AlertType.CONFIRMATION);
      alert.setTitle("Reconnect");
      alert.getDialogPane().setMinHeight(Region.USE_PREF_SIZE);
      alert.getDialogPane().setMinWidth(Region.USE_PREF_SIZE);
      alert.setHeaderText("Do you want to reconnect?");
      alert.setContentText("Previously modified variables will be overwritten by the server.");
      return alert.showAndWait().get() == ButtonType.OK;
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

      changeCollector = new ChangeCollector();

      return guiRegistries;
   }

   @Override
   public void submitChangedParameters(List<GuiParameter> changedParameters)
   {
      // For changes from the GUI set all properties of the parameter.
      changedParameters.stream().forEach(parameter -> {
         parameterMap.get(parameter.getUniqueName()).set(parameter);
         changeCollector.changed(parameter);
      });

      updateListener.changeVariables(changeCollector.getChangedParametersAndClear());
   }

   @Override
   public List<GuiParameter> pollUpdatedParameters()
   {
      List<GuiParameter> changedParameters = updateListener.getChangedParametersAndClear();

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
