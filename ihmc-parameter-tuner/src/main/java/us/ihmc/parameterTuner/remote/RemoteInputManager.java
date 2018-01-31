package us.ihmc.parameterTuner.remote;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.List;

import javafx.event.ActionEvent;
import javafx.geometry.Pos;
import javafx.scene.Node;
import javafx.scene.control.Button;
import javafx.scene.layout.HBox;
import javafx.stage.FileChooser;
import us.ihmc.parameterTuner.ParameterTuningTools;
import us.ihmc.parameterTuner.guiElements.GuiParameter;
import us.ihmc.parameterTuner.guiElements.GuiRegistry;
import us.ihmc.parameterTuner.guiElements.main.ParameterGuiInterface;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.yoVariables.parameters.xml.Registry;

public class RemoteInputManager extends HBox implements ParameterGuiInterface
{
   private GuiRegistry localRegistry;
   private final HashMap<String, GuiParameter> parameterMap = new HashMap<>();
   private final Button saveAs = new Button("Save as...");

   private final ParameterUpdateListener updateListener;

   public RemoteInputManager()
   {
      setupNode();
      updateListener = new ParameterUpdateListener();
      YoVariableClient client = new YoVariableClient(updateListener);
      client.start();
   }

   private void setupNode()
   {
      setMaxHeight(Double.NEGATIVE_INFINITY);
      setMaxWidth(Double.NEGATIVE_INFINITY);
      setSpacing(10.0);
      setAlignment(Pos.CENTER_LEFT);
      saveAs.setOnAction(this::saveAs);
      getChildren().add(saveAs);
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
      localRegistry = fullGuiRegistry.createFullCopy();
      parameterMap.clear();
      localRegistry.getAllParameters().stream().forEach(parameter -> {
         parameterMap.put(parameter.getUniqueName(), parameter);
      });

      return fullGuiRegistry;
   }

   @Override
   public void submitChangedParameters(List<GuiParameter> changedParameters)
   {
      updateLocalRegistry(changedParameters);
      updateListener.changeVariables(changedParameters);
   }

   @Override
   public List<GuiParameter> pollUpdatedParameters()
   {
      List<GuiParameter> changedParameters = updateListener.getChangedParametersAndClear();
      updateLocalRegistry(changedParameters);
      return changedParameters;
   }

   private void updateLocalRegistry(List<GuiParameter> changedParameters)
   {
      changedParameters.stream().forEach(parameter -> {
         parameterMap.get(parameter.getUniqueName()).set(parameter);
      });
   }

   @Override
   public Node getInputManagerNode()
   {
      return this;
   }

   @Override
   public void shutdown()
   {
      updateListener.exitActionPerformed();
   }

   private void saveAs(ActionEvent event)
   {
      FileChooser fileChooser = new FileChooser();
      FileChooser.ExtensionFilter extFilter = new FileChooser.ExtensionFilter("XML files (*.xml)", "*.xml");
      fileChooser.getExtensionFilters().add(extFilter);
      fileChooser.setTitle("Select Parameter File");
      File file = fileChooser.showSaveDialog(saveAs.getScene().getWindow());

      if (file != null)
      {
         List<Registry> xmlRegistries = ParameterTuningTools.buildXMLRegistryFromGui(localRegistry);
         try
         {
            ParameterTuningTools.save(xmlRegistries, file);
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }
   }
}
