package us.ihmc.parameterTuner.offline;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import javafx.event.ActionEvent;
import javafx.geometry.Pos;
import javafx.scene.Node;
import javafx.scene.control.Button;
import javafx.scene.layout.VBox;
import javafx.stage.FileChooser;
import us.ihmc.parameterTuner.ParameterSavingTools;
import us.ihmc.parameterTuner.ParameterTuningTools;
import us.ihmc.parameterTuner.guiElements.GuiParameter;
import us.ihmc.parameterTuner.guiElements.GuiRegistry;
import us.ihmc.parameterTuner.guiElements.main.ParameterGuiInterface;
import us.ihmc.parameterTuner.guiElements.main.ParameterSavingNode;
import us.ihmc.yoVariables.parameters.xml.Registry;

public class FileInputManager extends VBox implements ParameterGuiInterface
{
   private final Button open = new Button("Open");
   private final ParameterSavingNode savingNode = new ParameterSavingNode(true, true);

   private boolean reloadAll;
   private List<GuiRegistry> localRegistries;
   private HashMap<String, GuiParameter> parameterMap = new HashMap<>();

   private final List<GuiParameter> changedParameters = new ArrayList<>();

   public FileInputManager()
   {
      this(null);
   }

   public FileInputManager(File defaultFile)
   {
      setupNode();
      openFileSafe(defaultFile);
      savingNode.addSavedListener(file -> openFileSafe(file));
   }

   private void setupNode()
   {
      setMaxHeight(Double.NEGATIVE_INFINITY);
      setMaxWidth(Double.NEGATIVE_INFINITY);
      setSpacing(10.0);
      setAlignment(Pos.CENTER_LEFT);

      getChildren().add(open);
      getChildren().add(savingNode);

      open.setOnAction(event -> handleOpen(event));
   }

   @Override
   public Node getInputManagerNode()
   {
      return this;
   }

   @Override
   public boolean pollReloadAll()
   {
      boolean ret = reloadAll;
      reloadAll = false;
      return ret;
   }

   @Override
   public List<GuiRegistry> getRegistriesCopy()
   {
      List<GuiRegistry> registriesCopy = new ArrayList<>();
      localRegistries.stream().forEach(registry -> registriesCopy.add(registry.createFullCopy()));
      return registriesCopy;
   }

   @Override
   public void submitChangedParameters(List<GuiParameter> changedParameters)
   {
      changedParameters.stream().forEach(parameter -> {
         parameterMap.get(parameter.getUniqueName()).set(parameter);
      });

      this.changedParameters.clear();
      this.changedParameters.addAll(changedParameters);
   }

   @Override
   public List<GuiParameter> pollUpdatedParameters()
   {
      List<GuiParameter> changedParameters = new ArrayList<>();
      changedParameters.addAll(this.changedParameters);
      this.changedParameters.clear();
      return changedParameters;
   }

   @Override
   public void shutdown()
   {
   }

   private void handleOpen(ActionEvent event)
   {
      FileChooser fileChooser = new FileChooser();
      fileChooser.getExtensionFilters().add(ParameterSavingTools.getExtensionFilter());
      fileChooser.setTitle("Select Parameter File");

      // If a file is open initialize the folder path to the directory of the active file:
      File initialPath = ParameterSavingNode.getDefaultFilePath();
      if (initialPath != null)
      {
         fileChooser.setInitialDirectory(initialPath);
      }
      File file = fileChooser.showOpenDialog(open.getScene().getWindow());

      openFileSafe(file);
   }

   private void openFileSafe(File file)
   {
      try
      {
         openFile(file);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   private void openFile(File file) throws IOException
   {
      if (file != null)
      {
         List<Registry> xmlRegistries = ParameterTuningTools.getParameters(file);
         localRegistries = ParameterTuningTools.buildGuiRegistryFromXML(xmlRegistries);
         parameterMap.clear();
         List<GuiParameter> allParameters = new ArrayList<>();
         localRegistries.stream().forEach(registry -> allParameters.addAll(registry.getAllParameters()));
         allParameters.stream().forEach(parameter -> parameterMap.put(parameter.getUniqueName(), parameter));
         reloadAll = true;

         savingNode.setActiveFile(file);
         savingNode.setRegistries(localRegistries);
      }
   }

   @Override
   public void changeRootRegistries(List<String> rootRegistryNames)
   {
      savingNode.setRootRegistries(rootRegistryNames);
   }
}
