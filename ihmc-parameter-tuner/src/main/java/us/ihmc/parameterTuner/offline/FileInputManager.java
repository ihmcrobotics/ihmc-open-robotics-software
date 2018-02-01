package us.ihmc.parameterTuner.offline;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Node;
import javafx.scene.control.Alert;
import javafx.scene.control.Alert.AlertType;
import javafx.scene.control.Button;
import javafx.scene.control.ButtonType;
import javafx.scene.layout.HBox;
import javafx.scene.text.Text;
import javafx.stage.FileChooser;
import us.ihmc.parameterTuner.ParameterTuningTools;
import us.ihmc.parameterTuner.guiElements.GuiParameter;
import us.ihmc.parameterTuner.guiElements.GuiRegistry;
import us.ihmc.parameterTuner.guiElements.main.ParameterGuiInterface;
import us.ihmc.yoVariables.parameters.xml.Registry;

public class FileInputManager extends HBox implements ParameterGuiInterface
{
   private static final String FXML_PATH = "file_input_manager.fxml";

   @FXML
   private Button saveAs;
   @FXML
   private Button save;
   @FXML
   private Button open;
   @FXML
   private Text fileName;

   private boolean reloadAll;
   private File originalFile;

   private GuiRegistry localRegistry;
   private HashMap<String, GuiParameter> parameterMap = new HashMap<>();

   public FileInputManager()
   {
      this(null);
   }

   public FileInputManager(File defaultFile)
   {
      FXMLLoader loader = new FXMLLoader(getClass().getResource(FXML_PATH));
      loader.setRoot(this);
      loader.setController(this);
      try
      {
         loader.load();
         openFile(defaultFile);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
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
   public GuiRegistry getFullRegistryCopy()
   {
      return localRegistry.createFullCopy();
   }

   @Override
   public void submitChangedParameters(List<GuiParameter> changedParameters)
   {
      changedParameters.stream().forEach(parameter -> {
         parameterMap.get(parameter.getUniqueName()).set(parameter);
      });
   }

   @Override
   public List<GuiParameter> pollUpdatedParameters()
   {
      return null;
   }

   @Override
   public void shutdown()
   {
   }

   @FXML
   protected void handleOpen(ActionEvent event) throws IOException
   {
      FileChooser fileChooser = new FileChooser();
      FileChooser.ExtensionFilter extFilter = new FileChooser.ExtensionFilter("XML files (*.xml)", "*.xml");
      fileChooser.getExtensionFilters().add(extFilter);
      fileChooser.setTitle("Select Parameter File");
      if (originalFile != null)
      {
         fileChooser.setInitialDirectory(originalFile.getParentFile());
      }
      File file = fileChooser.showOpenDialog(open.getScene().getWindow());

      openFile(file);
   }

   private void openFile(File file) throws IOException
   {
      if (file != null)
      {
         fileName.setText(file.getName());
         originalFile = file;
         List<Registry> xmlRegistries = ParameterTuningTools.getParameters(originalFile);
         localRegistry = ParameterTuningTools.buildGuiRegistryFromXML(xmlRegistries);
         parameterMap.clear();
         localRegistry.getAllParameters().stream().forEach(parameter -> parameterMap.put(parameter.getUniqueName(), parameter));
         reloadAll = true;
      }
   }

   @FXML
   protected void handleSave(ActionEvent event) throws IOException
   {
      if (originalFile == null)
      {
         handleSaveAs(event);
         return;
      }
      if (localRegistry == null)
      {
         return;
      }

      Alert alert = new Alert(AlertType.CONFIRMATION);
      alert.setTitle("Confirmation Dialog");
      alert.setHeaderText("Confirm Save");
      alert.setContentText("This will overwrite the original XML parameter file.");
      Optional<ButtonType> result = alert.showAndWait();

      if (result.get() == ButtonType.OK)
      {
         List<Registry> xmlRegistries = ParameterTuningTools.buildXMLRegistryFromGui(localRegistry);
         ParameterTuningTools.save(xmlRegistries, originalFile);
      }
   }

   @FXML
   protected void handleSaveAs(ActionEvent event) throws IOException
   {
      if (localRegistry == null)
      {
         return;
      }

      FileChooser fileChooser = new FileChooser();
      FileChooser.ExtensionFilter extFilter = new FileChooser.ExtensionFilter("XML files (*.xml)", "*.xml");
      fileChooser.getExtensionFilters().add(extFilter);
      fileChooser.setTitle("Select Parameter File");
      if (originalFile != null)
      {
         fileChooser.setInitialDirectory(originalFile.getParentFile());
      }
      File file = fileChooser.showSaveDialog(saveAs.getScene().getWindow());

      if (file != null)
      {
         List<Registry> xmlRegistries = ParameterTuningTools.buildXMLRegistryFromGui(localRegistry);
         ParameterTuningTools.save(xmlRegistries, file);
         originalFile = file;
         fileName.setText(file.getName());
      }
   }

}
