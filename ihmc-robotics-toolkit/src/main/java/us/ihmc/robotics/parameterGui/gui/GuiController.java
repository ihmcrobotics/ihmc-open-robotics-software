package us.ihmc.robotics.parameterGui.gui;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import javafx.beans.Observable;
import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.scene.control.Alert;
import javafx.scene.control.Alert.AlertType;
import javafx.scene.control.Button;
import javafx.scene.control.ButtonType;
import javafx.scene.control.CheckBox;
import javafx.scene.control.TextField;
import javafx.scene.control.TextInputDialog;
import javafx.stage.FileChooser;
import us.ihmc.robotics.parameterGui.ParameterTuningTools;
import us.ihmc.robotics.parameterGui.tree.ParameterTree;
import us.ihmc.yoVariables.parameters.xml.Registry;

public class GuiController
{
   @FXML
   private TextField searchField;

   @FXML
   private CheckBox hideNamespaces;

   @FXML
   private ParameterTree tree;

   @FXML
   private Button saveAs;

   @FXML
   private Button save;

   @FXML
   private Button open;

   @FXML
   private Button create;

   private File originalFile;
   private List<Registry> registries;

   public void initialize()
   {
      searchField.textProperty().addListener(observable -> handleSearch(observable));
   }

   @FXML
   protected void handleNamespaceButton(ActionEvent event)
   {
      tree.setRegistries(registries, hideNamespaces.isSelected(), searchField.getText());
   }

   @FXML
   protected void handleCreate(ActionEvent event) throws IOException
   {
      TextInputDialog dialog = new TextInputDialog("NewRootRegistry");
      dialog.setTitle("Input Dialog");
      dialog.setHeaderText("Root Registry Name");
      dialog.setContentText("Provide the name of the new root registry.");

      Optional<String> result = dialog.showAndWait();
      if (result.isPresent())
      {
         originalFile = null;
         registries = new ArrayList<>();
         registries.add(new Registry(result.get()));
         tree.setRegistries(registries, hideNamespaces.isSelected(), searchField.getText());
      }
   }

   @FXML
   protected void handleOpen(ActionEvent event) throws IOException
   {
      FileChooser fileChooser = new FileChooser();
      FileChooser.ExtensionFilter extFilter = new FileChooser.ExtensionFilter("XML files (*.xml)", "*.xml");
      fileChooser.getExtensionFilters().add(extFilter);
      fileChooser.setTitle("Select Parameter File");
      File file = fileChooser.showOpenDialog(open.getScene().getWindow());

      if (file != null)
      {
         originalFile = file;
         registries = ParameterTuningTools.getParameters(originalFile);
         tree.setRegistries(registries, hideNamespaces.isSelected(), searchField.getText());
      }
   }

   @FXML
   protected void handleSaveAs(ActionEvent event) throws IOException
   {
      if (registries == null)
      {
         return;
      }

      FileChooser fileChooser = new FileChooser();
      FileChooser.ExtensionFilter extFilter = new FileChooser.ExtensionFilter("XML files (*.xml)", "*.xml");
      fileChooser.getExtensionFilters().add(extFilter);
      fileChooser.setTitle("Select Parameter File");
      File file = fileChooser.showSaveDialog(saveAs.getScene().getWindow());

      if (file != null)
      {
         ParameterTuningTools.save(registries, file);
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
      if (registries == null)
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
         ParameterTuningTools.save(registries, originalFile);
      }
   }

   protected void handleSearch(Observable observable)
   {
      tree.setRegistries(registries, hideNamespaces.isSelected(), searchField.getText());
   }
}
